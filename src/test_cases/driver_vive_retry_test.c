/* Regression test for handle_transfer()'s USB transfer retry/error-handling
 * logic (driver_vive.libusb.h). Three distinct bugs, all in the same
 * function:
 *
 * 1. On LIBUSB_TRANSFER_TIMED_OUT, below the consecutive-timeout/RF-device
 *    disconnect threshold, the function returned without resubmitting the
 *    transfer -- the endpoint then never receives another completion,
 *    since nothing else resubmits it.
 * 2. error_count was incremented twice per call (`error_count++` then
 *    `if (error_count++ < 10)`), reaching the disconnect threshold in half
 *    the intended number of errors.
 * 3. After a successful resubmit in the error path, execution fell through
 *    to `goto disconnect` regardless -- so a single transient error (one
 *    STALL, say) disconnected the device instead of retrying.
 *
 * A fourth, related fix: the error path never called libusb_clear_halt()
 * before retrying a LIBUSB_TRANSFER_STALL, so the retry was guaranteed to
 * stall again.
 *
 * This test drives the real, unmodified handle_transfer/AttachInterface
 * code (via #include of driver_vive.c, since both are `static`) against a
 * fake libusb (mock_libusb.c) that records submit/clear_halt calls, to
 * check the retry behavior directly rather than inferring it from side
 * effects.
 */
#include "../driver_vive.c"
#include "test_case.h"

extern int mock_libusb_submit_transfer_calls;
extern int mock_libusb_clear_halt_calls;
extern unsigned char mock_libusb_clear_halt_last_endpoint;

static SurviveUSBInterface *setup_one_interface(SurviveContext *ctx, SurviveViveData *sv, struct SurviveUSBInfo *usbInfo,
												 struct DeviceInfo *fake_device) {
	usbInfo->handle = (USBHANDLE)0x1234; /* opaque to our mock; never dereferenced */
	usbInfo->viveData = sv;
	usbInfo->device_info = fake_device;
	usbInfo->nextCfgSubmitTime = -1; /* pretend config negotiation already finished */

	SurviveObject *so = survive_create_device(ctx, "TST", usbInfo, "TS0", 0);
	usbInfo->so = so;

	AttachInterface(sv, usbInfo, &fake_device->endpoints[0], usbInfo->handle, survive_data_cb);
	return &usbInfo->interfaces[0];
}

TEST(ViveDriver, TimeoutBelowThresholdResubmits) {
	SurviveContext *ctx = survive_init(0, 0);
	ASSERT_EQ(ctx != 0, 1);

	SurviveViveData sv = {0};
	sv.ctx = ctx;
	struct DeviceInfo fake_device = {
		.name = "Test Tracker",
		.codename = "TS0",
		.vid = 0x28de,
		.pid = 0x2300,
		.type = USB_DEV_TRACKER1,
		.endpoints = {{.num = 0x81, .name = "IMU", .type = USB_IF_TRACKER1_IMU}},
	};
	struct SurviveUSBInfo usbInfo = {0};
	SurviveUSBInterface *iface = setup_one_interface(ctx, &sv, &usbInfo, &fake_device);

	mock_libusb_submit_transfer_calls = 0;
	iface->transfer->status = LIBUSB_TRANSFER_TIMED_OUT;
	handle_transfer(iface->transfer);

	TEST_PRINTF("consecutive_timeouts: %u, submit calls: %d, shutdown: %d\n", iface->consecutive_timeouts,
				mock_libusb_submit_transfer_calls, (int)iface->shutdown);

	/* Below the disconnect threshold (consecutive_timeouts < 3, not an RF
	 * device): the transfer must be resubmitted, and the interface must
	 * not have been torn down. */
	ASSERT_EQ((int)iface->consecutive_timeouts, 1);
	ASSERT_EQ(mock_libusb_submit_transfer_calls, 1);
	ASSERT_EQ((int)iface->shutdown, 0);

	return 0;
}

TEST(ViveDriver, SingleTransientErrorRetriesInsteadOfDisconnecting) {
	SurviveContext *ctx = survive_init(0, 0);
	ASSERT_EQ(ctx != 0, 1);

	SurviveViveData sv = {0};
	sv.ctx = ctx;
	struct DeviceInfo fake_device = {
		.name = "Test Tracker",
		.codename = "TS0",
		.vid = 0x28de,
		.pid = 0x2300,
		.type = USB_DEV_TRACKER1,
		.endpoints = {{.num = 0x81, .name = "IMU", .type = USB_IF_TRACKER1_IMU}},
	};
	struct SurviveUSBInfo usbInfo = {0};
	SurviveUSBInterface *iface = setup_one_interface(ctx, &sv, &usbInfo, &fake_device);

	mock_libusb_submit_transfer_calls = 0;
	mock_libusb_clear_halt_calls = 0; /* AttachInterface() above also calls clear_halt once, at setup */
	iface->transfer->status = LIBUSB_TRANSFER_STALL;
	handle_transfer(iface->transfer);

	TEST_PRINTF("error_count: %u (expected 1), submit calls: %d (expected 1), shutdown: %d (expected 0), "
				"clear_halt calls: %d (expected 1), clear_halt endpoint: 0x%02x (expected 0x81)\n",
				iface->error_count, mock_libusb_submit_transfer_calls, (int)iface->shutdown, mock_libusb_clear_halt_calls,
				mock_libusb_clear_halt_last_endpoint);

	/* A single transient error must increment error_count by exactly one
	 * (not two), clear the halt condition before retrying a STALL, and
	 * retry (resubmit) instead of disconnecting the device outright. */
	ASSERT_EQ((int)iface->error_count, 1);
	ASSERT_EQ(mock_libusb_clear_halt_calls, 1);
	ASSERT_EQ((int)mock_libusb_clear_halt_last_endpoint, 0x81);
	ASSERT_EQ(mock_libusb_submit_transfer_calls, 1);
	ASSERT_EQ((int)iface->shutdown, 0);
	ASSERT_EQ((int)usbInfo.active_transfers, 1);

	return 0;
}

TEST(ViveDriver, TenthConsecutiveErrorStillDisconnects) {
	SurviveContext *ctx = survive_init(0, 0);
	ASSERT_EQ(ctx != 0, 1);

	SurviveViveData sv = {0};
	sv.ctx = ctx;
	struct DeviceInfo fake_device = {
		.name = "Test Tracker",
		.codename = "TS0",
		.vid = 0x28de,
		.pid = 0x2300,
		.type = USB_DEV_TRACKER1,
		.endpoints = {{.num = 0x81, .name = "IMU", .type = USB_IF_TRACKER1_IMU}},
	};
	struct SurviveUSBInfo usbInfo = {0};
	SurviveUSBInterface *iface = setup_one_interface(ctx, &sv, &usbInfo, &fake_device);

	/* With the double-increment bug fixed, it now genuinely takes 10
	 * consecutive errors (not 5) to reach the disconnect threshold, and
	 * the interface must survive (keep retrying) through all of them. */
	for (int i = 0; i < 9 && iface->transfer; i++) {
		iface->transfer->status = LIBUSB_TRANSFER_STALL;
		handle_transfer(iface->transfer);
	}
	TEST_PRINTF("error_count after 9 errors: %u (expected 9), shutdown: %d (expected 0)\n", iface->error_count,
				(int)iface->shutdown);
	ASSERT_EQ((int)iface->error_count, 9);
	ASSERT_EQ((int)iface->shutdown, 0);

	/* The 10th consecutive error must disconnect. (Checking iface->shutdown
	 * rather than usbInfo.active_transfers here: survive_close_usb_device()
	 * sets shutdown unconditionally up front, before it gets to canceling
	 * transfers -- a separate, unrelated bug in that later step, where it
	 * doesn't skip the interface that originated the disconnect, is its
	 * own fix with its own regression test.) */
	iface->transfer->status = LIBUSB_TRANSFER_STALL;
	handle_transfer(iface->transfer);
	TEST_PRINTF("shutdown after 10th error: %d (expected 1)\n", (int)iface->shutdown);
	ASSERT_EQ((int)iface->shutdown, 1);

	return 0;
}
