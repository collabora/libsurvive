/* Regression test for a reentrancy bug in survive_disconnect_device()/
 * survive_close_usb_device() (driver_vive.libusb.h).
 *
 * survive_close_usb_device() cancels every interface's in-flight transfer,
 * including the one whose own handle_transfer() is still on the call
 * stack -- it doesn't skip the interface that originated the disconnect
 * (reachable here via the consecutive-timeout/RF-device path, or via
 * repeated transfer errors). If libusb's cancellation of that interface's
 * own transfer invokes its completion callback synchronously, this
 * function reenters itself: the shutdown cleanup (active_transfers--,
 * libusb_release_interface) ran once in the reentrant inner call, then
 * runs again when the outer frame resumes. active_transfers underflows
 * (0 - 1 wraps to SIZE_MAX on this unsigned counter) instead of landing on
 * 0, and libusb_release_interface is called twice for the same interface.
 *
 * This test drives the real, unmodified handle_transfer/AttachInterface
 * code (via #include of driver_vive.c, since both are `static`) against a
 * fake libusb (mock_libusb.c) whose libusb_cancel_transfer synchronously
 * re-invokes the target transfer's callback -- the worst-case reentrant
 * ordering a real libusb backend could produce -- to reproduce the bug
 * deterministically, without hardware. Reproduced below with exact
 * numbers; passes once the shutdown cleanup is made idempotent.
 */
#include "../driver_vive.c"
#include "test_case.h"

extern int mock_libusb_release_interface_calls[256];
extern int mock_libusb_max_cancel_depth;

TEST(ViveDriver, ReentrantCancelDuringDisconnect) {
	/* survive_init (not the export_config.c-style bare SV_CALLOC) because
	 * handle_transfer's SV_WARN calls survive_run_time(ctx), which needs
	 * ctx->private_members populated -- only the real init path does that. */
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
		.endpoints =
			{
				{.num = 0x81, .name = "IMU", .type = USB_IF_TRACKER1_IMU},
				{.num = 0x84, .name = "Buttons", .type = USB_IF_TRACKER1_BUTTONS},
			},
	};

	struct SurviveUSBInfo usbInfo = {0};
	usbInfo.handle = (USBHANDLE)0x1234; /* opaque to our mock; never dereferenced */
	usbInfo.viveData = &sv;
	usbInfo.device_info = &fake_device;
	usbInfo.nextCfgSubmitTime = -1; /* pretend config negotiation already finished */

	SurviveObject *so = survive_create_device(ctx, "TST", &usbInfo, "TS0", 0);
	usbInfo.so = so;

	ASSERT_EQ(AttachInterface(&sv, &usbInfo, &fake_device.endpoints[0], usbInfo.handle, survive_data_cb), 0);
	ASSERT_EQ(AttachInterface(&sv, &usbInfo, &fake_device.endpoints[1], usbInfo.handle, survive_data_cb), 0);
	ASSERT_EQ((int)usbInfo.interface_cnt, 2);
	ASSERT_EQ((int)usbInfo.active_transfers, 2);

	SurviveUSBInterface *buttons = &usbInfo.interfaces[1];
	int buttons_if_num = buttons->which_interface_am_i;

	/* Drive the Buttons interface with repeated STALLs until its error path
	 * takes `goto disconnect`, which calls survive_disconnect_device(buttons)
	 * -> survive_close_usb_device(usbInfo) -> libusb_cancel_transfer() on
	 * every interface, including buttons' own -- which our mock re-enters
	 * synchronously. Once that happens, buttons->transfer is freed and
	 * nulled, so stop driving it (how many STALLs that takes is an
	 * implementation detail of the error-retry logic, not of this bug). */
	for (int i = 0; i < 10 && buttons->transfer; i++) {
		buttons->transfer->status = LIBUSB_TRANSFER_STALL;
		handle_transfer(buttons->transfer);
	}

	TEST_PRINTF("max reentrant cancel depth: %d\n", mock_libusb_max_cancel_depth);
	TEST_PRINTF("active_transfers after disconnect: %zu (expected 0)\n", usbInfo.active_transfers);
	TEST_PRINTF("libusb_release_interface calls for buttons interface %d: %d (expected 1)\n", buttons_if_num,
				mock_libusb_release_interface_calls[buttons_if_num]);
	TEST_PRINTF("request_close: %d (expected 1)\n", (int)usbInfo.request_close);

	/* Reentrancy actually happened: libusb_cancel_transfer's synchronous
	 * callback invocation fired at least once (for the self-cancellation). */
	ASSERT_GE((double)mock_libusb_max_cancel_depth, 1.0);

	/* The bug: active_transfers should land exactly on 0 (both interfaces
	 * cleaned up once each). A double-decrement for the self-reentered
	 * interface underflows the unsigned counter instead. */
	ASSERT_EQ((int)usbInfo.active_transfers, 0);

	/* The bug: libusb_release_interface should be called exactly once per
	 * interface, not twice for the one that reentered itself. */
	ASSERT_EQ(mock_libusb_release_interface_calls[buttons_if_num], 1);

	/* request_close does get set here (by the inner, reentrant cleanup,
	 * before the outer frame's redundant decrement corrupts the counter
	 * further) -- the dangerous state is the corrupted active_transfers
	 * and the double-released interface above, not this flag. */
	ASSERT_EQ((int)usbInfo.request_close, 1);

	return 0;
}
