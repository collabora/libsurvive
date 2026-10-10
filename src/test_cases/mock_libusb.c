/* Fake libusb used only by driver_vive_reentrancy_test.c, to drive
 * driver_vive.c's handle_transfer/survive_disconnect_device without real
 * USB hardware. The real libusb library is NOT linked into that test
 * target -- these definitions are the only ones the linker sees.
 *
 * libusb_cancel_transfer is the one that matters: it synchronously
 * re-invokes the transfer's own completion callback with
 * LIBUSB_TRANSFER_CANCELLED, simulating the worst-case reentrant
 * dispatch a real libusb backend could produce when cancellation
 * completes inline. Everything else is a trivial stub -- present only
 * so the linker is satisfied; their behavior doesn't matter to the
 * scenario under test.
 */
/* Matches the include-path resolution in ../driver_vive.h. */
#ifdef SURVIVE_LIBUSB_NO_DIR
#include <libusb.h>
#elif defined(SURVIVE_LIBUSB_UNVER_DIR)
#include <libusb/libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#include <stdlib.h>
#include <string.h>

int mock_libusb_release_interface_calls[256];
int mock_libusb_cancel_depth = 0;
int mock_libusb_max_cancel_depth = 0;

int libusb_init(libusb_context **ctx) { return 0; }
void libusb_exit(libusb_context *ctx) {}
void libusb_set_debug(libusb_context *ctx, int level) {}
int libusb_set_option(libusb_context *ctx, enum libusb_option option, ...) { return 0; }
const struct libusb_version *libusb_get_version(void) {
	static struct libusb_version v = {1, 0, 27, 0, "", ""};
	return &v;
}
int libusb_has_capability(uint32_t capability) { return 0; }
const char *libusb_error_name(int errcode) { return "mock_error"; }

ssize_t libusb_get_device_list(libusb_context *ctx, libusb_device ***list) {
	*list = NULL;
	return 0;
}
void libusb_free_device_list(libusb_device **list, int unref_devices) {}

int libusb_open(libusb_device *dev, libusb_device_handle **handle) {
	*handle = (libusb_device_handle *)dev;
	return 0;
}
void libusb_close(libusb_device_handle *dev_handle) {}
libusb_device *libusb_get_device(libusb_device_handle *dev_handle) { return (libusb_device *)dev_handle; }

int libusb_get_device_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc) {
	memset(desc, 0, sizeof(*desc));
	return 0;
}
int libusb_get_config_descriptor(libusb_device *dev, uint8_t config_index, struct libusb_config_descriptor **config) {
	static struct libusb_config_descriptor conf;
	memset(&conf, 0, sizeof(conf));
	*config = &conf;
	return 0;
}
void libusb_free_config_descriptor(struct libusb_config_descriptor *config) {}

int libusb_reset_device(libusb_device_handle *dev_handle) { return 0; }
int libusb_set_auto_detach_kernel_driver(libusb_device_handle *dev_handle, int enable) { return 0; }
int libusb_claim_interface(libusb_device_handle *dev_handle, int interface_number) { return 0; }
int libusb_release_interface(libusb_device_handle *dev_handle, int interface_number) {
	if (interface_number >= 0 && interface_number < 256)
		mock_libusb_release_interface_calls[interface_number]++;
	return 0;
}
int libusb_clear_halt(libusb_device_handle *dev_handle, unsigned char endpoint) { return 0; }

struct libusb_transfer *libusb_alloc_transfer(int iso_packets) {
	struct libusb_transfer *tx = calloc(1, sizeof(struct libusb_transfer));
	return tx;
}
void libusb_free_transfer(struct libusb_transfer *transfer) {
	if (transfer)
		free(transfer);
}
int libusb_submit_transfer(struct libusb_transfer *transfer) { return 0; }

/* The scenario under test: survive_close_usb_device() cancels every
 * interface's in-flight transfer, including the one whose own
 * handle_transfer is still executing further up the call stack. Firing
 * the callback synchronously here reproduces the worst case ordering a
 * real libusb backend could produce. */
int libusb_cancel_transfer(struct libusb_transfer *transfer) {
	if (!transfer || !transfer->callback)
		return 0;
	mock_libusb_cancel_depth++;
	if (mock_libusb_cancel_depth > mock_libusb_max_cancel_depth)
		mock_libusb_max_cancel_depth = mock_libusb_cancel_depth;
	transfer->status = LIBUSB_TRANSFER_CANCELLED;
	transfer->callback(transfer);
	mock_libusb_cancel_depth--;
	return 0;
}

int libusb_handle_events_timeout(libusb_context *ctx, struct timeval *tv) { return 0; }
int libusb_handle_events(libusb_context *ctx) { return 0; }
int libusb_handle_event(libusb_context *ctx) { return 0; }

int libusb_hotplug_register_callback(libusb_context *ctx, int events, int flags, int vendor_id, int product_id,
									  int dev_class, libusb_hotplug_callback_fn cb_fn, void *user_data,
									  libusb_hotplug_callback_handle *callback_handle) {
	return LIBUSB_ERROR_NOT_SUPPORTED;
}
void libusb_hotplug_deregister_callback(libusb_context *ctx, libusb_hotplug_callback_handle callback_handle) {}
