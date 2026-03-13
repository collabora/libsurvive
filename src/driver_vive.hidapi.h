#include "driver_vive.h"
#include <assert.h>
#include <errno.h>
#include <hidapi.h>
#include <survive.h>
#define LIBUSB_TRANSFER_COMPLETED 0
#define LIBUSB_TRANSFER_STALL -1
#define LIBUSB_TRANSFER_TIMED_OUT -2

typedef struct survive_usb_transfer_t {
	enum survive_usb_transfer_type {
		survive_usb_transfer_type_read_feature_report,
		survive_usb_transfer_type_send_feature_report,
	} type;
	struct SurviveUSBInfo *dev;
	void (*cb)(struct survive_usb_transfer_t *);
	void *user_data;
	uint8_t *buffer;
	int status;
	bool processed;
	size_t length;
	size_t actual_length;
	uint32_t timeout;
} survive_usb_transfer_t;

static inline uint8_t *survive_usb_transfer_data(survive_usb_transfer_t *tx) { return tx->buffer; }
static inline size_t survive_usb_transfer_length(survive_usb_transfer_t *tx) { return tx->length; }
static inline void *survive_usb_transfer_alloc() { return calloc(1, sizeof(survive_usb_transfer_t)); }
static inline void survive_usb_transfer_free(survive_usb_transfer_t *tx) { free(tx); }
static inline void survive_usb_transfer_cancel(survive_usb_transfer_t *tx) {}
static inline int survive_usb_transfer_submit(survive_usb_transfer_t *tx) {
	int rtn = 0;
	// char tmp[64];
	// while (hid_read_timeout(tx->dev->handle->interfaces[0], tmp, sizeof(tmp), 0) > 0) {}

	switch (tx->type) {
	case survive_usb_transfer_type_read_feature_report: {
		rtn = hid_get_feature_report(tx->dev->handle->interfaces[0], tx->buffer, tx->actual_length);
		break;
	}
	case survive_usb_transfer_type_send_feature_report: {
		rtn = hid_send_feature_report(tx->dev->handle->interfaces[0], tx->buffer, tx->length);
		break;
	}
	}

	tx->processed = false;
	if (rtn > 0) {
		tx->status = LIBUSB_TRANSFER_COMPLETED;
		tx->length = rtn;
	} else if (rtn == -1) {
		tx->status = -2;
	} else {
		tx->status = LIBUSB_TRANSFER_TIMED_OUT;
	}

	return 0;
}
static inline void survive_usb_setup_get_feature_report(survive_usb_transfer_t *tx, uint8_t report_id) {
	tx->type = survive_usb_transfer_type_read_feature_report;
	tx->buffer[0] = report_id;
	tx->length = tx->actual_length;
}

static inline void survive_usb_setup_update_feature_report(survive_usb_transfer_t *tx, const uint8_t *data,
														   size_t datalen) {
	tx->type = survive_usb_transfer_type_send_feature_report;
	memcpy(tx->buffer, data, datalen);
	tx->length = datalen;
}

static inline void survive_usb_setup_control(survive_usb_transfer_t *tx, struct SurviveUSBInfo *usbInfo,
											 void (*cb)(survive_usb_transfer_t *), void *user, int32_t timeout) {
	tx->cb = cb;
	tx->dev = usbInfo;
	tx->user_data = user;
	tx->timeout = timeout;
}

static inline int update_feature_report(USBHANDLE dev, uint16_t iface, uint8_t *data, int datalen) {
	return hid_send_feature_report(dev->interfaces[iface], data, datalen);
}

static void *HAPIReceiver(void *v) {
	SurviveUSBInterface *iface = v;
	USB_INTERFACE_HANDLE *hp = &iface->uh;

	if ((iface->actual_len = hid_read(*hp, iface->buffer, sizeof(iface->swap_buffer[0]) )) > 0) {
		// if( iface->actual_len  == 52 ) continue;
		iface->packet_count++;
		survive_data_cb(OGGetAbsoluteTimeUS(), iface);
	}
	if (iface->actual_len < 0) {
		SurviveContext *ctx = iface->sv->ctx;
		iface->usbInfo->request_close = true;
	}
	return 0;
}

typedef struct hid_device_info *survive_usb_device_t;
typedef struct hid_device_info *survive_usb_devices_t;

static int survive_usb_subsystem_init(SurviveViveData *sv) { return hid_init(); }
static int survive_get_usb_devices(SurviveViveData *sv, survive_usb_devices_t *devs) {
	*devs = hid_enumerate(0, 0);
	SurviveContext * ctx = sv->ctx;
    int cnt = 0;
	for(survive_usb_devices_t d = *devs; d; d = d->next) {
	    cnt++;
        SV_VERBOSE(110, "HID Enumerate %04x:%04x:%02d (%S) %4d/%4d/%4d path: %s", d->vendor_id, d->product_id, d->interface_number, d->serial_number, d->release_number, d->usage, d->usage_page, d->path);
	}
    SV_VERBOSE(110, "HID Enumeration finished %d devices", cnt);
	return 0;
}
static void survive_free_usb_devices(survive_usb_devices_t devs) { hid_free_enumeration(devs); }
void survive_usb_handle_close(libsurvive_usb_handle *handle) {}
static int survive_get_ids(survive_usb_device_t d, uint16_t *idVendor, uint16_t *idProduct, uint8_t *class_id) {
	*idVendor = d->vendor_id;
	*idProduct = d->product_id;
	*class_id = 1;

	return 0;
}

static const char *survive_usb_error_name(int ret) { return ""; }

static int survive_open_usb_device(SurviveViveData *sv, survive_usb_device_t d, struct SurviveUSBInfo *usbInfo) {
	usbInfo->handle = SV_CALLOC(sizeof(struct HIDAPI_USB_Handle_t));

	struct SurviveContext *ctx = sv->ctx;

	survive_usb_devices_t devs;
	int ret = survive_get_usb_devices(sv, &devs);

	if (ret < 0) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Couldn't get list of USB devices %d (%s)", ret,
				 survive_usb_error_name(ret));
		return ret;
	}

	if (d->serial_number == 0) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Couldn't get serial number for device %s", usbInfo->device_info->name);
		return -1;
	}

	for (survive_usb_device_t c = devs; c; c = c->next) {
		int interface_num = c->interface_number;
		interface_num = interface_num < 0 ? 0 : interface_num;
		if (c && c->serial_number && wcscmp(c->serial_number, d->serial_number) == 0) {
            SV_VERBOSE(100, "Opening %04x:%04x:%02d (%S) (path %s)", c->vendor_id, c->vendor_id, interface_num, d->serial_number, c->path);
			usbInfo->handle->interfaces[interface_num] = hid_open_path(c->path);

			if (!usbInfo->handle->interfaces[interface_num]) {
				SV_INFO("Warning: Could not find vive device %04x:%04x", d->vendor_id, d->product_id);
				return -1;
			}
		}
	}

	SV_VERBOSE(40, "Successfully enumerated %s at %.7f", survive_colorize(usbInfo->device_info->name),
			   survive_run_time(ctx));

	survive_free_usb_devices(devs);

	return 0;
}
typedef survive_usb_device_t survive_usb_device_enumerator;

static survive_usb_device_t get_next_device(survive_usb_device_enumerator* d, survive_usb_device_t head) {
	if (*d == 0) {
		return *d = head;
	}
	
	// On Windows the wireless dongle interface is -1
	// That's why we check for <= 0
	for (survive_usb_device_t c = (*d)->next; c; c = c->next) {
		if (c->interface_number <= 0) {
			return *d = c; 
		}
	}
	return 0; 
}

static bool setup_hotplug(SurviveViveData *sv) { return true; }

static inline void survive_close_usb_device(struct SurviveUSBInfo *usbInfo) {
	for (int j = 0; j < 8; j++) {
		hid_close(usbInfo->handle->interfaces[j]);
	}

	free(usbInfo->handle);
#ifndef HID_NONBLOCKING
	for (int j = 0; j < MAX_INTERFACES_PER_DEVICE; j++) {
		OGJoinThread(sv->udev[i].interfaces->servicethread);
	}
#endif
	usbInfo->request_close = true;
}

void survive_usb_close(SurviveViveData *sv) {}
