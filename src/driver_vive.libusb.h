
typedef struct libusb_transfer survive_usb_transfer_t;
typedef libusb_device *survive_usb_device_t;
typedef libusb_device **survive_usb_devices_t;

static inline uint8_t *survive_usb_transfer_data(survive_usb_transfer_t *tx) { return tx->buffer + 8; }
static inline void *survive_usb_transfer_alloc() { return libusb_alloc_transfer(0); }
static inline void survive_usb_transfer_free(survive_usb_transfer_t *tx) { libusb_free_transfer(tx); }
static inline size_t survive_usb_transfer_length(survive_usb_transfer_t *tx) {
	return tx->actual_length >= 8 ? tx->actual_length - 8 : 0;
}
static inline void survive_usb_transfer_cancel(survive_usb_transfer_t *tx) { libusb_cancel_transfer(tx); }
static inline int survive_usb_transfer_submit(survive_usb_transfer_t *tx) { return libusb_submit_transfer(tx); }
static inline void survive_usb_setup_get_feature_report(survive_usb_transfer_t *tx, uint8_t report_id) {
	tx->buffer[8] = report_id;
	libusb_fill_control_setup(tx->buffer, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_IN,
							  0x01, 0x300 | report_id, 0, 256);
}

static inline void survive_usb_setup_update_feature_report(survive_usb_transfer_t *tx, const uint8_t *data,
														   size_t datalen) {
	memcpy(tx->buffer + 8, data, datalen);
	libusb_fill_control_setup(tx->buffer, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
							  0x09, 0x300 | tx->buffer[8], 0, datalen + 8);
}

static inline void survive_usb_setup_control(survive_usb_transfer_t *tx, struct SurviveUSBInfo *usbInfo,
											 void (*cb)(survive_usb_transfer_t *), void *user, int32_t timeout) {
	libusb_fill_control_transfer(tx, usbInfo->handle, tx->buffer, cb, user, timeout);
}

void survive_usb_handle_close(libsurvive_usb_handle *handle) { libusb_close(handle); }

void monitor_transfer(struct libusb_transfer *transfer) {
	assert(transfer->status == 0);
	survive_usb_transfer_free(transfer);
}

int libusb_control_transfer_async(libusb_device_handle *dev_handle, uint8_t bmRequestType, uint8_t bRequest,
								  uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength,
								  unsigned int timeout) {
	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	unsigned char *buffer;
	int r;
	if (!transfer)
		return LIBUSB_ERROR_NO_MEM;

	buffer = SV_MALLOC(LIBUSB_CONTROL_SETUP_SIZE + wLength);
	if (!buffer) {
		survive_usb_transfer_free(transfer);
		return LIBUSB_ERROR_NO_MEM;
	}
	libusb_fill_control_setup(buffer, bmRequestType, bRequest, wValue, wIndex, wLength);
	if ((bmRequestType & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT)
		memcpy(buffer + LIBUSB_CONTROL_SETUP_SIZE, data, wLength);
	libusb_fill_control_transfer(transfer, dev_handle, buffer, monitor_transfer, dev_handle, timeout);
	transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;
	r = survive_usb_transfer_submit(transfer);
	if (r < 0) {
		survive_usb_transfer_free(transfer);
		return r;
	}
	return wLength;
}

static inline int update_feature_report(libusb_device_handle *dev, uint16_t interface, uint8_t *data, int datalen) {
	return libusb_control_transfer_async(dev,
										 LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
										 0x09, 0x300 | data[0], interface, data, datalen, 1000);
}

static int survive_get_usb_devices(SurviveViveData *sv, survive_usb_devices_t *devs) {
	return libusb_get_device_list(sv->usbctx, devs);
}
static void survive_free_usb_devices(survive_usb_devices_t devs) { libusb_free_device_list(devs, 1); }

typedef int survive_usb_device_enumerator;
static survive_usb_device_t get_next_device(survive_usb_device_enumerator *iterator, survive_usb_devices_t list) {
	assert(iterator);
	return list[(*iterator)++];
}

STATIC_CONFIG_ITEM(LIBUSB_LOG_LEVEL, "libusb-log-level", 'i', "Log level of libusb", LIBUSB_LOG_LEVEL_WARNING)
static int survive_usb_subsystem_init(SurviveViveData *sv) {
	int rtn = libusb_init(&sv->usbctx);
	int log_level = survive_configi(sv->ctx, LIBUSB_LOG_LEVEL_TAG, SC_GET, LIBUSB_LOG_LEVEL_WARNING);
#if LIBUSB_API_VERSION < 0x01000106
	libusb_set_debug(NULL, log_level);
#else
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, log_level);
#endif

	const struct libusb_version *v = libusb_get_version();
	SurviveContext *ctx = sv->ctx;
	SV_VERBOSE(10, "libusb version %d.%d.%d.%d (log level %d)", v->major, v->minor, v->micro, v->nano, log_level);
	return rtn;
}

int survive_vive_add_usb_device(SurviveViveData *sv, survive_usb_device_t d);
int libusb_hotplug(libusb_context *usbctx, libusb_device *device, libusb_hotplug_event event, void *user_data) {
	SurviveViveData *sv = user_data;
	SurviveContext *ctx = sv->ctx;

	if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
		SV_VERBOSE(100, "Device added %p", device);
		survive_vive_add_usb_device(sv, device);
	} else {
		SV_VERBOSE(100, "Device removed %p", device);
	}

	return 0;
}
static bool setup_hotplug(SurviveViveData *sv) {
	SurviveContext *ctx = sv->ctx;
	if (!libusb_has_capability(LIBUSB_CAP_HAS_HOTPLUG)) {
		SV_WARN("Hotplug capabilities are not supported on this platform");
		return 1;
	}

	int rc = libusb_hotplug_register_callback(
		sv->usbctx, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT, LIBUSB_HOTPLUG_ENUMERATE,
		LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY, libusb_hotplug, sv,
		&sv->callback_handle);

	if (LIBUSB_SUCCESS != rc) {
		SV_WARN("Could not register hotplug callback err: %d", rc);
		return rc;
	}
	return LIBUSB_SUCCESS;
}

static int survive_get_ids(survive_usb_device_t d, uint16_t *idVendor, uint16_t *idProduct, uint8_t *class_id) {
	struct libusb_device_descriptor desc;

	int ret = libusb_get_device_descriptor(d, &desc);
	*idVendor = 0;
	*idProduct = 0;
	if (class_id)
		*class_id = 0;
	if (ret)
		return ret;

	*idVendor = desc.idVendor;
	*idProduct = desc.idProduct;
	if (class_id)
		*class_id = desc.bDeviceClass;

	return ret;
}

static const char *survive_usb_error_name(int ret) { return libusb_error_name(ret); }

static int survive_open_usb_device(SurviveViveData *sv, survive_usb_device_t d, struct SurviveUSBInfo *usbInfo) {
	struct libusb_config_descriptor *conf;
	int ret = libusb_get_config_descriptor(d, 0, &conf);
	if (ret)
		goto cleanup_and_rtn;

	const struct DeviceInfo *info = usbInfo->device_info;
	ret = libusb_open(d, &usbInfo->handle);

	uint16_t idVendor;
	uint16_t idProduct;
	survive_get_ids(d, &idVendor, &idProduct, 0);

	SurviveContext *ctx = sv->ctx;
	if (!usbInfo->handle || ret) {
		SV_WARN("Cannot open device \"%s\" with vid/pid %04x:%04x error %d (%s)", survive_colorize(info->name),
				idVendor, idProduct, ret, libusb_error_name(ret));
		goto cleanup_and_rtn;
	}

	libusb_set_auto_detach_kernel_driver(usbInfo->handle, 1);
	for (int j = 0; j < conf->bNumInterfaces; j++) {
		bool interface_is_microphone = false;

		for (int k = 0; k < conf->interface[j].num_altsetting; k++) {
			const struct libusb_interface_descriptor *d = &conf->interface[j].altsetting[k];
			if (d->bInterfaceClass == LIBUSB_CLASS_AUDIO) {
				interface_is_microphone = true;
			}
		}
		if (interface_is_microphone) {
		        SV_VERBOSE(10, "Not claiming interface %d of %s since it is an audio interface", j, survive_colorize(info->name));
			continue;
		}

		ret = libusb_claim_interface(usbInfo->handle, j);
		if (ret != 0) {
			SV_WARN("Could not claim interface %d of %s: %d %s", j, survive_colorize(info->name), ret,
					libusb_error_name(ret));
			goto cleanup_and_rtn;
		}
	}

	SV_VERBOSE(40, "Successfully enumerated %s (%d) %04x:%04x at %.7f", survive_colorize(info->name),
			   conf->bNumInterfaces, idVendor, idProduct, survive_run_time(ctx));

cleanup_and_rtn:
	libusb_free_config_descriptor(conf);
	return ret;
}

static inline void survive_close_usb_device(struct SurviveUSBInfo *usbInfo);

static void survive_disconnect_device(SurviveUSBInterface *iface) {
	iface->ctx = 0;
	survive_close_usb_device(iface->usbInfo);
}
static void handle_transfer(struct libusb_transfer *transfer) {
	uint64_t time = OGGetAbsoluteTimeUS();

	SurviveUSBInterface *iface = transfer->user_data;
	SurviveContext *ctx = iface->ctx;
	if (!iface->shutdown && transfer->status == LIBUSB_TRANSFER_TIMED_OUT) {
        iface->consecutive_timeouts++;
		bool is_rf_device = survive_device_is_rf(iface->usbInfo->device_info);
        if(iface->consecutive_timeouts >= 3 || is_rf_device) {
            SV_WARN("%f %s Device turned off: %d", survive_run_time(ctx), survive_colorize_codename(iface->assoc_obj),
                    transfer->status);
            goto object_turned_off;
        } else {
            return;
        }
	}

	if (!iface->shutdown && transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		SV_WARN("%f %s Device disconnect: %d", survive_run_time(ctx), survive_colorize_codename(iface->assoc_obj),
				transfer->status);
		iface->error_count++;
		if (iface->error_count++ < 10) {
			if (libusb_submit_transfer(transfer)) {
				goto shutdown;
			}
		}

		goto disconnect;
	}

	if (iface->shutdown) {
		goto shutdown;
	}

	iface->error_count = 0;
	iface->actual_len = transfer->actual_length;
	iface->buffer = iface->swap_buffer[iface->swap_buffer_idx++ % 2];

	transfer->buffer = iface->swap_buffer[iface->swap_buffer_idx % 2];

	uint64_t submit_cb_time = OGGetAbsoluteTimeUS() - iface->last_submit_time;

	iface->last_submit_time = OGGetAbsoluteTimeUS();

	// If we get at least one packet; start applying a timeout
	if (iface->assoc_obj && iface->assoc_obj->object_type != SURVIVE_OBJECT_TYPE_HMD) {
		transfer->timeout = 1000;
	}
    iface->consecutive_timeouts = 0;
	if (libusb_submit_transfer(transfer)) {
		goto shutdown;
	}

	if (iface->max_submit_time < submit_cb_time)
		iface->max_submit_time = submit_cb_time;
	uint64_t cb_start = OGGetAbsoluteTimeUS();
	iface->sum_submit_cb_time += submit_cb_time;
	iface->cb(time, iface);
	uint64_t cb_end = OGGetAbsoluteTimeUS();
	uint64_t cb_time = cb_end - cb_start;
	if (iface->max_cb_time < cb_time)
		iface->max_cb_time = cb_time;
	if (iface->time_constraint && cb_time > iface->time_constraint)
		iface->cb_time_violation++;

	iface->sum_cb_time += cb_time;
	iface->packet_count++;

	return;
object_turned_off:
	iface->usbInfo->request_reopen = true;
disconnect:
	survive_disconnect_device(iface);
shutdown:
	SV_VERBOSE(200, "Cleaning up transfer on %d %s", iface->which_interface_am_i, survive_colorize(iface->hname));
	iface->ctx = 0;

	libusb_release_interface(iface->usbInfo->handle, iface->which_interface_am_i);
	survive_usb_transfer_free(iface->transfer);
	iface->transfer = 0;

	iface->usbInfo->active_transfers--;
	if (iface->usbInfo->active_transfers == 0) {
		iface->usbInfo->request_close = true;
		SV_VERBOSE(100, "Requesting close for %s", survive_colorize_codename(iface->assoc_obj));
	}
}

static int survive_config_submit(struct SurviveUSBInfo *usbInfo);
static void survive_config_cancel(struct survive_config_packet *cfg);
static inline bool survive_handle_close_request_flag(struct SurviveUSBInfo *usbInfo);
static inline void survive_close_usb_device(struct SurviveUSBInfo *usbInfo) {
	usbInfo->interfaces[0].shutdown = 1;
	for (size_t j = 0; j < usbInfo->interface_cnt; j++) {
		usbInfo->interfaces[j].shutdown = 1;
		usbInfo->interfaces[j].assoc_obj = 0;
	}

	assert(usbInfo->interfaces[0].assoc_obj == 0);
	SurviveContext *ctx = usbInfo->viveData->ctx;
	if (usbInfo->nextCfgSubmitTime >= 0) {
		survive_config_submit(usbInfo);
	}

	struct survive_config_packet *cfg = usbInfo->cfg_user;
	if (cfg) {
		survive_config_cancel(cfg);
	}
	if (usbInfo->active_transfers == 0) {
		usbInfo->request_close = true;
		SV_VERBOSE(100, "Acking close for %s", survive_colorize_codename(usbInfo->so));
	}

	SV_VERBOSE(100, "Closing device on %s %p (%p)", survive_colorize_codename(usbInfo->so), cfg, usbInfo);

	for (int j = 0; j < usbInfo->interface_cnt; j++) {
		SurviveUSBInterface *iface = &usbInfo->interfaces[j];
		SV_VERBOSE(100, "Cleaning up interface on %d %s %s (%p)", iface->which_interface_am_i,
				   survive_colorize_codename(iface->usbInfo->so), survive_colorize(iface->hname),
				   usbInfo->interfaces[j].transfer);
		if (usbInfo->interfaces[j].transfer)
			libusb_cancel_transfer(usbInfo->interfaces[j].transfer);
	}
}

void survive_usb_close(SurviveViveData *sv) { libusb_exit(sv->usbctx); }
