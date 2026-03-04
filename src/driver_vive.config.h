
static int survive_config_submit(struct SurviveUSBInfo *usbInfo) {
	struct survive_config_packet *config_packet = usbInfo->cfg_user;
	SurviveContext *ctx = config_packet->sv->ctx;

	SV_VERBOSE(110, "Submitting config for %s %s at %f",
			   survive_colorize(usbInfo->so ? usbInfo->so->codename : usbInfo->device_info->codename),
			   survive_colorize(usbInfo->device_info->name), usbInfo->nextCfgSubmitTime);
	usbInfo->nextCfgSubmitTime = -1;

	int rc = survive_usb_transfer_submit(config_packet->tx);
	if (rc) {
		SV_WARN("Failed to submit transfer %s %s (%d)", survive_colorize_codename(usbInfo->so),
				survive_colorize(usbInfo->device_info->name), rc);
		return -6;
	}
	return 0;
}

static inline void setup_config_req(struct survive_config_packet *config_packet) {
	config_packet->state = SURVIVE_CONFIG_STATE_CONFIG;
	survive_usb_setup_get_feature_report(config_packet->tx, VIVE_REPORT_CONFIG_READMODE);
}
static inline void setup_version(struct survive_config_packet *packet) {
	packet->state = SURVIVE_CONFIG_STATE_VERSION;
	survive_usb_setup_get_feature_report(packet->tx, VIVE_REPORT_VERSION);
}
static inline void setup_config_imu_scales(struct survive_config_packet *packet) {
	packet->state = SURVIVE_CONFIG_STATE_IMU_SCALES;
	survive_usb_setup_get_feature_report(packet->tx, VIVE_REPORT_IMU_SCALES);
}

static inline void setup_magic(struct survive_config_packet *packet) {
	SurviveContext *ctx = packet->ctx;
	SurviveObject *so = packet->usbInfo->so;

	packet->state = SURVIVE_CONFIG_STATE_MAGICS;

	SV_VERBOSE(100, "Submitting magic %s at %f sec for %s - %s (length %d)",
			   survive_colorize(packet->current_magic->name), survive_run_time(ctx) - packet->start_time,
			   survive_colorize(so ? so->codename : "BRD"), survive_colorize(packet->usbInfo->device_info->name),
			   (int)packet->current_magic->length);
	survive_usb_setup_update_feature_report(packet->tx, packet->current_magic->magic, packet->current_magic->length);
	do {
		packet->current_magic++;
	} while (packet->current_magic->magic && packet->current_magic->code == 0);
}

void handle_config_tx(survive_usb_transfer_t *transfer);
static inline void setup_packet_state(struct survive_config_packet *packet) {
	switch (packet->state) {
	case SURVIVE_CONFIG_STATE_MAGICS:
		setup_magic(packet);
		break;
	case SURVIVE_CONFIG_STATE_VERSION:
		setup_version(packet);
		break;
	case SURVIVE_CONFIG_STATE_CONFIG:
		setup_config_req(packet);
		break;
	case SURVIVE_CONFIG_STATE_IMU_SCALES:
		setup_config_imu_scales(packet);
		return;
	case SURVIVE_CONFIG_STATE_DONE:
		return;
	}
	survive_usb_setup_control(packet->tx, packet->usbInfo, handle_config_tx, packet, 5000);
}

void survive_usb_feature_read(SurviveObject *so, const uint8_t *data, size_t length) {
	SurviveContext *ctx = so->ctx;
	switch (data[0]) {
	case VIVE_REPORT_CHANGE_MODE:
		SV_VERBOSE(100, "%s new mode %d", survive_colorize_codename(so), data[1]);
		break;
	case VIVE_REPORT_IMU_SCALES: {
		int gyro_scale_mode = data[1];
		int acc_scale_mode = data[2];
		survive_default_set_imu_scale_modes(so, gyro_scale_mode, acc_scale_mode);
		break;
	}
	case VIVE_REPORT_VERSION:
		parse_tracker_version_info(so, data + 1, length - 1);
		break;
	case VIVE_REPORT_CONFIG_READMODE:
	case VIVE_REPORT_CONFIG_READ:
		break;
	default:
		SV_VERBOSE(10, "============== Unknown feature %s read %x %d", survive_colorize_codename(so), data[0],
				   (int)length);
		break;
	}
}
void handle_config_tx(survive_usb_transfer_t *transfer) {
	struct survive_config_packet *packet = transfer->user_data;
	SurviveContext *ctx = packet->ctx;
	const uint8_t *buffer = survive_usb_transfer_data(transfer);
	SurviveObject *so = packet->usbInfo->so;
	uint8_t cmd = buffer[0];
	struct SurviveUSBInfo *usbInfo = packet->usbInfo;

	if (packet->usbInfo->interfaces[0].shutdown) {
		goto cleanup;
	}

	if (transfer->status == LIBUSB_TRANSFER_STALL || transfer->status == LIBUSB_TRANSFER_TIMED_OUT) {
		SV_VERBOSE(110, "Waiting, Transfer status %d at %f sec for %s (%d)", transfer->status,
				   survive_run_time(ctx) - packet->start_time, survive_colorize(so ? so->codename : "unknown"),
				   packet->usbInfo->request_close);
		packet->stall_counter++;
		if (packet->usbInfo->device_info->codename[0] == 0) {
			goto cleanup;
		}

		/* Watchman (WM0) dongle often doesn't respond to VERSION/IMU_SCALES - skip to data streaming */
		if (packet->stall_counter >= 1 &&
			strncmp(packet->usbInfo->device_info->codename, "WM", 2) == 0 &&
			(packet->state == SURVIVE_CONFIG_STATE_VERSION || packet->state == SURVIVE_CONFIG_STATE_IMU_SCALES)) {
			SV_INFO("Skipping VERSION/IMU_SCALES for %s (dongle doesn't support feature reports)",
					survive_colorize(packet->usbInfo->device_info->codename));
			packet->stall_counter = 0;
			if (packet->state == SURVIVE_CONFIG_STATE_VERSION && so) {
				SURVIVE_INVOKE_HOOK_SO(config, so, so->conf, so->conf_cnt);
			}
			/* Skip both VERSION and IMU_SCALES - go straight to data streaming */
			goto cleanup;
		}

		packet->usbInfo->nextCfgSubmitTime = survive_run_time(ctx) + .02;
		if (packet->stall_counter > 5) {
			packet->stall_counter = 0;
			packet->usbInfo->nextCfgSubmitTime++;
		}
		return;
	}
	packet->stall_counter = 0;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		SV_WARN("Transfer status %d at %f sec for %s", transfer->status, survive_run_time(ctx) - packet->start_time,
				survive_colorize(so ? so->codename : "unknown"));
		goto cleanup;
	}

	if (so == 0) {
		if (usbInfo->device_info->codename[0] != 0) {
			so = survive_create_device(ctx, "HTC", usbInfo, usbInfo->device_info->codename, survive_vive_send_haptic);
			survive_add_object(ctx, so);
			usbInfo->so = so;
			usbInfo->ownsObject = true;
		}
	}

	size_t tx_length = survive_usb_transfer_length(transfer);

	switch (packet->state) {
	case SURVIVE_CONFIG_STATE_MAGICS:
		/* For SET_REPORT OUT transfers, actual_length only reflects the data phase.
		 * On macOS, libusb may report fewer bytes than sent due to setup-header accounting
		 * differences. Any completed transfer (tx_length > 0 or status == COMPLETED) means
		 * the device received the command. */
		if (transfer->status == LIBUSB_TRANSFER_COMPLETED && tx_length == 0 && packet->current_magic->length > 0) {
			SV_WARN("Magic for %s was not acked; length %d vs %d", so ? survive_colorize(so->codename) : "unknown",
					(int)tx_length, (int)packet->current_magic->length);
			goto resubmit;
		}

		SV_VERBOSE(110, "Magic inc in %f sec for %s %s %d", survive_run_time(ctx) - packet->start_time,
				   so ? survive_colorize(so->codename) : "unknown",
				   survive_colorize(packet->usbInfo->device_info->name), (int)tx_length)

		if (!packet->current_magic->magic) {
			SV_VERBOSE(100, "Magics done in %f sec for %s %s", survive_run_time(ctx) - packet->start_time,
					   so ? survive_colorize(so->codename) : "unknown",
					   survive_colorize(packet->usbInfo->device_info->name));
			packet->usbInfo->lightcapMode = LightcapMode_raw0;
			if (packet->usbInfo->device_info->codename[0] == 0) {
				packet->state = SURVIVE_CONFIG_STATE_DONE;
				goto cleanup;
			}

			goto setup_next;
		}

		setup_magic(packet);
		goto resubmit;
	case SURVIVE_CONFIG_STATE_CONFIG:
		switch (cmd) {
		case VIVE_REPORT_CONFIG_READMODE:
			survive_usb_setup_get_feature_report(transfer, VIVE_REPORT_CONFIG_READ);
			packet->expected_cfg_length = (buffer[1] << 8 | buffer[2]);
			SV_VERBOSE(100, "Config readmode in %f sec for %s expected length: %d",
					   survive_run_time(ctx) - packet->start_time, survive_colorize(so->codename),
					   packet->expected_cfg_length);
			str_ensure_size(&packet->cfg, packet->expected_cfg_length);
			goto resubmit;
		case VIVE_REPORT_CONFIG_READ: {
			SV_VERBOSE(110, "Config response in %f sec for %s %d %d", survive_run_time(ctx) - packet->start_time,
					   survive_colorize(so->codename), buffer[0], buffer[1]);

			size_t size = buffer[1];
			str_append_n(&packet->cfg, (const char *)&buffer[2], size);
			if (size == 0) {
				// NOTE: Sometimes the actual length is 2 bytes longer than advertised; not sure why
				// Some devices (e.g. Watchman/dongle) report incorrect expected length - try using received data
				// if we have substantial config (>1KB) rather than retrying indefinitely
				int length_mismatch = packet->expected_cfg_length > (int)packet->cfg.length;
				if (length_mismatch && packet->cfg.length < 1000) {
					SV_WARN("Config request failed; trying again %f %s (%d > %d)",
							survive_run_time(ctx) - packet->start_time, survive_colorize(so->codename),
							packet->expected_cfg_length, (int)packet->cfg.length);
					str_clear(&packet->cfg);
					setup_config_req(packet);
					goto resubmit;
				}
				if (length_mismatch) {
					SV_WARN("Config length mismatch %s (expected %d, got %ld) - attempting to use received data",
							survive_colorize(so->codename), packet->expected_cfg_length, packet->cfg.length);
				}

				SV_VERBOSE(100, "Config done in %f sec for %s, len %ld", survive_run_time(ctx) - packet->start_time,
						   survive_colorize(so->codename), packet->cfg.length);

				uint8_t uncompressed_data[65536];
				int uncompressed_data_len = survive_simple_inflate(ctx, (uint8_t *)packet->cfg.d, packet->cfg.length,
																   uncompressed_data, sizeof(uncompressed_data) - 1);
				if (uncompressed_data_len < 0) {
					SV_ERROR(SURVIVE_ERROR_INVALID_CONFIG, "Invalid config for %s", survive_colorize(so->codename));
					goto cleanup;
				}
				packet->usbInfo->so->conf = SV_CALLOC(uncompressed_data_len + 1);
				packet->usbInfo->so->conf_cnt = uncompressed_data_len;
				memcpy(packet->usbInfo->so->conf, uncompressed_data, uncompressed_data_len);

				str_free(&packet->cfg);
				goto setup_next;
			}
			goto resubmit;
		}
		default:
			SV_ERROR(SURVIVE_ERROR_GENERAL, "Unknown config command 0x%x", cmd);
		}
		break;
	case SURVIVE_CONFIG_STATE_VERSION: {
		if (tx_length == 0)
			goto resubmit;

		parse_tracker_version_info(packet->usbInfo->so, &buffer[1], tx_length);
		SURVIVE_INVOKE_HOOK_SO(config, so, so->conf, so->conf_cnt);
		SV_VERBOSE(100, "Version done in %f sec for %s", survive_run_time(ctx) - packet->start_time,
				   survive_colorize(so->codename));

		goto setup_next;
	}
	case SURVIVE_CONFIG_STATE_IMU_SCALES: {
		int gyro_scale_mode = buffer[1];
		int acc_scale_mode = buffer[2];
		survive_default_set_imu_scale_modes(so, gyro_scale_mode, acc_scale_mode);
		goto setup_next;
	}
	default:
		SV_WARN("Config state machine saw packet of type %d at state %d; not sure how to proceed.", cmd, packet->state);
		goto cleanup;
	}
	return;

setup_next : {
	packet->state++;
	setup_packet_state(packet);
	if (packet->state == SURVIVE_CONFIG_STATE_DONE)
		goto cleanup;
	else
		goto resubmit;
};
resubmit : {
	SV_VERBOSE(110, "Resubmit startup packet for %s %s at %f", survive_colorize(packet->usbInfo->so->codename),
			   survive_colorize(packet->usbInfo->device_info->name), packet->usbInfo->nextCfgSubmitTime);
	int submit_transfer_error = survive_config_submit(packet->usbInfo);
	if (submit_transfer_error != 0) {
		SV_WARN("Config state machine could not submit transfer %d\n", submit_transfer_error);
		goto cleanup;
	}
	return;
}
cleanup:
	SV_VERBOSE(100, "Cleanup config for %s %s at %f %d/%d", survive_colorize_codename(packet->usbInfo->so),
			   survive_colorize(packet->usbInfo->device_info->name), survive_run_time(ctx),
			   usbInfo->interfaces[0].shutdown, (int)usbInfo->active_transfers);

	if (!usbInfo->interfaces[0].shutdown) {
		for (const struct Endpoint_t *endpoint = packet->usbInfo->device_info->endpoints; endpoint->name; endpoint++) {
			int errorCode =
				AttachInterface(packet->sv, packet->usbInfo, endpoint, packet->usbInfo->handle, survive_data_cb);
			if (errorCode < 0) {
				SV_WARN("Could not attach interface %s: %d", endpoint->name, errorCode);
			}
		}
	}

	packet->usbInfo->ignoreCnt = 10;
	packet->usbInfo->nextCfgSubmitTime = -1;
	packet->usbInfo->cfg_user = 0;
	packet->usbInfo->active_transfers--;

	if (usbInfo->interfaces[0].shutdown && usbInfo->active_transfers == 0) {
		usbInfo->request_close = true;
		SV_VERBOSE(100, "Acking close for %s", survive_colorize_codename(usbInfo->so));
	}

	survive_usb_transfer_free(transfer);
	str_free(&packet->cfg);
	free(packet);
}

static void survive_config_poll(struct SurviveUSBInfo *usbInfo) {
	FLT now = survive_run_time(usbInfo->viveData->ctx);
	bool active = true;
	while (active || usbInfo->nextCfgSubmitTime >= 0 && usbInfo->nextCfgSubmitTime < now) {
		active = false;
		if (usbInfo->nextCfgSubmitTime >= 0 && usbInfo->nextCfgSubmitTime < now) {
			if (survive_config_submit(usbInfo) == -6) {
				usbInfo->active_transfers--;
				survive_close_usb_device(usbInfo);
			}
		}
#ifdef HIDAPI
		struct survive_config_packet *config_packet = usbInfo->cfg_user;
		if (config_packet == 0)
			return;

		void (*cb)(struct survive_usb_transfer_t *) = config_packet->tx->cb;
		if (!config_packet->tx->processed && cb) {
			config_packet->tx->processed = true;
			cb(config_packet->tx);
		}
#else
#endif
	}
}

static void survive_config_cancel(struct survive_config_packet *config_packet) {
	survive_usb_transfer_cancel(config_packet->tx);
}
