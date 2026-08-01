#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "errno.h"
#include "os_generic.h"
#include "survive_config.h"
#include "survive_default_devices.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>

#include "pcap/usb.h"
#include <pcap.h>

#include "driver_vive.h"

#include <zlib.h>

STATIC_CONFIG_ITEM(USBMON_RECORD, "usbmon-record", 's', "File to save .pcap to.", 0)
STATIC_CONFIG_ITEM(USBMON_PLAYBACK, "usbmon-playback", 's', "File to replay .pcap from.", 0)
STATIC_CONFIG_ITEM(USBMON_RECORD_ALL, "usbmon-record-all", 'b', "Whether or not to record all usb traffic", 0)
STATIC_CONFIG_ITEM(USBMON_OUTPUT_EVERYTHING, "usbmon-output-all", 'b', "Whether or not to log all usb traffic", 0)
STATIC_CONFIG_ITEM(USBMON_OUTPUT, "usbmon-output", 'b', "Whether or not to log any generic usb traffic", 0)
STATIC_CONFIG_ITEM(USBMON_ONLY_RECORD, "usbmon-only-record", 'b', "Record only; don't forward to libsurvive", 0)
STATIC_CONFIG_ITEM(USBMON_ALLOW_FS_CONFIG, "usbmon-allow-fs-config", 'b',
				   "If we dont see a config section; try to read it from filesystem -- could be very wrong", 0)

typedef struct vive_device_t {
	uint16_t vid, pid;
	const char *codename;
	const char *def_config;
} vive_device_t;

typedef struct vive_device_inst_t {
	const struct vive_device_t *device;
	int bus_id;
	int dev_id;
	int devIdxForType;
	bool hasConfiged;
	SurviveObject *so;
	struct SurviveUSBInfo *usbInfo;
	char name[16];

	uint64_t last_config_id;
	uint8_t compressed_data[8192];
	uint16_t compressed_data_idx;
	size_t packets_without_config;
	bool tried_config_file;

	uint8_t serial[32];
} vive_device_inst_t;

typedef struct usb_info_t {
	uint16_t vid, pid;
	int bus_id, dev_id;
	uint8_t serial[32];
} usb_info_t;

#define VIVE_DEVICE_INST_MAX 32

struct vive_device_t devices[] = {{.vid = 0x28de, .pid = 0x2000, .codename = "HMD", .def_config = "HMD_config.json"},
								  {.vid = 0x0bb4, .pid = 0x2c87, .codename = "BD", .def_config = 0},
								  {.vid = 0x28de, .pid = 0x2101, .codename = "WM", .def_config = "WM%d_config.json"},
								  {.vid = 0x28de, .pid = 0x2022, .codename = "TR", .def_config = "TR%d_config.json"},
								  {.vid = 0x28de, .pid = 0x2300, .codename = "T2", .def_config = "T2%d_config.json"},
								  {.vid = 0x28de, .pid = 0x2012, .codename = "WW", .def_config = "WW%d_config.json"},
								  {.vid = 0x28de, .pid = 0x2102, .codename = "KN", .def_config = "KN%d_config.json"},
								  {0}};

static const int DEVICES_CNT = sizeof(devices) / sizeof(vive_device_t);

typedef struct SurviveDriverUSBMon {
	SurviveContext *ctx;
	pcap_t *pcap;
	int datalink;

	double playback_factor;
	double time_now;
	double run_time;

	pcap_dumper_t *pcapDumper;
	bool record_all;
	bool record_only;
	bool allow_fs_read;

	bool output_usb_stream;
	bool output_everything;

	char errbuf[PCAP_ERRBUF_SIZE];
	vive_device_inst_t usb_devices[VIVE_DEVICE_INST_MAX];
	size_t usb_devices_cnt;

	bool passiveMode;
	size_t packet_cnt;

	bool *keepRunning;
} SurviveDriverUSBMon;

#define USBPCAP_CONTROL_STAGE_SETUP 0
#define USBPCAP_CONTROL_STAGE_DATA 1
#define USBPCAP_CONTROL_STAGE_STATUS 2
#define USBPCAP_CONTROL_STAGE_COMPLETE 3

#pragma pack(push, 1)
typedef struct {
	uint16_t headerLen; /* This header length */
	uint64_t irpId;		/* I/O Request packet ID */
	uint32_t status;	/* USB status code
							   (on return from host controller) */
	uint16_t function;	/* URB Function */
	uint8_t info;		/* I/O Request info */

	uint16_t bus;	  /* bus (RootHub) number */
	uint16_t device;  /* device address */
	uint8_t endpoint; /* endpoint number and transfer direction */
	uint8_t transfer; /* transfer type */

	uint32_t dataLength; /* Data length */
} USBPCAP_BUFFER_PACKET_HEADER, *PUSBPCAP_BUFFER_PACKET_HEADER;

typedef struct {
	USBPCAP_BUFFER_PACKET_HEADER header;
	uint8_t stage;
} USBPCAP_BUFFER_CONTROL_HEADER, *PUSBPCAP_BUFFER_CONTROL_HEADER;

typedef union USBPCAP_BUFFER_UNION {
	USBPCAP_BUFFER_PACKET_HEADER hdr;
	USBPCAP_BUFFER_CONTROL_HEADER ctrl;
} USBPCAP_BUFFER_UNION;

#pragma pack(pop)

vive_device_inst_t *find_device_inst(SurviveDriverUSBMon *d, int bus_id, int dev_id) {
	for (size_t i = 0; i < d->usb_devices_cnt; i++) {
		if (d->usb_devices[i].bus_id == bus_id && d->usb_devices[i].dev_id == dev_id)
			return &d->usb_devices[i];
	}

	return 0;
}

static int interface_lookup(const vive_device_inst_t *dev, int endpoint) {
	int32_t id = dev->device->pid + (endpoint << 16);
	switch (id) {
	case 0x812000:
		return USB_IF_HMD_IMU;
	case 0x812101:
		return USB_IF_WATCHMAN1;
	case 0x812022:
		return USB_IF_TRACKER0_IMU;
	case 0x812300:
		return USB_IF_TRACKER1_IMU;
	case 0x812012:
		return USB_IF_W_WATCHMAN1_IMU;
	case 0x802300:
	case 0x802101:
		return USB_IF_TRACKER_INFO;
	case 0x822000:
		return USB_IF_HMD_LIGHTCAP;
	case 0x822101:
	case 0x812102:
		return USB_IF_WATCHMAN1;
	case 0x822022:
		return USB_IF_TRACKER0_LIGHTCAP;
	case 0x822300:
		return USB_IF_TRACKER1_LIGHTCAP;
	case 0x822012:
		return USB_IF_W_WATCHMAN1_LIGHTCAP;
	case 0x832000:
		return USB_IF_HMD_BUTTONS;
	case 0x832101:
		return USB_IF_W_WATCHMAN1_BUTTONS;
	case 0x832022:
		return USB_IF_TRACKER0_BUTTONS;
	case 0x832300:
		return USB_IF_TRACKER1_LIGHTCAP;
	case 0x832012:
		return USB_IF_W_WATCHMAN1_BUTTONS;
	case 0x842300:
		return USB_IF_TRACKER1_BUTTONS;
	default:
		return 0;
	}
}

static bool is_config_request(const struct _usb_header_mmapped *usbp) {
	return usbp->event_type == 'S' && usbp->s.setup.bmRequestType == 0xa1 && usbp->s.setup.bRequest == 1 &&
		   usbp->s.setup.wValue == 0x311;
}

static bool is_config_start(const struct _usb_header_mmapped *usbp) {
	return usbp->event_type == 'S' && usbp->s.setup.bmRequestType == 0xa1 && usbp->s.setup.bRequest == 1 &&
		   usbp->s.setup.wValue == 0x310;
}

static bool is_command_setup(const struct _usb_header_mmapped *usbp) {
	return usbp->event_type == 'S' && usbp->s.setup.bmRequestType == 0x21 && usbp->s.setup.bRequest == 0x09 &&
		   usbp->s.setup.wValue == 0x3ff;
}

static void ingest_config_request(vive_device_inst_t *dev, const struct _usb_header_mmapped *usbp,
								  const uint8_t *pktData) {
	if (dev->so == 0) {
		return;
	}
	struct SurviveContext *ctx = dev->so->ctx;
	uint16_t cnt = pktData[1];

	SV_VERBOSE(500, "Ingesting config data for %s(%p); %d bytes", dev->so->codename, (void *)dev, cnt);

	if (cnt) {
		// Some (Tracker at least?) devices send a uint64_t before data; not sure what it means but skip it for now.
		if (dev->compressed_data_idx == 0 && cnt >= 2 && pktData[2] != 0x78) {

		} else {
			memcpy(&dev->compressed_data[dev->compressed_data_idx], pktData + 2, cnt);
			dev->compressed_data_idx += cnt;
		}
	} else {
		char *uncompressed_data = SV_CALLOC(65536);
		SurviveContext *ctx = dev->so->ctx;

		int len = survive_simple_inflate(dev->so->ctx, dev->compressed_data, dev->compressed_data_idx,
										 (uint8_t *)uncompressed_data, 65536 - 1);

		if (len <= 0) {
			SV_WARN("Error: data for config descriptor");
		} else {
			SV_INFO("usbmon loaded %d total bytes of config data for %s", len, dev->so->codename);
		}

		if (!dev->hasConfiged) {
			if (ctx->configproc(dev->so, uncompressed_data, len) == 0) {
				dev->hasConfiged = true;
				dev->last_config_id = 0;
			} else {
				SV_WARN("Could not load from config");
			}
		}
	}
}

static double timestamp_in_s() {
	static double start_time_s = 0;
	if (start_time_s == 0.)
		start_time_s = OGGetAbsoluteTime();
	return OGGetAbsoluteTime() - start_time_s;
}

static int usbmon_close(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverUSBMon *driver = _driver;

	struct pcap_stat stats = {0};
	pcap_stats(driver->pcap, &stats);

	SV_INFO("usbmon saw %u/%u packets, %u dropped, %u dropped in driver in %.2f seconds (%.2fs runtime)",
			(uint32_t)driver->packet_cnt, stats.ps_recv, stats.ps_drop, stats.ps_ifdrop, driver->time_now,
			timestamp_in_s());
	if (driver->pcapDumper) {
		pcap_dump_close(driver->pcapDumper);
	}
	pcap_close(driver->pcap);

	for (int i = 0; i < driver->usb_devices_cnt; i++) {
		vive_device_inst_t *dev = &driver->usb_devices[i];
		free(dev->usbInfo);
	}
	survive_install_run_time_fn(ctx, 0, 0);
	free(driver);
	return 0;
}
static usb_info_t *get_usb_info_from_file(const char *fname) {
	usb_info_t *rtn = SV_CALLOC_N(MAX_USB_DEVS, sizeof(usb_info_t));
	size_t count = 0;
	FILE *f = fopen(fname, "r");
	while (f && !feof(f)) {
		char name[128];
		if (fscanf(f, "%hu %hu %d %d %s ", &rtn[count].vid, &rtn[count].pid, &rtn[count].bus_id, &rtn[count].dev_id,
				   name) == 5) {
			count++;
		}
	}
	return rtn;
}

#ifdef HIDAPI
static usb_info_t *get_usb_info_from_os() { return 0; }
#else
static usb_info_t *get_usb_info_from_os() {
	usb_info_t *rtn = 0;

	libusb_context *context = NULL;
	libusb_device **list = NULL;
	int rc = 0;
	ssize_t count = 0;

	rc = libusb_init(&context);
	if (rc != 0)
		return 0;

	count = libusb_get_device_list(context, &list);
	rtn = (usb_info_t *)SV_CALLOC_N(count + 1, sizeof(usb_info_t));
	size_t fill_cnt = 0;
	for (size_t idx = 0; idx < count; ++idx) {
		libusb_device *device = list[idx];
		struct libusb_device_descriptor desc = {0};

		rc = libusb_get_device_descriptor(device, &desc);
		if (rc != 0)
			break;

		rtn[fill_cnt] = (usb_info_t){.vid = desc.idVendor,
									 .pid = desc.idProduct,
									 .bus_id = libusb_get_bus_number(device),
									 .dev_id = libusb_get_device_address(device)};

		if (desc.iSerialNumber != 0) {
			libusb_device_handle *handle;
			int error = libusb_open(device, &handle);
			if (error >= 0) {
				libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, rtn[fill_cnt].serial, 32);
				libusb_close(handle);
			}
		}
		fill_cnt++;
	}

	libusb_free_device_list(list, count);
	libusb_exit(context);

	return rtn;
}
#endif

static size_t fill_device_inst(SurviveContext *ctx, vive_device_inst_t *insts, const usb_info_t *usb_dev,
							   FILE *save_file) {
	size_t rtn = 0;
	while (usb_dev->vid != 0 && usb_dev->pid != 0) {
		bool foundDevice = false;
		for (vive_device_t *dev = devices; dev->vid != 0; dev++) {
			if (usb_dev->vid == dev->vid && usb_dev->pid == dev->pid) {
				foundDevice = true;
				insts->device = dev;
				insts->bus_id = usb_dev->bus_id;
				insts->dev_id = usb_dev->dev_id;
				memcpy(insts->serial, usb_dev->serial, 32);
				if (save_file) {
					fprintf(save_file, "%d %d %d %d %s\n", usb_dev->vid, usb_dev->pid, insts->bus_id, insts->dev_id,
							dev->codename);
				}

				SV_INFO("Device instance %s for %04x:%04x at %d %d", dev->codename, usb_dev->vid, usb_dev->pid,
						usb_dev->bus_id, usb_dev->dev_id);
				insts++;
				rtn++;
			}
		}

		if (!foundDevice) {
			SV_WARN("Didn't find device instance for %04x:%04x at %d %d", usb_dev->vid, usb_dev->pid, usb_dev->bus_id,
					usb_dev->dev_id);
		}

		usb_dev++;
	}

	return rtn;
}

static const char *usbmon_record_file(SurviveContext *ctx) {
	const char *usbmon_record = survive_configs(ctx, "usbmon-record", SC_GET, 0);
	if (usbmon_record == 0) {
		usbmon_record = survive_configs(ctx, "record", SC_GET, 0);
		if (strstr(usbmon_record, ".pcap"))
			return usbmon_record;
		return NULL;
	}
	return usbmon_record;
}

static const char *usbmon_playback_file(SurviveContext *ctx) {
	const char *usbmon_playback = survive_configs(ctx, "usbmon-playback", SC_GET, 0);
	if (usbmon_playback == 0)
		return survive_configs(ctx, "playback", SC_GET, 0);
	return usbmon_playback;
}

static int setup_usb_devices(SurviveDriverUSBMon *sp) {
	SurviveContext *ctx = sp->ctx;
	int rtn = 0;

	int *device_cnts = alloca(sizeof(int) * DEVICES_CNT);
	memset(device_cnts, 0, sizeof(int) * DEVICES_CNT);

	const char *usbmon_record = usbmon_record_file(ctx);
	const char *usbmon_playback = usbmon_playback_file(ctx);

	FILE *listing_file = 0;

	if (usbmon_record && *usbmon_record) {
		char fname[256] = {0};
		sprintf(fname, "%s.usbdevs", usbmon_record);
		listing_file = fopen(fname, "w");
	}

	usb_info_t *usbInfo = 0;
	if (usbmon_playback && *usbmon_playback) {
		char fname[256] = {0};
		sprintf(fname, "%s.usbdevs", usbmon_playback);
		usbInfo = get_usb_info_from_file(fname);
	} else {
		usbInfo = get_usb_info_from_os();
	}

	sp->usb_devices_cnt = fill_device_inst(ctx, sp->usb_devices, usbInfo, listing_file);
	if (listing_file) {
		fclose(listing_file);
	}
	free(usbInfo);

	for (int i = 0; i < sp->usb_devices_cnt; i++) {
		int dev_idx = sp->usb_devices[i].device - devices;
		char buff[16] = "HMD";
		if (dev_idx != 0) {
			sprintf(buff, "%s%d", sp->usb_devices[i].device->codename, device_cnts[dev_idx]++);
		}
		memcpy(sp->usb_devices[i].name, buff, 16);

		if (sp->passiveMode == false) {
			memcpy(sp->usb_devices[i].name, buff, 16);

			if (sp->usb_devices[i].device->def_config) {
				SurviveObject *so = survive_create_device(ctx, "UMN", 0, buff, 0);
				sp->usb_devices[i].so = so;
				sp->usb_devices[i].usbInfo =
					survive_vive_register_driver(so, sp->usb_devices[i].device->vid, sp->usb_devices[i].device->pid);
				survive_add_object(ctx, so);
			}
		}
		char filter[256] = {0};
		sprintf(filter, "(usb.bus_id = %d and usb.device_address = %d)", sp->usb_devices[i].bus_id,
				sp->usb_devices[i].dev_id);

		sp->usb_devices[i].devIdxForType = device_cnts[dev_idx];
		rtn++;
	}

	return rtn;
}

double make_time(double start, const struct _usb_header_mmapped *pMmapped) {
	return pMmapped->ts_sec + pMmapped->ts_usec * 1.e-6 - start;
}

const char *requestTypeToStr(uint8_t requestType) {
	switch (requestType) {
	case 0:
		return "GET_STATUS";
	case 1:
		return "CLEAR_FEATURE";
	case 3:
		return "SET_FEATURE";
	case 5:
		return "SET_ADDRESS";
	case 6:
		return "GET_DESCRIPTOR";
	case 7:
		return "SET_DESCRIPTOR";
	case 8:
		return "GET_CONFIGURATION";
	case 9:
		return "SET_CONFIGURATION";
	case 10:
		return "GET_INTERFACE";
	case 11:
		return "SET_INTERFACE";
	case 12:
		return "SYNC_FRAME";
	}
	return "<unknown>";
}

static double survive_usbmon_playback_run_time(const SurviveContext *ctx, void *_driver) {
	SurviveDriverUSBMon *driver = _driver;
	return driver->time_now;
}

#define USBPCAP_TRANSFER_ISOCHRONOUS 0
#define USBPCAP_TRANSFER_INTERRUPT 1
#define USBPCAP_TRANSFER_CONTROL 2
#define USBPCAP_TRANSFER_BULK 3
#define USBPCAP_TRANSFER_IRP_INFO 0xFE
#define USBPCAP_TRANSFER_UNKNOWN 0xFF

const uint8_t *fill_usb_header(const void *_hdr, struct pcap_pkthdr *pkthdr, pcap_usb_header_mmapped *usbp) {
	const USBPCAP_BUFFER_PACKET_HEADER *hdr = _hdr;

	usbp->id = hdr->irpId;
	usbp->event_type = 0; // ???
	usbp->transfer_type = hdr->transfer;
	usbp->endpoint_number = hdr->endpoint;
	usbp->device_address = hdr->device;
	usbp->bus_id = hdr->bus;

	usbp->status = hdr->status;
	usbp->urb_len = 0; // ???
	usbp->data_len = hdr->dataLength;
	usbp->data_flag = hdr->dataLength == 0;
	usbp->ts_sec = pkthdr->ts.tv_sec;
	usbp->ts_usec = pkthdr->ts.tv_usec;

	usbp->setup_flag = 1;

	const uint8_t *data_ptr = (const uint8_t *)_hdr + hdr->headerLen;
	if (hdr->transfer == USBPCAP_TRANSFER_CONTROL) {
		const USBPCAP_BUFFER_CONTROL_HEADER *ctrl = _hdr;
		switch (ctrl->stage) {
		case USBPCAP_CONTROL_STAGE_SETUP:
			usbp->setup_flag = 0;
			usbp->event_type = 'S';
			assert(hdr->dataLength == 8);
			memcpy(&usbp->s.setup, data_ptr, hdr->dataLength);
			break;
		case USBPCAP_CONTROL_STAGE_COMPLETE:
			usbp->event_type = 'C';
			break;
		case USBPCAP_CONTROL_STAGE_DATA:
			break;
		case USBPCAP_CONTROL_STAGE_STATUS:
			break;
		}
	} else if (hdr->transfer == USBPCAP_TRANSFER_INTERRUPT) {
	}

	return data_ptr;
}
void *pcap_thread_fn(void *_driver) {
	SurviveDriverUSBMon *driver = _driver;
	struct SurviveContext *ctx = driver->ctx;
	typedef pcap_usb_header_mmapped usb_header_t;

	struct pcap_pkthdr *pkthdr = 0;
	const usb_header_t *usbp = 0;
	pcap_usb_header_mmapped usbpcap_translation = {0};

	SV_INFO("Pcap thread started");
	double start_time = 0;
	double real_time_start = timestamp_in_s();
	while ((driver->keepRunning == 0 || *driver->keepRunning) && ctx->currentError == SURVIVE_OK) {
		void *hdr = 0;
		int result = pcap_next_ex(driver->pcap, &pkthdr, (const uint8_t **)&hdr);

		switch (result) {
		case 0:
			goto continue_loop;
		case 1: {
			const uint8_t *pktData = 0;
			if (driver->datalink == DLT_USBPCAP) {
				pktData = fill_usb_header(hdr, pkthdr, &usbpcap_translation);
				usbp = &usbpcap_translation;
			} else {
				usbp = hdr;
				// Packet data is directly after the packet header
				pktData = (uint8_t *)&usbp[1];
			}

			vive_device_inst_t *dev = find_device_inst(driver, usbp->bus_id, usbp->device_address);

			if (driver->pcapDumper && (dev || driver->record_all)) {
				pcap_dump((uint8_t *)driver->pcapDumper, pkthdr, (uint8_t *)usbp);
			}

			if (dev) {
				driver->packet_cnt++;
				const char *dev_name = dev->name;
				if (dev->so)
					dev_name = dev->so->codename;
				assert(dev_name);
				const char *color_dev_name = survive_colorize(dev_name);

				if (start_time == 0) {
					start_time = make_time(0, usbp);
				}
				double this_real_time = timestamp_in_s();
				double this_time = make_time(start_time, usbp);
				if (driver->playback_factor > 0.) {
					double next_time_s_scaled = this_time * driver->playback_factor;
					while (this_real_time < next_time_s_scaled) {
						int sleep_time_ms = 1 + (next_time_s_scaled - this_real_time) * 1000.;
						OGUSleep(sleep_time_ms * 1000);
						this_real_time = timestamp_in_s();
					}
				}

				while (survive_input_event_count(ctx) > 0) {
					// OGUSleep(1000);
				}

#define COLORIZED_ID_STR SURVIVE_COLORIZED_FORMAT("%016lx")
#define COLORIZED_ID SURVIVE_COLORIZED_DATA(usbp->id)
				char color_set[16] = "";

				unsigned hash = ((usbp->id + (usbp->id >> 8) + (usbp->id >> 16) + (usbp->id >> 24)) & 0xFF) % 8;
				sprintf(color_set, "\033[0;%dm", (int)(hash + 30));
				const char *color_reset = "\033[0m";
				driver->time_now = this_time;
				if (this_time > driver->run_time && driver->run_time > 0)
					*driver->keepRunning = false;

				survive_get_ctx_lock(ctx);
				// Print setup flags, then just bail
				if (!usbp->setup_flag) {
					if (is_config_start(usbp)) {
						dev->last_config_id = 0;
						dev->compressed_data_idx = 0;
						SV_VERBOSE(200, "%s start of config", color_dev_name);
					} else if (is_config_request(usbp)) {
						dev->last_config_id = usbp->id;
					} else if (is_command_setup(usbp)) {
						SV_INFO("%s sent command 0x%02x with %u bytes:", color_dev_name, pktData[1], pktData[2]);
						survive_dump_buffer(ctx, pktData + 3, pktData[2]);
					}
					if (driver->output_usb_stream) {
						SURVIVE_INVOKE_HOOK(printf, ctx,
											"--> %10.6f S: %s " COLORIZED_ID_STR
											" event_type: %c transfer_type: %d bmRequestType: 0x%02x "
											"bRequest: 0x%02x (%s) "
											"wValue: 0x%04x wIndex: 0x%04x wLength: %4d (%4d)\n",
											this_time, color_dev_name, SURVIVE_COLORIZED_DATA(usbp->id),
											usbp->event_type, usbp->transfer_type, usbp->s.setup.bmRequestType,
											usbp->s.setup.bRequest, requestTypeToStr(usbp->s.setup.bRequest),
											usbp->s.setup.wValue, usbp->s.setup.wIndex, usbp->s.setup.wLength,
											usbp->data_len);

						survive_dump_buffer(ctx, pktData, usbp->data_len);
					}

					if (dev->so) {
						survive_data_on_setup_write(dev->so, usbp->s.setup.bmRequestType, usbp->s.setup.bRequest,
													usbp->s.setup.wValue, usbp->s.setup.wIndex, pktData,
													usbp->data_len);
					}
					survive_release_ctx_lock(ctx);
					goto continue_loop;
				}

				if (!(usbp->endpoint_number & 0x80u)) {

					if (driver->output_usb_stream) {
						if (usbp->event_type == 'C') {
							SURVIVE_INVOKE_HOOK(printf, ctx,
												"<-- %10.6f C: %s " COLORIZED_ID_STR
												" event_type: %c transfer_type: %d 0x%02x (0x%02x):\n",
												this_time, color_dev_name, SURVIVE_COLORIZED_DATA(usbp->id),
												usbp->event_type, usbp->transfer_type, usbp->endpoint_number,
												usbp->data_len);
						} else {
							SURVIVE_INVOKE_HOOK(printf, ctx,
												"--> %10.6f W: %s " COLORIZED_ID_STR
												" event_type: %c transfer_type: %d 0x%02x (0x%02x):\n",
												this_time, color_dev_name, SURVIVE_COLORIZED_DATA(usbp->id),
												usbp->event_type, usbp->transfer_type, usbp->endpoint_number,
												usbp->data_len);
						}
						survive_dump_buffer(ctx, pktData, usbp->data_len);
					}
					survive_release_ctx_lock(ctx);
					goto continue_loop; // Only want incoming data
				}

				int interface = interface_lookup(dev, usbp->endpoint_number);

				if (usbp->status != 0) {
					// EINPROGRESS is normal, EPIPE means stalled
					if (driver->output_usb_stream) {
						if ((usbp->status != -115 && usbp->status != -32) || driver->output_everything)
							SURVIVE_INVOKE_HOOK(printf, ctx,
												"<-- %10.6f E: %s " COLORIZED_ID_STR
												" event_type: %c transfer_type: %d status: %d endpoint: 0x%02x (%s)\n",
												this_time, color_dev_name, SURVIVE_COLORIZED_DATA(usbp->id),
												usbp->event_type, usbp->transfer_type, usbp->status,
												usbp->endpoint_number, survive_usb_interface_str(interface));
					}
					if (usbp->id == dev->last_config_id) {
						dev->last_config_id = 0;
					}
					survive_release_ctx_lock(ctx);
					goto continue_loop; // Only want responses
				}

				bool output_read = driver->output_usb_stream &&
								   (interface == 0 || driver->output_everything || interface == USB_IF_TRACKER_INFO) &&
								   interface != USB_IF_W_WATCHMAN1_IMU && interface != USB_IF_TRACKER1_IMU &&
								   interface != USB_IF_TRACKER0_IMU;

				if (output_read) {
					SURVIVE_INVOKE_HOOK(printf, ctx,
										"<-- %10.6f R: %s " COLORIZED_ID_STR
										" event_type: %c transfer_type: %d endpoint: 0x%02x (%s) (0x%02x): \n",
										this_time, color_dev_name, SURVIVE_COLORIZED_DATA(usbp->id), usbp->event_type,
										usbp->transfer_type, usbp->endpoint_number,
										survive_usb_interface_str(interface), usbp->data_len);
					survive_dump_buffer(ctx, pktData, usbp->data_len);
				}

				if (usbp->event_type == 'C' && usbp->transfer_type == 2 && dev->so) {
					survive_usb_feature_read(dev->so, pktData, usbp->data_len);
				}

				if (usbp->id == dev->last_config_id && usbp->event_type == 'C' && dev->hasConfiged == false) {
					ingest_config_request(dev, usbp, pktData);
					dev->last_config_id = 0;
					dev->packets_without_config = 0;
					survive_release_ctx_lock(ctx);
					goto continue_loop;
				}

				bool is_standard_endpoint =
					interface == USB_IF_TRACKER_INFO && ((usbp->endpoint_number >> 5) & 0x3) == 0;
				bool forward_to_data_cb = driver->record_only == false &&
										  (interface != 0 && (dev->hasConfiged || interface == USB_IF_TRACKER_INFO)) &&
										  dev->so != 0 && !is_standard_endpoint && usbp->data_len > 0 &&
										  usbp->status == 0;
				survive_release_ctx_lock(ctx);
				if (forward_to_data_cb) {
					SurviveUSBInterface si = {.ctx = ctx,
											  .actual_len = pkthdr->len,
											  .assoc_obj = dev->so,
											  .which_interface_am_i = interface,
											  .hname = dev->so->codename,
											  .buffer = si.swap_buffer[0]};

					// memcpy(si.buffer, (u_char*)&usbp[1], usbp->data);

					si.actual_len = usbp->data_len;
					memset(si.buffer, 0xCA, sizeof(si.swap_buffer));
					memcpy(si.buffer, pktData, usbp->data_len);
					uint64_t time = usbp->ts_sec * 1000000 + usbp->ts_usec;
					survive_data_cb(time, &si);
				} else if (!dev->hasConfiged) {
					if (driver->allow_fs_read && dev->packets_without_config++ > 1000 &&
						dev->tried_config_file == false) {
						for (int i = 0; i < 2 && !dev->hasConfiged; i++) {
							char filename[128] = {0};
							snprintf(filename, sizeof(filename), "%s_config.json",
									 i == 0 ? dev->serial : (uint8_t *)dev_name);
							int res = survive_load_htc_config_format_from_file(dev->so, filename);
							SV_VERBOSE(50,
									   "Too long without config packet for %s; trying to read config from file %s: %d",
									   color_dev_name, filename, res);
							if (res == 0) {
								dev->hasConfiged = true;
							}
							dev->tried_config_file = true;
						}
					}
				}
			}

			break;
		}
		case PCAP_ERROR:
			SV_WARN("Pcap error %s", pcap_geterr(driver->pcap));
		case PCAP_ERROR_BREAK:
			*driver->keepRunning = false;
			goto exit_loop;
		default:
			SV_WARN("Pcap next got %d", result);
		}

	continue_loop:
		continue;
	}

exit_loop:

	SV_VERBOSE(100, "Exiting usbmon thread");
	return 0;
}

#if defined(HAVE_FOPENCOOKIE)
static ssize_t gzip_cookie_write(void *cookie, const char *buf, size_t size) {
	return gzwrite((gzFile)cookie, (voidpc)buf, size);
}

static int gzip_cookie_close(void *cookie) { return gzclose((gzFile)cookie); }

static ssize_t gzip_cookie_read(void *cookie, char *buf, size_t nbytes) { return gzread((gzFile)cookie, buf, nbytes); }

static int gzip_cookie_seek(void *cookie, off64_t *pos, int whence) {
	z_off64_t r = gzseek64((gzFile)cookie, (z_off64_t)*pos, whence);
	if (r < 0)
		return -1;

	*pos = (off64_t)r;
	return 0;
}

cookie_io_functions_t gzip_cookie = {
	.close = gzip_cookie_close, .write = gzip_cookie_write, .read = gzip_cookie_read, .seek = gzip_cookie_seek};
#endif

static FILE *open_playback(const char *fn, const char *mode) {
#if defined(HAVE_FOPENCOOKIE)
	gzFile z = gzopen(fn, mode);
	return fopencookie(z, mode, gzip_cookie);
#else
	return fopen(fn, mode);
#endif
}

static int DriverRegUSBMon_(SurviveContext *ctx, int driver_id) {
	int enable = survive_configi(ctx, "usbmon", SC_GET, 0);
	const char *usbmon_record = usbmon_record_file(ctx);
	const char *usbmon_playback = usbmon_playback_file(ctx);

	if (enable && driver_id != 0)
		return -1;

	SurviveDriverUSBMon *sp = SV_CALLOC(sizeof(SurviveDriverUSBMon));
	sp->playback_factor = -1;
	sp->ctx = ctx;
	sp->passiveMode = !enable && driver_id == 1;
	if (sp->passiveMode) {
		SV_INFO("Starting usbmon in passive (record) mode.");
	} else {
		SV_INFO("Starting usbmon");
	}

	bool isPlaybackMode = usbmon_playback && *usbmon_playback;
	if (isPlaybackMode) {
		sp->playback_factor = survive_configf(ctx, "playback-factor", SC_GET, 1.0);
		sp->run_time = survive_configf(ctx, "run-time", SC_GET, -1);

		SV_INFO("Opening '%s' for usb playback for %4.2f seconds at time factor %f", usbmon_playback, sp->run_time,
				sp->playback_factor);
		FILE *pF = open_playback(usbmon_playback, "r");
		sp->pcap = pcap_fopen_offline(pF, sp->errbuf);

#if !defined(HAVE_FOPENCOOKIE)
		if (strcmp(".gz", usbmon_playback + strlen(usbmon_playback) - 3) == 0) {
			SV_WARN("Trying to open a compressed file without FOPENCOOKIE support in the usbmon driver.");
		}
#endif
		survive_install_run_time_fn(ctx, survive_usbmon_playback_run_time, sp);
	} else {
		sp->pcap = pcap_open_live("usbmon0", PCAP_ERRBUF_SIZE, 0, -1, sp->errbuf);
	}

	if (sp->pcap == NULL) {

		const char *playback_error = "pcap_fopen_offline failed due to [%s] - The file either doesn't exist, is "
									 "corrupted, or uses compression which isn't enabled for this driver binary";
		const char *live_error =
			"pcap_open_live() failed due to [%s] - You probably need to call 'sudo modprobe usbmon'. If you want "
			"to capture as a normal user; try 'sudo setfacl -m u:$USER:r /dev/usbmon*'";
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, isPlaybackMode ? playback_error : live_error, sp->errbuf);
		return SURVIVE_DRIVER_ERROR;
	}

	sp->datalink = pcap_datalink(sp->pcap);
	if (sp->datalink != DLT_USBPCAP && sp->datalink != DLT_USB_LINUX_MMAPPED) {
		SV_ERROR(SURVIVE_ERROR_INVALID_CONFIG, "USBMON driver does not support datalink type %d", sp->datalink);
		return SURVIVE_DRIVER_ERROR;
	}

	sp->output_everything = survive_configi(ctx, "usbmon-output-all", SC_GET, 0);
	sp->output_usb_stream = sp->output_everything || survive_configi(ctx, "usbmon-output", SC_GET, 0);
	sp->record_only = survive_configi(ctx, "usbmon-only-record", SC_GET, 0);
	sp->allow_fs_read = survive_configi(ctx, USBMON_ALLOW_FS_CONFIG_TAG, SC_GET, 0);

	if (usbmon_record && *usbmon_record) {
		FILE *fd = open_playback(usbmon_record, "w");
		SV_INFO("Opening %s for usb recording (%p)", usbmon_record, (void *)fd);
		sp->pcapDumper = pcap_dump_fopen(sp->pcap, fd);
		sp->record_all = survive_configi(ctx, "usbmon-record-all", SC_GET, 0);
		if (sp->record_all) {
			SV_WARN("All USB traffic is being captured. Don't use 'usbmon-record-all' if you don't want to expose "
					"things like input from keyboards.");
		}
	}

	int device_count = setup_usb_devices(sp);
	if (device_count) {
		// sp->keepRunning = true;
		// sp->pcap_thread = OGCreateThread(pcap_thread_fn, sp);
		// OGNameThread(sp->pcap_thread, "pcap_thread");

		sp->keepRunning = survive_add_threaded_driver(ctx, sp, "pcap_thread", pcap_thread_fn, usbmon_close);
		// survive_add_driver(ctx, sp, usbmon_poll, usbmon_close, 0);
	} else {
		usbmon_close(ctx, sp);
		SV_ERROR(SURVIVE_ERROR_NO_TRACKABLE_OBJECTS, "USBMon found no devices");
		return SURVIVE_DRIVER_ERROR;
	}
	return SURVIVE_DRIVER_NORMAL;
}

int DriverRegUSBMon(SurviveContext *ctx) { return DriverRegUSBMon_(ctx, 0); }
REGISTER_LINKTIME(DriverRegUSBMon)

int DriverRegUSBMon_Record(SurviveContext *ctx) { return DriverRegUSBMon_(ctx, 1); }
REGISTER_LINKTIME(DriverRegUSBMon_Record)

int DriverRegUSBMon_Playback(SurviveContext *ctx) { return DriverRegUSBMon_(ctx, 2); }
REGISTER_LINKTIME(DriverRegUSBMon_Playback)
