#include <errno.h>
#include <jsmn.h>
#include <os_generic.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>
#include <sys/stat.h>
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <assert.h>
#include <malloc.h> // for alloca

#endif

#include "json_helpers.h"
#include "survive_config.h"
#include "survive_default_devices.h"
#include "survive_str.h"
#include "driver_vive.h"
#include "lfsr_lh2.h"
//#define DEBUG_WATCHMAN 1

struct SurviveViveData;

struct DeviceInfo {
	const char *name;
	const char *codename;

	uint16_t vid;
	uint16_t pid;
	enum USB_DEV_t type;

	struct Endpoint_t {
		uint8_t num;
		const char *name;
		enum USB_IF_t type;
	} endpoints[MAX_INTERFACES_PER_DEVICE];

	struct Magic_t {
		const char *name;
		bool code;
		const uint8_t *magic;
		const size_t length;
	} magics[8];
};

enum vive_report_ids {
	// Possibly 0x03 - Gyro range, 0x02 - Accel range
	// Tracker/lh1/[rf,usb]: (device -> steamvr):  01 03 02
	VIVE_REPORT_IMU_SCALES = 1,
	
	// You can get this from doing errors in lighthouse_console:
	/*
	0300000000000000000000000000000000000000000000000000000000000000000000000000000000010000000000325160d6066f0394c59affa228a02870f7

	missing_rising_edge         : 0
	backward_time               : 0
	pulse_queue_overflow        : 0
	short_sync                  : 0
	long_sync                   : 0
	invalid_calibration_size    : 0
	invalid_calibration_crc     : 0
	invalid_calibration_version : 0
	queue_overflow              : 0
	spammy_sensor               : 0
	flags                       : 1
	long_optical_packet_delay   : 0

	0300000000000000000000000080c44300020000000000000000000000000000005700000000000000010000000000000006010000f3ff0357a915ca8ebd47d5
	missing_rising_edge         : 0
	backward_time               : 0
	pulse_queue_overflow        : 0
	short_sync                  : 4441216
	long_sync                   : 2
	invalid_calibration_size    : 0
	invalid_calibration_crc     : 0
	invalid_calibration_version : 0
	queue_overflow              : 87
	spammy_sensor               : 0
	flags                       : 1
	long_optical_packet_delay   : 0

	*/
	VIVE_REPORT_ERRORS = 0x03,
	VIVE_REPORT_IMU = 0x20,
	VIVE_REPORT_USB_TRACKER_LIGHTCAP_V1 = 0x21,
	VIVE_REPORT_RF_WATCHMAN = 0x23,
	VIVE_REPORT_RF_WATCHMANx2 = 0x24,
	VIVE_REPORT_USB_LIGHTCAP_REPORT_V1 = 0x25,

	// I've seen this with a byte of data; 01. Maybe this turns on and off?
	// Note: I've now seen this at connection too? Maybe its just what happens when you hold the button down?
	VIVE_REPORT_RF_TURN_OFF = 0x26,
	VIVE_REPORT_USB_LIGHTCAP_REPORT_V2 = 0x27,
	VIVE_REPORT_USB_LIGHTCAP_REPORT_RAW_MODE_1 = 0x28,

	VIVE_REPORT_COMMAND = 0xff,
	// Below is from https://github.com/nairol/LighthouseRedox/blob/master/docs/USB%20Protocol.md and not necessarilly
	// up to date / used
	VIVE_REPORT_HMD_PROXIMITY = 0x3,

	VIVE_REPORT_VERSION = 0x05,

	// Tracker/lh1/[rf,usb]: (steamvr -> device): 04 00 00 00   00
	VIVE_REPORT_CHANGE_MODE = 0x04,
	// Tracker/lh1/[rf,usb]: (steamvr -> device):  07 03 00 00   00
	VIVE_REPORT_LIGHTCAP_VERBOSITY = 0x07,

	// Seems like 0x4/0x7 are used in conjunction:
	// Sync mode 0:
	// --> 04 00 00 00   00
	// --> 07 03 00 00   00
	// Sync mode 1:
	// --> 04 01 00 00   00
	// --> 07 03 00 00   00
	// Sync mode 2:
	// --> 04 03 00 00   00
	// --> 07 03 00 00   00

	// Tracker/lh1/rf: (steamvr -> device): (CLEAR_FEATURE)
	//                 (device -> steamvr): 08 03 00 00
	// If clear_feature is sent; seems to be asking how much data is in the user alloc area.
	// Seems like its maybe current offset as uint32_t, total size as uint32_t.
	// The tracker seems to not have a userdata section; and responds with 0x8 and then whatever was in the buffer.
	VIVE_REPORT_HMD_SET_READOFFSET_USERDATA = 0x08,
	VIVE_REPORT_HMD_READ_USERDATA = 0x09,

	VIVE_REPORT_CONFIG_READMODE = 0x10,
	VIVE_REPORT_CONFIG_READ = 0x11,

	// Sent and echoed by steamvr; no data
	// Directly proceeds a 0x04...
	// Possibly a different version packet. Sent when you do 'version' in lh_console.
	// knuckles -> lighthouse_console:
	// 13 b5 35 28   5d 03 00 00   00 77 61 74   63 68 6d 61   6e 00 00 00   00 00 00 00   00 56 61 6c   76 65 42 75 |
	// ..5(  ]...  .wat  chma  n...  ....  .Val  veBu
	// 69 6c 64 65   72 30 32 00   00 09 00 0e   11 00 00 bd   26 1a 02 0a   02 f1 52 2c   5c 00 00 00   00 00 00 00 |
	// ilde  r02.  ....  ....  &...  ..R,  \...  ....
	VIVE_REPORT_VERSION_ALT = 0x13,

	// Tracker/lh1/usb: (steamvr -> device): 16 01 00 00   00
	VIVE_REPORT_REBOOT = 0x16

};

enum vive_commands {
	// This command seems to dictate protocol and sync model. I think the first byte is the 'protocol',
	// the second second and fourth together dictate the sync mode.
	// Tracker/lh1/rf: (steamvr -> device):  01 00 00 02   00 00 |    ....  ..
	// Tracker/lh1/rf: (steamvr -> device):  01 00 00            |    ....  ..

	// Tracker/lh1/rf: (lhconsole -> device):  sync mode 0: 01 00 00 02   00 00
	// Tracker/lh1/rf: (lhconsole -> device):  sync mode 1: 01 01 00 02   00 00
	// Tracker/lh1/rf: (lhconsole -> device):  sync mode 2: 01 01 00 02   01 00

	// Knuckles/lh2/rf: (steamvr -> device): 01 00 00 02   00 00
	//                                       00 01 00
	//                                       01 01 00 02   00 00

	VIVE_COMMAND_CHANGE_PROTOCOL = 0x87,

	// Tracker/lh1/rf: (steamvr -> device)
	//   - Sends 2x with no data as SET_CONFIGURATION
	//   - Sends 1x as CLEAR_FEATURE
	//   - Recv back:
	//                ff 83 2d 00   b1 d7 b3 ba   01 00 23 00   00 02 0b 00   00 00 04 6a   19 e6 5b 09   06 00 02 84 0b
	//                b1 d7 b3     |    ..-.  ....  ..#.  ....  ...j  ..[.  ....  ....
	//                ba 05 09 3a   87 5a 0c 00   02 00 00 0a   cf 3b 3c 5a |    ...:  .Z..  ....  .;<Z  ....  .... ....
	//                ....
	VIVE_COMMAND_UNKNOWN3 = 0x83,
	VIVE_COMMAND_HAPTIC_PULSE = 0x8F,

	// Wants '6f 66 66 21 |    off!' as data
	// -->  22.307137 S: T20 ffff8dc3fcadc000 event_type: S transfer_type: 2 bmRequestType: 0x21 bRequest: 0x09
	// (SET_CONFIGURATION) wValue: 0x0300 wIndex: 0x0002 wLength:   64 (  64)
	// 22.3068941 9f 04 6f 66   66 21 00 00   00 00 00 00   00 00 00 00   00 00 00 00   00 00 00 00   00 00 00 00   00
	// 00 00 00     |    ..of  f!..  ....  ....  ....  ....  ....  ....
	//            00 00 00 00   00 00 00 00   00 00 00 00   00 00 00 00   00 00 00 00   00 00 00 00   00 00 00 00   00
	//            00 00 00     |    ....  ....  ....  ....  ....  ....  ....  ....
	// <--  22.307294 C: T20 ffff8dc3fcadc000 event_type: C transfer_type: 2 0x00 (0x00):
	VIVE_COMMAND_POWER_OFF = 0x9F,

	// Knuckles/lh2/rf: (steamvr -> device):  be 5b 32 54   11 cf 83 75   53 8a 08 6a   53 58 d0 b1
	// Tracker/lh1/rf:  (steamvr -> device):  be 5b 32 54   11 cf 83 75   53 8a 08 6a   53 58 d0 b1 |    .[2T  ...u S..j
	// SX..
	// Note: This is sent regardless of whether there is a device setup
	VIVE_COMMAND_CONFIGURE_RADIO = 0x96,

	// Info: WM sent command 0xad with 3 bytes:
	// 01 10 27
	VIVE_COMMAND_PAIR = 0xAD,

	// Keepalive?
	// Tracker/lh1/rf: (steamvr -> device):
	//                 (device -> steamvr): 57 16 a0 b9   32 33 46 45   41 33 44 44   44 41 00 00   00 00 00 00   00 00
	//                 00 00   00 00 00 00   00 00 00 00     |    W...  23FE  A3DD  DA..  ....  ....  ....  ....
	//                                      00 00 |    ..
	VIVE_COMMAND_CHECK_ALIVE = 0xa1,

	// Knuckles/lh2/rf: (steamvr->sevice): 00
	VIVE_COMMAND_UNKNOWN4 = 0xb5,

	// Knuckles/lh2/rf: (steamvr->sevice)
	VIVE_COMMAND_UNKNOWN5 = 0x8c,
};

static uint8_t vive_magic_power_on[64] = {0x04, 0x78, 0x29, 0x38, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01};
static uint8_t vive_magic_enable_lighthouse[5] = {VIVE_REPORT_CHANGE_MODE};
static uint8_t vive_magic_enable_lighthouse_more[5] = {VIVE_REPORT_LIGHTCAP_VERBOSITY, 0x03};
static uint8_t vive_magic_power_off[] = {
	0x04, 0x78, 0x29, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x30, 0x05, 0x77,
	0x00, 0xe4, 0xf7, 0x33, 0x00, 0xe4, 0xf7, 0x33, 0x00, 0x60, 0x6e, 0x72, 0x00, 0xb4, 0xf7, 0x33,
	0x00, 0x04, 0x00, 0x00, 0x00, 0x70, 0xb0, 0x72, 0x00, 0x90, 0xf7, 0x33, 0x00, 0x7c, 0xf8, 0x33,
	0x00, 0xd0, 0xf7, 0x33, 0x00, 0x3c, 0x68, 0x29, 0x65, 0x24, 0xf9, 0x33, 0x00, 0x00, 0x00, 0x00,
};
static uint8_t vive_magic_raw_mode_1[] = {VIVE_REPORT_CHANGE_MODE, 0x01, 0x00, 0x00, 0x00};
static uint8_t vive_request_version_info[] = {VIVE_REPORT_VERSION};

#ifdef __linux__
static uint8_t vive_magic_rf_raw_mode_0[] = {
#else
// Windows needs 64 size for the wireless dongle
static uint8_t vive_magic_rf_raw_mode_0[64] = {
#endif
	VIVE_REPORT_COMMAND, VIVE_COMMAND_CHANGE_PROTOCOL, 0x6, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00};
static uint8_t vive_magic_rf_raw_mode_1[] = {
	VIVE_REPORT_COMMAND, VIVE_COMMAND_CHANGE_PROTOCOL, 0x6, 0x01, 0x01, 0x00, 0x02, 0x00, 0x00};

#ifdef __linux__
static uint8_t vive_magic_protocol_switch[] = {
#else
// Windows needs 64 size for the wireless dongle
static uint8_t vive_magic_protocol_switch[64] = {
#endif
	VIVE_REPORT_COMMAND, VIVE_COMMAND_CHANGE_PROTOCOL, 0x3, 0x00, 0x01, 0x00};
static uint8_t vive_request_pairing[] = {VIVE_REPORT_COMMAND, VIVE_COMMAND_PAIR, 0x03, 0x01, 0x10, 0x27};

#ifdef __linux__
static uint8_t vive_magic_protocol_super_magic[] = {
#else
// Windows needs 64 size for the wireless dongle
static uint8_t vive_magic_protocol_super_magic[64] = {
#endif
	VIVE_REPORT_COMMAND,
	VIVE_COMMAND_CONFIGURE_RADIO,
	0x10,
	0xbe,
	0x5b,
	0x32,
	0x54,
	0x11,
	0xcf,
	0x83,
	0x75,
	0x53,
	0x8a,
	0x08,
	0x6a,
	0x53,
	0x58,
	0xd0,
	0xb1};

#define MAGIC_CTOR(ison, buffer)                                                                                       \
	{ .name = #buffer, .code = ison, .magic = buffer, .length = sizeof(buffer) }
const struct DeviceInfo KnownDeviceTypes[] = {
	{.vid = 0x0bb4,
	 .pid = 0x2c87,
	 .type = USB_DEV_HMD,
	 .name = "BRD",
	 .codename = "",
	 .endpoints = {{.num = 0x81, .name = "Mainboard", .type = USB_IF_HMD_HEADSET_INFO}},
	 .magics = {MAGIC_CTOR(true, vive_magic_power_on), MAGIC_CTOR(false, vive_magic_power_off)}},
	{.vid = 0x0bb4,
	 .pid = 0x030e,
	 .type = USB_DEV_HMD,
	 .name = "BRD",
	 .codename = "",
	 .endpoints = {{.num = 0x81, .name = "Mainboard Pro", .type = USB_IF_HMD_HEADSET_INFO}},
	 .magics = {MAGIC_CTOR(true, vive_magic_power_on), MAGIC_CTOR(false, vive_magic_power_off)}},
	{.vid = 0x28de,
	 .pid = 0x2000,
	 .type = USB_DEV_HMD_IMU_LH,
	 .name = "HMD IMU & LH",
	 .codename = "HMD",
	 .endpoints = {{.num = 0x81, .name = "IMU", .type = USB_IF_HMD_IMU},
				   {.num = 0x82, .name = "Lightcap", .type = USB_IF_HMD_LIGHTCAP}},
	 .magics = {MAGIC_CTOR(true, vive_magic_enable_lighthouse), MAGIC_CTOR(true, vive_magic_enable_lighthouse_more)}},
	{.vid = 0x28de,
	 .pid = 0x2101,
	 .type = USB_DEV_WATCHMAN1,
	 .name = "Watchman",
	 .codename = "WM0",
	 .endpoints = {{.num = 0x81, .name = "IMU/Lightcap/Buttons", .type = USB_IF_WATCHMAN1}},
	 .magics =
		 {// MAGIC_CTOR(true, vive_magic_enable_lighthouse),
		  // MAGIC_CTOR(true, vive_magic_enable_lighthouse_more),
		  MAGIC_CTOR(true, vive_magic_protocol_super_magic), MAGIC_CTOR(true, vive_magic_rf_raw_mode_0),
		  MAGIC_CTOR(true, vive_magic_protocol_switch)}},
	{.vid = 0x28de,
	 .pid = 0x2022,
	 .type = USB_DEV_TRACKER0,
	 .name = "Tracker",
	 .codename = "TR0",
	 .endpoints =
		 {
			 {.num = 0x81, .name = "IMU", .type = USB_IF_TRACKER0_IMU},
			 {.num = 0x82, .name = "Lightcap", .type = USB_IF_TRACKER0_LIGHTCAP},
			 {.num = 0x83, .name = "Buttons", .type = USB_IF_TRACKER0_BUTTONS},
		 },
	 .magics = {MAGIC_CTOR(true, vive_magic_enable_lighthouse), MAGIC_CTOR(true, vive_magic_enable_lighthouse_more)}},
	{.vid = 0x28de,
	 .pid = 0x2300,
	 .type = USB_DEV_TRACKER1,
	 .name = "Tracker (2018)",
	 .codename = "T20",
	 .endpoints =
		 {
			 {.num = 0x81, .name = "IMU", .type = USB_IF_TRACKER1_IMU},
			 {.num = 0x83, .name = "Lightcap", .type = USB_IF_TRACKER1_LIGHTCAP},
			 {.num = 0x84, .name = "Buttons", .type = USB_IF_TRACKER1_BUTTONS},
		 },
	 .magics = {MAGIC_CTOR(true, vive_magic_enable_lighthouse), MAGIC_CTOR(true, vive_magic_enable_lighthouse_more)}},
	{.vid = 0x28de,
	 .pid = 0x2012,
	 .type = USB_DEV_W_WATCHMAN1,
	 .name = "Wired Watchman",
	 .codename = "WW0",
	 .endpoints = {{.num = 0x81, .name = "IMU", .type = USB_IF_W_WATCHMAN1_IMU},
				   {.num = 0x82, .name = "Lightcap", .type = USB_IF_W_WATCHMAN1_LIGHTCAP},
				   {.num = 0x83, .name = "Buttons", .type = USB_IF_W_WATCHMAN1_BUTTONS}},
	 .magics = {MAGIC_CTOR(true, vive_magic_enable_lighthouse), MAGIC_CTOR(true, vive_magic_enable_lighthouse_more)}},
	{.vid = 0x28de,
	 .pid = 0x2102,
	 .type = USB_DEV_WATCHMAN1,
	 .name = "Knuckles",
	 .codename = "KN0",
	 .endpoints = {{.num = 0x81, .name = "IMU/Lightcap/Buttons", .type = USB_IF_WATCHMAN1}},
	 .magics = {MAGIC_CTOR(true, vive_magic_protocol_super_magic), MAGIC_CTOR(true, vive_magic_rf_raw_mode_0),
				MAGIC_CTOR(true, vive_magic_protocol_switch)}},
	{0}};

// typedef struct SurviveUSBInterface SurviveUSBInterface;
typedef struct SurviveViveData SurviveViveData;

const char *survive_usb_interface_str(enum USB_IF_t iface) {
	switch (iface) {
	case USB_IF_HMD_HEADSET_INFO:
		return "USB_IF_HMD_HEADSET_INFO";
	case USB_IF_HMD_IMU:
		return "USB_IF_HMD_IMU";
	case USB_IF_WATCHMAN1:
		return "USB_IF_WATCHMAN1";
	case USB_IF_WATCHMAN2:
		return "USB_IF_WATCHMAN2";
	case USB_IF_TRACKER0_IMU:
		return "USB_IF_TRACKER0_IMU";
	case USB_IF_TRACKER_INFO:
		return "USB_IF_TRACKER_INFO";
	case USB_IF_TRACKER1_IMU:
		return "USB_IF_TRACKER1_IMU";
	case USB_IF_W_WATCHMAN1_IMU:
		return "USB_IF_W_WATCHMAN1_IMU";
	case USB_IF_HMD_LIGHTCAP:
		return "USB_IF_HMD_LIGHTCAP";
	case USB_IF_TRACKER0_LIGHTCAP:
		return "USB_IF_TRACKER0_LIGHTCAP";
	case USB_IF_TRACKER1_LIGHTCAP:
		return "USB_IF_TRACKER1_LIGHTCAP";
	case USB_IF_W_WATCHMAN1_LIGHTCAP:
		return "USB_IF_W_WATCHMAN1_LIGHTCAP";
	case USB_IF_HMD_BUTTONS:
		return "USB_IF_HMD_BUTTONS";
	case USB_IF_TRACKER0_BUTTONS:
		return "USB_IF_TRACKER0_BUTTONS";
	case USB_IF_TRACKER1_BUTTONS:
		return "USB_IF_TRACKER1_BUTTONS";
	case USB_IF_W_WATCHMAN1_BUTTONS:
		return "USB_IF_W_WATCHMAN1_BUTTONS";
	}
	return "UNKNOWN";
}

void survive_dump_buffer(SurviveContext *ctx, const uint8_t *data, size_t length) {
	int bytes_per_row = 32;
	ctx->printfproc(ctx, "%.7f ", survive_run_time(ctx));
	for (size_t i = 0; i < length; i += bytes_per_row) {
		for (int j = 0; j < bytes_per_row; j++) {
			if (j > 0 && j % 4 == 0)
				SURVIVE_INVOKE_HOOK(printf, ctx, "  ");

			if (i + j < length) {
				SURVIVE_INVOKE_HOOK(printf, ctx, "%02x ", data[i + j]);
			} else {
				SURVIVE_INVOKE_HOOK(printf, ctx, "   ");
			}
		}

		SURVIVE_INVOKE_HOOK(printf, ctx, "    |    ");

		for (int j = 0; j < bytes_per_row; j++) {
			if (j > 0 && j % 4 == 0)
				SURVIVE_INVOKE_HOOK(printf, ctx, "  ");
			if (i + j < length) {
				uint8_t d = data[i + j];
				SURVIVE_INVOKE_HOOK(printf, ctx, "%c", d >= 32 && d < 127 ? d : '.');
			} else {
				SURVIVE_INVOKE_HOOK(printf, ctx, "   ");
			}
		}

		SURVIVE_INVOKE_HOOK(printf, ctx, "\n");
	}
}

struct SurviveUSBInfo {
	USBHANDLE handle;
	SurviveViveData *viveData;
	const struct DeviceInfo *device_info;
	struct SurviveObject *so;
	bool ownsObject;

	size_t interface_cnt;
	SurviveUSBInterface interfaces[MAX_INTERFACES_PER_DEVICE];

	enum LightcapMode {
		LightcapMode_unknown = 0,

		// This is the starting mode after an 0x40 0 0 0 0. Used for LH1
		LightcapMode_raw0,

		// This is the mode used for LH2
		LightcapMode_raw1,

		// Not sure what this is used for tbh
		LightcapMode_raw2
	} lightcapMode;

	size_t timeWithoutFlag;
	size_t packetsSeenWaitingForV2;
	size_t ignoreCnt;

	size_t active_transfers;
	FLT nextCfgSubmitTime;
	struct survive_config_packet *cfg_user;

	bool request_close, request_reopen;
};

struct SurviveViveData {
	SurviveContext *ctx;
	size_t udev_cnt;
	struct SurviveUSBInfo *udev[MAX_USB_DEVS];
	struct libusb_context *usbctx;
	size_t read_count;
	int seconds_per_hz_output;

	int cnt_per_device_type[sizeof(KnownDeviceTypes) / sizeof(KnownDeviceTypes[0])];
	struct SurviveUSBInfo *hmd_mainboard, *hmd_imu;

	FLT lastPairTime;
	bool requestPairing;
#ifndef HIDAPI
	libusb_hotplug_callback_handle callback_handle;
#endif
};

static void parse_tracker_version_info(SurviveObject *so, const uint8_t *data, size_t size);
static int AttachInterface(SurviveViveData *sv, struct SurviveUSBInfo *usbObject, const struct Endpoint_t *endpoint,
						   USBHANDLE devh, usb_callback cb);
static int survive_vive_send_haptic(SurviveObject *so, FLT frequency, FLT amplitude, FLT duration_seconds);

static bool survive_device_is_rf(const struct DeviceInfo *device_info) {
	switch (device_info->type) {
	case USB_DEV_HMD:
	case USB_DEV_HMD_IMU_LH:
	case USB_DEV_W_WATCHMAN1:
	case USB_DEV_TRACKER0:
	case USB_DEV_TRACKER1:
		return false;
	}
	return true;
}

#ifdef HIDAPI
#include "driver_vive.hidapi.h"
#else
#include "driver_vive.libusb.h"
#endif

struct survive_config_packet {
	SurviveContext *ctx;
	SurviveViveData *sv;
	struct SurviveUSBInfo *usbInfo;
	const struct Magic_t *current_magic;

	uint8_t buffer[256];

	uint16_t expected_cfg_length;
	cstring cfg;
	double start_time;

	enum {
		/**
		 * CONFIG must come first; it effectively gates progress for controllers which might not be turned on yet.
		 */
		SURVIVE_CONFIG_STATE_CONFIG,
		SURVIVE_CONFIG_STATE_MAGICS,
		SURVIVE_CONFIG_STATE_VERSION,
		SURVIVE_CONFIG_STATE_IMU_SCALES,
		SURVIVE_CONFIG_STATE_DONE
	} state;
	uint16_t stall_counter;
	survive_usb_transfer_t *tx;
};

#include "driver_vive.config.h"

static struct SurviveUSBInfo *survive_get_usb_info(SurviveObject *so) { return (struct SurviveUSBInfo *)so->driver; }

static int survive_vive_send_haptic(SurviveObject *so, FLT frequency, FLT amplitude, FLT duration_seconds) {
	SurviveViveData *sv = ((struct SurviveUSBInfo *)so->driver)->viveData;
	SurviveContext *ctx = so->ctx;

	if (so->object_type == SURVIVE_OBJECT_TYPE_HMD || so->conf == 0) {
		return 0;
	}

	if (NULL == sv) {
		return -500;
	}

	// Due to https://gitlab.freedesktop.org/monado/monado/-/blob/master/src/xrt/drivers/vive/vive_controller.c#L398
	FLT high_plus_low = 1000.f * 1000.f / frequency;
	FLT pulse_low = amplitude * high_plus_low / 2.;

	/* Vive Controller doesn't vibrate with value == 0.
	 * Not sure if this actually happens, but let's fix it anyway. */
	if (pulse_low == 0)
		pulse_low = 1;

	FLT pulse_high = high_plus_low - pulse_low;

	uint16_t repeat_count = duration_seconds * frequency;
	if (repeat_count == 0)
		repeat_count = 1;

	uint16_t pulse_high16 = pulse_high, pulse_low16 = pulse_low;
	if (pulse_high > UINT16_MAX)
		pulse_high16 = UINT16_MAX;
	if (pulse_high < 0)
		pulse_high16 = 1;
	if (pulse_low > UINT16_MAX)
		pulse_low16 = UINT16_MAX;
	if (pulse_low < 0)
		pulse_low16 = 1;

	SV_VERBOSE(100, "Sending haptic pulse to %s %f %f %f %f %f %d", survive_colorize(so->codename), frequency,
			   amplitude, duration_seconds, pulse_high, pulse_low, repeat_count);
	uint8_t vive_controller_haptic_pulse[] = {
		VIVE_REPORT_COMMAND, VIVE_COMMAND_HAPTIC_PULSE, 0x07,		 0x00,
		pulse_high16,		 pulse_high16 >> 8u,		pulse_low16, pulse_low16 >> 8u,
		repeat_count,		 repeat_count >> 8u,
	};

	struct SurviveUSBInfo *usbInfo = survive_get_usb_info(so);

	int r =
		update_feature_report(usbInfo->handle, 0, vive_controller_haptic_pulse, sizeof(vive_controller_haptic_pulse));

	if (r != sizeof(vive_controller_haptic_pulse)) {
		SV_WARN("HAPTIC FAILED %d", r);
		return -1;
	}

	return 0;
}

void vive_switch_mode(struct SurviveUSBInfo *driverInfo, enum LightcapMode lightcapMode) {
	SurviveContext *ctx = driverInfo->so->ctx;
	SurviveObject *w = driverInfo->so;
	if (driverInfo->timeWithoutFlag == 0) {
		driverInfo->timeWithoutFlag = 1;
		uint8_t buffer[64] = {0};
		size_t buffer_length = 0;
		if (survive_device_is_rf(driverInfo->device_info)) {
			buffer[0] = VIVE_REPORT_COMMAND;
			buffer[1] = VIVE_COMMAND_CHANGE_PROTOCOL;
			buffer[2] = 6;

			buffer[3] = 1;
			buffer[4] = lightcapMode == LightcapMode_raw0 ? 0 : 1;
			buffer[5] = 0;
			buffer[6] = 2;
			buffer[7] = lightcapMode == LightcapMode_raw2 ? 1 : 0;
			buffer[8] = 0;
			buffer_length = 64;
		} else {
			buffer[0] = VIVE_REPORT_CHANGE_MODE;
			buffer[1] = (lightcapMode == LightcapMode_raw1) ? 1 : (lightcapMode == LightcapMode_raw2) ? 3 : 0;
			buffer_length = 64;
		}

		if (driverInfo->handle) {
			int r = update_feature_report(driverInfo->handle, 0, buffer, buffer_length);
			if (r != buffer_length) {
				SV_WARN("Could not send raw mode to %s (%d)", w->codename, r);
				return;
			}

			if (!survive_device_is_rf(driverInfo->device_info)) {
				r = update_feature_report(driverInfo->handle, 0, vive_magic_enable_lighthouse_more,
										  sizeof(vive_magic_enable_lighthouse_more));
				if (r != buffer_length) {
					SV_WARN("Could not lighthouse more to %s (%d)", w->codename, r);
				}
			}

			SV_INFO("LightcapMode (%s) %d -> %d (%x)", w->codename, driverInfo->lightcapMode, lightcapMode, buffer[0]);
			driverInfo->lightcapMode = lightcapMode;

		} else {
			static bool transfer_null_warning = false;
			if (!transfer_null_warning) {
				SV_WARN("Can't update the usb device %s out of raw 0 mode; dumping data", w->codename);
				transfer_null_warning = true;
			}
		}
	}
}

static bool is_mode_switch_usb(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
							   uint16_t length) {
	return bmRequestType == 0x21 && bRequest == 0x09 && wValue == 0x304 && length >= 8;
}

static bool is_mode_switch_rf(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
							  const uint8_t *data, uint16_t length) {
	return bmRequestType == 0x21 && bRequest == 0x09 && wValue == 0x3ff && length >= 8 &&
		   data[1] == VIVE_COMMAND_CHANGE_PROTOCOL && data[2] >= 6;
}

void survive_data_on_setup_write(SurviveObject *so, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
								 uint16_t wIndex, const uint8_t *data, size_t length) {
	SurviveContext *ctx = so->ctx;
	struct SurviveUSBInfo *driverInfo = so->driver;
	driverInfo->timeWithoutFlag = 1;
	if (is_mode_switch_usb(bmRequestType, bRequest, wValue, wIndex, length)) {
		enum LightcapMode m = data[1] == 0 ? LightcapMode_raw0 : data[1] == 1 ? LightcapMode_raw1 : LightcapMode_raw2;
		SV_INFO("LightcapMode usb %s %d -> %d", so->codename, driverInfo->lightcapMode, m);
		driverInfo->lightcapMode = m;
		driverInfo->ignoreCnt = 10;
	}

	if (is_mode_switch_rf(bmRequestType, bRequest, wValue, wIndex, data, length)) {
		enum LightcapMode m = data[4] == 0 ? LightcapMode_raw0 : data[7] == 1 ? LightcapMode_raw2 : LightcapMode_raw1;
		SV_INFO("LightcapMode rf %s %d -> %d", so->codename, driverInfo->lightcapMode, m);
		driverInfo->lightcapMode = m;
		driverInfo->ignoreCnt = 10;
	}

	// SV_INFO("Setup %s write of %x %d", survive_colorize_codename(so), wValue, length);
}

void survive_data_cb_locked(uint64_t time_received_us, SurviveUSBInterface *si);
void survive_data_cb(uint64_t time_received_us, SurviveUSBInterface *si) {
	SurviveContext *ctx = si->ctx;
	survive_get_ctx_lock(ctx);
	survive_data_cb_locked(time_received_us, si);
	survive_release_ctx_lock(ctx);
}

// USB Subsystem
static int survive_usb_init(SurviveViveData *sv);
int survive_usb_poll(SurviveContext *ctx);

static int AttachInterface(SurviveViveData *sv, struct SurviveUSBInfo *usbObject, const struct Endpoint_t *endpoint,
						   USBHANDLE devh, usb_callback cb) {
	SurviveContext *ctx = sv->ctx;
	size_t iface_cnt = usbObject->interface_cnt++;
	int which_interface_am_i = endpoint->type;
	const char *hname = endpoint->name;
	int endpoint_num = endpoint->num;

	SurviveUSBInterface *iface = &usbObject->interfaces[iface_cnt];
	SurviveObject *assocobj = usbObject->so;
	iface->ctx = ctx;
	iface->sv = sv;
	iface->which_interface_am_i = which_interface_am_i;
	iface->assoc_obj = usbObject->so;
	iface->usbInfo = usbObject;
	iface->hname = hname;
	iface->cb = cb;

#ifdef HIDAPI
	iface->buffer = iface->swap_buffer[0];
	iface->uh = usbObject->handle->interfaces[endpoint - usbObject->device_info->endpoints];
	assert(iface->uh);

#ifndef HID_NONBLOCKING
	iface->servicethread = OGCreateThread(HAPIReceiver, iface);
	OGUSleep(100000);
#else
	hid_set_nonblocking(iface->uh, 1);

	// Empty the queue out. If you don't, you might get stale data
	while (hid_read(iface->uh, iface->buffer, sizeof(iface->buffer)) > 0) {
	}
#endif
#else
	struct libusb_transfer *tx = iface->transfer = libusb_alloc_transfer(0);
	usbObject->active_transfers++;

	// printf( "%p %d %p %p\n", iface, which_interface_am_i, tx, devh );
	SV_VERBOSE(50, "Attaching %s(0x%x) for %s", hname, endpoint_num,
			   survive_colorize(assocobj ? assocobj->codename : "(unknown)"));

	if (!iface->transfer) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Error: failed on libusb_alloc_transfer for %s", hname);
		return 4;
	}
	memset(iface->swap_buffer, 0xCA, sizeof(iface->swap_buffer));
	libusb_fill_interrupt_transfer(tx, devh, endpoint_num, iface->swap_buffer[0], INTBUFFSIZE, handle_transfer, iface,
								   0);

	iface->last_submit_time = OGGetAbsoluteTimeUS();
	int rc = libusb_submit_transfer(tx);
	if (rc) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Error: Could not submit transfer for %s 0x%02x (Code %d, %s)", hname,
				 endpoint_num, rc, libusb_error_name(rc));
		return 6;
	}
#endif
	return 0;
}

static const struct DeviceInfo *find_known_device(SurviveContext *ctx, uint16_t idVendor, uint16_t idProduct) {
	const char *blacklist = survive_configs(ctx, "blacklist-devs", SC_GET, 0);
	for (const struct DeviceInfo *info = KnownDeviceTypes; info->name && blacklist; info++) {
		if (info == 0 || strstr(blacklist, info->name)) {
			continue;
		}

		if (info->vid == idVendor && info->pid == idProduct) {
			return info;
		}
	}
	return 0;
}

static int survive_start_get_config(SurviveViveData *sv, struct SurviveUSBInfo *usbInfo, int iface);
int survive_vive_add_usb_device(SurviveViveData *sv, survive_usb_device_t d) {
	SurviveContext *ctx = sv->ctx;
	uint16_t idVendor;
	uint16_t idProduct;
	uint8_t class_id;
	int ret = survive_get_ids(d, &idVendor, &idProduct, &class_id);
	if (ret < 0) {
		SV_WARN("Could not get vid:pid for usb device.")
		return -2;
	}

	const struct DeviceInfo *info = find_known_device(ctx, idVendor, idProduct);
	if (info == 0) {
		SV_VERBOSE(110, "USB device %04x:%x4x in an unknown type; ignoring", idVendor, idProduct);
		return -1;
	}

	SV_VERBOSE(10, "Enumerating USB device %04x:%04x %s", idVendor, idProduct, survive_colorize(info->name));
	
	struct SurviveUSBInfo *usbInfo = sv->udev[sv->udev_cnt++] = SV_CALLOC(sizeof(struct SurviveUSBInfo));
	usbInfo->handle = 0;
	usbInfo->device_info = info;
	usbInfo->viveData = sv;
	ret = survive_open_usb_device(sv, d, usbInfo);

	if (ret) {
		sv->udev_cnt--;
		return -5;
	}

	if (info->type == USB_DEV_HMD) {
		SV_VERBOSE(10, "Mainboard class %d", class_id);
		if (sv->hmd_mainboard != 0 || class_id != 0) {
			free(sv->udev[sv->udev_cnt--]);
			sv->udev[sv->udev_cnt] = 0;
			return -3;
		}
		sv->hmd_mainboard = usbInfo;
	} else if (info->type == USB_DEV_HMD_IMU_LH) {
		if (sv->hmd_imu != 0) {
			SV_WARN("Multiple HMDs are not supported currently.")
			return -4;
		}
		sv->hmd_imu = usbInfo;
	}

	int *cnt = (sv->cnt_per_device_type + (usbInfo->device_info - KnownDeviceTypes));

#ifdef HIDAPI
	if (usbInfo->device_info->codename[0] != 0) {
		char codename[4] = {0};
		strcpy(codename, usbInfo->device_info->codename);
		codename[2] += (*cnt);
		*cnt = *cnt + 1;

		SurviveObject *so = survive_create_device(ctx, "HTC", usbInfo, codename, survive_vive_send_haptic);
		survive_add_object(ctx, so);
		usbInfo->so = so;
		usbInfo->ownsObject = true;
	}
#endif

	survive_start_get_config(sv, usbInfo, 0);

	// There should only be one HMD, tie the mainboard interface to the surviveobject
	if (info->type == USB_DEV_HMD_IMU_LH || info->type == USB_DEV_HMD) {
		if (sv->hmd_imu != 0 && sv->hmd_mainboard != 0) {
			sv->hmd_mainboard->so = sv->hmd_imu->so;
			usbInfo->ownsObject = false;
		}
	}

#ifdef HIDAPI
	/*
	for (const struct Endpoint_t *endpoint = usbInfo->device_info->endpoints; endpoint->name; endpoint++) {
		int errorCode = AttachInterface(sv, usbInfo, endpoint, usbInfo->handle, survive_data_cb);
		if (errorCode < 0) {
			SV_WARN("Could not attach interface %s: %d", endpoint->name, errorCode);
		}
	}
	 */
#endif

	return 0;
}

int survive_usb_init(SurviveViveData *sv) {
	SurviveContext *ctx = sv->ctx;

	int r = survive_usb_subsystem_init(sv);
	if (r) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "usb fault %d (%s)\n", r, survive_usb_error_name(r));
		return r;
	}

	if (setup_hotplug(sv) != 0) {
		survive_usb_devices_t devs;
		int ret = survive_get_usb_devices(sv, &devs);

		if (ret < 0) {
			SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Couldn't get list of USB devices %d (%s)", ret,
					 survive_usb_error_name(ret));
			return ret;
		}
		
		// Open all interfaces.
		survive_usb_device_enumerator e = 0;
		for (survive_usb_device_t d = 0; (d = get_next_device(&e, devs)) && sv->udev_cnt < MAX_USB_DEVS;) {
			survive_vive_add_usb_device(sv, d);
		}
		survive_free_usb_devices(devs);

		SV_INFO("All enumerated devices attached.");
	}

	// libUSB initialized.  Continue.
	return 0;
}

STATIC_CONFIG_ITEM(PAIR_DEVICE, "pair-device", 'b', "Turn on pairing mode", 0)
STATIC_CONFIG_ITEM(SECONDS_PER_HZ_OUTPUT, "usb-hz-output", 'i', "Seconds between outputing usb stats", -1)
void survive_vive_usb_close(SurviveViveData *sv) {
	survive_release_ctx_lock(sv->ctx);
	survive_usb_close(sv);
	survive_get_ctx_lock(sv->ctx);
	survive_detach_config(sv->ctx, SECONDS_PER_HZ_OUTPUT_TAG, &sv->seconds_per_hz_output);
}

static inline bool survive_handle_close_request_flag(struct SurviveUSBInfo *usbInfo) {
	SurviveViveData *sv = usbInfo->viveData;
	SurviveContext *ctx = sv->ctx;

	if (usbInfo->request_close) {
		int idx = 0;
		for (idx = 0; idx < sv->udev_cnt && sv->udev[idx] != usbInfo; idx++)
			;

		for (size_t j = 0; j < usbInfo->interface_cnt; j++) {
			usbInfo->interfaces[j].assoc_obj = 0;
		}

		SV_VERBOSE(10, "Closing device %s %s (%d/%zu)", survive_colorize_codename(usbInfo->so),
				   survive_colorize(usbInfo->device_info->name), idx, sv->udev_cnt);
		if (usbInfo == sv->hmd_imu) {
			sv->hmd_imu = 0;
			if (sv->hmd_mainboard != 0) {
				survive_close_usb_device(sv->hmd_mainboard);
				sv->hmd_mainboard->so = 0;
			}
		}
		if (usbInfo == sv->hmd_mainboard)
			sv->hmd_mainboard = 0;

		sv->udev_cnt--;
		sv->udev[idx] = sv->udev[sv->udev_cnt];
		sv->udev[sv->udev_cnt] = 0;

		if (usbInfo->ownsObject) {
			survive_destroy_device(usbInfo->so);
		}
		survive_usb_device_t dev = 0;
#ifndef HIDAPI
		dev = libusb_get_device(usbInfo->handle);
#endif
		bool reopen = usbInfo->request_reopen;
		survive_usb_handle_close(usbInfo->handle);
		free(usbInfo);

		if (reopen && dev) {
			survive_vive_add_usb_device(sv, dev);
		}
		return true;
	}
	return false;
}

static int survive_start_get_config(SurviveViveData *sv, struct SurviveUSBInfo *usbInfo, int iface) {
	SurviveContext *ctx = sv->ctx;
	survive_usb_transfer_t *tx = survive_usb_transfer_alloc();
	if (!tx) {
		SV_WARN("Could not allocate transfer frame");
		return -4;
	}
	usbInfo->active_transfers++;

	struct survive_config_packet *config_packet = SV_CALLOC(sizeof(struct survive_config_packet));
	config_packet->tx = tx;
	config_packet->tx->buffer = config_packet->buffer;
	config_packet->tx->actual_length = sizeof(config_packet->buffer);

	usbInfo->cfg_user = config_packet;

	config_packet->ctx = ctx;
	config_packet->sv = sv;
	config_packet->usbInfo = usbInfo;
	config_packet->current_magic = config_packet->usbInfo->device_info->magics;

	USBHANDLE dev = usbInfo->handle;
	config_packet->start_time = survive_run_time(ctx);

	if (config_packet->usbInfo->device_info->codename[0] == 0) {
		config_packet->state = SURVIVE_CONFIG_STATE_MAGICS;
	}
	setup_packet_state(config_packet);

	SV_VERBOSE(10, "Requesting config for %s %p %d",
			   survive_colorize(usbInfo->so ? usbInfo->so->codename : usbInfo->device_info->name), (void *)0,
			   config_packet->state);

	usbInfo->nextCfgSubmitTime = survive_run_time(ctx);

	return 0;
}
static void survive_config_poll(struct SurviveUSBInfo *usbInfo);
int survive_vive_usb_poll(SurviveContext *ctx, void *v) {
	SurviveViveData *sv = v;
	sv->read_count++;

	static FLT last_print = 0;
	FLT now = survive_run_time(ctx);

	FLT time_diff = now - last_print;
	bool print = sv->seconds_per_hz_output > 0 && time_diff > sv->seconds_per_hz_output;

	if (print) {
		size_t total_packets = 0;
		for (int i = 0; i < sv->udev_cnt; i++) {
			const char *codename = sv->udev[i]->so->codename;
			if (sv->udev[i]->so == 0)
				continue;

			for (int j = 0; j < sv->udev[i]->interface_cnt; j++) {
				SurviveUSBInterface *iface = &sv->udev[i]->interfaces[j];
				if (iface->assoc_obj)
					codename = iface->assoc_obj->codename;

				total_packets += iface->packet_count;

				FLT avg_cb_time = iface->sum_cb_time / (FLT)(iface->packet_count + .0001) / 1000.;
				FLT avg_cb_submit_latency = iface->sum_submit_cb_time / (FLT)(iface->packet_count + .0001) / 1000.;

				if (iface->time_constraint == 0 && iface->packet_count) {
					iface->time_constraint = 1000. * avg_cb_submit_latency;
					SV_INFO("Iface %3s %-32s has time constraint of %5.2fms", survive_colorize(codename),
							survive_colorize(iface->hname), avg_cb_submit_latency);
				}
				SV_INFO("Iface %3s %-32s has %5zu packets (%8.2f hz) Avg CB Time: %5.2fms Avg CB Latency: %5.2fms Max "
						"CB Time: %5.2fms Max CB Latency: %5.2fms Time Violations %4d (%7.5f%%)",
						survive_colorize(codename), survive_colorize(iface->hname), iface->packet_count,
						iface->packet_count / time_diff, avg_cb_time, avg_cb_submit_latency, iface->max_cb_time / 1000.,
						iface->max_submit_time / 1000., iface->cb_time_violation,
						100. * iface->cb_time_violation / (FLT)(iface->packet_count + .0001));
				iface->max_cb_time = iface->max_submit_time = iface->sum_cb_time = iface->sum_submit_cb_time = 0;
				iface->cb_time_violation = 0;
				iface->packet_count = 0;
			}
		}

		SV_INFO("Total                  %4zu packets (%6.2f hz) at %7.3fs", total_packets, total_packets / time_diff,
				now);
		last_print = now;
	}

	for (int i = 0; i < sv->udev_cnt; i++) {
		struct SurviveUSBInfo *usbInfo = sv->udev[i];

		if ((usbInfo->device_info->pid == 0x2102 || usbInfo->device_info->pid == 0x2101) && usbInfo->so == 0 &&
			sv->requestPairing && (sv->lastPairTime + 1) < now && now > 3) {
			survive_release_ctx_lock(ctx);
			int r = update_feature_report(usbInfo->handle, 0, vive_request_pairing, sizeof(vive_request_pairing));
			survive_get_ctx_lock(ctx);
			SV_VERBOSE(10, "Pairing attempt...");
			sv->lastPairTime = now;
		}

		survive_config_poll(usbInfo);

		if (survive_handle_close_request_flag(usbInfo)) {
			i--;
		}
	}

#ifdef HIDAPI
#ifdef HID_NONBLOCKING
	survive_release_ctx_lock(ctx);
	for (int i = 0; i < sv->udev_cnt; i++) {
		for (int j = 0; j < sv->udev[i]->interface_cnt; j++) {
			SurviveUSBInterface *iface = &sv->udev[i]->interfaces[j];
			if (iface->assoc_obj)
				HAPIReceiver(iface);
		}
	}
	survive_get_ctx_lock(ctx);
#else
	OGUSleep(1);
	return 0;
#endif
#else
	// int r = libusb_handle_events(sv->usbctx);
	struct timeval tv = {.tv_usec = 10 * 1000};
	survive_release_ctx_lock(ctx);

	int r = libusb_handle_events_timeout(sv->usbctx, &tv);
	survive_get_ctx_lock(ctx);

	if (r) {
		SV_WARN("Libusb poll failed. %d (%s)", r, libusb_error_name(r));
	}
#endif
	return 0;
}

static inline survive_timecode fix_time24(SurviveContext *ctx, survive_timecode time24, survive_timecode refTime) {
	survive_timecode upper_ref = refTime & 0xFF000000u;
	survive_timecode lower_ref = refTime & 0x00FFFFFFu;

	if (lower_ref > time24 && lower_ref - time24 > (1 << 23u)) {
		upper_ref += 0x01000000;
	} else if (lower_ref < time24 && time24 - lower_ref > (1 << 23u) && upper_ref > 0) {
		upper_ref -= 0x01000000;
	}

	//	SV_VERBOSE(250, "fix_time24 time24: %x refTime: %x upper_ref: %x (%x) rtn: %x", time24, refTime, upper_ref,
	//refTime & 0xFF000000u, upper_ref | time24);

	return upper_ref | time24;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define POP1 (*(readdata++))

#ifndef _MSC_VER
struct __attribute__((__packed__)) unaligned_16_t {
	int16_t v;
};
struct __attribute__((__packed__)) unaligned_32_t {
	int32_t v;
};
struct __attribute__((__packed__)) unaligned_u16_t {
	uint16_t v;
};
struct __attribute__((__packed__)) unaligned_u32_t {
	uint32_t v;
};
#else
struct unaligned_16_t {
	int16_t v;
};
struct unaligned_32_t {
	int32_t v;
};
struct unaligned_u16_t {
	uint16_t v;
};
struct unaligned_u32_t {
	uint32_t v;
};
#endif

#define POP2 (((((struct unaligned_u16_t *)((readdata += 2) - 2))))->v)
#define POP4 (((((struct unaligned_u32_t *)((readdata += 4) - 4))))->v)

typedef struct {
	// could use a bitfield here, but since this data is short-lived,
	// the space savings probably isn't worth the processing overhead.
	uint8_t pressedButtonsValid;
	uint32_t touchedButtonsValid;
	uint8_t triggerOfBatteryValid;
	uint8_t batteryChargeValid;
	uint8_t hardwareIdValid;
	uint8_t touchpadHorizontalValid;
	uint8_t touchpadVerticalValid;
	uint8_t triggerHighResValid;
	uint8_t proximityValid;
	uint8_t rawAxisCnt;

	SurviveAxisVal_t rawAxis[16];
	SurviveAxisVal_t /*uint8_t*/ proximity[6];
	uint32_t pressedButtons;
	uint32_t touchedButtons;
	uint16_t triggerOrBattery;
	uint8_t batteryCharge;
	uint32_t hardwareId;
	SurviveAxisVal_t /*uint16_t*/ touchpadHorizontal;
	SurviveAxisVal_t /*uint16_t*/ touchpadVertical;
	SurviveAxisVal_t /*uint16_t*/ triggerHighRes;
} buttonEvent;

static ButtonQueueEntry *prepareNextButtonEvent(SurviveObject *so) {
	ButtonQueueEntry *entry = &(so->ctx->buttonQueue.entry[so->ctx->buttonQueue.nextWriteIndex]);
	memset(entry, 0, sizeof(ButtonQueueEntry));
	assert(so);
	entry->so = so;
	for (int i = 0; i < 16; i++) {
		entry->ids[i] = SURVIVE_AXIS_UNKNOWN;
	}
	entry->buttonId = SURVIVE_BUTTON_UNKNOWN;
	return entry;
}

static ButtonQueueEntry *incrementAndPostButtonQueue(SurviveObject *so) {
	SurviveContext *ctx = so->ctx;

	if (ctx->buttonQueue.buttonservicesem == 0)
		return 0;

	ButtonQueueEntry *entry = &(ctx->buttonQueue.entry[ctx->buttonQueue.nextWriteIndex]);

	SV_VERBOSE(110, "%s Button event %s %d %s %f", survive_colorize_codename(so),
			   SurviveInputEventStr(entry->eventType), entry->buttonId,
			   SurviveAxisStr(so->object_subtype, entry->ids[0]), entry->axisValues[0]);

	for (int i = 0; i < (sizeof(entry->ids) / sizeof(entry->ids[0])) && entry->ids[i] != 255; i++)
		so->axis[entry->ids[i]] = entry->axisValues[i];

	if (entry->buttonId != 255) {
		assert(entry->buttonId < 32);
		bool isTouch =
			entry->eventType == SURVIVE_INPUT_EVENT_TOUCH_UP || entry->eventType == SURVIVE_INPUT_EVENT_TOUCH_DOWN;
		bool isClear =
			entry->eventType == SURVIVE_INPUT_EVENT_TOUCH_UP || entry->eventType == SURVIVE_INPUT_EVENT_BUTTON_UP;

		uint32_t mask = 1u << entry->buttonId;
		uint32_t *maskp = isTouch ? &so->touchmask : &so->buttonmask;
		uint32_t maskv = *maskp;
		if (isClear)
			*maskp &= ~mask;
		else
			*maskp |= mask;
		// assert(maskv != *maskp);
	}

	if ((ctx->buttonQueue.nextWriteIndex + 1) % BUTTON_QUEUE_MAX_LEN == ctx->buttonQueue.nextReadIndex) {
		// There's not enough space to write this entry.  Clear it out and move along
		SV_WARN("Button buffer full");
		memset(entry, 0, sizeof(ButtonQueueEntry));
		return 0;
	}
	entry->isPopulated = 1;
	ctx->buttonQueue.nextWriteIndex++;
	// if we've exceeded the size of the buffer, loop around to the beginning.
	if (ctx->buttonQueue.nextWriteIndex >= BUTTON_QUEUE_MAX_LEN) {
		ctx->buttonQueue.nextWriteIndex = 0;
	}
	OGUnlockSema(ctx->buttonQueue.buttonservicesem);

	// clear out any old data in the entry so we always start with a clean slate.
	entry = &(ctx->buttonQueue.entry[ctx->buttonQueue.nextWriteIndex]);
	memset(entry, 0, sizeof(ButtonQueueEntry));
	return prepareNextButtonEvent(so);
}

enum ButtonEventSource {
	BUTTON_EVENT_SOURCE_DEFAULT = 0,
	BUTTON_EVENT_SOURCE_RF = 1,

	BUTTON_TOUCH_EVENT_SOURCE = 0x80
};

struct DeviceMapping {
	bool isRF;
	SurviveObjectSubtype objectSubtype;
	bool isTouch[64];
	enum SurviveButton buttonMap[64];
	enum SurviveAxis axisMap[16];
};

static void init_device_mapping(struct DeviceMapping *mapping) {
	memset(mapping, 0, sizeof(struct DeviceMapping));
	for (int i = 0; i < 64; i++)
		mapping->buttonMap[i] = SURVIVE_BUTTON_UNKNOWN;
	for (int i = 0; i < 16; i++)
		mapping->axisMap[i] = SURVIVE_AXIS_UNKNOWN;

	for (int i = 32; i < 64; i++) {
		mapping->isTouch[i] = true;
	}
}

static struct DeviceMapping *RFWandMapping() {
	static struct DeviceMapping mapping = {0};
	if (mapping.objectSubtype == SURVIVE_OBJECT_SUBTYPE_GENERIC) {
		init_device_mapping(&mapping);

		mapping.isRF = true;
		mapping.objectSubtype = SURVIVE_OBJECT_SUBTYPE_WAND;
		mapping.axisMap[1] = SURVIVE_AXIS_TRIGGER;
		mapping.axisMap[2] = SURVIVE_AXIS_TRACKPAD_X;
		mapping.axisMap[3] = SURVIVE_AXIS_TRACKPAD_Y;

		mapping.buttonMap[0] = SURVIVE_BUTTON_TRIGGER;
		mapping.buttonMap[1] = SURVIVE_BUTTON_TRACKPAD;
		mapping.isTouch[1] = true;
		mapping.buttonMap[2] = SURVIVE_BUTTON_TRACKPAD;
		mapping.buttonMap[3] = SURVIVE_BUTTON_SYSTEM;
		mapping.buttonMap[4] = SURVIVE_BUTTON_GRIP;
		mapping.buttonMap[5] = SURVIVE_BUTTON_MENU;
	}
	return &mapping;
}
static struct DeviceMapping *WiredWandMapping() {
	static struct DeviceMapping mapping = {0};
	if (mapping.objectSubtype == SURVIVE_OBJECT_SUBTYPE_GENERIC) {
		init_device_mapping(&mapping);

		mapping.isRF = false;
		mapping.objectSubtype = SURVIVE_OBJECT_SUBTYPE_WAND;
		mapping.axisMap[1] = SURVIVE_AXIS_TRIGGER;
		mapping.axisMap[2] = SURVIVE_AXIS_TRACKPAD_X;
		mapping.axisMap[3] = SURVIVE_AXIS_TRACKPAD_Y;

		mapping.buttonMap[0] = SURVIVE_BUTTON_TRIGGER;
		mapping.buttonMap[2] = SURVIVE_BUTTON_GRIP;
		mapping.buttonMap[12] = SURVIVE_BUTTON_MENU;
		mapping.buttonMap[13] = SURVIVE_BUTTON_SYSTEM;
		mapping.buttonMap[18] = SURVIVE_BUTTON_TRACKPAD;
		mapping.buttonMap[20] = SURVIVE_BUTTON_TRACKPAD;
		mapping.isTouch[20] = true;
	}
	return &mapping;
}
static struct DeviceMapping *RFKnuckles() {
	static struct DeviceMapping mapping = {0};
	if (mapping.objectSubtype == SURVIVE_OBJECT_SUBTYPE_GENERIC) {
		init_device_mapping(&mapping);

		mapping.isRF = false;
		mapping.objectSubtype = SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L;
		mapping.axisMap[1] = SURVIVE_AXIS_TRIGGER;
		mapping.axisMap[2] = SURVIVE_AXIS_TRACKPAD_X;
		mapping.axisMap[3] = SURVIVE_AXIS_TRACKPAD_Y;
		mapping.axisMap[4] = SURVIVE_AXIS_MIDDLE_FINGER_PROXIMITY;
		mapping.axisMap[5] = SURVIVE_AXIS_RING_FINGER_PROXIMITY;
		mapping.axisMap[6] = SURVIVE_AXIS_PINKY_FINGER_PROXIMITY;
		mapping.axisMap[7] = SURVIVE_AXIS_TRIGGER_FINGER_PROXIMITY;
		mapping.axisMap[8] = SURVIVE_AXIS_GRIP_FORCE;
		mapping.axisMap[9] = SURVIVE_AXIS_TRACKPAD_FORCE;
		mapping.axisMap[10] = SURVIVE_AXIS_JOYSTICK_X;
		mapping.axisMap[11] = SURVIVE_AXIS_JOYSTICK_Y;

		mapping.buttonMap[0] = SURVIVE_BUTTON_TRIGGER;
		mapping.buttonMap[1] = SURVIVE_BUTTON_TRACKPAD;
		mapping.isTouch[1] = true;

		mapping.buttonMap[2] = SURVIVE_BUTTON_THUMBSTICK;
		mapping.buttonMap[3] = SURVIVE_BUTTON_SYSTEM;
		mapping.buttonMap[4] = SURVIVE_BUTTON_A;
		mapping.buttonMap[5] = SURVIVE_BUTTON_B;
		mapping.buttonMap[6] = SURVIVE_BUTTON_MENU;

		for (int i = 0; i < 32; i++) {
			mapping.buttonMap[i + 32] = mapping.buttonMap[i];
		}
		mapping.buttonMap[1 + 32] = SURVIVE_BUTTON_UNKNOWN;
	}
	return &mapping;
}
static struct DeviceMapping *WiredTracker() {
	static struct DeviceMapping mapping = {0};
	if (mapping.objectSubtype == SURVIVE_OBJECT_SUBTYPE_GENERIC) {
		init_device_mapping(&mapping);

		mapping.isRF = false;
		mapping.objectSubtype = SURVIVE_OBJECT_SUBTYPE_TRACKER;
		mapping.buttonMap[13] = SURVIVE_BUTTON_SYSTEM;
	}
	return &mapping;
}
struct DeviceMapping *getDeviceMapping(SurviveObjectSubtype type, enum ButtonEventSource source) {
	static struct DeviceMapping *DeviceMappings[SURVIVE_OBJECT_SUBTYPE_COUNT * 2] = {0};
	static bool init = false;
	if (init == false) {
		init = true;

		DeviceMappings[SURVIVE_OBJECT_SUBTYPE_WAND] = WiredWandMapping();
		DeviceMappings[SURVIVE_OBJECT_SUBTYPE_TRACKER] = WiredTracker();
		DeviceMappings[SURVIVE_OBJECT_SUBTYPE_TRACKER_GEN2] = WiredTracker();

		DeviceMappings[SURVIVE_OBJECT_SUBTYPE_WAND + SURVIVE_OBJECT_SUBTYPE_COUNT] = RFWandMapping();
		DeviceMappings[SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R + SURVIVE_OBJECT_SUBTYPE_COUNT] = RFKnuckles();
		DeviceMappings[SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L + SURVIVE_OBJECT_SUBTYPE_COUNT] = RFKnuckles();
	}
	return DeviceMappings[type + SURVIVE_OBJECT_SUBTYPE_COUNT * ((source & (BUTTON_EVENT_SOURCE_RF)) != 0)];
}
static enum SurviveButton get_button_id_for_idx_from_mapping(struct DeviceMapping *mapping, uint8_t idx,
															 bool *isTouchEvent, enum ButtonEventSource source) {
	size_t buttonIdx = idx + ((source & BUTTON_TOUCH_EVENT_SOURCE) != 0) * 32;
	*isTouchEvent = mapping->isTouch[buttonIdx];
	enum SurviveButton rtn = mapping->buttonMap[buttonIdx];
	assert(rtn == 255 || rtn < 32);
	return rtn;
}

static enum SurviveButton get_button_id_for_idx(const SurviveObject *so, uint8_t idx, bool *isTouchEvent,
												enum ButtonEventSource source) {
	struct DeviceMapping *mapping = getDeviceMapping(so->object_subtype, source);
	if (mapping) {
		return get_button_id_for_idx_from_mapping(mapping, idx, isTouchEvent, source);
	}

	return idx;
}

#define HAS_BIT_FLAG(x, idx) (((x) & (1u << (idx))) != 0u)

static inline ButtonQueueEntry *registerButtonOnOff(SurviveObject *so, ButtonQueueEntry *entry, uint32_t incoming_mask,
													enum ButtonEventSource source) {
	struct SurviveContext *ctx = so->ctx;
	for (uint8_t a = 0; a < 32; a++) {
		bool isTouchEvent = (source & BUTTON_TOUCH_EVENT_SOURCE) != 0;
		enum SurviveButton id = get_button_id_for_idx(so, a, &isTouchEvent, source);
		enum SurviveInputEvent eventTypeDown =
			isTouchEvent ? SURVIVE_INPUT_EVENT_TOUCH_DOWN : SURVIVE_INPUT_EVENT_BUTTON_DOWN;
		enum SurviveInputEvent eventTypeUp =
			isTouchEvent ? SURVIVE_INPUT_EVENT_TOUCH_UP : SURVIVE_INPUT_EVENT_BUTTON_UP;

		uint8_t current_mask = eventTypeDown == SURVIVE_INPUT_EVENT_BUTTON_DOWN ? so->buttonmask : so->touchmask;

		if (id == 255) {
			if (HAS_BIT_FLAG(incoming_mask, a))
				SV_WARN("%s has unknown button input %d %d", so->codename, a, source);
			// assert(
			continue;
		}

		if (entry && HAS_BIT_FLAG(incoming_mask, a) != HAS_BIT_FLAG(current_mask, id)) {
			entry->eventType = HAS_BIT_FLAG(incoming_mask, a) ? eventTypeDown : eventTypeUp;
			entry->buttonId = id;

			entry = incrementAndPostButtonQueue(so);
		}
	}

	return entry;
}

// important!  This must be the only place that we're posting to the buttonEntryQueue
// if that ever needs to be changed, you will have to add locking so that only one
// thread is posting at a time.
static void registerButtonEvent(SurviveObject *so, buttonEvent *event, enum ButtonEventSource source) {
	ButtonQueueEntry *entry = prepareNextButtonEvent(so);
	struct SurviveContext *ctx = so->ctx;

	if (event->pressedButtonsValid) {
		SV_VERBOSE(1000, "buttons %8x", event->pressedButtons);
		entry = registerButtonOnOff(so, entry, event->pressedButtons, source);
	}

	if (event->touchedButtonsValid) {
		SV_VERBOSE(1000, "touched %8x", event->touchedButtons);
		entry = registerButtonOnOff(so, entry, event->touchedButtons, source | BUTTON_TOUCH_EVENT_SOURCE);
	}

	if (event->triggerHighResValid) {
		if (so->axis[1] != event->triggerHighRes) {
			entry->eventType = SURVIVE_INPUT_EVENT_AXIS_CHANGED;
			entry->ids[0] = 1;
			entry->axisValues[0] = event->triggerHighRes;
			entry = incrementAndPostButtonQueue(so);
		}
	}

	if ((event->touchpadHorizontalValid) && (event->touchpadVerticalValid)) {
		enum SurviveAxis ax = SURVIVE_AXIS_TRACKPAD_X;
		enum SurviveAxis ay = SURVIVE_AXIS_TRACKPAD_Y;

		if (so->object_subtype == SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R ||
			so->object_subtype == SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L) {
			if (!HAS_BIT_FLAG(so->touchmask, SURVIVE_BUTTON_TRACKPAD)) {

				if ((so->axis[ax] != 0) || (so->axis[ay] != 0)) {
					entry->eventType = SURVIVE_INPUT_EVENT_AXIS_CHANGED;

					entry->ids[0] = ax;
					entry->ids[1] = ay;
					entry->axisValues[0] = 0;
					entry->axisValues[1] = 0;

					entry = incrementAndPostButtonQueue(so);
				}

				ax = SURVIVE_AXIS_JOYSTICK_X;
				ay = SURVIVE_AXIS_JOYSTICK_Y;
			}
		}

		if ((so->axis[ax] != event->touchpadHorizontal) || (so->axis[ay] != event->touchpadVertical)) {
			entry->eventType = SURVIVE_INPUT_EVENT_AXIS_CHANGED;

			entry->ids[0] = ax;
			entry->ids[1] = ay;
			entry->axisValues[0] = event->touchpadHorizontal;
			entry->axisValues[1] = event->touchpadVertical;

			entry = incrementAndPostButtonQueue(so);
		}
	}

	if (event->proximityValid) {
		size_t axisCnt = 0;
		for (int i = 0; i < 6; i++) {
			if (event->proximity[i] != so->axis[SURVIVE_AXIS_MIDDLE_FINGER_PROXIMITY + i]) {
				entry->eventType = SURVIVE_INPUT_EVENT_AXIS_CHANGED;
				entry->ids[axisCnt] = SURVIVE_AXIS_MIDDLE_FINGER_PROXIMITY + i;
				entry->axisValues[axisCnt++] = event->proximity[i];
			}
		}

		if (axisCnt > 0) {
			entry = incrementAndPostButtonQueue(so);
		}
	}

	for (int i = 0; i < event->rawAxisCnt; i++) {
		if (event->rawAxis[i] != so->axis[i]) {
			entry->eventType = SURVIVE_INPUT_EVENT_AXIS_CHANGED;
			entry->ids[0] = i;
			entry->axisValues[0] = event->rawAxis[i];
			entry = incrementAndPostButtonQueue(so);
		}
	}

	if (event->batteryChargeValid) {
		so->charge = event->batteryCharge;
	}
}

#define FAILURE_ON_FALSE(x)                                                                                            \
	if (!(x)) {                                                                                                        \
		SV_WARN("Assert failure for parsing watchman: " #x);                                                           \
		goto failure;                                                                                                  \
	};

//#define DEBUG_WATCHMAN
#define DEBUG_WATCHMAN_ERRORS
#ifdef DEBUG_WATCHMAN
#define DEBUG_WATCHMAN_ERRORS
#endif

#define AS_SHORT(a, b) ((uint16_t)(((uint16_t)a) << 8) | (0x00ff & b))
#define POP_BYTE(ptr) ((uint8_t) * ((ptr)++))
#define POP_SHORT(ptr) (((((struct unaligned_u16_t *)(((ptr) += 2) - 2))))->v)

#define HAS_FLAG(flags, flag) ((flags & (flag)) == (flag))

char hexstr[512];
static char *packetToHex(const uint8_t *packet, const uint8_t *packetEnd) {
	int count = packetEnd - packet;
	int i;
	for (i = 0; i < count; i++)
		sprintf(&hexstr[i * 3], "%02x ", packet[i]);
	hexstr[i * 3] = 0;

	return hexstr;
}

char bin[9] = {0};
static char *byteToBin(uint8_t b) {
	for (int i = 0; i < 8; i++) {
		bin[i] = ((b >> (7 - i)) & 1) ? '1' : '0';
	}
	bin[8] = 0;
	return bin;
}

struct sensorData {
	uint8_t sensorId;
	uint8_t edgeCount;
};

static int32_t read_light_data(SurviveObject *w, uint16_t time, uint8_t **readPtr, uint8_t *payloadEndPtr,
							   LightcapElement *output, int output_cnt) {
	uint8_t *payloadPtr = *readPtr;
	SurviveContext *ctx = w->ctx;
	uint32_t reference_time = w->activations.last_imu;

	bool isGen2 = 0; // w->ctx->lh_version == 1;
	(void)isGen2;

	if (payloadEndPtr - payloadPtr <= 3) {
		return 0;
	}

	// DEBUG
	if ((*payloadPtr & 0xE0) == 0xE0) {
		SV_WARN("Light contains probable non-light data : 0x%02hX [Time:%04hX] [Payload: %s]", *payloadPtr, time,
				packetToHex(payloadPtr, payloadEndPtr));
	}

	/*
	 * ---=== LIGHT DATA STRUCTURE ===---
	 *
	 * | SensorData  | Time Deltas          | End Timestamp |
	 * ╔═════════════╦══════════════════════╦═══════════════╗
	 * ║ SS SS .. SS ║ DD DD DD DD DD .. DD ║ TT TT TT      ║
	 * ╚═════════════╩══════════════════════╩═══════════════╝
	 *
	 * Three parts to the packet, the sensor data which contains which sensors were triggered, and the times
	 * deltas between rising and falling of the sensor event. There are always two rising/falling events per
	 * sensor though the ordering is not simple as new sensor events may start before others are finished.
	 *
	 * The meaning and associated led with each 'event' is determined by the edge count as encoded within the
	 * sensor data (see below)
	 *
	 * The time deltas use variable length encoding, so we can't determine how many sensors are in the packet
	 * just from the packet length. However, we do know that there are always two times per sensor (rise and
	 * fall), so there are (2*Sensor)-1 deltas in the packet (-1 because the end time is 'known' yielding two
	 * times). Therefore, the general read process is thus:
	 *
	 *  1) Read off timestamp
	 *  2) Read the first byte from the start of the packet
	 *  3) Read one time delta from the end of the packet (We get two times from this since we know the 'end' time)
	 *  4) Repeatedly:
	 *     a) Read one byte from the start of the packet (Led/Flag)
	 *     b) Read two time deltas from the end of the packet (not including timestamp) [See encoding below]
	 *     c) Stop once we've read all data in the packet
	 *
	 *
	 * TT TT TT
	 * ~~~~~~~~
	 *  Timestamp of the last event [Little Endian]
	 *
	 * eg:
	 *  f6 b4 5b = 6010102
	 *
	 * DD
	 * ~~
	 *  Time deltas between events stored as variable length sequences. The lower 7 bits of each byte are
	 *  summed until a byte with the 8th bit set is encountered. [Little endian]
	 *
	 *   8 76543210
	 *  ╔═╦════════╗
	 *  ║S│Value   ║
	 *  ╚═╩════════╝
	 *    ╲  ╲_________ 7 Bits of time delta
	 *     ╲___________ Stop bit (1 = Value complete, 0 = Continue reading)
	 *
	 *  eg:
	 *      0 = 80       [(80 & 7F)                  = 0]
	 *    127 = FF       [(FF & 7F)                  = 127]
	 *    128 = 80 01    [(80 & 7F) + ((01 & 7F)<<7) = 128]
	 *    255 = FF 01    [(FF & 7F) + ((01 & 7F)<<7) = 255]
	 *    256 = 80 02    [(80 & 7F) + ((02 & 7F)<<7) = 256]
	 *  16383 = FF 7F    [(FF & 7F) + ((7F & 7F)<<7) = 16383]
	 *  16384 = 80 80 01 [(80 & 7F) + ((80 & 7F)<<7) + ((01 & 7F)<<14) = 16384]
	 *
	 *
	 *
	 * SS
	 * ~~
	 *  Packed data about which sensor was detected and how many time deltas it's associated event straddles.
	 *  1 Byte per sensor
	 *
	 *   876543 210
	 *  ╔══════╦═══╗
	 *  ║Sensor│EC ║
	 *  ╚══════╩═══╝
	 *    ╲      ╲____ Edge count
	 *     ╲__________ Sensor ID of the event
	 *
	 *  eg:
	 *    2B = Sensor 5, 3 edges [2B>>3 = 5, 2B & 03 = 3]
	 *
	 *
	 * Example full packet
	 * ===================
	 *
	 *   ┌──────┬───────┐
	 *   │Sensor│ Edges │
	 *   ├──────┼───────┤
	 *   │   5  │   3   │
	 *   │  10  │   1   │
	 *   │   9  │   2   │
	 *   │   5  │   0   │
	 *   │  12  │   0   │
	 *   └──────┴───────┘                           End Time : 6010102
	 *              ╲                               ╱
	 *               ╲                             ╱
	 *            ╔════════════════╦═┄┄┄┄┄┄┄┄┄┄┄═╦══════════╗
	 *            ║ 2b 51 4a 28 60 ║ Time Deltas ║ f6 b4 5b ║
	 *            ╚════════════════╩═┄┄┄┄┄┄┄┄┄┄┄═╩══════════╝
	 *                             ╱              ╲
	 *     _______________________╱                ╲_______________________
	 *    ╱                                                                ╲
	 *   ╱                                                                  ╲
	 *  ╔═══════╤═══════╤═══════╤══════════╤═══════╤════╤════╤═══════╤═══════╗
	 *  ║ c7 2e │ e6 66 │ 84 1f │ fb 31 0b │ d9 01 │ da │ ca │ db 02 │ e4 02 ║
	 *  ╠═══════╪═══════╪═══════╪══════════╪═══════╪════╪════╪═══════╪═══════╣
	 *  ║ 5959  | 13158 | 3972  | 186619   | 217   | 90 | 74 | 347   |356    ║
	 *  ╚═══════╧═══════╧═══════╧══════════╧═══════╧════╧════╧═══════╧═══════╝
	 *  │       │       │       │          │       │    │    │       │       │
	 *  ┕━━━━━━«E       │       │          │       │    │    │       │       │ -> Led 12 : 5799310 -> 5805269
	 *                  ┕━━━━━━«D          │       │    │    │       │       │ -> Led 5  : 5818427 -> 5822399
	 *                                     ┕━━━━━━━2━━━━2━━━«C       │       │ -> Led 9  : 6009018 -> 6009399
	 *                                             │    │    ┊       │       │
	 *                                             ┕━━━━3━━━━3━━━━━━━3━━━━━━«A -> Led 5  : 6009235 -> 6010102
	 *                                                  |    ┊       |
	 *                                                  ┕━━━━1━━━━━━«B         -> Led 10 : 6009325 -> 6009746
	 * Read order :
	 *  A : Ends at 'A' - Skip 3 edges to find start
	 *  B : Ends at 'B' - Skip 1 edge to find start
	 *  C : Ends at 'C' - Skip 2 edges to find start
	 *  D : Ends at 'D' (Since the 'end' edges from ABC have already been 'used'), ends at next edge
	 *  E : Ends at 'E' - Ends at next edge
	 */

	// Step 1 - Extract deltas between events the corresponding sensors
	size_t timeIndex = 0;
	uint32_t times[16] = {0};
	size_t maxTimeIndex = sizeof(times) / sizeof(times[0]);
	size_t maxEvents = maxTimeIndex >> 1;
	uint8_t *reportOrder = alloca(sizeof(uint8_t*) * maxTimeIndex);

	struct sensorData* sensors = alloca(sizeof(struct sensorData) * maxEvents);

	uint8_t *idsPtr = payloadPtr;
	uint8_t *eventPtr = payloadEndPtr;

	// Last three bytes of light data are the LSB of the time of the last event
	eventPtr -= 4;
	uint32_t lastEventTime =
		((uint32_t)(time >> 8) << 24) | (eventPtr[3] << 16) | (eventPtr[2] << 8) | (eventPtr[1] << 0);

	// The general issue is that the 'time1' field isn't super in sync with the last 3 bytes we use for timing
	// light events -- it can tip a smidge before those bytes see it or sometimes after. This can cause
	// wild 1<<24 tick differences which break everything.
	//
	// We base it off IMU as a reference because light events can get blocked and it's not out of the ordinary
	// to not see them for a second or two. (1 << 23) on a 48mhz clock is ~150ms; and the IMU is consistently
	// much faster than that.
	if (lastEventTime > reference_time && lastEventTime - reference_time > (1 << 23)) {
		lastEventTime -= (1 << 24);
	} else if (reference_time > lastEventTime && reference_time - lastEventTime > (1 << 23)) {
		lastEventTime += (1 << 24);
	}

	times[0] = lastEventTime;
	SV_VERBOSE(750, "Packet Start Time: %u (0x%x) (ref: %u) Payload: %s", lastEventTime, lastEventTime, reference_time,
			   packetToHex(payloadPtr, payloadEndPtr));

	while (idsPtr + (timeIndex >> 1u) < eventPtr) {
		// Obtain the timing to the previous event
		// Variable length encoding [if bit 8 is 0, continue into next byte]
		uint32_t timeDelta = 0;
		uint8_t *eventPtrStart = eventPtr;
		while (true) {
			timeDelta <<= 7;
			timeDelta |= (*eventPtr & 0x7F);
			if (((*(eventPtr--)) & 0x80) == 0x80)
				break;

			if (idsPtr + (timeIndex >> 1u) > eventPtr) {
				eventPtr = eventPtrStart;
				goto exit_while;
			}
		}
		lastEventTime -= timeDelta;
		if (timeIndex >= 16) {
			return -8;
		}
		// Store the event time
		times[++timeIndex] = lastEventTime;
		SV_VERBOSE(750, "Time: [%zd] %u (%u) %s", timeIndex, lastEventTime, timeDelta,
				   packetToHex(eventPtr + 1, eventPtrStart + 1));
	}

exit_while:
	if (timeIndex % 2 == 0 && timeIndex > 0) {
		timeIndex--;
		do {
			eventPtr++;
		} while ((*eventPtr & 0x80) == 0);
	}
	assert(timeIndex % 2 == 1);

	size_t sensor_byte_cnt = eventPtr - idsPtr + 1;
	if (sensor_byte_cnt > ((timeIndex >> 1) + 1)) {
		isGen2 = true;
		//	    SV_VERBOSE(100, "Likely gen2 data; %x %d %d %d", *idsPtr, sensor_byte_cnt, (timeIndex >> 1), idsPtr -
		//(eventPtr - (timeIndex >> 1)));
		idsPtr = eventPtr - (timeIndex >> 1);
	}

	// There are two time deltas per 'event'
	for (int i = 0; i < (timeIndex >> 1) + 1; i++) {
		sensors[i].sensorId = ((*idsPtr) >> 3) & 0x1F;
		sensors[i].edgeCount = (*idsPtr) & 0x7;
		SV_VERBOSE(750, "Sensor: %2d Edge: %d (%02x)", sensors[i].sensorId, sensors[i].edgeCount, (*idsPtr));
		idsPtr++;
	}

	// Step 2 - Convert events to pulses
	LightcapElement* les = alloca(sizeof(LightcapElement) * maxEvents);
	size_t eventCount = (timeIndex + 1) >> 1; // timeIndex>>1 = There are always twice as many time events as sensors

	memset(les, 0, maxEvents * sizeof(LightcapElement));
	memset(reportOrder, 0, maxTimeIndex * sizeof(uint8_t));
	timeIndex = -1;

	for (int i = 0; i < eventCount; i++) {
		// Get the end time (Increment and find the next 'unused' time)
		while (times[++timeIndex] == 0)
			if (timeIndex + 1 >= maxTimeIndex) {
				return -2;
			}
		if (timeIndex >= maxTimeIndex) {
			return -3;
		}

		// Get the start time
		size_t startTimeIndex = timeIndex + (sensors[i].edgeCount + 1);
		if (startTimeIndex >= maxTimeIndex) {
			return -4;
		}

		// Store the start index so we can return in ascending time order
		if (reportOrder[startTimeIndex] != 0) {
			return -5;
		}
		reportOrder[startTimeIndex] = i + 1;
		LightcapElement *le = &les[i];

		// Fill in the LightcapElement data
		assert(le->sensor_id == 0);

		le->sensor_id = sensors[i].sensorId;
		le->timestamp = times[startTimeIndex];
		le->length = times[timeIndex] - times[startTimeIndex];

		// Flag the start time as 'used'
		times[startTimeIndex] = 0;

		// SV_INFO("Light Event : [%i|%i] %li -> %li (%li) [%i-%i]", les[i].sensor_id, sensors[i].edgeCount,
		// les[i].timestamp, les[i].timestamp + les[i].length, les[i].length, startTimeIndex, timeIndex);
	}

	// Output the events in ascending time order
	uint8_t orderedIndex;
	for (int i = 0; (i < maxTimeIndex) && output_cnt > 0; i++) {
		if ((orderedIndex = reportOrder[i]) != 0) {
			LightcapElement *ol = &les[orderedIndex - 1];

			if (!(ol->length != 0 || ol->timestamp != 0)) {
				return -6;
			}

			*(output++) = *ol;
			output_cnt--;
			SV_VERBOSE(750, "Light Event [Ordered]: %i [%2i] %u -> %u (%4hu)", i, ol->sensor_id, ol->timestamp,
					   ol->timestamp + ol->length, ol->length);
		}
	}

	return eventCount;
}

static bool read_imu_data(SurviveObject *w, uint64_t time_in_us, uint16_t time, uint8_t **readPtr,
						  const uint8_t *payloadEndPtr) {
	uint8_t *payloadPtr = *readPtr;

	SurviveContext *ctx = w->ctx;
	if (payloadEndPtr - payloadPtr < 7) {
		return false;
	}

	// First byte is higher res time, followed by 6 shorts
	uint8_t timeLSB = POP_BYTE(payloadPtr);
	int16_t aX = POP_SHORT(payloadPtr);
	int16_t aY = POP_SHORT(payloadPtr);
	int16_t aZ = POP_SHORT(payloadPtr);
	int16_t rX = POP_SHORT(payloadPtr);
	int16_t rY = POP_SHORT(payloadPtr);
	int16_t rZ = POP_SHORT(payloadPtr);

	FLT agm[9] = {aX, aY, aZ, rX, rY, rZ};

	SV_VERBOSE(750, "%s IMU: %d " Point3_format " " Point3_format " From: %s", w->codename, timeLSB,
			   LINMATH_VEC3_EXPAND(agm), LINMATH_VEC3_EXPAND(agm + 3), packetToHex(*readPtr, payloadPtr));
	SURVIVE_INVOKE_HOOK_SO(raw_imu, w, 3, agm, ((uint32_t)time << 16) | (timeLSB << 8), 0);

	SurviveSensorActivations_register_runtime(&w->activations, w->activations.last_imu, time_in_us);

	*readPtr = payloadPtr;

	return true;
}
#define UPDATE_PTR_AND_RETURN                                                                                          \
	*readPtr = payloadPtr;                                                                                             \
	return true;

static void handle_battery(SurviveObject *w, uint8_t batStatus) {
	// int8_t percent = (int8_t)((((FLT)(batStatus & 0x7f)) / 0x7f) * 100);
	int8_t percent = (batStatus & 0x7fu);
	bool charging = (batStatus & 0x80) == 0x80;
	if (percent != 0 || charging) {
		w->charging = charging;
		w->charge = percent;
		SurviveContext *ctx = w->ctx;
		SV_VERBOSE(110, "%s Battery charge %d%%(%s)", w->codename, percent, charging ? "Charging" : "Not charging");
	}
}

static bool read_event(SurviveObject *w, uint64_t time_in_us, uint16_t time, uint8_t **readPtr,
					   uint8_t *payloadEndPtr) {
	uint8_t *payloadPtr = *readPtr;
	SurviveContext *ctx = w->ctx;

	// If we're looking at light data, return
	if (!HAS_FLAG(*payloadPtr, 0xE0))
		return true;

	// This is some kind of heartbeat
	if (*payloadPtr == 0xE2) {
		*readPtr = payloadEndPtr;
		return true;
	}

	/*
	 * Event Flags
	 * ===========
	 *
	 * If input (button, touch, motion) data is present in packet:
	 *   ┄╦═╤═╤═╦═╤═╤═╤═╤═╦┄
	 *    ║1│1│1│1│I│-│-│-║
	 *   ┄╩═╧═╧═╩═╧═╧═╧═╧═╩┄
	 *
	 * If battery data is present in a packet:
	 *   ┄╦═╤═╤═╦═╤═╤═╤═╤═╦═══════════════╦┄
	 *    ║1│1│1│0│?│?│?│1║ Battery       ║
	 *   ┄╩═╧═╧═╩═╧═╧═╧═╧═╩═══════════════╩┄
	 *    ▲                               │
	 *    ╰───────────────────────────────╯
	 *
	 * If battery data is not present in packet (EG IMU only packets):
	 *   ┄╦═╤═╤═╦═╤═╤═╤═╤═╦┄
	 *    ║1│1│1│0│I│?│?│0║
	 *   ┄╩═╧═╧═╩═╧═╧═╧═╧═╩┄
	 *
	 * I: IMU Data      1 = IMU Data present after event (13 Bytes)
	 *                  0 = No IMU Data present after event
	 */

	const uint8_t flags = POP_BYTE(payloadPtr);

	bool flagInput = HAS_FLAG(flags, 0x10);
	bool flagIMU = HAS_FLAG(flags, 0x08);
	if (flagInput) {
		/*
		 * Flags for input events are as follows:
		 *
		 * ┄╦═╤═╤═╦═╤═╤═╤═╤═╦┄
		 *  │1│1│1│1│-│t│m│b│
		 * ┄╩═╧═╧═╩═╧═╧═╧═╧═╩┄
		 *
		 * t: Trigger    1 = Trigger data present in event [1 Byte]  ╮
		 * m: Motion     1 = Motion data present in event [4 Byte]   ├ If all 0, this is a gen 2 event [See below]
		 * b: Button     1 = Button data present in event [1 Byte]   ╯
		 *
		 * Order of data in payload is as follows:
		 * ┄╦═══════════════╦═══════════════╦═══════════════╦═══════════════╦
		 *  ║ [Button/b]    ║ [Trigger/t]   ║ [Motion/t]    ║ [IMU Data/I]  ║
		 * ┄╩═══════════════╩═══════════════╩═══════════════╩═══════════════╩
		 */

		bool firstGen = ((flags & 0x7) != 0);

		buttonEvent bEvent = {0};
		if (firstGen) {
			bool flagTrigger = HAS_FLAG(flags, 0x4);
			bool flagMotion = HAS_FLAG(flags, 0x2);
			bool flagButton = HAS_FLAG(flags, 0x1);

			if (flagButton) {
				bEvent.pressedButtonsValid = 1;
				bEvent.pressedButtons = POP_BYTE(payloadPtr);
			}

			if (flagTrigger) {
				bEvent.triggerHighResValid = 1;
				bEvent.triggerHighRes = POP_BYTE(payloadPtr) / 255.;
			}

			if (flagMotion) {
				bEvent.touchpadHorizontalValid = 1;
				bEvent.touchpadVerticalValid = 1;

				bEvent.touchpadHorizontal = (int16_t)POP_SHORT(payloadPtr) / 32768.;
				bEvent.touchpadVertical = (int16_t)POP_SHORT(payloadPtr) / 32768.;
			}

		} else {
			// Second gen event (Eg Knuckles proximity)
			uint8_t genTwoType =
				POP_BYTE(payloadPtr); // May be flags, but currently only observed to be 'a1' when knuckles

			if (genTwoType == 0xA1) {
				// Knucles
				/*SV_INFO("GRIP 0x%02hX [Time:%04hX] [Payload: %s] <<ABORT FURTHER READ>>",
				 *(payloadPtr-1), time, packetToHex(payloadPtr, payloadEndPtr));*/

				// Resistive contact sensors in buttons?
				uint8_t touchFlags = POP_BYTE(payloadPtr);
				// 0x01 = Trigger
				// 0x08 = Menu
				// 0x10 = Button A
				// 0x20 = Button B
				// 0x40 = Thumbstick

				bEvent.touchedButtonsValid = 1;
				bEvent.touchedButtons = (touchFlags & ~0x40u);
				bEvent.touchedButtons |= ((touchFlags & 0x40u) >> 4);
				bEvent.proximityValid = 1;

				// Non-touching proximity to fingers
				bEvent.proximity[0] = POP_BYTE(payloadPtr) / 255.; // Middle finger
				bEvent.proximity[1] = POP_BYTE(payloadPtr) / 255.; // Ring finger
				bEvent.proximity[2] = POP_BYTE(payloadPtr) / 255.; // Pinky finger
				bEvent.proximity[3] = POP_BYTE(payloadPtr) / 255.; // Index finger (trigger)

				// Contact force (Squeeze strength)
				bEvent.proximity[4] = POP_BYTE(payloadPtr) / 255.;
				bEvent.proximity[5] = POP_BYTE(payloadPtr) / 255.;
			} else {
				SV_WARN("Unknown gen two event %s 0x%02hX 0b%s [Time:%04hX] [Payload: %s] <<ABORT FURTHER READ>>",
						w->codename, *(payloadPtr - 1), byteToBin(*(payloadPtr - 1)), time,
						packetToHex(payloadPtr, payloadEndPtr));
				// Since we don't know how much data this should consume, proceeding to IMU/Light decode is likely
				// to choke.
				return false;
			}
		}
		registerButtonEvent(w, &bEvent, BUTTON_EVENT_SOURCE_RF);
	} else {
		/*
		 * Flags for non-input (status) events are as follows:
		 *
		 * ┄╦═╤═╤═╦═╤═╤═╤═╤═╦┄
		 *  │1│1│1│0│-│?│?│b│
		 * ┄╩═╧═╧═╩═╧═╧═╧═╧═╩┄
		 *
		 * b: Battery     1 = Battery data present in event [1 Byte] possibly followed by an another event or light data
		 */

		bool flagBatteryStatus = HAS_FLAG(flags, 0x1);
		bool flagUnknown = ((flags & 0x6) != 0);

		// flagUnknown = true;

		if (flagUnknown) {
			SV_WARN("Unknown status event 0x%02hX [Time:%04hX] [Payload: %s] <<ABORT FURTHER READ>>", *(payloadPtr - 1),
					time, packetToHex(payloadPtr, payloadEndPtr));
			// Since we don't know how much data this should consume, proceeding to IMU/Light decode is likely
			// to choke.
			*readPtr = payloadPtr;
			return false;
		}

		if (flagBatteryStatus) {
			// Battery Status
			// Happens On USB plugged in. Switch to wired mode?
			uint8_t batStatus = POP_BYTE(payloadPtr);
			handle_battery(w, batStatus);
#ifdef KNUCKLES_INFO
			SV_INFO("KAS: @%04hX | Status  [Battery: % 3i%%] [%s]", time, percent,
					(charging ? "CHARGING" : "ON BATTERY"));
#endif
			// Maybe read another event, IMU data or Light Data
			bool rtn = read_event(w, time_in_us, time, &payloadPtr, payloadEndPtr);
			*readPtr = payloadPtr;
			return rtn;
		}
	}

	// Read off any IMU data present
	if (flagIMU)
		read_imu_data(w, time_in_us, time, &payloadPtr, payloadEndPtr);

	UPDATE_PTR_AND_RETURN
}

static inline void parse_and_process_lightcap(SurviveObject *w, uint16_t time, uint8_t *payloadPtr,
											  uint8_t *payloadEndPtr) {
	LightcapElement les[10] = {0};
	uint8_t *payloadPtrStart = payloadPtr;
	int32_t cnt = read_light_data(w, time, &payloadPtr, payloadEndPtr, les, 10);
	SurviveContext *ctx = w->ctx;
#ifndef NDEBUG
	for (int i = (int)cnt - 1; i >= 0; i--) {
		uint8_t sensor = survive_map_sensor_id(w, les[i].sensor_id);
		if (sensor == 255) {
			cnt = -255;
			break;
		}
	}
#endif

	if (cnt < 0) {
		SV_WARN("Read light data error %d   [Time:%04hX] [Payload: %s]", (int)cnt, time,
				packetToHex(payloadPtr, payloadEndPtr));
		SV_WARN("Full payload: %s", packetToHex(payloadPtrStart, payloadEndPtr));
	} else {

#ifdef VERIFY_LIGHTCAP
		LightcapElement les_old[10] = {0};
		int les_old_cnt = parse_watchman_lightcap(w->ctx, w->codename, time >> 8, w->activations.last_imu, payloadPtr,
												  payloadEndPtr - payloadPtr, les, 10);

		assert(cnt == les_old_cnt);
#endif
		for (int i = (int)cnt - 1; i >= 0; i--) {
#ifdef DEBUG_WATCHMAN
			printf("%d: %u [%u]\n", les[i].sensor_id, les[i].length, les[i].timestamp);
#endif
#ifdef VERIFY_LIGHTCAP
			assert(memcmp(&les[i], &les_old[i], sizeof(LightcapElement)) == 0);
#endif
			handle_lightcap(w, &les[i]);
		}
	}
}

static int parse_and_process_raw1_lightcap(SurviveObject *obj, uint16_t time, uint8_t *packet, uint8_t length) {
	bool has_errors = false;
	uint8_t idx = 0;
	uint8_t channel = 255;
	SurviveContext *ctx = obj->ctx;
	bool dump_binary = false;

	while (idx < length) {
		uint8_t data = packet[idx];

		if (data & 0x1u) {
			// Since they flag for this; I assume multiples can appear in a single packet. Need to plug in
			// second LH to find out...

			if ((data & 0x0Au) != 0) {
				static int unknown_count = 0;
				if(unknown_count++ < 10) {
					// Currently I've only ever seen 0x1 if the 1 bit is set; I doubt they left 3 bits on the table
					// though....
					SV_WARN("Not entirely sure what this data is; errors may occur (%d, 0x%02x)\n", idx, data);
					dump_binary = true;
				}
				// has_errors = true;
				goto exit_loop;
			}

			// encodes like so: 0bcccc ?F?C
			bool hasConflict = data & 0x04u;
			if (hasConflict) {
				uint8_t conflicted_channel = data >> 4u;
				SV_WARN("Two or more lighthouses are on channel %d; tracking is most likely going to fail.",
						conflicted_channel);
			}
			channel = data >> 4u;
			SV_VERBOSE(750, "%s Channel %d (0x%02x)", obj->codename, channel, data);
			idx++;
		} else {
			uint32_t timecode = 0;
			memcpy(&timecode, packet + idx, sizeof(uint32_t));

			uint32_t reference_time = (obj->activations.last_imu);

			bool sync = timecode & 0x2u;
			if (!sync) {
				//                          O                               SC
				//                         GO                               YH
				//                         ET                               NA
				//                    [??] NX[    24 bit time @ 48mhz      ]CN
				// encodes like so: 0bXXXX ABTTT TTTT TTTT TTTT TTTT TTTT TTSC
				bool ootx = (timecode >> 26u) & 1u;
				bool g = (timecode >> 27u) & 1u;
				uint32_t time24 = (timecode >> 2u) & 0xFFFFFFu;
				timecode = fix_time24(ctx, time24, reference_time);
				uint8_t unused = timecode >> 28;
				if (unused && dump_binary) {
					SV_WARN("Not sure what this is: %x", unused);
				}
				SV_VERBOSE(750, "Sync %s %02d %d %8u", obj->codename, channel, ootx, timecode);
				if (channel == 255) {
					SV_WARN("No channel specified for sync");
					dump_binary = true;
					// has_errors = true;
				} else {
					SURVIVE_INVOKE_HOOK_SO(sync, obj, channel, timecode, ootx, g);
				}
			} else {
				//                                                         SC
				//                                                         YH
				//                                                         NA
				//                    [SNSR][    25 bit time @ 96mhz      ]CN
				// encodes like so: 0bSSSS STTT TTTT TTTT TTTT TTTT TTTT TFSC

				// Since nothing in libsurvive thinks anything is 96mhz; pass in a flag
				bool half_clock_flag = timecode & 0x4u;
				uint8_t sensor = (timecode >> 27u);
				timecode = fix_time24(ctx, (timecode >> 3u) & 0xFFFFFFu, reference_time);
				SV_VERBOSE(750, "Sweep %s %02d.%02d %8u", obj->codename, channel, sensor, timecode);
				if (channel == 255) {
					SV_WARN("No channel specified for sweep");
					dump_binary = true;
					// has_errors = true;
				} else {
					SURVIVE_INVOKE_HOOK_SO(sweep, obj, channel, survive_map_sensor_id(obj, sensor), timecode,
										   half_clock_flag);
				}
			}

			if (channel == 255) {
				dump_binary = true;
				has_errors = true;
				goto exit_loop;
			}
			idx += 4;
		}
	}

exit_loop:

	if (dump_binary) {
		for (int i = 0; i < length; i++) {
			if ((i + 2) % 4 == 0)
				SURVIVE_INVOKE_HOOK(printf, ctx, "  ");
			SURVIVE_INVOKE_HOOK(printf, ctx, "%02x ", packet[i]);
		}

		SURVIVE_INVOKE_HOOK(printf, ctx, "\n");
	}

	return has_errors ? -1 : idx;
}

#define VERIFY_LENGTH_OR_FAIL(payloadPtr, len)                                                                         \
	if (payloadEndPtr - payloadPtr < (len)) {                                                                          \
		SV_WARN("%s handle_input needed %d bytes but had %u", w->codename, (len),                                      \
				(uint32_t)(payloadEndPtr - payloadPtr));                                                               \
		goto exit_failure;                                                                                             \
	}

static bool handle_input(SurviveObject *w, uint8_t flags, uint8_t **payloadPtr, uint8_t *payloadEndPtr) {
	/*
	 * Flags for input events are as follows:
	 *
	 * ┄╦═╤═╤═╦═╤═╤═╤═╤═╦┄
	 *  │1│1│1│1│-│t│m│b│
	 * ┄╩═╧═╧═╩═╧═╧═╧═╧═╩┄
	 *
	 * t: Trigger    1 = Trigger data present in event [1 Byte]  ╮
	 * m: Motion     1 = Motion data present in event [4 Byte]   ├ If all 0, this is a gen 2 event [See below]
	 * b: Button     1 = Button data present in event [1 Byte]   ╯
	 *
	 * Order of data in payload is as follows:
	 * ┄╦═══════════════╦═══════════════╦═══════════════╦═══════════════╦
	 *  ║ [Button/b]    ║ [Trigger/t]   ║ [Motion/t]    ║ [IMU Data/I]  ║
	 * ┄╩═══════════════╩═══════════════╩═══════════════╩═══════════════╩
	 */
	struct SurviveContext *ctx = w->ctx;
	bool firstGen = ((flags & 0x7) != 0);
	buttonEvent bEvent = { 0 };
	uint8_t *payloadStart = *payloadPtr;

	if (firstGen) {
		bool flagUnknown = HAS_FLAG(flags, 0x8);
		bool flagTrigger = HAS_FLAG(flags, 0x4);
		bool flagMotion = HAS_FLAG(flags, 0x2);
		bool flagButton = HAS_FLAG(flags, 0x1);

		if (flagButton) {
			bEvent.pressedButtonsValid = 1;
			VERIFY_LENGTH_OR_FAIL(*payloadPtr, 1);
			bEvent.pressedButtons = POP_BYTE(*payloadPtr);
		}

		if (flagTrigger) {
			bEvent.triggerHighResValid = 1;
			VERIFY_LENGTH_OR_FAIL(*payloadPtr, 1);
			bEvent.triggerHighRes = POP_BYTE(*payloadPtr) / 255.;
		}

		if (flagMotion) {
			bEvent.touchpadHorizontalValid = 1;
			bEvent.touchpadVerticalValid = 1;

			VERIFY_LENGTH_OR_FAIL(*payloadPtr, 4);
			bEvent.touchpadHorizontal = (int16_t)POP_SHORT(*payloadPtr) / 32768.;
			bEvent.touchpadVertical = (int16_t)POP_SHORT(*payloadPtr) / 32768.;
		}

		if (flagUnknown) {
			SV_VERBOSE(150, "Unknown flag in handle_inut");
		}
		SV_VERBOSE(150, "handle_input flags %d %d %d", flagButton, flagTrigger, flagMotion);
	} else {
		// Second gen event (Eg Knuckles proximity)
		uint8_t genTwoType =
			POP_BYTE(*payloadPtr); // May be flags, but currently only observed to be 'a1' when knuckles

		if (genTwoType == 0xA1) {
			// Knuckles

			// Resistive contact sensors in buttons?
			VERIFY_LENGTH_OR_FAIL(*payloadPtr, 7);
			uint8_t touchFlags = POP_BYTE(*payloadPtr);
			// 0x01 = Trigger
			// 0x08 = Menu
			// 0x10 = Button A
			// 0x20 = Button B
			// 0x40 = Thumbstick

			bEvent.touchedButtonsValid = 1;
			bEvent.touchedButtons = (touchFlags & ~0x40u);
			bEvent.touchedButtons |= ((touchFlags & 0x40u) >> 4);

			bEvent.proximityValid = 1;

			// Non-touching proximity to fingers
			bEvent.proximity[0] = POP_BYTE(*payloadPtr) / 255.; // Middle finger
			bEvent.proximity[1] = POP_BYTE(*payloadPtr) / 255.; // Ring finger
			bEvent.proximity[2] = POP_BYTE(*payloadPtr) / 255.; // Pinky finger
			bEvent.proximity[3] = POP_BYTE(*payloadPtr) / 255.; // Index finger (trigger)

			// Contact force (Squeeze strength)
			bEvent.proximity[4] = POP_BYTE(*payloadPtr) / 255.;
			// Trackpad force
			bEvent.proximity[5] = POP_BYTE(*payloadPtr) / 255.;
		} else {
			SV_WARN("Unknown gen two event 0x%02hX 0b%s [Payload: %s] <<ABORT FURTHER READ>>", *(*payloadPtr - 1),
					byteToBin(*(*payloadPtr - 1)), packetToHex(*payloadPtr, payloadEndPtr));
			// Since we don't know how much data this should consume, proceeding to IMU/Light decode is likely
			// to choke.
			return false;
		}
	}

	registerButtonEvent(w, &bEvent, BUTTON_EVENT_SOURCE_RF);
	return true;

exit_failure:
	return false;
}

static void handle_watchman_v2(SurviveObject *w, uint64_t time_in_us, uint16_t time, uint8_t *payloadPtr,
							   uint8_t *payloadEndPtr) {
	struct SurviveContext *ctx = w->ctx;
	const uint8_t *originPayloadPtr = payloadPtr;
	struct SurviveUSBInfo *driverInfo = w->driver;
	if (driverInfo->nextCfgSubmitTime >= 0) {
		return;
	}

	if (w->ctx->lh_version == 1 && driverInfo->lightcapMode != LightcapMode_raw1) {
		vive_switch_mode(driverInfo, LightcapMode_raw1);
		return;
	}

	uint8_t flags = POP_BYTE(payloadPtr);
	bool has_errors = false;

	// Some kind of startup heartbeat?
	if (flags == 0xe2) {
		SV_VERBOSE(500, "Heartbeat(?) packet %s: '%s'", w->codename, packetToHex(payloadPtr, payloadEndPtr));
		return;
	}

	// Info: Watchman v2(KN1): '34 f2 3f 9f a2 c9             | 61 06 8c 0a 44 '
	// Info: Watchman v2(KN1): '34 f2 7a 9e fe c8             | 61 b4 6e 27 02 7e 95 7c 7c d6 b3 7c 5c a6 da 7c bc '
	// Info: Watchman v2(KN1): '38 f0 a1 00 2a 00 00 1c 00 01 | 61 32 9d 62 15 2e d8 62 0d 62 26 63 45 '
	// Info: Watchman v2(KN1): '31 f4 ff                      | 61 de 25 d1 62 '

	// Touch (not press) A
	// Info: Watchman v2(KN1): '20 f0 a1 20 00 01 00 00 00 00 '

	// Pulling trigger:
	// Info: Watchman v2(KN1): '20 f4 39 '
	// Info: Watchman v2(KN1): '20 f0 a1 00 bb f5 ff e5 07 00 '
	// Info: Watchman v2(KN1): '20 f0 a1 00 00 03 00 00 00 00 '

	// Info: Watchman v2(KN1): '31 f1 00                      | 61 42 a4 ef 80 ee b4 ef c0 '
	// Info: Watchman v2(KN1): '31 f4 0c 61 44 56 97 09 46 88 5b b3 8a a1 5b 1b 9a e4 5b 3b 76 f0 5b bb '

	// Hitting A:
	// Info: Watchman v2(KN1): '31 f4 f8                      | 61 ca 25 9f 7f 26 49 9f 5f 06 c5 9f bf f6 db 9f 17 8a e8
	// 9f 0f '

	// Hitting B
	// Info: Watchman v2(KN1): '31 f4 12                      | 61 68 62 56 01 '

	// Info: Watchman v2(KN1): '35 f6 5b ef e5 6d 3e          | 61 cc 3d de 00 c6 51 ea 59 72 7b ea b9 9a 81 ea 11 '
	// Info: Watchman v2(KN1): '35 f6 2d 36 ec 07 1e          | 61 da 56 93 c0 02 aa 93 60 '
	// Info: Watchman v2(KN1): '35 f6 15 b6 25 98 1b          | 61 20 c4 fa 03 fa 89 23 78 f6 c9 23 58 3a 2d 24 b8 '

	// Possibly this means the protocol is done switching:
	// Info: Watchman v2(KN1): '51 20 01                      | 61 50 d2 57 01 '
	// Info: Watchman v2(KN1): '36 f7 02 18 12 e1 b8 06       | 61 74 7e bf 0b 26 b3 ac 7f 76 d2 ac 5f e2 00 ad 17 '

	// Info: Watchman v2(KN1): '80 a5 4c fb 8f f1 f2 f9 d3 ff 2c 00 df ff '
	// Info: Watchman v2(KN1): 'c0 92 52 fb 7d f1 ea f9 ce ff 2f 00 dc ff 20 01 '

	// bool has_errors = !read_event(w, time, &payloadPtr, payloadEndPtr);

	bool flagUnknown01 = HAS_FLAG(flags, 0x01);
	bool flagUnknown02 = HAS_FLAG(flags, 0x02);
	// Haptic on the trackpad thing on the knuckles?
	bool flagUnknown04 = HAS_FLAG(flags, 0x04);
	bool flagUnknown08 = HAS_FLAG(flags, 0x08);

	bool flagLightcap = HAS_FLAG(flags, 0x10);
	bool flagInput = HAS_FLAG(flags, 0x20);
	bool flagMetaData = HAS_FLAG(flags, 0x40);
	bool flagIMU = HAS_FLAG(flags, 0x80);

	if (HAS_FLAG(flags, ~0xD1)) {
		SV_VERBOSE(100, "%s Unknown flag %02x", w->codename, flags);
	}

	if (flagIMU)
		read_imu_data(w, time_in_us, time, &payloadPtr, payloadEndPtr);

	// These things happen every 10 seconds
	if (flagMetaData) {
		uint8_t marker_byte = POP_BYTE(payloadPtr);

		switch (marker_byte) {
		case 0x80: {
			uint8_t battery_byte = POP_BYTE(payloadPtr);
			handle_battery(w, battery_byte);
			break;
		}
		case 0x20: {
			// Mode?
			uint8_t unknown_byte = POP_BYTE(payloadPtr);
			break;
		}
		case 0x30: {
			// Only have seen zeros here
			uint8_t unknown_byte1 = POP_BYTE(payloadPtr);
			uint8_t unknown_byte2 = POP_BYTE(payloadPtr);
			break;
		}
		default:
			SV_VERBOSE(100, "%.7f %s Unknown metadata marker in v2 (%02x %02x) bytes dropping rest of data %s",
					   survive_run_time(ctx), w->codename, flags, marker_byte,
					   packetToHex(originPayloadPtr, payloadEndPtr));
			return;
		}
	} else {
		SV_VERBOSE(700, "%.7f %s ref flag 0x40(0x%x) bytes rest of data %s", survive_run_time(ctx), w->codename, flags,
				   packetToHex(originPayloadPtr, payloadEndPtr));
	}

	if (flagInput) {
		VERIFY_LENGTH_OR_FAIL(payloadPtr, 1);
		uint8_t input_flags = POP_BYTE(payloadPtr);
		has_errors = !handle_input(w, input_flags, &payloadPtr, payloadEndPtr);
	}

	if (driverInfo->packetsSeenWaitingForV2 > 200) {
		driverInfo->packetsSeenWaitingForV2 = 0;
		driverInfo->timeWithoutFlag = 0;
	}

	if (driverInfo->timeWithoutFlag > 0 && driverInfo->timeWithoutFlag < 20) {
		driverInfo->packetsSeenWaitingForV2++;
		if (flagLightcap) {
			driverInfo->timeWithoutFlag = 1;
			flagLightcap = false;

			SV_VERBOSE(200, "Discard %s %lu: '%s'", w->codename, driverInfo->timeWithoutFlag,
					   packetToHex(payloadPtr, payloadEndPtr));
		} else {
			driverInfo->timeWithoutFlag++;
		}
	}

	if (flagLightcap && !has_errors) {
		if (driverInfo->lightcapMode == LightcapMode_raw0) {
			parse_and_process_lightcap(w, time, payloadPtr, payloadEndPtr);
		} else {
			int read = parse_and_process_raw1_lightcap(w, time, payloadPtr, payloadEndPtr - payloadPtr);
			if (read == -1)
				has_errors = true;
			else
				payloadPtr += read;
		}
	}

	if (driverInfo->timeWithoutFlag == 20) {
		if (payloadPtr != payloadEndPtr) {
			has_errors = true;
			SV_WARN("Did not read full input packet; %ld bytes remain", payloadEndPtr - payloadPtr);
		}
	}

	if (!has_errors) {
		return;
	}

exit_failure:
	survive_dump_buffer(ctx, originPayloadPtr, payloadEndPtr - originPayloadPtr);
}

static bool use_watchman_v2(SurviveObject *w) {
	if (w->ctx->lh_version == 1) {
		return true;
	}

	switch (w->object_subtype) {
	case SURVIVE_OBJECT_SUBTYPE_TRACKER:
	case SURVIVE_OBJECT_SUBTYPE_WAND:
		return false;
	default:
		return true;
	}
}

void survive_handle_watchman(SurviveObject *w, uint64_t time_in_us, uint8_t *readdata) {
	// KASPER'S DECODE
	SurviveContext *ctx = w ? w->ctx : 0;

	struct SurviveUSBInfo *driver = w->driver;
	if (driver->ignoreCnt) {
		driver->ignoreCnt--;
		return;
	}
	/*
	 * ---=== PACKET STRUCTURE ===---
	 * Key:
	 *  [Optional] - Element may or may not be present depending on flags
	 *  1 = Bit is set
	 *  0 = Bit is not set
	 *  - = Bit is "don't care"
	 *  ? = Bit meaning is unknown
	 *  ║ = Byte boundary
	 *  │ = Bit boundary
	 *
	 *
	 *                                 ┊<------- Size is the length of this section in bytes ------->┊
	 *                                 ┊                                                             ┊
	 * ╔═══════════════╦═══════════════╬═══════════════╦┄┄┄                                          ┊
	 * ║Time MSB       ║Size           ║Time LSB       ║                                             ┊
	 * ╚═══════════════╩═══════════════╩═══════════════╩┄┄┄                                          ┊
	 *                                                 ┊                                             ┊
	 *                                              ┄┄┄╬═╤═╤═╦═╤═╤═╤═╤═╦═┄┄┄┄┄┄┄┄┄┄┄═╦═┄┄┄┄┄┄┄┄┄┄┄┄══╣
	 *  For Non-Light events (First three bits = 111)  ║1│1│1│  FLAGS  ║ [Payload]   ║ [Light Data]  ║
	 *                                              ┄┄┄╬═╧═╧═╩═╧═╧═╧═╧═╩═┄┄┄┄┄┄┄┄┄┄┄═╩═┄┄┄┄┄┄┄┄┄┄┄┄══╣
	 *                                                 ┊                                             ┊
	 *                                              ┄┄┄╬═══════════════════┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄══╣
	 *  For light-only events (First three bits != 111)║ Light Data                                  ║
	 *                                              ┄┄┄╩═══════════════════┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄══╝
	 *
	 * Since bits 7&8 are part of the input event or the sensor id, this places limits on the available id space thus :
	 *   The first byte of the light data is sensor data, of which the upper 5 bits are the sensor id, and given
	 *   bits 7&8 cannot both be 1, results in the range 00000XXX to 10111XXX yielding a maximum of 24 light
	 *   elements per device (In the current format).
	 *
	 * Packet Decode Process:
	 *   1) Read off Time (MSB and LSB) and Payload Size
	 *   2) If bits 6-8 of the byte 4 are set:
	 *       a) Read the input/status event
	 *       b) Compute the remaining bytes after the input/status event has been read (The size of the input/status
	 *          event is not explicitly provided, so must be implied by the event type.
	 *   3) Read an interpret any remaining bytes as light data
	 *
	 *   Watchman V2:
	 *
	 *   Watchman protocol V2 has the same time / size breakdown as V1 but is followed with a flags byte:
	 *   0bIMPL xxxx
	 *     ||||
	 *     ||||- Has Light data
	 *     |||- Has Input
	 *     ||- Has Metadata
	 *     |- Has IMU data
	 *
	 */

	uint16_t time = AS_SHORT(readdata[0], readdata[2]);
	uint8_t payloadSize = readdata[1] - 1;
	uint8_t *payloadPtr = &readdata[3];
	uint8_t *payloadEndPtr = payloadPtr + payloadSize;

	if (use_watchman_v2(w)) {
		SV_VERBOSE(750, "Watchman v2(%s): '%s'", w->codename, packetToHex(readdata, payloadEndPtr));
		handle_watchman_v2(w, time_in_us, time, payloadPtr, payloadEndPtr);
		return;
	}

	SV_VERBOSE(750, "Watchman v1(%s): %s", w->codename, packetToHex(payloadPtr, payloadEndPtr));
	/*
	if (w->ctx->lh_version == -1) {
		attempt_lh_detection(w, payloadPtr, payloadEndPtr);
		return;
	}
	 */
	if (driver->lightcapMode != LightcapMode_raw0) {
		return;
	}

	// Read any non-light events that may be in the packet
	if (!read_event(w, time_in_us, time, &payloadPtr, payloadEndPtr)) {
		SV_WARN("Read event failed; full payload: %s", packetToHex(&readdata[3], payloadEndPtr));
		return;
	}

	// Any remaining data after events (if any) have been read off is light data
	if (payloadPtr < payloadEndPtr) {
		LightcapElement les[10] = {0};
		int32_t cnt = read_light_data(w, time, &payloadPtr, payloadEndPtr, les, 10);

		if (cnt < 0) {
			SV_WARN("Read light data error %d   [Time:%04hX] [Payload: %s]", (int)cnt, time,
					packetToHex(payloadPtr, payloadEndPtr));

		} else {

#ifdef VERIFY_LIGHTCAP
			LightcapElement les_old[10] = {0};
			int les_old_cnt = parse_watchman_lightcap(w->ctx, w->codename, time >> 8, w->activations.last_imu,
													  payloadPtr, payloadEndPtr - payloadPtr, les, 10);

			assert(cnt == les_old_cnt);
#endif
			for (int i = (int)cnt - 1; i >= 0; i--) {
#ifdef DEBUG_WATCHMAN
				printf("%d: %u [%u]\n", les[i].sensor_id, les[i].length, les[i].timestamp);
#endif
#ifdef VERIFY_LIGHTCAP
				assert(memcmp(&les[i], &les_old[i], sizeof(LightcapElement)) == 0);
#endif
				handle_lightcap(w, &les[i]);
			}
		}
	}
}
#define DEBUG_WATCHMAN_PRINTF(...)                                                                                     \
	if (ctx && ctx->log_level > 500) {                                                                                 \
		SURVIVE_INVOKE_HOOK(printf, ctx, __VA_ARGS__);                                                                 \
	}

SURVIVE_EXPORT int parse_watchman_lightcap(struct SurviveContext *ctx, const char *codename, uint8_t time1,
										   survive_timecode reference_time, uint8_t *readdata, size_t qty,
										   LightcapElement *les, size_t output_cnt) {

	assert(qty > 0);
	uint8_t *mptr = readdata + qty - 3 - 1; //-3 for timecode, -1 to

	DEBUG_WATCHMAN_PRINTF("_%s lc Data: ", codename);
	for (int i = 0; i < qty; i++) {
		DEBUG_WATCHMAN_PRINTF("%02x ", readdata[i]);
	}
	DEBUG_WATCHMAN_PRINTF("\n");

	uint32_t mytime = (mptr[3] << 16) | (mptr[2] << 8) | (mptr[1] << 0);

	uint32_t times[20] = {0};
	int timecount = 0;
	int leds;
	int fault = 0;

	/// Handle uint32_tifying (making sure we keep it incrementing)
	mytime |= ((uint32_t)time1 << 24);

	// The general issue is that the 'time1' field isn't super in sync with the last 3 bytes we use for timing
	// light events -- it can tip a smidge before those bytes see it or sometimes after. This can cause
	// wild 1<<24 tick differences which break everything.
	//
	// We base it off IMU as a reference because light events can get blocked and it's not out of the ordinary
	// to not see them for a second or two. (1 << 23) on a 48mhz clock is ~150ms; and the IMU is consistently
	// much faster than that.
	if (mytime > reference_time && mytime - reference_time > (1 << 23)) {
		mytime -= (1 << 24);
	} else if (reference_time > mytime && reference_time - mytime > (1 << 23)) {
		mytime += (1 << 24);
	}

	times[timecount++] = mytime; //
	DEBUG_WATCHMAN_PRINTF("_%s Packet Start Time: %u\n", codename, mytime);

	// First, pull off the times, starting with the current time, then all the delta times going backwards.
	{
		while (mptr - readdata > (timecount >> 1)) {
			uint32_t time_delta = 0;

			DEBUG_WATCHMAN_PRINTF("%s\t", codename);

			// https://en.wikipedia.org/wiki/Variable-length_quantity
			uint8_t codebyte = 0;
			int codelength = 0;
			while ((codebyte & 0x80) == 0 && mptr >= readdata) {
				codebyte = *(mptr--);
				time_delta = (time_delta << 7) | (codebyte & 0x7f);
				codelength++;

				if (codelength > 5) {
					SV_WARN("Code word too long");
					fault = 7;
					goto end;
				}
				DEBUG_WATCHMAN_PRINTF("%02x ", codebyte);
			}
			times[timecount++] = (mytime -= time_delta);
			DEBUG_WATCHMAN_PRINTF(" newtime: %u (%u)\n", mytime, time_delta);
		}

		leds = timecount >> 1;
		// Check that the # of sensors at the beginning match the # of parameters we would expect.
		if (timecount & 1) {
			SV_WARN("Uneven time count -- %d %d", leds, timecount);
			fault = 1;
			goto end;
		} // Inordinal LED count
		if (leds != mptr - readdata + 1) {
			fault = 2;
			SV_WARN("Bad LED count in packet %d; should be %d", leds, (int)(mptr - readdata + 1));
			goto end;
		} // LED Count does not line up with parameters
	}

	size_t times_size = timecount;

	int lese = 0; // les's end

	// Second, go through all LEDs and extract the lightevent from them.
	{
		uint8_t *marked = alloca(timecount);
		memset(marked, 0, timecount);

		int i, parpl = 0;
		timecount--;
		int timepl = 0;

		// This works, but usually returns the values in reverse end-time order.
		for (i = 0; i < leds; i++) {
			uint8_t led = readdata[i] >> 3;
			int adv = readdata[i] & 0x07;

			while (marked[timepl])
				timepl++;

			DEBUG_WATCHMAN_PRINTF("TP %d   TC: %d : ", timepl, timecount);
			for (int i = 0; i < timecount; i++) {
				DEBUG_WATCHMAN_PRINTF("%d", marked[i]);
			}
			DEBUG_WATCHMAN_PRINTF("\n");

			if (timepl > timecount) {
				fault = 3;
				goto end;
			} // Ran off max of list.
			uint32_t endtime = times[timepl++];

			int end = timepl + adv;
			if (end > timecount) {
				SV_WARN("Lightfault 4: %d > %d", end, timecount);
				fault = 4;
				goto end;
			} // end referencing off list
			if (marked[end] > 0) {
				fault = 5;
				goto end;
			} // Already marked trying to be used.
			uint32_t starttime = times[end];
			marked[end] = 1;

			// Insert all lighting things into a sorted list.  This list will be
			// reverse sorted, but that is to minimize operations.  To read it
			// in sorted order simply read it back backwards.
			// Use insertion sort, since we should most of the time, be in order.
			assert(lese < output_cnt);
			LightcapElement *le = &les[lese++];
			le->sensor_id = led;

			if ((uint32_t)(endtime - starttime) > 65535) {
				fault = 6;
				goto end;
			} // Length of pulse dumb.
			le->length = endtime - starttime;
			le->timestamp = starttime;

			DEBUG_WATCHMAN_PRINTF("_%s Event: %d %u %u-%u\n", codename, led, le->length, endtime, starttime);

			int swap = lese - 2;
			while (swap >= 0 && les[swap].timestamp < les[swap + 1].timestamp) {
				LightcapElement l;
				memcpy(&l, &les[swap], sizeof(l));
				memcpy(&les[swap], &les[swap + 1], sizeof(l));
				memcpy(&les[swap + 1], &l, sizeof(l));
				swap--;
			}
		}
	}

	return lese;

end : {

	DEBUG_WATCHMAN_PRINTF("Light decoding fault: %d", fault);
	DEBUG_WATCHMAN_PRINTF("Info: _%s %u %u ", codename, time1, reference_time);
	for (int i = 0; i < qty; i++) {
		DEBUG_WATCHMAN_PRINTF("%02x ", readdata[i]);
	}
	DEBUG_WATCHMAN_PRINTF("\n");

	return -fault;
	}
	}

struct SurviveUSBInfo *survive_vive_register_driver(SurviveObject *so, uint16_t vid, uint16_t pid) {
	struct SurviveUSBInfo *d = calloc(1, sizeof(struct SurviveUSBInfo));
	so->driver = d;
	d->so = so;
	d->lightcapMode = LightcapMode_raw0;
	for (const struct DeviceInfo *info = KnownDeviceTypes; info->name; info++) {
		if (info == 0) {
			continue;
		}

		if (info->vid != vid || info->pid != pid) {
			continue;
		}

		d->device_info = info;
		break;
	}

	return d;
}

static inline uint16_t read_buffer16(uint8_t *readdata, int idx) {
	uint16_t rtn;
	memcpy(&rtn, readdata + idx, sizeof(uint16_t));
	return rtn;
}
static inline uint32_t read_buffer32(uint8_t *readdata, int idx) {
	uint32_t rtn;
	memcpy(&rtn, readdata + idx, sizeof(uint32_t));
	return rtn;
}

static uint32_t earliest_working_revision(uint32_t hw_id) { return 1462663157; }

static uint32_t latest_working_revision(uint32_t hw_id) { return 1632556731; }

static void parse_tracker_version_info(SurviveObject *so, const uint8_t *data, size_t size) {
	SurviveContext *ctx = so->ctx;

#pragma pack(push, 1)
	struct {
		uint32_t revision;
		uint32_t board_model;
		char fw_name[32];
		uint32_t hardware_id;
		uint32_t a;
		uint16_t fpga_major_version;
		uint8_t fpga_minor_version;
		uint8_t fpga_patch_version;
		uint8_t h;
		uint64_t k;
	} version_info;
#pragma pack(pop)
	char fw_name[33] = {0};
	memcpy(&version_info, data, sizeof(version_info));
	memcpy(fw_name, version_info.fw_name, 32);
	for (int i = 0; i < 32; i++) {
		fw_name[i] = 0x7f & fw_name[i];
	}

	SV_INFO("Device %s has watchman FW version %u and FPGA version %u/%u/%u; named '%31s'. Hardware id 0x%08x Board "
			"rev: %d (len %d)",
			survive_colorize(so->codename), version_info.revision, version_info.fpga_major_version,
			version_info.fpga_minor_version, version_info.fpga_patch_version, fw_name, version_info.hardware_id,
			version_info.board_model, (int)size);
	SV_VERBOSE(105, "Extra version info: 0x%x / 0x%x / 0x%lx", version_info.a, version_info.h, version_info.k);
	uint32_t earliest_version = earliest_working_revision(version_info.hardware_id);
	uint32_t latest_version = latest_working_revision(version_info.hardware_id);
	if (earliest_version > version_info.revision) {
		SV_WARN("The detected version for device %s is %d; the earliest that is verified to work is %d. You may want "
				"to upgrade. If this version seems to work, please create an issue at "
				"https://github.com/cntools/libsurvive/issues with this message so we can update the version list.",
				so->codename, version_info.revision, earliest_version)
	} else if (latest_version < version_info.revision) {
		SV_WARN("The detected version for device %s is %d; the latest that is verified to work is %d. You may have to "
				"upgrade libsurvive to support this device.",
				so->codename, version_info.revision, latest_version);
	}
}

static void parse_tracker_info(SurviveObject *so, uint8_t id, uint8_t *readdata, size_t size) {
	SurviveContext *ctx = so->ctx;
	switch (id) {
	case VIVE_REPORT_IMU_SCALES: {
		SV_INFO("IMU Scales 1: 0x%x 0x%x", readdata[0], readdata[1]);
		goto dump_data;
		break;
	}
	case VIVE_REPORT_CHANGE_MODE: {
		SV_INFO("Info 4: ")
		goto dump_data;
	}
	case VIVE_REPORT_VERSION_ALT:
	case VIVE_REPORT_VERSION: {
		parse_tracker_version_info(so, readdata, size);
		break;
	}
	case VIVE_REPORT_HMD_SET_READOFFSET_USERDATA: {
		SV_INFO("Info 8: 0x%x", readdata[0]);
		break;
	}
	// Start config stream
	case VIVE_REPORT_CONFIG_READMODE: {
		break;
	}
	// Request more config stream
	case VIVE_REPORT_CONFIG_READ: {
		break;
	}
	case VIVE_REPORT_COMMAND: {
		SV_INFO("Received cmd 0x%02x with %d bytes of data", readdata[0], readdata[1]);
		survive_dump_buffer(ctx, readdata + 2, readdata[1]);
		break;
	}
	default: {
		SV_INFO("Unknown field 0x%02x for %s", id, so->codename);
		goto dump_data;
	}
	}

	return;
dump_data:
	survive_dump_buffer(ctx, readdata, size);
}

void survive_data_cb_locked(uint64_t time_received_us, SurviveUSBInterface *si) {
	int size = si->actual_len;
	SurviveContext *ctx = si->ctx;
	int iface = si->which_interface_am_i;
	SurviveObject *obj = si->assoc_obj;
	uint8_t *readdata = si->buffer;
	uint8_t *enddata = readdata + size;

	if (obj == 0)
		return;

	int id = POP1;
	size--;

	if (!obj->driver) {
		struct SurviveUSBInfo *d = obj->driver = calloc(1, sizeof(struct SurviveUSBInfo));
		d->so = obj;
	}

	// We handle this here since it's one of the few things we use without a config loaded.
	if (si->which_interface_am_i == USB_IF_TRACKER_INFO) {
		parse_tracker_info(obj, id, readdata, size);
		return;
	}

	if (obj->conf == 0 || (si->usbInfo && si->usbInfo->cfg_user)) {
		if (si->usbInfo) {
				//si->usbInfo->tryConfigLoad = 1;

		}
		return;
	}

	switch (si->which_interface_am_i) {
	case USB_IF_HMD_HEADSET_INFO: {
		SurviveObject *headset = obj;
		readdata += 2;
		buttonEvent event = {0};

		event.touchedButtonsValid = 0;
		event.touchedButtons = POP1; // Lens
		event.rawAxisCnt = 3;

		event.rawAxis[1] = POP2; // Lens Separation
		readdata += 2;
		event.touchedButtons |= POP1; // Button
		readdata += 3;
		readdata++; // Proxchange, No change = 0, Decrease = 1, Increase = 2
		readdata++;
		event.rawAxis[2] = POP2; // Proximity  	<< how close to face are you?  Less than 80 = not on face.
		event.rawAxis[0] = POP2; // IPD   		<< what is this?
		headset->ison = 1;

		registerButtonEvent(headset, &event, BUTTON_EVENT_SOURCE_DEFAULT);

		break;
	}
	case USB_IF_HMD_IMU:
	case USB_IF_W_WATCHMAN1_IMU:
	case USB_IF_TRACKER0_IMU:
	case USB_IF_TRACKER1_IMU: {
		int i;
		// printf( "%d -> ", size );
		for (i = 0; i < 3 && readdata < enddata; i++) {
			struct unaligned_16_t *acceldata = (struct unaligned_16_t *)readdata;
			readdata += 12;
			uint32_t timecode = POP4;
			uint8_t code = POP1;
			// printf( "%d ", code );
			int8_t cd = code - obj->oldcode;

			if (cd > 0) {
				obj->oldcode = code;

				// XXX XXX BIG TODO!!! Actually recal gyro data.
				FLT agm[9] = {acceldata[0].v,
							  acceldata[1].v,
							  acceldata[2].v,
							  acceldata[3].v,
							  acceldata[4].v,
							  acceldata[5].v,
							  0,
							  0,
							  0};

				SV_VERBOSE(300, "%s %s %7.6f %7.6f %2u", survive_colorize(obj->codename),
						   survive_colorize("IMU"), survive_run_time(ctx), timecode / 48000000., code);

				// assert(timecode <= obj->timebase_hz);
				SURVIVE_INVOKE_HOOK_SO(raw_imu, obj, 3, agm, timecode, code);
				SurviveSensorActivations_register_runtime(&obj->activations, obj->activations.last_imu,
														  time_received_us);
			}
		}
		// DONE OK.
		break;
	}
	case USB_IF_WATCHMAN1:
	case USB_IF_WATCHMAN2: {
		SurviveObject *w = obj;
		if (id == VIVE_REPORT_RF_WATCHMAN) {
			survive_handle_watchman(w, time_received_us, readdata);
		} else if (id == VIVE_REPORT_RF_WATCHMANx2) {
			survive_handle_watchman(w, time_received_us, readdata);
			survive_handle_watchman(w, time_received_us, readdata + 29);
		} else if (id == VIVE_REPORT_RF_TURN_OFF) {
			w->ison = 0; // turning off
		} else if (id != 0) {
			SV_WARN("Unknown watchman code %d", id);
		}
		break;
	}
	case USB_IF_HMD_LIGHTCAP:
	case USB_IF_TRACKER1_LIGHTCAP: {

		bool dump_binary = false;
		if (id == VIVE_REPORT_USB_LIGHTCAP_REPORT_V1) { // LHv1
			int i;
			for (i = 0; i < 9 && readdata < enddata; i++) {
				assert((enddata - readdata) >= 7);
				LightcapElement le;
				le.sensor_id = POP1;
				le.length = POP2;
				le.timestamp = POP4;
				if (le.sensor_id > 0xfd)
					continue;
				SV_VERBOSE(300, "%s %s %7.6f %7.6f %2u %2u %5u %08x %4d", survive_colorize(obj->codename),
						   survive_colorize("LIGHTCAP"), survive_run_time(ctx), le.timestamp / 48000000., id,
						   le.sensor_id, le.length, le.timestamp, (int)(si->buffer + size - readdata));

				if (obj->ctx->lh_version != 1) {
					bool success = handle_lightcap(obj, &le);
					if (!success)
						dump_binary = 1;
				}

				if (dump_binary) {
					SV_VERBOSE(100, "%s sensor: %2d         time: %3.5f length: %4d end_time: %8u", obj->codename,
							   le.sensor_id, le.timestamp / 48000000., le.length, le.length + le.timestamp);
				}
			}
		} else if (id == VIVE_REPORT_USB_LIGHTCAP_REPORT_V2) { // LHv2
			if (obj->ctx->lh_version == 0) {
				return;
			}
			survive_notify_gen2(obj, "Report id 39");

			// Implies that the user forced gen1
			if (obj->ctx->lh_version != 1) {
				// Shouldn't see this if the user said to use gen1 -- dump the output.
				static bool force_gen_warning = false;
				if (!force_gen_warning) {
					SV_WARN("LH Gen is %d, dumping data", obj->ctx->lh_version);
					force_gen_warning = true;
				}

				dump_binary = true;
			} else {
				vive_switch_mode(obj->driver, LightcapMode_raw1);
			}

			if (dump_binary) {
#pragma pack(push, 1)
				struct lh2_entry {
					uint8_t code; // sensor with some bit flag. Continuation flag?
					uint32_t time;
					uint32_t data;
					uint32_t mask;
				};
#pragma pack(pop)
				uint32_t samples[4] = {0}, masks[4] = {0}, times[4] = {0};

				struct lh2_entry *entries = (struct lh2_entry *)readdata;
				static uint32_t last_time = 0;

				for (int i = 0; i < 4; i++) {
					struct lh2_entry *entry = &entries[i];
					if (entry->code == 0xff)
						break;
					fprintf(stderr, "sensor: %2u flag: %u time: %8u %3.5f (%7u) ", entry->code & 0x7fu,
							(entry->code & 0x80u) > 0, entry->time, entry->time / 48000000., entry->time - last_time);

					for (int k = 0; k < 32; k++) {
						uint8_t idx = 32 - k - 1;
						bool d = ((entry->data >> (idx)) & 1u);
						bool m = ((entry->mask >> (idx)) & 1u);
						if (m)
							fprintf(stderr, "%d", d);
						else
							fprintf(stderr, "_");
					}
					samples[i] = entry->data;
					masks[i] = entry->mask;
					times[i] = entry->time;
					last_time = entry->time;
					fprintf(stderr, "\n");
				}

				uint32_t time_since_sync[4] = {0};
				survive_channel chan = survive_decipher_channel(samples, masks, times, time_since_sync, 4);
				fprintf(stderr, "Chan ootx: %d %d\n", chan / 2, chan & 1);
				for (int i = 0; i < 4; i++) {
					fprintf(stderr, "time since sync: %8u time of last sync: %8u\n", time_since_sync[i],
							entries[i].time - time_since_sync[i] * 8);
				}

				for (int i = 59 - 7; i < 59; i++) {
					fprintf(stderr, "%02x ", readdata[i]);
				}
				fprintf(stderr, "\n");
			}
		} else if (id == VIVE_REPORT_USB_LIGHTCAP_REPORT_RAW_MODE_1) {
			survive_notify_gen2(obj, "Report ID 40");

			uint8_t *packet = readdata + 1;
			uint8_t length = readdata[0];

			parse_and_process_raw1_lightcap(obj, 0, packet, length);
		} else if (id == VIVE_REPORT_USB_TRACKER_LIGHTCAP_V1) {
			SV_INFO("USB lightcap report is of an unexpected type for %s: %d (0x%02x)", obj->codename, id, id);
		} else {
			SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "USB lightcap report is of an unknown type for %s: %d (0x%02x)",
					 obj->codename, id, id);
		}

		break;
	}
	case USB_IF_W_WATCHMAN1_LIGHTCAP:
	case USB_IF_TRACKER0_LIGHTCAP: {
		if (id == VIVE_REPORT_USB_TRACKER_LIGHTCAP_V1) {
			int i = 0;
			for (i = 0; i < 7; i++) {
				uint16_t *sensorId = (uint16_t *)readdata;
				uint16_t *length = (uint16_t *)(&(readdata[2]));
				unsigned long *time = (unsigned long *)(&(readdata[4]));
				LightcapElement le;
				le.sensor_id = (uint8_t)POP2;
				le.length = POP2;
				le.timestamp = POP4;
				if (le.sensor_id > 0xfd)
					continue; //
				handle_lightcap(obj, &le);
			}
		} else {
			SV_WARN("Unrecognized report id: 0x%02x for %s", id, obj->codename);
		}
		break;
	}
	case USB_IF_TRACKER0_BUTTONS:
	case USB_IF_TRACKER1_BUTTONS:
	case USB_IF_W_WATCHMAN1_BUTTONS: {
		if (VIVE_REPORT_IMU_SCALES == id) {
			// 0x00	uint8	1	reportID	HID report identifier(= 1)
			// 0x02	uint16	2	reportType(? )	0x0B04: Ping(every second) / 0x3C01 : User input
			// 0x04	uint32	4	reportCount	Counter that increases with every report
			// 0x08	uint32	4	pressedButtons	Bit field, see below for individual buttons
			// 0x0C	uint16	2	triggerOrBattery	Analog trigger value(user input) / Battery voltage ? (ping)
			// 0x0E	uint8	1	batteryCharge	Bit 7 : Charging / Bit 6..0 : Battery charge in percent
			// 0x10	uint32	4	hardwareID	Hardware ID(user input) / 0x00000000 (ping)
			// 0x14	int16	2	touchpadHorizontal	Horizontal thumb position(Left : -32768 / Right : 32767)
			// 0x16	int16	2	touchpadVertical	Vertical thumb position(Bottom : -32768 / Top : 32767)
			// 0x18 ? 2 ? unknown
			// 0x1A	uint16	2	triggerHighRes	Analog trigger value with higher resolution
			// 0x1C ? 24 ? unknown
			// 0x34	uint16	2	triggerRawMaybe	Analog trigger value, maybe raw sensor data
			// 0x36 ? 8 ? unknown
			// 0x3E	uint8	1	someBitFieldMaybe	0x00 : ping / 0x64 : user input
			// 0x3F ? 1 ? unknown

			// typedef struct
			//{
			//	//uint8_t reportId;
			//	uint16_t reportType;
			//	uint32_t reportCount;
			//	uint32_t pressedButtons;
			//	uint16_t  triggerOrBattery;
			//	uint8_t batteryCharge;
			//	uint32_t hardwareId;
			//	int16_t  touchpadHorizontal;
			//	int16_t  touchpadVertical;
			//	uint16_t unknown1;
			//	uint16_t triggerHighRes;
			//	uint8_t  unknown2;
			//	uint8_t  unknown3;
			//	uint8_t  unknown4;
			//	uint16_t triggerRaw;
			//	uint8_t  unknown5;
			//	uint8_t  unknown6; // maybe some bitfield?
			//	uint8_t  unknown7;
			//} usb_buttons_raw;

			// usb_buttons_raw *raw = (usb_buttons_raw*) readdata;
			uint16_t reportType = read_buffer16(readdata, 0);

			switch(reportType) {
			case 0x0400: {
				// Very infrequent; doesnt seem periodic.
				// 00 04 0b 68   00 00 00 00   00 00 00 3e   0f 93 00 00   00 00 00 00   00 00 00 00   00 00 00 00   00 00 00 00     |    ...h  ....  ...>  ....  ....  ....  ....  ....
				// 00 04 0b 17   00 00 00 00   00 00 00 3e   0f 93 00 00   00 00 00 00   00 00 00 00   00 00 00 00   00 00 00 00     |    ....  ....  ...>  ....  ....  ....  ....  ....
				uint8_t second_cnt = readdata[3];
				uint8_t battery_info = readdata[13];
				handle_battery(obj, battery_info);
				break;
			}
			case 0x0700: {
				// Not sure what this is exacty; comes in at ~20hz as an incrementing counter.
				uint8_t unsure = readdata[2]; // Was always 0x3c?
				uint8_t timer = readdata[3];
				SV_VERBOSE(2000, "%f    0x700 Report %2x %2x", OGRelativeTime(), unsure, timer);
				break;
			}
			case 0x0800: {
				// HMD Spews this at a frequent interval
				//                                      [ IPD][prox]              [F]
				// 00 08 16 00   00 00 00 00   00 00 00 45 2b 3d 00 00   00 00 00 00   01 00 00 96   01 00 00 00   00 00
				// 00 00
				uint16_t IPD = readdata[11] | (readdata[12] << 8u);
				int16_t proximity = readdata[13] | (readdata[14] << 8u);
				uint8_t onFace = readdata[19];

				buttonEvent evt = {0};
				evt.rawAxisCnt = 2;
				evt.rawAxis[0] = IPD / 65536.;
				evt.rawAxis[1] = proximity / 32768.;

				evt.pressedButtonsValid = 1;
				evt.pressedButtons = onFace;
				registerButtonEvent(obj, &evt, BUTTON_EVENT_SOURCE_DEFAULT);

				break;
			}
			case 0x100: {
					buttonEvent bEvent;
					memset(&bEvent, 0, sizeof(bEvent));

					bEvent.pressedButtonsValid = 1;
					bEvent.pressedButtons = read_buffer32(readdata, 0x7);
					bEvent.triggerHighResValid = 1;

					bEvent.triggerHighRes = read_buffer16(readdata, 0x19) / 32768.;
					bEvent.touchpadHorizontalValid = 1;
					bEvent.touchpadVerticalValid = 1;

					bEvent.touchpadHorizontal = (int16_t)read_buffer16(readdata, 0x13) / 32768.;
					bEvent.touchpadVertical = (int16_t)read_buffer16(readdata, 0x15) / 32768.;

					registerButtonEvent(obj, &bEvent, BUTTON_EVENT_SOURCE_DEFAULT);
				}
				default: {
					survive_dump_buffer(ctx, readdata, size);
				}
			}
		} else {
			SV_WARN("Unknown report id 0x%02x for %s", id, obj->codename);
		}
	} break;
	default: { SV_WARN("Unknown interface %d for %s", si->which_interface_am_i, obj->codename); }
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int survive_vive_close(SurviveContext *ctx, void *driver) {
	SurviveViveData *sv = driver;
#ifndef HIDAPI
	libusb_hotplug_deregister_callback(sv->usbctx, sv->callback_handle);
#endif
	for (int i = 0; i < sv->udev_cnt; i++) {
		survive_close_usb_device(sv->udev[i]);
	}
	while (sv->udev_cnt) {
#ifndef HIDAPI
		survive_release_ctx_lock(ctx);
		libusb_handle_events(sv->usbctx);
		survive_get_ctx_lock(ctx);
#endif
		for (int i = 0; i < sv->udev_cnt; i++) {
			struct SurviveUSBInfo *usbInfo = sv->udev[i];
			if (survive_handle_close_request_flag(usbInfo)) {
				i--;
			}
		}
	}
	survive_vive_usb_close(sv);
	free(sv);
	return 0;
}

int DriverRegHTCVive(SurviveContext *ctx) {
	SurviveViveData *sv = SV_CALLOC(sizeof(SurviveViveData));

	survive_attach_configi(ctx, SECONDS_PER_HZ_OUTPUT_TAG, &sv->seconds_per_hz_output);
	sv->requestPairing = survive_configi(ctx, PAIR_DEVICE_TAG, SC_GET, 0);

	if(sv->seconds_per_hz_output > 0) {
	  SV_INFO("Reporting usb hz in %d second intervals", sv->seconds_per_hz_output);
	}
	sv->ctx = ctx;

	// USB must happen last.
	if (survive_usb_init(sv)) {
		// TODO: Cleanup any libUSB stuff sitting around.
		SV_WARN("USB Init failed");
		goto fail_gracefully;
	}

	DeviceDriver dd = (DeviceDriver)GetDriver("DriverRegGatt");
	if (dd && (survive_config_is_set(ctx, "gatt") == 0 || survive_configi(ctx, "gatt", SC_GET, 0) == 1)) {
		SurviveDeviceDriverReturn r = dd(ctx);
		if (r < 0) {
			SV_WARN("GATT could not start error %d", r);
		}
	}

	bool hasHotplug = true;
#ifdef HIDAPI
	hasHotplug = false;
#endif

	if (sv->udev_cnt || hasHotplug) {
		survive_add_driver(ctx, sv, survive_vive_usb_poll, survive_vive_close);
	} else {
		SV_INFO("No USB devices detected");
		goto fail_gracefully;
	}

#ifdef HIDAPI
	/*
	// Tricky: use other interface for actual lightcap.  XXX THIS IS NOT YET RIGHT!!!
	if (sv->udev[USB_DEV_HMD_IMU_LHB] && AttachInterface(sv, hmd, USB_IF_HMD_LIGHTCAP, sv->udev[USB_DEV_HMD_IMU_LHB],
														 0x82, survive_data_cb, "Lightcap")) {
		return -12;
	}

	// This is a HACK!  But it works.  Need to investigate further
	sv->uiface[USB_DEV_TRACKER0_LIGHTCAP].actual_len = 64;
	sv->uiface[USB_DEV_TRACKER1_LIGHTCAP].actual_len = 64;
*/
#endif
	// Note: don't sleep for HTCVive, the handle_events call can block
	ctx->poll_min_time_ms = 0;

	return 0;
fail_gracefully:
	survive_vive_usb_close(sv);
	free(sv);
	return -1;
}

REGISTER_LINKTIME(DriverRegHTCVive)
