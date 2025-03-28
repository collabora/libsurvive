#include "os_generic.h"
#include "survive.h"

#include "gattlib.h"
#include <os_generic.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/queue.h>
#include <sys/socket.h>
#include <unistd.h>

STATIC_CONFIG_ITEM(GATT_ENABLE, "gatt", 'i', "Enable GATT communication to basestation", 0)
STATIC_CONFIG_ITEM(GATT_POWER_DOWN, "gatt-sleep-at-exit", 'i', "Put basestations to sleep at exit", 0)

struct gatt_info;
struct connection_t {
	struct gatt_info *driver;
	og_thread_t thread;
	char *addr;
	char name[32];

	gattlib_connection_t *gconn;
	LIST_ENTRY(connection_t) entries;
};

typedef struct gatt_info {
	SurviveContext *ctx;
	gattlib_adapter_t *adapter;
	LIST_HEAD(listhead, connection_t) ble_connections;
	og_thread_t thread;

	struct connection_t *lhs[NUM_GEN2_LIGHTHOUSES];
} gatt_info;

const char *LH_SERVICE_UUID = "00001523-1212-efde-1523-785feabcd124";
const char *MODE_UUID = "00001524-1212-EFDE-1523-785FEABCD124";
const char *IDENTIFY_UUID = "00008421-1212-EFDE-1523-785FEABCD124";
const char *POWER_UUID = "00001525-1212-EFDE-1523-785FEABCD124";

static uuid_t *mode_uuid() {
	static uuid_t uuid = {0};
	if (uuid.type == 0) {
		gattlib_string_to_uuid(MODE_UUID, sizeof(MODE_UUID), &uuid);
	}
	return &uuid;
}

static uuid_t *power_uuid() {
	static uuid_t uuid = {0};
	if (uuid.type == 0) {
		gattlib_string_to_uuid(POWER_UUID, sizeof(POWER_UUID), &uuid);
	}
	return &uuid;
}

static uint32_t read_char_byte(gattlib_connection_t *gatt_connection, uuid_t *uuid) {
	void *buffer;
	size_t buffer_length;

	int success = gattlib_read_char_by_uuid(gatt_connection, uuid, &buffer, &buffer_length);
	if (success != 0 || buffer_length != 1) {
		goto exit_error;
	}
	uint32_t ret = *(uint8_t *)buffer;

	free(buffer);
	return ret;

exit_error:
	free(buffer);
	return 0xFFFFFFFF;
}

static void ble_connect_device_done(gattlib_adapter_t *adapter, const char *dst,
		gattlib_connection_t *gatt_connection, int error, void *user_data) {
	struct connection_t *connection = user_data;
	char *addr = connection->addr;
	SurviveContext *ctx = connection->driver->ctx;

	gattlib_primary_service_t *services = 0;
	int services_count = 0;
	char uuid_str[MAX_LEN_UUID_STR + 1] = {0};

	int ret = gattlib_discover_primary(gatt_connection, &services, &services_count);
	if (ret != 0) {
		SV_WARN("Fail to discover primary services for %s", addr);
		goto disconnect_exit;
	}

	bool has_lh_service = false;
	for (int i = 0; i < services_count; i++) {
		gattlib_uuid_to_string(&services[i].uuid, uuid_str, sizeof(uuid_str));
		if (strcmp(LH_SERVICE_UUID, uuid_str) == 0) {
			has_lh_service = true;
		}
	}
	free(services);

	if (has_lh_service == false)
		goto disconnect_exit;

	connection->gconn = gatt_connection;
	uint32_t mode = read_char_byte(gatt_connection, mode_uuid()) - 1;
	uint32_t power_mode = read_char_byte(gatt_connection, power_uuid());

	if (mode == 0xffffffff || power_mode == 0xffffffff) {
		SV_WARN("GATT: Could not get mode/power settings for %s", addr);
		goto disconnect_exit;
	}

	SV_INFO("GATT: Found base station %s(%s) with mode %d in power state %d", connection->name, addr, mode, power_mode);

	if (connection->driver->lhs[mode] != 0) {
		uint8_t newmode = 0;
		for (newmode = 0; newmode < NUM_GEN2_LIGHTHOUSES && connection->driver->lhs[newmode]; newmode++)
			;
		SV_WARN("GATT: Lighthouse conflict with %s; changing mode on %s to %d", connection->driver->lhs[mode]->addr,
				addr, newmode);
		mode = newmode;
		newmode++; // 1 based in lighthouse
		gattlib_write_char_by_uuid(gatt_connection, mode_uuid(), &newmode, sizeof(newmode));
	}
	connection->driver->lhs[mode] = connection;

	if (power_mode == 0) {
		SV_INFO("Turning on base station at %s", addr);
		uint8_t power_mode_enable = 1;
		gattlib_write_char_by_uuid(gatt_connection, power_uuid(), &power_mode_enable, sizeof(power_mode_enable));
	}

	return;

disconnect_exit:
	gattlib_disconnect(gatt_connection, false);
}

static void *ble_connect_device(void *arg) {
	struct connection_t *connection = arg;
	char *addr = connection->addr;
	SurviveContext *ctx = connection->driver->ctx;

	int ret = gattlib_connect(connection->driver->adapter, addr, 0, ble_connect_device_done, connection);
	if (ret != GATTLIB_SUCCESS) {
		SV_WARN("Fail to connect to the bluetooth device %s: %d", addr, ret);
	}

	return 0;
}

static void ble_discovered_device(gattlib_adapter_t *adapter, const char *addr, const char *name,
		void *user_data) {
	gatt_info *driver = (gatt_info *)user_data;
	SurviveContext *ctx = driver->ctx;

	if (name == 0 || strncmp(name, "LHB", strlen("LHB")) != 0)
		return;

	struct connection_t *connection = SV_MALLOC(sizeof(struct connection_t));
	connection->driver = driver;
	connection->addr = strdup(addr);
	strncpy(connection->name, name, sizeof(connection->name) - 1);
	connection->thread = OGCreateThread(ble_connect_device, "ble-connect", connection);

	LIST_INSERT_HEAD(&driver->ble_connections, connection, entries);
}

static void *gatt_thread(void *arg) {
	gatt_info *driver = (gatt_info *)arg;
	SurviveContext *ctx = driver->ctx;

	int ret = gattlib_adapter_open(NULL, &driver->adapter);

	if (ret) {
		SV_WARN("GATT: Could not open adapater");
		return (void *)-1;
	}

	ret = gattlib_adapter_scan_enable(driver->adapter, ble_discovered_device, 1, driver);

	if (ret) {
		SV_WARN("GATT: Failed to scan.");
		return (void *)-1;
	}

	return 0;
}
static int gatt_close(SurviveContext *ctx, void *arg) {
	gatt_info *driver = (gatt_info *)arg;

	OGJoinThread(driver->thread);
	gattlib_adapter_scan_disable(driver->adapter);

	if (survive_configi(ctx, "gatt-sleep-at-exit", SC_GET, 0) == 1) {
		for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
			if (driver->lhs[i]) {
				if (driver->lhs[i]->gconn) {
					uint8_t powerdown[] = {1, 0};
					SV_INFO("GATT: Powering down base station %s", driver->lhs[i]->name);
					gattlib_write_char_by_uuid(driver->lhs[i]->gconn, power_uuid(), &powerdown[0], 1);
					gattlib_write_char_by_uuid(driver->lhs[i]->gconn, power_uuid(), &powerdown[1], 1);
					gattlib_disconnect(driver->lhs[i]->gconn, true);
				}
			}
		}
	}

	while (driver->ble_connections.lh_first != NULL) {
		struct connection_t *connection = driver->ble_connections.lh_first;
		OGJoinThread(connection->thread);
		LIST_REMOVE(driver->ble_connections.lh_first, entries);
		free(connection->addr);
		free(connection);
	}

	gattlib_adapter_close(driver->adapter);
	free(driver);
	return 0;
}
int DriverRegGatt(SurviveContext *ctx) {
	gatt_info *driver = SV_CALLOC(sizeof(gatt_info));
	driver->ctx = ctx;

	LIST_INIT(&driver->ble_connections);

	driver->thread = OGCreateThread(gatt_thread, "gatt", driver);

	survive_add_driver(ctx, driver, NULL, gatt_close);
	return 0;
}
REGISTER_LINKTIME(DriverRegGatt)
