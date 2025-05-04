#include "survive.h"

#include "survive_recording.h"
#include <assert.h>
#include <os_generic.h>
#include <stdio.h>

uint8_t survive_map_sensor_id(SurviveObject *so, uint8_t reported_id) {
	if (so->channel_map) {
		assert(reported_id < 32);
		int ole = reported_id;
		uint8_t mapped_id = so->channel_map[ole];
		if (mapped_id >= so->sensor_ct) {
			SurviveContext *ctx = so->ctx;
			SV_VERBOSE(110, "Invalid sensor %d detected hit (%d)", mapped_id, ole);
			return -1;
		}
		return mapped_id;
	}
	return reported_id;
}

typedef struct disambiguate_version {
	size_t pulse_count;
	size_t total_count;
	uint32_t last_pulse_times[SENSORS_PER_OBJECT];
} disambiguate_version;

static disambiguate_version *disambiguate_version_init(disambiguate_version *self) { return self; }

static void remove_all_disambiguate_version(SurviveContext *ctx) {
	for (int i = 0; i < ctx->objs_ct; i++) {
		free(ctx->objs[i]->disambiguator_data);
		ctx->objs[i]->disambiguator_data = 0;
	}
}
bool handle_lightcap(SurviveObject *so, const LightcapElement *_le) {
	// Gen2 devices can trigger this on startup; but later packets should
	// reliably change to lh_version == 1. If we see 50+ lightcap packets
	// without these gen2 packets we can just call it for gen1.

	SurviveContext *ctx = so->ctx;

	// "assert(_le->length > 0);" sometimes breaks the whole program, so instead just return success = false and go on
	uint16_t le_length = _le->length;
	if (le_length == 0 ) {
		SV_WARN("LightcapElement has zero length [SensorId:%d] [Timestamp:%04hX]", _le->sensor_id, _le->timestamp)
		return false;
	}

	if (so->ctx->lh_version == -1) {
		disambiguate_version *dv = so->disambiguator_data;
		if (dv == 0) {
			so->disambiguator_data = dv = SV_NEW(disambiguate_version);
		}

		dv->total_count++;

		// If the device is close to the LH, it's very possible to see lengths in this range. To prevent false
		// positives while on gen2; we also check the timing -- it must see x correctly distanced 60/120hz pulses
		// of the right length to get flagged in. This should be pretty solid -- LH2 all operate at <55hz, so they
		// would have a hard time generating this signature.
		bool isOOTXPulseLength = _le->length >= 3000 && _le->length < 6500;
		if (isOOTXPulseLength) {
			bool firstPulse = dv->last_pulse_times[_le->sensor_id] == 0;

			uint32_t pulse_dist = _le->timestamp - dv->last_pulse_times[_le->sensor_id];
			dv->last_pulse_times[_le->sensor_id] = _le->timestamp;

			if (!firstPulse) {
				uint32_t dist_at_120hz = 400000;
				uint32_t dist_at_60hz = 2 * dist_at_120hz;
				bool matches_60hz_lh1 = (pulse_dist > dist_at_60hz * .95 && pulse_dist < dist_at_60hz * 1.05);
				bool matches_120hz_lh1 = (pulse_dist > dist_at_120hz * .95 && pulse_dist < dist_at_120hz * 1.05);
				if (matches_120hz_lh1 || matches_60hz_lh1) {
					if (dv->pulse_count++ > 10) {
						remove_all_disambiguate_version(ctx);
						survive_notify_gen1(so, "OOTX pulses detected");
						return true;
					}
				}
			}
		}

		if (dv->total_count > 500) {
			remove_all_disambiguate_version(ctx);
			survive_notify_gen2(so, "no OOTX pulses detected");
		}
		return true;
	}

	LightcapElement le = *_le;
	survive_recording_lightcap(so, &le);

	uint8_t original_sensor = le.sensor_id;
	le.sensor_id = survive_map_sensor_id(so, original_sensor);
	if (le.sensor_id == (uint8_t)-1) {
		return false;
	}
	SURVIVE_INVOKE_HOOK_SO(lightcap, so, &le);

	return true;
}
