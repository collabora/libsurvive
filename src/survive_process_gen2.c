#include "ootx_decoder.h"
#include "survive.h"
#include "survive_config.h"
#include "survive_internal.h"
#include "survive_kalman_tracker.h"
#include "survive_recording.h"
#include <assert.h>
#include <math.h>
#include <survive.h>

static FLT freq_per_channel[NUM_GEN2_LIGHTHOUSES] = {
	50.0521, 50.1567, 50.3673, 50.5796, 50.6864, 50.9014, 51.0096, 51.1182,

	51.2273, 51.6685, 52.2307, 52.6894, 52.9217, 53.2741, 53.7514, 54.1150,
};

static void ootx_bad_crc_clbk(ootx_decoder_context *ct, ootx_packet *pkt, uint32_t checksum) {
	SurviveContext *ctx = ((SurviveObject *)(ct->user))->ctx;
	int id = ct->user1;

	if (!ctx->bsd[id].OOTXSet)
		SV_VERBOSE(200, "(%d) Failed CRC", ctx->bsd[id].mode != 255 ? ctx->bsd[id].mode : id);
}
static void ootx_error_clbk_d(ootx_decoder_context *ct, const char *msg) {
	SurviveContext *ctx = ((SurviveObject *)(ct->user))->ctx;
	int id = ct->user1;

	if (!ctx->bsd[id].OOTXSet)
		SV_INFO("(%d) %s", ctx->bsd[id].mode != 255 ? ctx->bsd[id].mode : id, msg);
}

STATIC_CONFIG_ITEM(SERIALIZE_OOTX, "serialize-ootx", 'b', "Serialize out ootx", 0)
static void ootx_packet_clbk_d_gen2(ootx_decoder_context *ct, ootx_packet *packet) {
	SurviveContext *ctx = ((SurviveObject *)(ct->user))->ctx;
	int id = ct->user1;

	lighthouse_info_v15 v15;
	init_lighthouse_info_v15(&v15, packet->data);

	if (survive_configi(ctx, SERIALIZE_OOTX_TAG, SC_GET, 0) == 1) {
		char filename[128];
		snprintf(filename, 128, "LH%02d_%08x.ootx", v15.mode_current & 0x7F, (unsigned)v15.id);
		FILE *f = fopen(filename, "w");
		fwrite(packet->data, packet->length, 1, f);
		fclose(f);
	}

	BaseStationData *b = &ctx->bsd[id];
	b->OOTXChecked |= true;
	FLT accel[3] = {v15.accel_dir[0], v15.accel_dir[1], v15.accel_dir[2]};
	bool upChanged = norm3d(b->accel) != 0.0 && dist3d(b->accel, accel) > 1e-3;

	if (upChanged) {
		SV_VERBOSE(10, "OOTX up direction changed for %x (%f)", b->BaseStationID, norm3d(b->accel));
	}
	bool doSave = b->BaseStationID != v15.id || b->OOTXSet == false || upChanged;
	b->OOTXSet = 1;

	if (doSave) {
	  SV_INFO("Got OOTX packet %d %08x", ctx->bsd[id].mode, (unsigned)v15.id);

		b->BaseStationID = v15.id;

	    const char *whitelist = survive_configs(ctx, "use-lighthouses", SC_GET, "");
		SV_INFO("PRINT WHITELIST '%s' is whitelisted", whitelist);
	    if (whitelist && whitelist[0] != 0) {
	        char id_str[16] = {0};
	        sprintf(id_str, "%X", b->BaseStationID);
			SV_WARN("Checking lighthouse %s (idx %d) ", id_str, id);
	        if (strstr(whitelist, id_str) == NULL) {
	            SV_WARN("Lighthouse %s (idx %d) is not in the whitelist. Disabling.", id_str, id);
	            b->disable = 1;
	        } else {
				SV_INFO("Lighthouse %s (idx %d) discovered and in whitelist. Enabling.", id_str, id);
				b->disable = 0;
			}
	    }

		for (int i = 0; i < 2; i++) {
			b->fcal[i].phase = v15.fcal_phase[i];
			b->fcal[i].tilt = v15.fcal_tilt[i];
			b->fcal[i].curve = v15.fcal_curve[i];
			b->fcal[i].gibpha = v15.fcal_gibphase[i];
			b->fcal[i].gibmag = v15.fcal_gibmag[i];
			b->fcal[i].ogeephase = v15.fcal_ogeephase[i];
			b->fcal[i].ogeemag = v15.fcal_ogeemag[i];
		}

		for (int i = 0; i < 3; i++) {
			b->accel[i] = v15.accel_dir[i];
		}
		b->sys_unlock_count = v15.sys_unlock_count;

		// Although we know this already....
		b->mode = v15.mode_current & 0x7F;

		survive_reset_lighthouse_position(ctx, id);

		SURVIVE_INVOKE_HOOK(ootx_received, ctx, id);
	}
}

static void ootx_packet_cblk_d_gen1(ootx_decoder_context *ct, ootx_packet *packet) {

	SurviveContext *ctx = ((SurviveObject *)(ct->user))->ctx;
	int id = ct->user1;

	lighthouse_info_v6 v6;
	init_lighthouse_info_v6(&v6, packet->data);

	BaseStationData *b = &ctx->bsd[id];
	b->OOTXChecked = true;
	FLT accel[3] = {v6.accel_dir_x, v6.accel_dir_y, v6.accel_dir_z};
	bool upChanged = norm3d(b->accel) != 0.0 && dist3d(b->accel, accel) > 1e-3;

	bool doSave = b->BaseStationID != v6.id || b->OOTXSet == false || upChanged || b->mode != v6.mode_current;
	b->sys_unlock_count = v6.sys_unlock_count;
	b->OOTXSet = 1;

	if (doSave) {
		SV_VERBOSE(50, "Got OOTX packet %d %08x", ctx->bsd[id].mode, (unsigned)v6.id);

		b->BaseStationID = v6.id;
		b->fcal[0].phase = v6.fcal_0_phase;
		b->fcal[1].phase = v6.fcal_1_phase;
		b->fcal[0].tilt = tan(v6.fcal_0_tilt);
		b->fcal[1].tilt = tan(v6.fcal_1_tilt);
		b->fcal[0].curve = v6.fcal_0_curve;
		b->fcal[1].curve = v6.fcal_1_curve;
		b->fcal[0].gibpha = v6.fcal_0_gibphase;
		b->fcal[1].gibpha = v6.fcal_1_gibphase;
		b->fcal[0].gibmag = v6.fcal_0_gibmag;
		b->fcal[1].gibmag = v6.fcal_1_gibmag;
		b->accel[0] = v6.accel_dir_x;
		b->accel[1] = v6.accel_dir_y;
		b->accel[2] = v6.accel_dir_z;
		b->mode = v6.mode_current;

		survive_reset_lighthouse_position(ctx, id);

		SURVIVE_INVOKE_HOOK(ootx_received, ctx, id);
	}
}
void survive_ootx_dump_decoder_context(struct SurviveContext *ctx, int bsd_idx) {
	ootx_decoder_context *decoderContext = ctx->bsd[bsd_idx].ootx_data;
	if (decoderContext == 0)
		return;

	SV_VERBOSE(105, "OOTX stats for LH%d (mode: %d, %u)", bsd_idx, ctx->bsd[bsd_idx].mode, ctx->bsd[bsd_idx].BaseStationID);
	SV_VERBOSE(105, "\tBits seen:         %u (%d bytes)", decoderContext->stats.bits_seen,
			   decoderContext->stats.bits_seen / 8);
	SV_VERBOSE(105, "\tBad CRCs:          %u", decoderContext->stats.bad_crcs);
	SV_VERBOSE(105, "\tBad sync bits:     %u", decoderContext->stats.bad_sync_bits);
	SV_VERBOSE(105, "\tPackets found:     %u", decoderContext->stats.packets_found);
	SV_VERBOSE(105, "\tPayload size:      %u", decoderContext->stats.used_bytes);
	SV_VERBOSE(105, "\tPackage bits:      %u", decoderContext->stats.package_bits);
	SV_VERBOSE(105, "\tGuessed bits:      %u (%5.2f%%)", decoderContext->stats.guess_bits,
			   decoderContext->stats.guess_bits / (FLT)decoderContext->stats.package_bits * 100.);
	FLT d = survive_run_time(ctx) - decoderContext->stats.started_s;
	SV_VERBOSE(105, "\tTime:              %2.2f (%2.2fb/s, %2.2fb/s)", d, decoderContext->stats.bits_seen / d,
			   decoderContext->stats.used_bytes * 8 / d);
}
void survive_ootx_free_decoder_context(struct SurviveContext *ctx, int bsd_idx) {
	ootx_decoder_context *decoderContext = ctx->bsd[bsd_idx].ootx_data;
	ctx->bsd[bsd_idx].ootx_data = 0;
	if (decoderContext == 0)
		return;

	survive_ootx_dump_decoder_context(ctx, bsd_idx);
	ootx_free_decoder_context(decoderContext);
	free(decoderContext);
}
void survive_ootx_behavior(SurviveObject *so, int8_t bsd_idx, int8_t lh_version, int ootx) {
	struct SurviveContext *ctx = so->ctx;
	if (ctx->bsd[bsd_idx].OOTXChecked == false) {
		ootx_decoder_context *decoderContext = ctx->bsd[bsd_idx].ootx_data;

		if (decoderContext == 0) {
			if (lh_version == 1) {
				SV_INFO("OOTX not set for LH in channel %d; attaching ootx decoder using device %s",
						ctx->bsd[bsd_idx].mode, so->codename);
			} else {
				SV_INFO("OOTX not set for LH %d; attaching ootx decoder using device %s", bsd_idx, so->codename);
			}
			decoderContext = ctx->bsd[bsd_idx].ootx_data = SV_CALLOC(sizeof(ootx_decoder_context));
			ootx_init_decoder_context(decoderContext, survive_run_time(ctx));
			decoderContext->user1 = bsd_idx;
			decoderContext->user = so;
			decoderContext->ignore_sync_bit_error = survive_configi(ctx, "ootx-ignore-sync-error", SC_SETCONFIG, 0);
			decoderContext->ootx_packet_clbk = lh_version ? ootx_packet_clbk_d_gen2 : ootx_packet_cblk_d_gen1;
			decoderContext->ootx_error_clbk = ootx_error_clbk_d;
			decoderContext->ootx_bad_crc_clbk = ootx_bad_crc_clbk;
		}
		if (decoderContext->user == so) {
			ootx_pump_bit(decoderContext, ootx);

			if (ctx->bsd[bsd_idx].OOTXChecked) {
				ctx->bsd[bsd_idx].OOTXChecked = false;
				survive_ootx_dump_decoder_context(ctx, bsd_idx);
				// survive_ootx_free_decoder_context(ctx, bsd_idx);
			}
		}
	}
}

SURVIVE_EXPORT void survive_default_sync_process(SurviveObject *so, survive_channel channel, survive_timecode timecode,
												 bool ootx, bool gen) {
	struct SurviveContext *ctx = so->ctx;
	int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);
	if (bsd_idx == -1) {
		SV_WARN("Invalid channel requested(%d) for %s", channel, so->codename)
		return;
	}
	// Run OOTX FIRST. This allows a disabled LH to be identified and (if whitelisted) re-enabled
	survive_ootx_behavior(so, bsd_idx, ctx->lh_version, ootx);
	if (so->ctx->bsd[bsd_idx].disable)
		return;

	assert(channel <= NUM_GEN2_LIGHTHOUSES);

	survive_recording_sync_process(so, channel, timecode, ootx, gen);

	bool isIniting = so->last_sync_time[bsd_idx] == 0;

	if (!isIniting) {
		survive_timecode time_delta = survive_timecode_difference(timecode, so->last_sync_time[bsd_idx]);
		survive_timecode predicted_time = 48000000. / freq_per_channel[channel];
		int rotations_since = (time_delta + predicted_time / 2) / predicted_time;
		int skipped_syncs = rotations_since - 1;
		time_delta -= predicted_time * skipped_syncs;

		FLT hz = 48000000. / time_delta;
		FLT err = fabs(hz - freq_per_channel[channel]);

		SV_DATA_LOG("lh_freq_err[%d]", &err, 1, channel);

		if (err > 1) {
			if (skipped_syncs > 10) {
				so->last_sync_time[bsd_idx] = 0;
				so->stats.sync_resets[bsd_idx]++;
			}
			SV_VERBOSE(100, "Sync hz %2d: %8.6fhz (err: %0.6fhz) ootx: %d gen: %d time: %u count: %u", channel, hz, err,
					   ootx, gen, timecode, so->stats.syncs[bsd_idx]);
			so->stats.bad_syncs[bsd_idx] += skipped_syncs;
			so->sync_count[bsd_idx] += skipped_syncs;
			return;
		} else {
			SV_VERBOSE(400, "%s Sync   ch%2d       %6.3fhz (err: %0.6fhz) ootx: %d gen: %d time: %u count: %u",
					   survive_colorize(so->codename), channel, hz, err, ootx, gen, timecode, so->stats.syncs[bsd_idx]);
		}

		so->stats.skipped_syncs[bsd_idx] += skipped_syncs;
		// Every skipped sync halves our ootx success rate; but if we don't send anything in it will be wrong 100% of
		// the time
		for (int i = 0; i < skipped_syncs; i++) {
			survive_ootx_behavior(so, bsd_idx, ctx->lh_version, -1);
		}

		so->last_time_between_sync[bsd_idx] = time_delta;
	}

	so->stats.syncs[bsd_idx]++;
	so->sync_count[bsd_idx]++;
	so->last_sync_time[bsd_idx] = timecode;

	survive_ootx_behavior(so, bsd_idx, ctx->lh_version, ootx);

	PoserDataLightGen2 l = {.common = {
								.hdr =
									{
										.pt = POSERDATA_SYNC_GEN2,
										.timecode = SurviveSensorActivations_long_timecode_light(&so->activations, timecode),
									},
								.lh = bsd_idx,
							}};

	if (bsd_idx < ctx->activeLighthouses) {
		if (SurviveSensorActivations_add_gen2(&so->activations, &l) == false) {
			so->stats.rejected_data[bsd_idx]++;
		} else {
			so->stats.accepted_data[bsd_idx]++;
		}
	}

	so->stats.hit_from_lhs[bsd_idx]++;

	if (ctx->lh_version != -1) {
		survive_kalman_tracker_integrate_light(so->tracker, &l.common);
		SURVIVE_POSER_INVOKE(so, &l);
	}
}

static inline int8_t determine_plane(SurviveObject *so, int8_t bsd_idx, FLT angle) {
	static int naive_plane_only = -1;
	if (naive_plane_only == -1)
		naive_plane_only = survive_configi(so->ctx, "naive-plane-only", SC_GET, 0);
	int8_t naive_plane = angle > LINMATHPI;
	if (naive_plane_only != 0)
		return naive_plane;

	int8_t plane = naive_plane;
	FLT m = .05;
	bool borderLine = angle > LINMATHPI * (1 - 2 * m) && angle < LINMATHPI * (1 + 2 * m);
	bool veryBorderLine = angle > LINMATHPI * (1 - m) && angle < LINMATHPI * (1 + m);
	FLT angle_for_axis[2] = {angle - 2. / 3. * LINMATHPI, angle - 4. / 3. * LINMATHPI};

	if (borderLine) {
		if (veryBorderLine)
			plane = -1;

		FLT err[2] = {0};
		for (int i = 0; i < 2; i++) {
			err[i] = fabsf((float)(angle_for_axis[i] - so->activations.angles_center_x[bsd_idx][i]));
			if (isnan(err[i]))
				err[i] = 1.;
		}

		if (err[0] <= .1 || err[1] <= .1) {
			plane = err[1] < err[0];
		}

		if (plane != -1 && naive_plane != plane) {
			so->stats.extent_hits++;
			so->stats.min_extent = linmath_min(angle - LINMATHPI, so->stats.min_extent);
			so->stats.max_extent = linmath_max(angle - LINMATHPI, so->stats.max_extent);
		} else if (plane == -1) {
			so->stats.extent_misses++;
		} else if (plane == naive_plane) {
			so->stats.naive_hits++;
		}
	}

	return plane;
}
SURVIVE_EXPORT void survive_default_sweep_process(SurviveObject *so, survive_channel channel, int sensor_id,
												  survive_timecode timecode, bool half_clock_flag) {
	struct SurviveContext *ctx = so->ctx;

	int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);
	if (bsd_idx == -1) {
		SV_WARN("Invalid channel requested(%d) for %s", channel, so->codename)
		return;
	}

	if (so->ctx->bsd[bsd_idx].disable)
		return;

	survive_notify_gen2(so, "sweep called");

	survive_recording_sweep_process(so, channel, sensor_id, timecode, half_clock_flag);

	survive_timecode last_sweep = so->last_sync_time[bsd_idx];
	if (last_sweep == 0) {
		return;
	}
	assert(channel <= NUM_GEN2_LIGHTHOUSES);

	// SV_INFO("Sensor ch%2d %2d %d %12x %6d", channel, sensor_id, flag, timecode, timecode
	// so->last_sync_time[bsd_idx]);

	survive_timecode time_delta = survive_timecode_difference(timecode, last_sweep);
	FLT time_since_sync = (time_delta / 48000000.);
	if (half_clock_flag)
		time_since_sync += 0.5 / 48000000.;

	FLT hz = 48000000. / so->last_time_between_sync[bsd_idx];
	if (fabs(hz - freq_per_channel[channel]) > 1.0) {
		hz = freq_per_channel[channel];
	}

	survive_timecode predicted_time = 48000000. / freq_per_channel[channel];
	int rotations_since = time_delta / predicted_time;

	FLT time_per_rot = 1. / hz;
	time_since_sync -= time_per_rot * rotations_since;

	if (rotations_since > 10) {
		SV_VERBOSE(100, "Dropping light data ch %d sync: %fms rotations missed: %d (at %u)", channel,
				   time_since_sync * 1000., rotations_since, timecode);
		so->stats.dropped_light[bsd_idx]++;
		return;
	}

	FLT angle = time_since_sync / time_per_rot * 2. * LINMATHPI;
	FLT angle2 = (time_since_sync + .5 / 48000000.) / time_per_rot * 2. * LINMATHPI;

	SV_VERBOSE(450, "%s %7.3f Sensor ch%2d.%02d   %+8.3fdeg %12f %d time_since_sync: %.16f rot: %5u tc: %u",
			   survive_colorize(so->codename), survive_run_time(ctx), channel, sensor_id, angle / LINMATHPI * 180.,
			   angle2 / LINMATHPI * 180., half_clock_flag, time_since_sync, rotations_since + so->sync_count[bsd_idx],
			   timecode);

	FLT angle_for_axis[2] = {angle - 2. / 3. * LINMATHPI, angle - 4. / 3. * LINMATHPI};
	int8_t plane = determine_plane(so, bsd_idx, angle);
	SV_DATA_LOG("time_since_sync[%d,%d,%d]", &time_since_sync, 1, channel, sensor_id, plane);

	so->stats.hit_from_lhs[bsd_idx]++;

	if (plane >= 0)
		SURVIVE_INVOKE_HOOK_SO(sweep_angle, so, channel, sensor_id, timecode, plane, angle_for_axis[plane]);
}

SURVIVE_EXPORT void survive_default_sweep_angle_process(SurviveObject *so, survive_channel channel, int sensor_id,
														survive_timecode timecode, int8_t plane, FLT angle) {
	struct SurviveContext *ctx = so->ctx;
	int8_t bsd_idx = survive_get_bsd_idx(ctx, channel);
	if (bsd_idx == -1) {
		SV_WARN("Invalid channel requested(%d) for %s", channel, so->codename)
		return;
	}

	PoserDataLightGen2 l = {
		.common =
			{
				.hdr =
					{
						.pt = POSERDATA_LIGHT_GEN2,
						.timecode = SurviveSensorActivations_long_timecode_light(&so->activations, timecode),
					},
				.sensor_id = sensor_id,
				.angle = angle,
				.lh = bsd_idx,
			},
		.plane = plane,
		.sync = so->sync_count[bsd_idx]};

	SV_VERBOSE(500, "%s %7.3f Sensor ch%2d.%02d.%d %+8.3fdeg", survive_colorize(so->codename), survive_run_time(ctx),
			   channel, sensor_id, plane, angle / LINMATHPI * 180.);

	// Simulate the use of only one lighthouse in playback mode.
	if (bsd_idx < ctx->activeLighthouses) {
		if (SurviveSensorActivations_add_gen2(&so->activations, &l) == false) {
			so->stats.rejected_data[bsd_idx]++;
		} else {
			so->stats.accepted_data[bsd_idx]++;
			survive_kalman_tracker_integrate_light(so->tracker, &l.common);
		}
	}

	survive_recording_sweep_angle_process(so, channel, sensor_id, timecode, plane, angle);
	SURVIVE_POSER_INVOKE(so, &l);
}

SURVIVE_EXPORT void survive_default_gen_detected_process(SurviveObject *so, int lh_version) {
	SurviveContext *ctx = so->ctx;

	if (ctx->lh_version != -1) {
		static bool seenWarning = false;
		if (seenWarning == false) {
			SV_WARN("Detected both LH gen1 and LH gen2 systems. Mixed mode is only supported for experimentation.");
			seenWarning = true;
		}

		ctx->lh_version = 3;
		return;
	}
	assert(ctx->lh_version == -1);

	SV_INFO("Detected LH gen %d system.", lh_version + 1);
	if (lh_version != ctx->lh_version_configed && ctx->lh_version_configed != -1) {
		SV_WARN("Configuration was valid for gen %d; resetting BSD positions and OOTX", ctx->lh_version_configed + 1);
		for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
			ctx->bsd[i].PositionSet = ctx->bsd[i].OOTXSet = 0;
			ctx->bsd[i].mode = -1;
		}
	}

	for (int i = 0; i < ctx->objs_ct; i++) {
		// When the USB devices transition; they reset their clock or something -- so clear out old data.
		SurviveSensorActivations_reset(&ctx->objs[i]->activations);
	}

	ctx->lh_version = lh_version;
	survive_configi(ctx, "configed-lighthouse-gen", SC_OVERRIDE | SC_SETCONFIG, lh_version + 1);
	config_save(ctx);
}
