#include "survive_optimizer.h"

#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif

#include "mpfit/mpfit.h"
#include "poser.h"
#include <survive.h>
#include <variance.h>

#include "assert.h"
#include "linmath.h"
#include "math.h"
#include "poser_general_optimizer.h"
#include "string.h"
#include "survive_async_optimizer.h"
#include "survive_config.h"
#include "survive_kalman_tracker.h"
#include "survive_recording.h"
#include "survive_reproject.h"
#include "survive_reproject_gen2.h"

#ifndef _WIN32
//#define DEBUG_NAN
#endif
#ifdef DEBUG_NAN
#include <fenv.h>
#endif

STATIC_CONFIG_ITEM(SERIALIZE_SOLVE, "serialize-lh-mpfit", 's', "Serialize MPFIT formulization", 0)
STATIC_CONFIG_ITEM(USE_JACOBIAN_FUNCTION, "use-jacobian-function", 'i',
				   "If set to false, a slower numerical approximation of the jacobian is used. Set to -1 to see debug output", 1)
STATIC_CONFIG_ITEM(SENSOR_VARIANCE_PER_SEC, "sensor-variance-per-sec", 'f',
				   "Variance per second to add to the sensor input -- discounts older data", 0.0)
STATIC_CONFIG_ITEM(SENSOR_VARIANCE, "sensor-variance", 'f', "Base variance for each sensor input", 1.0e-4)

STATIC_CONFIG_ITEM(DISABLE_LIGHTHOUSE, "disable-lighthouse", 'i', "Disable given lighthouse from tracking", -1)
STATIC_CONFIG_ITEM(RUN_EVERY_N_SYNCS, "syncs-per-run", 'i', "Number of sync pulses before running optimizer", 1)
STATIC_CONFIG_ITEM(RUN_POSER_ASYNC, "poser-async", 'i', "Run the poser in it's own thread", 0)

STATIC_CONFIG_ITEM(PRECISE_POSE, "precise", 'b', "Always calculate precise pose", 0)
STATIC_CONFIG_ITEM(USE_STATIONARY_SENSOR_WINDOW, "use-stationary-sensor-window", 'i',
				   "Use larger time window when stationary", 1)
STATIC_CONFIG_ITEM(MPFIT_FULL_COV, "mpfit-use-cov", 'b', "Use the mpfit covariance output", 1)

typedef struct MPFITStats {
	int meas_failures;
	int total_iterations;
	int total_fev;
	int total_runs;
	FLT sum_errors;
	FLT sum_origerrors;
	int status_cnts[9];

	uint32_t total_meas_cnt;
	uint32_t total_lh_cnt;
	uint32_t dropped_meas_cnt;
	uint32_t dropped_lh_cnt;
} MPFITStats;

typedef struct MPFITGlobalData {
	size_t instances;
	MPFITStats stats;
} MPFITGlobalData;

static MPFITGlobalData g;

typedef struct MPFITData {
	GeneralOptimizerData opt;

	int disable_lighthouse;
	int sensor_time_window;
	// > 0; use jacobian, 0 don't use, < 0 debug
	int use_jacobian_function_obj;
	int use_jacobian_function_lh;
	int required_meas;
  int syncs_per_run;

  int syncs_per_run_cnt;
  int syncs_seen;
  int syncs_to_setup;

  FLT sensor_variance;
  FLT sensor_variance_per_second;

  bool alwaysPrecise;

  bool useStationaryWindow;
  const char *serialize_prefix;
  MPFITStats stats;

  FLT record_reprojection_error;
  FLT obj_up_variance, lh_up_variance, calibration_stationary_obj_up_variance, stationary_obj_up_variance;
  FLT sensor_variance_cal;
  bool model_velocity;
  bool globalDataAvailable;
  struct survive_async_optimizer *async_optimizer;

  survive_optimizer_settings optimizer_settings;
} MPFITData;

STRUCT_CONFIG_SECTION(MPFITData)
STRUCT_CONFIG_ITEM("mpfit-cal-sensor-variance", "Light sensor variance for calibration", 1e-4, t->sensor_variance_cal)
STRUCT_CONFIG_ITEM("mpfit-model-velocity", "Model velocity in non mpfit process", true, t->model_velocity)
STRUCT_CONFIG_ITEM("mpfit-record-reprojection-error", "", 0, t->record_reprojection_error)
STRUCT_CONFIG_ITEM("mpfit-object-up-variance",
				   "How much to weight having the accel direction on tracked objects pointing up", -1,
				   t->obj_up_variance)
STRUCT_CONFIG_ITEM("mpfit-stationary-object-up-variance",
				   "How much to weight having the accel direction on tracked objects pointing up", 1e-2,
				   t->stationary_obj_up_variance)
STRUCT_CONFIG_ITEM("mpfit-cal-stationary-object-up-variance",
				   "How much to weight having the accel direction on tracked objects pointing up during calibration",
				   1e-3, t->calibration_stationary_obj_up_variance)
STRUCT_CONFIG_ITEM("mpfit-lighthouse-up-variance",
				   "How much to weight having the accel direction on lighthouses pointing up", 1e-2, t->lh_up_variance)
END_STRUCT_CONFIG_SECTION(MPFITData)

static size_t remove_lh_from_meas(survive_optimizer *mpfitctx, int lh) {
	size_t rtn = mpfitctx->measurementsCnt;
	survive_optimizer_measurement *meas = mpfitctx->measurements;
	for (int i = 0; i < rtn; i++) {
		while (meas[i].meas_type == survive_optimizer_measurement_type_light && meas[i].light.lh == lh && i < rtn) {
			meas[i] = meas[rtn - 1];
			rtn--;
		}
	}
	return rtn;
}

struct async_optimizer_user {
	MPFITData *d;
	PoserDataLight pdl;
	bool canPossiblySolveLHS;
	bool worldEstablished;
	size_t meas_for_lhs_axis[NUM_GEN2_LIGHTHOUSES * 2];
	struct variance_measure meas_variance[NUM_GEN2_LIGHTHOUSES * 2];

	struct {
		survive_long_timecode old_measurements_age;
		uint32_t time_window;
		uint32_t old_measurements;
	} stats;
};

static size_t construct_input_from_scene(const MPFITData *d, survive_long_timecode timecode,
										 const SurviveSensorActivations *scene, size_t *meas_for_lhs_axis,
										 survive_optimizer *mpfitctx, struct async_optimizer_user *user) {
	size_t rtn = 0;
	SurviveObject *so = d->opt.so;
	SurviveContext *ctx = so->ctx;
	survive_long_timecode *most_recent_time = user ? &user->pdl.hdr.timecode : 0;

	bool isStationary = SurviveSensorActivations_stationary_time(scene) > so->timebase_hz;
	survive_timecode sensor_time_window =
		isStationary && d->useStationaryWindow ? (so->timebase_hz) : d->sensor_time_window;

	if (user) {
		user->stats.time_window = sensor_time_window;
	}

	for (uint8_t lh = 0; lh < ctx->activeLighthouses; lh++) {
		if (d->disable_lighthouse == lh) {
			continue;
		}

		if (!ctx->bsd[lh].PositionSet && (!isStationary || !ctx->bsd[lh].OOTXSet)) {
			continue;
		}

		if (!ctx->bsd[lh].PositionSet) {
			SV_VERBOSE(500, "Allowing data from %d", lh);
		}

		bool isCandidate = !ctx->bsd[lh].PositionSet;
		size_t candidate_meas = 10;
		size_t required_meas_for_lh = 0;

		size_t meas_for_lh = 0;
		for (uint8_t sensor = 0; sensor < so->sensor_ct; sensor++) {
			for (uint8_t axis = 0; axis < 2; axis++) {
				survive_long_timecode last_reading =
					SurviveSensorActivations_time_since_last_reading(scene, sensor, lh, axis);
				bool isReadingValue = last_reading < sensor_time_window;

				if (isReadingValue) {
					const FLT *a = scene->angles[sensor][lh];

					survive_optimizer_measurement *meas =
						survive_optimizer_emplace_meas(mpfitctx, survive_optimizer_measurement_type_light);

					meas->light.object = 0;
					meas->light.axis = axis;
					meas->light.value = a[axis];
					meas->light.sensor_idx = sensor;
					meas->light.lh = lh;
					if (user) {
						variance_measure_add(&user->meas_variance[lh * 2 + axis], &meas->light.value);
					}
					survive_long_timecode diff = timecode - scene->timecode[sensor][lh][axis];
					meas->time = scene->timecode[sensor][lh][axis] / (FLT)so->timebase_hz;
					meas->variance = d->sensor_variance + diff * d->sensor_variance_per_second / (FLT)so->timebase_hz;
					if (most_recent_time && scene->timecode[sensor][lh][axis] > *most_recent_time) {
						*most_recent_time = scene->timecode[sensor][lh][axis];
					}
					// SV_INFO("Adding meas %d %d %d %f", lh, sensor, axis, meas->value);
					rtn++;
					meas_for_lh++;
					if (meas_for_lhs_axis) {
						meas_for_lhs_axis[lh * 2 + axis]++;
					}
				} else if (last_reading != UINT32_MAX && user) {
					user->stats.old_measurements_age += last_reading;
					user->stats.old_measurements++;
				}
			}
		}
		if ((isCandidate && meas_for_lh < candidate_meas) || meas_for_lh < required_meas_for_lh) {
			survive_optimizer_pop_meas(mpfitctx, meas_for_lh);
			rtn -= meas_for_lh;

			for (int axis = 0; axis < 2 && meas_for_lhs_axis; axis++)
				meas_for_lhs_axis[lh * 2 + axis] = 0;
		}
	}
	return rtn;
}

static bool invalid_starting_condition(MPFITData *d, size_t meas_size, const size_t *meas_for_lhs_axis) {
	static int failure_count = 500;
	struct SurviveObject *so = d->opt.so;

	size_t meas_size_known_lh = 0;
	size_t axis_known_lh = 0;
	if (meas_for_lhs_axis) {
		for (uint8_t lh = 0; lh < so->ctx->activeLighthouses; lh++) {
			if (so->ctx->bsd[lh].PositionSet) {
				meas_size_known_lh += meas_for_lhs_axis[2 * lh] + meas_for_lhs_axis[2 * lh + 1];
				for (int axis = 0; axis < 2; axis++)
					axis_known_lh += meas_for_lhs_axis[2 * lh + axis] > 0;
			}
		}
	} else {
		meas_size_known_lh = meas_size;
	}

	if (meas_size_known_lh < d->required_meas || axis_known_lh < 2) {
		if (failure_count++ == 500) {
			SurviveContext *ctx = so->ctx;
			SV_INFO("Can't solve for position with just %u measurements", (unsigned int)meas_size_known_lh);
			failure_count = 0;
		}
		if (meas_size_known_lh < d->required_meas || axis_known_lh < 2) {
			d->stats.meas_failures++;
		}
		return true;
	}
	failure_count = 0;
	return false;
}

static inline void serialize_mpfit(MPFITData *d, survive_optimizer *mpfitctx) {
	if (d->serialize_prefix) {
		char path[1024] = {0};
		snprintf(path, 1023, "%s_%s_%d.opt", d->serialize_prefix, d->opt.so->codename, d->stats.total_runs);
		survive_optimizer_serialize(mpfitctx, path);
	}
}

static inline bool has_data_for_lh(const size_t *meas_for_lhs_axis, int lh) {
	return meas_for_lhs_axis[2 * lh] > 0 && meas_for_lhs_axis[2 * lh + 1] > 0;
}
static inline int get_strong_axis_count(const size_t *meas_for_lhs_axis, int min_support) {
	int num_axis = 0;
	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES * 2; i++) {
		if (meas_for_lhs_axis[i] >= min_support)
			num_axis++;
	}
	return num_axis;
}
static inline int get_axis_count(const size_t *meas_for_lhs_axis) {
	int num_axis = 0;
	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES * 2; i++) {
		if (meas_for_lhs_axis[i] > 0)
			num_axis++;
	}
	return num_axis;
}
static inline int get_lh_count(const size_t *meas_for_lhs_axis) {
	int num_lh = 0;
	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (meas_for_lhs_axis[2 * i] > 0 || meas_for_lhs_axis[2 * i + 1] > 0)
			num_lh++;
	}
	return num_lh;
}

static int setup_optimizer(struct async_optimizer_user *user, survive_optimizer *mpfitctx,
						   SurviveSensorActivations *scene) {
	MPFITData *d = user->d;
	PoserDataLight *pdl = &user->pdl;
	size_t *meas_for_lhs_axis = user->meas_for_lhs_axis;

	SurviveObject *so = d->opt.so;
	struct SurviveContext *ctx = so->ctx;

	SurvivePose *soLocation = survive_optimizer_get_pose(mpfitctx);
	survive_optimizer_setup_cameras(mpfitctx, so->ctx, true, d->use_jacobian_function_lh, false);
	bool objectStationary = SurviveSensorActivations_stationary_time(&so->activations) > 3 * so->timebase_hz;

	bool worldEstablished = false;
	for (int lh = 0; lh < ctx->activeLighthouses && !worldEstablished; lh++)
		worldEstablished |= ctx->bsd[lh].PositionSet;

	survive_optimizer_setup_pose(mpfitctx, 0, !worldEstablished, d->use_jacobian_function_obj);
	SurviveVelocity *velocity = survive_optimizer_get_velocity(mpfitctx);
	if (velocity) {
		velocity[0] = so->velocity;
	}
	if (norm3d(so->activations.accel) == 0 || !isfinite(norm3d(so->activations.accel))) {
		return -2;
	}

	*soLocation = *survive_object_last_imu2world(so);
	bool currentPositionValid = quatmagnitude(soLocation->Rot) != 0;

	if (worldEstablished && !general_optimizer_data_record_current_pose(&d->opt, pdl, soLocation)) {
		return -1;
	}

	FLT upVectorBias = d->lh_up_variance;
	if (!worldEstablished && upVectorBias > 0) {
		FLT accel_mag = norm3d(so->activations.accel);
		const FLT up[3] = {0, 0, 1};
		if (accel_mag != 0.0 && !isnan(accel_mag)) {
			// quatfrom2vectors(soLocation->Rot, so->activations.accel, up);
			// so->OutPoseIMU = *soLocation;
		}
	}

	if (quatiszero(soLocation->Rot)) {
		soLocation->Rot[0] = 1;
	}

	size_t meas_size = construct_input_from_scene(d, pdl->hdr.timecode, scene, meas_for_lhs_axis, mpfitctx, user);

	if (worldEstablished && invalid_starting_condition(d, meas_size, meas_for_lhs_axis)) {
		return -1;
	}

	bool canPossiblySolveLHS = false;
	bool bestObjForCal = true;
	for (int obj_id = 0; obj_id < so->ctx->objs_ct; obj_id++) {
		if (so->ctx->objs[obj_id]->sensor_ct > so->sensor_ct) {
			bestObjForCal = false;
		}
	}

	bool needsInitialEstimate = false;

	// This just spaces out the controllers; forces an order onto what trys to calibrate what
	int syncs_required = 200 - so->sensor_ct * 2 - (so->codename[2] - '0') * 10;

	if (bestObjForCal || (d->syncs_seen > syncs_required && objectStationary)) {
		for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
			if (!so->ctx->bsd[lh].OOTXSet) {
				// Wait til this thing gets OOTX, and then solve for as much as we can. Avoids doing
				// two solves in a row because of OOTX timing.
				canPossiblySolveLHS = false;
				break;
			}

			bool needsSolve =
				!so->ctx->bsd[lh].PositionSet || (canPossiblySolveLHS == false && (so->ctx->bsd[lh].confidence) < 0);

			if (needsSolve && has_data_for_lh(meas_for_lhs_axis, lh)) {
				canPossiblySolveLHS = !d->globalDataAvailable;
				needsInitialEstimate = !so->ctx->bsd[lh].PositionSet;
			}
		}
	}

	SurvivePose lhs[NUM_GEN2_LIGHTHOUSES] = {0};
	if (canPossiblySolveLHS) {
		if (!needsInitialEstimate || general_optimizer_data_record_current_lhs(&d->opt, pdl, lhs)) {
			uint32_t reference_basestation = survive_configi(ctx, "reference-basestation", SC_GET, 0);

			for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
				bool needsSolve = !so->ctx->bsd[lh].PositionSet;
				if (needsSolve) {
					// Reference basestations are used mostly for unit testing; if we can't solve for it we don't solve
					// for anything
					if (reference_basestation != 0 && quatiszero(lhs[lh].Rot)) {
						if (reference_basestation == so->ctx->bsd[lh].BaseStationID) {
							canPossiblySolveLHS = false;
						}
					}

					assert(!isnan(lhs[lh].Rot[0]));

					if (quatiszero(lhs[lh].Rot) && has_data_for_lh(meas_for_lhs_axis, lh)) {
						SV_WARN("Seed poser failed for %d, not trying to solve LH system", lh);
						canPossiblySolveLHS = false;
						break;
					}
				}
			}
		} else {
			canPossiblySolveLHS = false;
		}
	}

	size_t skipped_lh_cnt = 0;
	for (int lh = 0; lh < so->ctx->activeLighthouses; lh++) {
		survive_optimizer_set_cam_up_vector(mpfitctx, lh, d->lh_up_variance, ctx->bsd[lh].accel);
		FLT meas[2] = { meas_for_lhs_axis[2 * lh] + 1e-2 * lh, meas_for_lhs_axis[2 * lh + 1] + 1e-2 * lh + 1e-3};
		SV_DATA_LOG("meas_for_lh_axis[%d]", meas, 2, lh);
		if (!so->ctx->bsd[lh].PositionSet) {
			if (canPossiblySolveLHS) {
				if (has_data_for_lh(meas_for_lhs_axis, lh)) {
					SV_INFO("Attempting to solve for %d with %lu/%lu meas from device %s", lh,
							meas_for_lhs_axis[2 * lh], meas_for_lhs_axis[2 * lh + 1], survive_colorize(so->codename));
					survive_optimizer_setup_camera(mpfitctx, lh, &lhs[lh], false, d->use_jacobian_function_lh);
				} else {
					skipped_lh_cnt++;
				}
			} else {
				// SV_VERBOSE(200, "Removing data for %d", lh);
				meas_size = remove_lh_from_meas(mpfitctx, lh);
				meas_for_lhs_axis[2 * lh] = meas_for_lhs_axis[2 * lh + 1] = 0;
			}
		} else if (canPossiblySolveLHS) {
			SV_INFO("Assuming %d with %lu/%lu meas from device %s as given", lh, meas_for_lhs_axis[2 * lh],
					meas_for_lhs_axis[2 * lh + 1], survive_colorize(so->codename));
		}
	}

	if (skipped_lh_cnt > 0 && survive_run_time(ctx) < 4) {
		// Heurestic -- stuff can come on in weird orders; give a little time to solve everything at once.
		canPossiblySolveLHS = false;
	}

	if (canPossiblySolveLHS) {
		SV_INFO("Assuming object position of " SurvivePose_format, SURVIVE_POSE_EXPAND(*soLocation));
	}

	user->worldEstablished = worldEstablished;
	user->canPossiblySolveLHS = canPossiblySolveLHS;

	if (!worldEstablished && !canPossiblySolveLHS) {
		return -1;
	}
	if (!worldEstablished && d->globalDataAvailable) {
		return -2;
	}

	/*
	if ((d->useKalman || d->useIMU)) {
		survive_long_timecode tc = so->activations.last_light;
		//survive_kalman_tracker_predict(so->tracker, tc, soLocation);
	}
	 */

	serialize_mpfit(d, mpfitctx);
	if (canPossiblySolveLHS || d->alwaysPrecise) {
		mpfitctx->cfg = survive_optimizer_precise_config();
	}

	if (canPossiblySolveLHS || get_strong_axis_count(meas_for_lhs_axis, 2) < 4) {
		survive_optimizer_disable_sensor_scale(mpfitctx);
		// SV_VERBOSE(110, "%s Scale not solving %d", survive_colorize_codename(so),
		//		   get_strong_axis_count(meas_for_lhs_axis, 2));
	} else {
		// SV_VERBOSE(110, "%s Scale solving", survive_colorize_codename(so));
	}

	if (canPossiblySolveLHS) {
		// mpfitctx->iteration_cb = iteration_cb;
	}
	return 0;
}


void print_stats_results(SurviveContext* ctx, const survive_optimizer *mpfitctx, const mp_result *result, int verbosity) {
	SV_VERBOSE(
		verbosity,
		"Results %10.10f/%10.10f %d iter %s (%3d meas, %3d pars (%3d free), %d lhs, %d dev "
		"sensor_err %7.7f up_err %7.7f bias %7.7f",
		result->orignorm, result->bestnorm, result->niter, survive_optimizer_error(result->status),
		(int)mpfitctx->measurementsCnt, (int)mpfitctx->parametersCnt, result->nfree,
		mpfitctx->cameraLength, mpfitctx->poseLength,
		sqrtf(mpfitctx->stats.sensor_error / mpfitctx->stats.sensor_error_cnt),
		sqrtf(mpfitctx->stats.object_up_error / mpfitctx->stats.object_up_error_cnt),
		sqrtf(mpfitctx->stats.params_error / mpfitctx->stats.params_error_cnt));
}
static FLT handle_optimizer_results(survive_optimizer *mpfitctx, int res, const mp_result *result,
									struct async_optimizer_user *user_data, CnMat *R, SurvivePose *out) {
	FLT rtn = -1;

	size_t meas_size = mpfitctx->measurementsCnt;
	SurvivePose *soLocation = survive_optimizer_get_pose(mpfitctx);
	bool canPossiblySolveLHS = user_data->canPossiblySolveLHS;
	bool worldEstablished = user_data->worldEstablished;
	size_t *meas_for_lhs_axis = user_data->meas_for_lhs_axis;
	MPFITData *d = user_data->d;
	PoserDataLight *pdl = &user_data->pdl;

	SurviveObject *so = d->opt.so;
	struct SurviveContext *ctx = so->ctx;

	SV_DATA_LOG("mpfit_norm_best", &result->bestnorm, 1);
	SV_DATA_LOG("mpfit_norm_orig", &result->orignorm, 1);

	FLT iterations = result->niter, fev = result->nfev;
	SV_DATA_LOG("mpfit_iterations", &iterations, 1);
	SV_DATA_LOG("mpfit_nfevals", &fev, 1);

	FLT meas_f = mpfitctx->measurementsCnt;
	SV_DATA_LOG("mpfit_measurement_cnt", &meas_f, 1);

	bool status_failure = res <= 0;
	if (status_failure) {
		SV_WARN("MPFIT status failure %s %f/%f (%d measurements, %d)", survive_colorize(so->codename), result->orignorm,
				result->bestnorm, (int)meas_size, res);

		general_optimizer_data_record_failure(&d->opt);
		return -1;
	}
	bool solvedLHPoses = false;
	FLT sensor_error = sqrtf(mpfitctx->stats.sensor_error / mpfitctx->stats.sensor_error_cnt);
	FLT norm_error = sensor_error; // result->bestnorm * d->sensor_variance * d->sensor_variance;
	bool error_failure = !general_optimizer_data_record_success(&d->opt, norm_error, soLocation, canPossiblySolveLHS);
	if (!status_failure && !error_failure) {
		quatnormalize(soLocation->Rot, soLocation->Rot);

		if (canPossiblySolveLHS) {
			if (!worldEstablished && mpfitctx->objectUpVectorVariance <= 0.0)
				*soLocation = (SurvivePose){0};

			SurvivePose *opt_cameras = survive_optimizer_get_camera(mpfitctx);
			SurvivePose cameras[NUM_GEN2_LIGHTHOUSES] = {0};
			FLT variances[NUM_GEN2_LIGHTHOUSES] = {0};

			for (int i = 0; i < mpfitctx->cameraLength; i++) {
				if (has_data_for_lh(meas_for_lhs_axis, i) > 0 && !quatiszero(opt_cameras[i].Rot)) {
					cameras[i] = InvertPoseRtn(&opt_cameras[i]);
				}
				for (int axis = 0; axis < 2; axis++) {
					FLT v;
					variance_measure_calc(&user_data->meas_variance[i * 2 + axis], &v);
					variances[i] += 0.5 * v;
				}
			}

			if (!worldEstablished) {
				PoserData_normalize_scene(ctx, cameras, ctx->activeLighthouses, soLocation, R);
			}

			PoserData_lighthouse_poses_func(&pdl->hdr, so, cameras, R, ctx->activeLighthouses,
											soLocation);
			solvedLHPoses = true;
		}

		int axis_count = 0, lh_count = 0, sensor_ct = 0;
		for (int i = 0; i < mpfitctx->cameraLength; i++) {
			if(has_data_for_lh(meas_for_lhs_axis, i)) {
				lh_count++;
			}
			for(int axis = 0;axis < 2;axis++) {
				if(meas_for_lhs_axis[i * 2 + axis])
					axis_count++;
				sensor_ct += meas_for_lhs_axis[i * 2 + axis];
			}
		}

		FLT penalty = 1. / pow(2, sensor_ct / 3.) + 1. / pow(2, axis_count * 5);
		*out = *soLocation;
		rtn = result->bestnorm; // + penalty;

		if (d->record_reprojection_error) {
			for (int i = 0; i < mpfitctx->measurementsCnt; i++) {
				survive_optimizer_measurement *meas = &mpfitctx->measurements[i];
				if (meas->meas_type == survive_optimizer_measurement_type_light) {
					SurvivePose world2lh = InvertPoseRtn(survive_get_lighthouse_position(ctx, meas->light.lh));
					FLT v = survive_reproject_model(ctx)->reprojectAxisFullFn[meas->light.axis](
						soLocation, &so->sensor_locations[meas->light.sensor_idx * 3], &world2lh,
						ctx->bsd[meas->light.lh].fcal + meas->light.axis);
					survive_recording_write_to_output(ctx->recptr, "%s RA %d %d %f %u\r\n", so->codename,
													  meas->light.sensor_idx, meas->light.axis, v - meas->light.value,
													  meas->light.lh);
				}
			}
		}

		FLT v[] = {axis_count, lh_count, sensor_ct, rtn};
		SV_DATA_LOG("mpfit_confidence_measures", v, 4);

		FLT scale = so->sensor_scale;
		SV_VERBOSE(
			!solvedLHPoses ? 110 : 100,
			"MPFIT success %s %f7.5s %s %f/%10.10f/%10.10f (%3d measurements, %s result, %d lighthouses, %d axis, "
			"%6.3fms "
			"time_window, %2d old_meas (avg %6.3fms) run #%d) scale %7.7f sensor_err %7.7f up_err %7.7f bias %7.7f",
			survive_colorize(so->codename), survive_run_time(ctx),
			survive_colorize(SurviveSensorActivations_stationary_time(&so->activations) > 4800000 ? "STILL" : "MOVE "),
			result->orignorm, result->bestnorm,
			sqrt(result->bestnorm / (mpfitctx->measurementsCnt - result->nfree + 1) * d->sensor_variance *
				 d->sensor_variance),
			(int)meas_size, survive_optimizer_error(res), get_lh_count(meas_for_lhs_axis),
			get_axis_count(meas_for_lhs_axis), user_data->stats.time_window / 48000000. * 1000.,
			user_data->stats.old_measurements,
			user_data->stats.old_measurements_age / 48000000. * 1000. / (.001 + user_data->stats.old_measurements),
			d->stats.total_runs, scale,
			sqrtf(mpfitctx->stats.sensor_error / mpfitctx->stats.sensor_error_cnt),
			sqrtf(mpfitctx->stats.object_up_error / mpfitctx->stats.object_up_error_cnt),
			sqrtf(mpfitctx->stats.params_error / mpfitctx->stats.params_error_cnt));
	} else {
		SV_VERBOSE(
			100,
			"MPFIT failure %s %f7.5s %f/%10.10f/%10.10f (%d measurements, %s result, %d lighthouses, %d axis, %d "
			"canSolveLHs, %d "
			"since success, "
			"run #%d) scale %7.7f sensor_err %7.7f param %7.7f",
			survive_colorize(so->codename), survive_run_time(ctx), result->orignorm, result->bestnorm,
			sqrt(result->bestnorm / (mpfitctx->measurementsCnt - result->nfree + 1) * d->sensor_variance *
				 d->sensor_variance),
			(int)meas_size, survive_optimizer_error(res), get_lh_count(meas_for_lhs_axis),
			get_axis_count(meas_for_lhs_axis), canPossiblySolveLHS, d->opt.failures_since_success, d->stats.total_runs,
			so->sensor_scale,
			sqrtf(mpfitctx->stats.sensor_error / mpfitctx->stats.sensor_error_cnt),
			sqrtf(mpfitctx->stats.params_error / mpfitctx->stats.params_error_cnt));

		if (canPossiblySolveLHS && d->opt.failures_since_success > 10 && d->opt.stats.successes < 10 &&
			(SurviveSensorActivations_stationary_time(&so->activations) > (48000000 / 10))) {
			SV_WARN("Tracker lost for %s", so->codename);
			survive_kalman_tracker_lost_tracking(so->tracker, true);
		}
	}

	d->stats.dropped_meas_cnt += mpfitctx->stats.dropped_meas_cnt;
	d->stats.dropped_lh_cnt += mpfitctx->stats.dropped_lh_cnt;
	d->stats.total_meas_cnt += mpfitctx->stats.total_meas_cnt;
	d->stats.total_lh_cnt += mpfitctx->stats.total_lh_cnt;
	d->stats.total_fev += result->nfev;
	d->stats.total_iterations += result->niter;
	d->stats.total_runs++;
	d->stats.sum_errors += result->bestnorm;
	d->stats.sum_origerrors += result->orignorm;
	if (result->status > 0) {
		assert(result->status < 10);
		d->stats.status_cnts[result->status - 1]++;
	}
	return rtn;
}

static void handle_results(MPFITData *d, PoserDataLight *lightData, FLT error, SurvivePose *estimate, const CnMat *R) {
	SurviveObject *so = d->opt.so;
	if (error > 0) {
		if (so->object_type == SURVIVE_OBJECT_TYPE_HMD && so->ctx->request_floor_set) {
			FLT adjust = estimate->Pos[2];
			estimate->Pos[2] = 0;

			for (int i = 0; i < so->ctx->activeLighthouses; i++) {
				SurvivePose new_pos = *survive_get_lighthouse_position(so->ctx, i);
				if (so->ctx->bsd[i].PositionSet) {
					new_pos.Pos[2] -= adjust;
				}
				SURVIVE_INVOKE_HOOK(raw_lighthouse_pose, so->ctx, i, &new_pos);
			}
			so->ctx->request_floor_set = false;
		}

		PoserData_poser_pose_func(&lightData->hdr, so, estimate, error, R);
	}
}

typedef void (*handle_results_fn)(MPFITData *d, PoserDataLight *lightData, FLT error, SurvivePose *estimate);

static FLT run_mpfit_find_3d_structure(MPFITData *d, PoserDataLight *pdl, SurviveSensorActivations *scene,
									   SurvivePose *out, CnMat *R) {
	SurviveObject *so = d->opt.so;
	struct SurviveContext *ctx = so->ctx;

	bool objectStationary = SurviveSensorActivations_stationary_time(&so->activations) > so->timebase_hz;
	survive_optimizer mpfitctx = {
	        .settings = &d->optimizer_settings,
	        .reprojectModel = survive_reproject_model(ctx),
								  .poseLength = 1,
								  .cameraLength = so->ctx->activeLighthouses,
								  .timecode = pdl->hdr.timecode / (FLT)so->timebase_hz,
								  .objectUpVectorVariance =
									  objectStationary ? d->stationary_obj_up_variance : d->obj_up_variance,
								  .disableVelocity = d->model_velocity == false || objectStationary,
								  .user = d};
	// stationary_obj_up_variance;
	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(mpfitctx, so);

	struct async_optimizer_user user_data = {.d = d, .pdl = *pdl};

	int setup_results = setup_optimizer(&user_data, &mpfitctx, scene);
	if (setup_results < 0) {
		return setup_results;
	}

	mp_result result = {0};

	int nfree = survive_optimizer_get_free_parameters_count(&mpfitctx);
	survive_release_ctx_lock(ctx);
	int res = survive_optimizer_run(&mpfitctx, &result, R);
//	cn_print_mat(R);
	survive_get_ctx_lock(ctx);

	return handle_optimizer_results(&mpfitctx, res, &result, &user_data, R, out);
}

static inline void print_stats(SurviveContext *ctx, MPFITStats *stats) {
	// if (stats->total_iterations == 0)
	//		return;
	int total_runs = stats->total_runs;
	if (total_runs == 0)
		total_runs++;

	SV_INFO("\tmeas failures     %d", stats->meas_failures);
	SV_INFO("\ttotal iterations  %d", stats->total_iterations);
	SV_INFO("\tavg iterations    %f", (FLT)stats->total_iterations / total_runs);
	SV_INFO("\ttotal fevals      %d", stats->total_fev);
	SV_INFO("\tavg fevals        %f", (FLT)stats->total_fev / total_runs);
	SV_INFO("\ttotal runs        %d", stats->total_runs);
	SV_INFO("\tavg error         %10.10f", stats->sum_errors / total_runs);
	SV_INFO("\tavg orig error    %10.10f", stats->sum_origerrors / total_runs);
	if (stats->total_meas_cnt)
		SV_INFO("\tnoisy meas cnt    %7d / %8d (%4.2f%%)", stats->dropped_meas_cnt, stats->total_meas_cnt,
				100. * (stats->dropped_meas_cnt / (FLT)stats->total_meas_cnt));

	if (stats->total_lh_cnt)
		SV_INFO("\tdropped lh cnt    %7d / %8d (%4.2f%%)", stats->dropped_lh_cnt, stats->total_lh_cnt,
				100. * (stats->dropped_lh_cnt / (FLT)stats->total_lh_cnt));

	for (int i = 0; i < sizeof(stats->status_cnts) / sizeof(int); i++) {
		SV_INFO("\tStatus %10s %d", survive_optimizer_error(i + 1), stats->status_cnts[i]);
	}
}

bool find_initial_camera(PoserDataGlobalScenes *gss, int lh, SurvivePose *pose) {
	if (!quatiszero(pose->Rot))
		return false;

	int best_scene = -1;
	int best_scene_cnt = 5;
	for (int i = 0; i < gss->scenes_cnt; i++) {
		int cnt = 0;
		for (int m = 0; m < gss->scenes[i].meas_cnt; m++) {
			if (!quatiszero(gss->scenes[i].pose.Rot) && gss->scenes[i].meas[m].lh == lh)
				cnt++;
		}

		if (cnt > best_scene_cnt) {
			best_scene = i;
			best_scene_cnt = cnt;
		}
	}

	if (best_scene == -1)
		return false;

	return 0;
}

struct global_data {
	bool updated;
	SurvivePose *camera;
	SurvivePose *pose;
};

void global_pose(SurviveObject *so, uint32_t lighthouse, const SurvivePose *pose, void *user) {
	struct global_data *gd = user;
	*gd->pose = *pose;
	gd->updated |= true;

	SurviveContext *ctx = so->ctx;
	SV_VERBOSE(10, "Initial pose (%s) " SurvivePose_format, survive_colorize(so->codename), SURVIVE_POSE_EXPAND(*pose));
}
void global_lh_pose(SurviveObject *so, uint8_t lighthouse, SurvivePose *lighthouse_pose, SurvivePose *object_pose,
					void *user) {
	struct global_data *gd = user;
	gd->camera[lighthouse] = InvertPoseRtn(lighthouse_pose);
	gd->updated |= true;

	SurviveContext *ctx = so->ctx;
	SV_VERBOSE(10, "Initial LH pose from %s (%d) " SurvivePose_format, survive_colorize_codename(so), lighthouse,
			   SURVIVE_POSE_EXPAND(*lighthouse_pose));
}

bool solve_global_scene(struct SurviveContext *ctx, MPFITData *d, PoserDataGlobalScenes *gss) {
	if (gss->scenes_cnt == 0 || gss->scenes == 0)
		return false;

	size_t meas_cnt = 0;
	size_t scenes_cnt = gss->scenes_cnt;
	for (int i = 0; i < scenes_cnt; i++) {
		meas_cnt += gss->scenes[i].meas_cnt;
	}

	survive_optimizer mpfitctx = {.settings = &d->optimizer_settings,
								  .reprojectModel = survive_reproject_model(ctx),
								  .poseLength = scenes_cnt,
								  .cameraLength = ctx->activeLighthouses,
								  .objectUpVectorVariance = d->calibration_stationary_obj_up_variance,
								  .disableVelocity = true,
								  .nofilter = scenes_cnt < 8};

	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(mpfitctx, 0);
	int useJacobians = 1;
	survive_optimizer_setup_cameras(&mpfitctx, ctx, false, useJacobians, true); // d->use_jacobian_function_obj);
	size_t lh_meas[NUM_GEN2_LIGHTHOUSES][2] = {0};

	struct variance_measure lh_meas_variance[NUM_GEN2_LIGHTHOUSES] = {0};

	for (int i = 0; i < scenes_cnt; i++) {
		meas_cnt += gss->scenes[i].meas_cnt;
		mpfitctx.sos[i] = gss->scenes[i].so;

		survive_optimizer_set_obj_up_vector(&mpfitctx, i, mpfitctx.objectUpVectorVariance, gss->scenes[i].accel);
		survive_optimizer_setup_pose_n(&mpfitctx, &gss->scenes[i].pose, i, false, useJacobians);//d->use_jacobian_function_lh);

		LinmathPoint3d err_up;
		quatrotatevector(err_up, gss->scenes[i].pose.Rot, gss->scenes[i].accel);
		normalize3d(err_up, err_up);

		SV_VERBOSE(10, "Scene with pose (%s) " SurvivePose_format " %6.4f", mpfitctx.sos[i]->codename,
				   SURVIVE_POSE_EXPAND(gss->scenes[i].pose), fabs(err_up[2] - 1.));
		for (int j = 0; j < gss->scenes[i].meas_cnt; j++) {
			survive_optimizer_measurement *meas =
				survive_optimizer_emplace_meas(&mpfitctx, survive_optimizer_measurement_type_light);
			meas->light.object = i;
			meas->variance = d->sensor_variance_cal;
			meas->light.value = gss->scenes[i].meas[j].value;
			meas->light.lh = gss->scenes[i].meas[j].lh;
			meas->light.axis = gss->scenes[i].meas[j].axis;
			lh_meas[meas->light.lh][meas->light.axis]++;
			meas->light.sensor_idx = gss->scenes[i].meas[j].sensor_idx;
			meas->invalid = false;

			variance_measure_add(&lh_meas_variance[meas->light.lh * 2 + meas->light.axis], &meas->light.value);
			meas++;
		}
	}

	for (int i = 0; i < ctx->activeLighthouses; i++) {
		if (lh_meas[i] > 0)
			SV_VERBOSE(10, "%d %d Measurements for %d", (int)lh_meas[i][0], (int)lh_meas[i][1], i);

		if (lh_meas[i][0] < 5 || lh_meas[i][1] < 5) {
			lh_meas[i][1] = lh_meas[i][0] = 0;
			survive_optimizer_fix_camera(&mpfitctx, i);
			survive_optimizer_remove_data_for_lh(&mpfitctx, i);
		} else {
			if (!ctx->bsd[i].PositionSet) {
				memset(survive_optimizer_get_camera(&mpfitctx)[i].Rot, 0, sizeof(FLT) * 4);
			}
			SV_VERBOSE(10, "Scene with lh (%d) " SurvivePose_format, i,
					   SURVIVE_POSE_EXPAND(*survive_get_lighthouse_position(ctx, i)));

			survive_optimizer_set_cam_up_vector(&mpfitctx, i, d->lh_up_variance, ctx->bsd[i].accel);
		}
	}

	int worldEstablishedLh = survive_get_ctx_reference_bsd(ctx);

	int bestObjForCal = -1;
	int bestObjForCalSensorCnt = 0;
	for (int scene = 0; scene < gss->scenes_cnt; scene++) {
		if (gss->scenes[scene].so->sensor_ct > bestObjForCalSensorCnt) {
			bestObjForCal = scene;
			bestObjForCalSensorCnt = (int)gss->scenes[scene].so->sensor_ct;
		}
	}

	// in some cases my setup did not found a bestObjForCal...
	// maybe my Controller was just hidden
	if (bestObjForCal == -1) {
		SV_VERBOSE(10, "No best object for fixing found");
		return false;
	}

	SV_VERBOSE(10, "Best object for fixing was %s (%d)", survive_colorize(gss->scenes[bestObjForCal].so->codename),
			   bestObjForCal);
	SurvivePose *opt_cameras = survive_optimizer_get_camera(&mpfitctx);

	if (quatiszero(survive_optimizer_get_pose(&mpfitctx)[bestObjForCal].Rot)) {
		const FLT up[3] = {0, 0, 1};
		quatfrom2vectors(survive_optimizer_get_pose(&mpfitctx)[bestObjForCal].Rot, gss->scenes[bestObjForCal].accel,
						 up);
	}

	if (worldEstablishedLh != -1) {
		survive_optimizer_fix_cam_pos(&mpfitctx, worldEstablishedLh);
		survive_optimizer_fix_cam_yaw(&mpfitctx, worldEstablishedLh);
		SV_VERBOSE(10, "Locking LH %d", worldEstablishedLh);
	} else {
		for (int i = 0; i < 3; i++) {
			mpfitctx.mp_parameters_info[bestObjForCal * 7 + i].fixed = true;
		}
		survive_optimizer_fix_obj_yaw(&mpfitctx, bestObjForCal);
		SV_VERBOSE(10, "Locking Obj %d", bestObjForCal);
	}

	bool updates = true;
	while (updates) {
		updates = false;

		for (int s = 0; s < scenes_cnt; s++) {
			SurviveObject *so = gss->scenes[s].so;

			struct global_data gd = {.camera = survive_optimizer_get_camera(&mpfitctx),
									 .pose = survive_optimizer_get_pose(&mpfitctx) + s};

			if (quatiszero(gd.camera->Rot) && quatiszero(gd.pose->Rot))
				continue;

			struct PoserDataGlobalScene scene = gss->scenes[s];
			memcpy(&scene.pose, gd.pose, sizeof(SurvivePose));

			struct PoserDataGlobalScenes seed_gss = {.scenes_cnt = 1,
													 .scenes = &scene,
													 .world2lhs = gd.camera,
													 .hdr = {.pt = POSERDATA_GLOBAL_SCENES,
															 .lighthouseposeproc = global_lh_pose,
															 .poseproc = global_pose,
															 .userdata = &gd}};
			if (d->opt.seed_poser) {
				d->opt.seed_poser(so, &seed_gss.hdr);
			}

			updates |= gd.updated;
		}
	}

	for (int i = 0; i < ctx->activeLighthouses; i++) {
		if (quatiszero(survive_optimizer_get_camera(&mpfitctx)[i].Rot)) {
			// survive_optimizer_fix_camera(&mpfitctx, i);
			if (lh_meas[i][0] < 5 || lh_meas[i][1] < 5) {
				lh_meas[i][0] = lh_meas[i][1] = 0;
				survive_optimizer_fix_camera(&mpfitctx, i);
				survive_optimizer_remove_data_for_lh(&mpfitctx, i);
				SV_VERBOSE(10, "No data for %d", i);
			} else {
				SurvivePose initial_guess = {.Pos = {0, 0, 10}, .Rot = {1, 0, 0, 0}};
				survive_optimizer_get_camera(&mpfitctx)[i] = initial_guess;
				SV_VERBOSE(10, "No estimate for %d", i);
			}
		}
	}

	mp_result result = {0};
	mpfitctx.cfg = survive_optimizer_precise_config();

	survive_release_ctx_lock(ctx);
	CN_CREATE_STACK_MAT(R_free, (scenes_cnt + ctx->activeLighthouses) * 7, (scenes_cnt + ctx->activeLighthouses) * 7);
	int res = survive_optimizer_run(&mpfitctx, &result, &R_free);
	CN_CREATE_STACK_MAT(R, (scenes_cnt + ctx->activeLighthouses) * 7, (scenes_cnt + ctx->activeLighthouses) * 7);
	survive_optimizer_covariance_expand(&mpfitctx, &R_free, &R);

	survive_get_ctx_lock(ctx);
	survive_recording_write_matrix(ctx->recptr, 0, 5, "GSS", &R);
	bool status_failure = res <= 0;
	FLT sensor_covariance = d->sensor_variance * d->sensor_variance;
	FLT sensor_error = sqrtf(mpfitctx.stats.sensor_error / mpfitctx.stats.sensor_error_cnt);
	if (status_failure || sensor_error > d->opt.max_cal_error) {
		SV_WARN("MPFIT status failure %f/%f/%f (%d measurements, %d, %s)", result.orignorm, result.bestnorm,
				sensor_error, (int)mpfitctx.measurementsCnt, res, survive_optimizer_error(res));

		return false;
	} else {
		SV_INFO("MPFIT success %f/%10.10f/%7.7f (%d measurements, %d, %s, %d iters, up err %7.7f, trace %7.7f)",
				result.orignorm, result.bestnorm, sensor_error, (int)mpfitctx.measurementsCnt, res,
				survive_optimizer_error(res), result.niter,
				mpfitctx.stats.object_up_error / mpfitctx.stats.object_up_error_cnt, cn_trace(&R) / R.rows);

		print_stats_results(ctx, &mpfitctx, &result, 100);

		SurvivePose *opt_cameras = survive_optimizer_get_camera(&mpfitctx);
		SurvivePose cameras[NUM_GEN2_LIGHTHOUSES] = {0};

		for (int i = 0; i < mpfitctx.cameraLength; i++) {
			if (!quatiszero(opt_cameras[i].Rot) && lh_meas[i][0] > 0 && lh_meas[i][1] > 0) {
				cameras[i] = InvertPoseRtn(&opt_cameras[i]);

				LinmathPoint3d up = {ctx->bsd[i].accel[0], ctx->bsd[i].accel[1], ctx->bsd[i].accel[2]};
				normalize3d(up, up);
				LinmathPoint3d err;
				quatrotatevector(err, cameras[i].Rot, up);
				SV_INFO("Global solve with %d scenes for %d with error of %f/%10.10f (acc err %5.4f)", (int)scenes_cnt,
						i, result.orignorm, result.bestnorm, fabs(err[2] - 1));
				FLT v1 = 0, v2 = 0;
				variance_measure_calc(&lh_meas_variance[i * 2], &v1);
				variance_measure_calc(&lh_meas_variance[i * 2 + 1], &v2);
			}
		}

		if (worldEstablishedLh == -1) {
			int ref = survive_get_reference_bsd(ctx, cameras, mpfitctx.cameraLength);
			if (ref == -1)
				return false;

			SurvivePose reflh2objUp = cameras[ref];
			FLT ang = atan2(reflh2objUp.Pos[1], reflh2objUp.Pos[0]);
			FLT ang_target = LINMATHPI / 2.;
			FLT euler[3] = {0, 0, ang_target - ang};
			SurvivePose objUp2World = {0};
			quatfromeuler(objUp2World.Rot, euler);

			bool centerOnLh0 = survive_configi(ctx, "center-on-lh0", SC_GET, 0);
			if(centerOnLh0) {
				SurvivePose offset = { .Rot = {1} };
				scalend(offset.Pos, reflh2objUp.Pos, -1, 3);
				ApplyPoseToPose(&objUp2World, &objUp2World, &offset);
			}

			for (int i = 0; i < mpfitctx.cameraLength; i++) {
				ApplyPoseToPose(&cameras[i], &objUp2World, &cameras[i]);
			}
			for (int i = 0; i < mpfitctx.poseLength; i++) {
				SurvivePose *p = &survive_optimizer_get_pose(&mpfitctx)[i];
				ApplyPoseToPose(p, &objUp2World, p);
			}
		}

		int camera_idx = survive_optimizer_get_camera_index(&mpfitctx);
		CnMat LH_Rs = cnMatView(ctx->activeLighthouses * 7, ctx->activeLighthouses * 7, &R, camera_idx, camera_idx);
		PoserData_lighthouse_poses_func(0, mpfitctx.sos[0], cameras, &LH_Rs, ctx->activeLighthouses,
										&survive_optimizer_get_pose(&mpfitctx)[bestObjForCal]);

		for (int i = 0; i < mpfitctx.poseLength; i++) {
			SurvivePose *p = &survive_optimizer_get_pose(&mpfitctx)[i];

			memcpy(&gss->scenes[i].pose, p, sizeof(SurvivePose));

			LinmathPoint3d err_up;
			quatrotatevector(err_up, p->Rot, gss->scenes[i].accel);
			normalize3d(err_up, err_up);

			CnMat OBJ_Rs = cnMatView(7, 7, &R, i * 7, i * 7);
			FLT v[7] = {0};
			cn_get_diag(&OBJ_Rs, v, 7);
			SV_VERBOSE(10, "Solved scene with pose (%s) " SurvivePose_format " %5.4f Var: " Point7_format,
					   survive_colorize_codename(mpfitctx.sos[i]), SURVIVE_POSE_EXPAND(*p), fabs(err_up[2] - 1),
					   LINMATH_VEC7_EXPAND(v));

			if (!quatiszero(p->Rot)) {
				survive_recording_write_to_output(ctx->recptr, "SPHERE %s_%d %f %d " Point3_format "\n",
												  gss->scenes[i].so->codename, (int)gss->scenes_cnt, .05, 0xFF,
												  LINMATH_VEC3_EXPAND(p->Pos));
			}
		}

		for (int i = 0; i < mpfitctx.cameraLength; i++) {
			SV_VERBOSE(10, "Solved scene with lh %d " SurvivePose_format, i, SURVIVE_POSE_EXPAND(cameras[i]));
		}
		for (int i = 0; i < ctx->objs_ct; i++) {
			// survive_kalman_tracker_lost_tracking(ctx->objs[i]->tracker);
		}
	}

	return true;
}
int PoserMPFIT(SurviveObject *so, PoserData *pd) {
	void **user = survive_object_plugin_data(so, PoserMPFIT);
	SurviveContext *ctx = so->ctx;
	if (*user == 0 && pd->pt == POSERDATA_DISASSOCIATE) {
		return 0;
	}
	if (*user == 0) {
		*user = SV_CALLOC(sizeof(MPFITData));
		g.instances++;
		MPFITData *d = *user;

		general_optimizer_data_init(&d->opt, so);

		d->alwaysPrecise = (bool)survive_configi(ctx, "precise", SC_GET, 0);
		d->useStationaryWindow = (bool)survive_configi(ctx, USE_STATIONARY_SENSOR_WINDOW_TAG, SC_GET, 1);

		d->syncs_to_setup = 16;
		d->required_meas = survive_configi(ctx, "required-meas", SC_GET, 8);
		d->syncs_per_run = survive_configi(ctx, "syncs-per-run", SC_GET, 1);
		d->sensor_time_window = survive_configi(ctx, "time-window", SC_GET, SurviveSensorActivations_default_tolerance);
		d->use_jacobian_function_obj = survive_configi(ctx, "use-jacobian-function", SC_GET, 1);
		d->use_jacobian_function_lh = survive_configi(ctx, "use-jacobian-function-lh", SC_GET, 1);
		if (d->use_jacobian_function_lh * d->use_jacobian_function_obj < 0) {
			d->use_jacobian_function_lh = d->use_jacobian_function_obj = -1;
			SV_WARN("MPFit doesn't properly support having some analytical parameters and debug parameters. Setting "
					"all to debug.");
		}
		d->serialize_prefix = survive_configs(ctx, "serialize-lh-mpfit", SC_GET, 0);
		survive_attach_configi(ctx, "disable-lighthouse", &d->disable_lighthouse);
		survive_attach_configf(ctx, "sensor-variance-per-sec", &d->sensor_variance_per_second);
		survive_attach_configf(ctx, "sensor-variance", &d->sensor_variance);

#ifdef DEBUG_NAN
		feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif
		MPFITData_attach_config(ctx, d);
        survive_optimizer_settings_attach_config(ctx, &d->optimizer_settings);

		SV_VERBOSE(110, "Initializing MPFIT:");
		SV_VERBOSE(110, "\trequired-meas: %d", d->required_meas);
		SV_VERBOSE(110, "\ttime-window: %d", d->sensor_time_window);
		SV_VERBOSE(110, "\tsensor-variance: %f", d->sensor_variance);
		SV_VERBOSE(110, "\tsensor-variance-per-sec: %f", d->sensor_variance_per_second);
		SV_VERBOSE(110, "\tuse-jacobian-function: %d", d->use_jacobian_function_obj);
	}

	MPFITData *d = *user;
	switch (pd->pt) {
	case POSERDATA_GLOBAL_SCENES: {
		d->globalDataAvailable = true;
		PoserDataGlobalScenes *gs = (PoserDataGlobalScenes *)pd;
		return solve_global_scene(ctx, d, gs) ? 0 : -1;
	}
	case POSERDATA_SYNC_GEN2:
	case POSERDATA_SYNC: {
		// No poses if calibration is ongoing
		d->syncs_seen++;
		if (d->syncs_seen < d->syncs_to_setup) {
			return 0;
		}
		SurviveSensorActivations *scene = &so->activations;
		PoserDataLight *lightData = (PoserDataLight *)pd;
		SurvivePose estimate = {0};

		FLT error = -1;
		if (++d->syncs_per_run_cnt >= d->syncs_per_run) {
			d->syncs_per_run_cnt = 0;
			CN_CREATE_STACK_MAT(R, 7 * 4, 7 * 4);
			bool useCovariance = survive_configf(ctx, MPFIT_FULL_COV_TAG, SC_GET, 1.);
			error = run_mpfit_find_3d_structure(d, lightData, scene, &estimate, useCovariance ? &R : 0);
			handle_results(d, lightData, error, &estimate, useCovariance ? &R : 0);
		}
		return 0;
	}

	case POSERDATA_DISASSOCIATE: {
		SV_INFO("MPFIT stats for %s:", so->codename);
		if (ctx->log_level > 5) {
			print_stats(ctx, &d->stats);

			if (d->async_optimizer) {
				SV_INFO("\tjobs submitted     %lu", d->async_optimizer->submitted);
				SV_INFO("\tjobs completed     %lu", d->async_optimizer->completed);
			}
		}

		g.stats.total_lh_cnt += d->stats.total_lh_cnt;
		g.stats.dropped_lh_cnt += d->stats.dropped_lh_cnt;
		g.stats.total_meas_cnt += d->stats.total_meas_cnt;
		g.stats.dropped_meas_cnt += d->stats.dropped_meas_cnt;
		g.stats.total_fev += d->stats.total_fev;
		g.stats.total_runs += d->stats.total_runs;
		g.stats.sum_errors += d->stats.sum_errors;
		g.stats.meas_failures += d->stats.meas_failures;
		g.stats.total_iterations += d->stats.total_iterations;
		g.stats.sum_origerrors += d->stats.sum_origerrors;
		for (int i = 0; i < sizeof(d->stats.status_cnts) / sizeof(int); i++) {
			g.stats.status_cnts[i] += d->stats.status_cnts[i];
		}

		g.instances--;
		if (ctx->log_level >= 1) {
			if (g.instances == 0) {
				SV_INFO("MPFIT overall stats:");
				print_stats(ctx, &g.stats);
			}
		}
		general_optimizer_data_dtor(&d->opt);
		MPFITData_detach_config(ctx, d);
        survive_optimizer_settings_detach_config(ctx, &d->optimizer_settings);
		survive_detach_config(ctx, "disable-lighthouse", &d->disable_lighthouse);
		survive_detach_config(ctx, "sensor-variance-per-sec", &d->sensor_variance_per_second);
		survive_detach_config(ctx, "sensor-variance", &d->sensor_variance);
		survive_async_free(d->async_optimizer);
		*user = 0;
		free(d);
		return 0;
	}
	case POSERDATA_IMU: {
		PoserDataIMU *imu = (PoserDataIMU *)pd;
		if (d->stats.total_runs == 0)
			return 0;

		general_optimizer_data_record_imu(&d->opt, imu);
	}
	}
	return -1;
}

REGISTER_POSER(PoserMPFIT)
