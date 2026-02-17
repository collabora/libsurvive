#include "survive_kalman_tracker.h"
#include "generated/imu_model.gen.h"
#include "generated/kalman_kinematics.gen.h"
#include "linmath.h"
#include "math.h"
#include "survive_internal.h"

#include <cnkalman/kalman.h>

#include <assert.h>
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif
#include <memory.h>
#include <stddef.h>
#include <survive_reproject.h>
#include <survive_reproject_gen2.h>

#include "generated/lighthouse_model.gen.h"

#include "generated/survive_reproject.aux.generated.h"
#include "survive_kalman_lighthouses.h"
#include "survive_recording.h"

#define SURVIVE_MODEL_MAX_STATE_CNT (sizeof(SurviveKalmanModel) / sizeof(FLT))

// clang-format off
STRUCT_CONFIG_SECTION(SurviveKalmanTracker)
	STRUCT_CONFIG_ITEM("light-error-threshold",  "Error limit to invalidate position",
					   -1., t->light_error_threshold)
	STRUCT_CONFIG_ITEM("min-report-time",
					   "Minimum kalman report time in s (-1 defaults to 1. / imu_hz)", -1., t->min_report_time)
	STRUCT_CONFIG_ITEM("report-covariance", "Report covariance matrix every n poses", -1, t->report_covariance_cnt);
	STRUCT_CONFIG_ITEM("report-sampled-cloud", "Show sample cloud from covariance", false, t->report_sampled_cloud);

	STRUCT_CONFIG_ITEM("report-ignore-start",  "Number of reports to ignore at startup", 0, t->report_ignore_start)
	STRUCT_CONFIG_ITEM("report-ignore-threshold",
					   "Minimum variance to report pose from the kalman filter", 1e-1, t->report_threshold_var)
	STRUCT_CONFIG_ITEM("light-ignore-threshold",
					   "Minimum variance to allow light data into the kalman filter", 1., t->light_threshold_var)
	STRUCT_CONFIG_ITEM("light-required-obs",
					   "Minimum observations to allow light data into the kalman filter", 16, t->light_required_obs)

    STRUCT_CONFIG_ITEM("light-max-error",  "Maximum error to integrate into lightcap", -1, t->lightcap_max_error)
    STRUCT_CONFIG_ITEM("kalman-light-variance",  "Variance of raw light sensor readings", 1e-2, t->light_var)
    STRUCT_CONFIG_ITEM("obs-cov-scale",  "Covariance matrix scaling for obs",
                       1, t->obs_cov_scale)
	STRUCT_CONFIG_ITEM("kalman-obs-axisangle",  "Process observation updates as axis angle poses", false, t->obs_axisangle_model)
    STRUCT_CONFIG_ITEM("obs-pos-variance",  "Variance of position integration from light capture",
					   1e-6, t->obs_pos_var)
	STRUCT_CONFIG_ITEM("obs-rot-variance",  "Variance of rotation integration from light capture",
					   1e-7, t->obs_rot_var)

	STRUCT_CONFIG_ITEM("use-raw-obs",  "If true; the raw position from the solver is used and no filtering is applied", 0, t->use_raw_obs)

	STRUCT_CONFIG_ITEM("show-raw-obs", "Show position of raw poser output", 0, t->show_raw_obs)

	STRUCT_CONFIG_ITEM("light-error-for-lh-confidence",
					   "Whether or not to invalidate LH positions based on kalman errors", 0, t->use_error_for_lh_pos)

	STRUCT_CONFIG_ITEM("process-weight-jerk", "Jerk variance per second", 1874161, t->params.process_weight_jerk)
	STRUCT_CONFIG_ITEM("process-weight-acc", "Acc variance per second", 0, t->params.process_weight_acc)
	STRUCT_CONFIG_ITEM("process-weight-ang-vel", "Angular velocity variance per second", 60,
					   t->params.process_weight_ang_velocity)
	STRUCT_CONFIG_ITEM("process-weight-vel", "Velocity variance per second", 0, t->params.process_weight_vel)
	STRUCT_CONFIG_ITEM("process-weight-pos", "Position variance per second", 0, t->params.process_weight_pos)
	STRUCT_CONFIG_ITEM("process-weight-rot", "Rotation variance per second", 0, t->params.process_weight_rotation)
	STRUCT_CONFIG_ITEM("process-weight-acc-bias", "Acc bias variance per second", 1e-8, t->params.process_weight_acc_bias)
	STRUCT_CONFIG_ITEM("process-weight-gyro-bias", "Gyro bias variance per second", 1e-8, t->params.process_weight_gyro_bias)
	STRUCT_CONFIG_ITEM("kalman-minimize-state-space", "Minimize the state space", 1, t->minimize_state_space)
	STRUCT_CONFIG_ITEM("kalman-use-error-space", "Model using error state", true, t->use_error_state)

	STRUCT_CONFIG_ITEM("kalman-joint-model-lightcap", "Ratio of confidence of LH over tracked object to use joint filter", -1, t->joint_lightcap_ratio)
	STRUCT_CONFIG_ITEM("kalman-joint-lightcap-minimum-sensors", "Minimum number of sensors for the joint model to run", 5, t->joint_min_sensor_cnt)
	STRUCT_CONFIG_ITEM("kalman-lightcap-minimum-sensors", "Minimum number of sensors for the lightcap model to run", 5, t->lightcap_min_sensor_cnt)

	STRUCT_CONFIG_ITEM("kalman-initial-imu-variance", "Initial variance in IMU frame", 0, t->params.initial_variance_imu_correction)
	STRUCT_CONFIG_ITEM("kalman-initial-acc-scale-variance", "Initial variance in IMU frame", 1e-6, t->params.initial_acc_scale_variance)
	STRUCT_CONFIG_ITEM("kalman-initial-acc-bias-variance", "Initial variance in IMU frame", 1e-6, t->params.initial_acc_bias_variance)
	STRUCT_CONFIG_ITEM("kalman-initial-gyro-variance", "Initial variance in gyro", 1e-6, t->params.initial_gyro_variance)

	STRUCT_CONFIG_ITEM("kalman-zvu-moving", "", -1, t->zvu_moving_var)
	STRUCT_CONFIG_ITEM("kalman-zvu-stationary", "", 1e-5, t->zvu_stationary_var)
	STRUCT_CONFIG_ITEM("kalman-zvu-no-light", "", 1e-4, t->zvu_no_light_var)
	STRUCT_CONFIG_ITEM("kalman-zvu-no-light-time", "", .1, t->zvu_no_light_time)

	STRUCT_CONFIG_ITEM("kalman-noise-model", "0 is jerk acceleration model, 1 is simple model", 0, t->noise_model)

	STRUCT_CONFIG_ITEM("imu-acc-norm-penalty", "Penalty to IMU variance when reading high accel values", 0, t->acc_norm_penalty)
	STRUCT_CONFIG_ITEM("imu-acc-variance", "Variance of accelerometer", 1e-3, t->acc_var)
	STRUCT_CONFIG_ITEM("imu-gyro-variance", "Variance of gyroscope", 0.0000304617, t->gyro_var)

	STRUCT_CONFIG_ITEM("light-batch-size", "", 32, t->light_batchsize)
END_STRUCT_CONFIG_SECTION(SurviveKalmanTracker)

// clang-format off

MEAS_MDL_CONFIG(obj, obs, 10, -1)
MEAS_MDL_CONFIG(obj, imu, 0, -1)
MEAS_MDL_CONFIG(obj, zvu, 0, 0)
MEAS_MDL_CONFIG(obj, lightcap, 10, .1)
MEAS_MDL_CONFIG(joint, lightcap, 10, .1)

static inline void integrate_variance_tracker(SurviveKalmanTracker *tracker, struct variance_tracker* vtracker, const FLT* v, size_t size) {
	bool isStationary = SurviveSensorActivations_stationary_time(&tracker->so->activations) > 4800000;
	if(!isStationary) {
		variance_tracker_reset(vtracker);
	} else {
		variance_tracker_add(vtracker, v, size);
	}
}

FLT pid_update(struct pid_t* pid, FLT err, FLT dt) {
	FLT der = err - pid->err;
	pid->integration += err;
	FLT output = pid->Kp * err + (pid->Ki * pid->integration * dt) + (pid->Kd * der /dt);
	pid->err = err;
	return output;
}

static SurviveKalmanModel copy_model(const FLT *src, size_t state_size) {
	SurviveKalmanModel rtn = {
		.IMUBias = {
				.IMUCorrection = { 1. },
				.AccScale = { 1., 1, 1 }
		}
	};
	assert(state_size >= 7 && state_size * sizeof(FLT) <= sizeof(SurviveKalmanModel));
	memcpy(rtn.Pose.Pos, src, sizeof(FLT) * state_size);
    quatnormalize(rtn.Pose.Rot, rtn.Pose.Rot);
	quatnormalize(rtn.IMUBias.IMUCorrection, rtn.IMUBias.IMUCorrection);
	return rtn;
}
static SurviveJointKalmanModel copy_joint_model(const FLT *src, size_t state_size) {
	assert(state_size >= 14);
	SurviveJointKalmanModel rtn = {
		0
	};
	memcpy(&rtn, src, sizeof(FLT) * state_size);
	quatnormalize(rtn.Lighthouse.Rot, rtn.Lighthouse.Rot);
	return rtn;
}
static SurviveKalmanErrorModel copy_error_model(const CnMat* src) {
	SurviveKalmanErrorModel rtn = {
		0
	};
	assert(src->rows >= 7);
	memcpy(rtn.Pose.Pos, src->data, sizeof(FLT) * src->rows);
	return rtn;
}

static inline FLT survive_kalman_tracker_position_var2(SurviveKalmanTracker *tracker, FLT *var_diag, size_t cnt) {
	FLT _var_diag[SURVIVE_MODEL_MAX_STATE_CNT] = {0};
	if (var_diag == 0)
		var_diag = _var_diag;

	for (int i = 0; i < cnt; i++) {
		var_diag[i] = cnMatrixGet(&tracker->model.P, i, i);
	}

	return normnd2(var_diag, cnt);
}

void kalman_model_normalize(void *user, struct CnMat* x) {
    SurviveKalmanModel state = copy_model(x->data, x->rows);
    quatnormalize(state.Pose.Rot, state.Pose.Rot);
    quatnormalize(state.IMUBias.IMUCorrection, state.IMUBias.IMUCorrection);
    memcpy(x->data, &state, sizeof(FLT) * x->rows);
}
static void normalize_model(SurviveKalmanTracker *pTracker) {
	/*
	FLT d = magnitude3d(pTracker->state.Pose.Rot + 1);
	pTracker->state.Pose.Rot[0] = 0;
	if(d < 1) {
		pTracker->state.Pose.Rot[0] = FLT_SQRT(1 - d * d);
	}
*/
	quatnormalize(pTracker->state.Pose.Rot, pTracker->state.Pose.Rot);
	quatnormalize(pTracker->state.IMUBias.IMUCorrection, pTracker->state.IMUBias.IMUCorrection);

	for (int i = 0; i < 3; i++) {
		pTracker->state.IMUBias.AccScale[i] = linmath_enforce_range(pTracker->state.IMUBias.AccScale[i], .95, 1.05);
		pTracker->state.IMUBias.GyroBias[i] = linmath_enforce_range(pTracker->state.IMUBias.GyroBias[i], -1e-1, 1e-1);
		pTracker->state.IMUBias.AccBias[i] = linmath_enforce_range(pTracker->state.IMUBias.AccBias[i], -1e-1, 1e-1);
	}
	for (int i = 0; i < 3; i++) {
		assert(isfinite(pTracker->state.Pose.Pos[i]));
	}
	for (int i = 0; i < 4; i++) {
		assert(isfinite(pTracker->state.Pose.Rot[i]));
	}
}

struct map_light_data_ctx {
	SurviveKalmanTracker *tracker;
};

typedef void (*SurviveJointKalmanModel_LightMeas_jac_x0_with_hx)(CnMat* Hx, CnMat* hx, const FLT dt, const SurviveJointKalmanModel * _x0, const FLT* sensor_pt, const BaseStationCal* bsc0);
typedef void (*SurviveJointKalmanErrorModel_LightMeas_jac_x0_with_hx)(CnMat* Hx, CnMat* hx, const FLT dt, const SurviveJointKalmanModel * _x0, const SurviveJointKalmanErrorModel* error_model, const FLT* sensor_pt);


const static SurviveJointKalmanErrorModel zero_error_joint_model = { 0 };
/*
SurviveJointKalmanModel_LightMeas_jac_x0_with_hx SurviveJointKalmanModel_LightMeas_jac_x0_with_hx_fns[2][2] = {
	{SurviveJointKalmanModel_LightMeas_x_gen1_jac_x0_with_hx, SurviveJointKalmanModel_LightMeas_y_gen1_jac_x0_with_hx},
	{SurviveJointKalmanModel_LightMeas_x_gen2_jac_x0_with_hx, SurviveJointKalmanModel_LightMeas_y_gen2_jac_x0_with_hx},
};
 */
SurviveJointKalmanErrorModel_LightMeas_jac_x0_with_hx SurviveJointKalmanErrorModel_LightMeas_jac_x0_with_hx_fns[2][2] = {
	{SurviveJointKalmanErrorModel_LightMeas_x_gen1_jac_error_model_with_hx, SurviveJointKalmanErrorModel_LightMeas_y_gen1_jac_error_model_with_hx},
	{SurviveJointKalmanErrorModel_LightMeas_x_gen2_jac_error_model_with_hx, SurviveJointKalmanErrorModel_LightMeas_y_gen2_jac_error_model_with_hx},
};

typedef void (*SurviveKalmanModel_LightMeas_jac_x0_with_hx)(CnMat* Hx, CnMat* hx, const FLT dt, const SurviveKalmanModel* _x0, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0);
typedef void (*SurviveKalmanErrorModel_LightMeas_jac_x0_with_hx)(CnMat* Hx, CnMat* hx, const FLT dt, const SurviveKalmanModel* _x0, const SurviveKalmanErrorModel* error_model, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0);

const static SurviveKalmanErrorModel zero_error_model = { 0 };
SurviveKalmanModel_LightMeas_jac_x0_with_hx SurviveKalmanModel_LightMeas_jac_x0_with_hx_fns[2][2] = {
	{SurviveKalmanModel_LightMeas_x_gen1_jac_x0_with_hx, SurviveKalmanModel_LightMeas_y_gen1_jac_x0_with_hx},
	{SurviveKalmanModel_LightMeas_x_gen2_jac_x0_with_hx, SurviveKalmanModel_LightMeas_y_gen2_jac_x0_with_hx},
};
SurviveKalmanErrorModel_LightMeas_jac_x0_with_hx SurviveKalmanErrorModel_LightMeas_jac_x0_with_hx_fns[2][2] = {
	{SurviveKalmanErrorModel_LightMeas_x_gen1_jac_error_model_with_hx, SurviveKalmanErrorModel_LightMeas_y_gen1_jac_error_model_with_hx},
	{SurviveKalmanErrorModel_LightMeas_x_gen2_jac_error_model_with_hx, SurviveKalmanErrorModel_LightMeas_y_gen2_jac_error_model_with_hx},
};

static bool map_joint_light_data(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
						   struct CnMat *H_k) {
	struct map_light_data_ctx *cbctx = (struct map_light_data_ctx *)user;
	const SurviveKalmanTracker *tracker = cbctx->tracker;
	SurviveObject *so = tracker->so;
	struct SurviveContext *ctx = tracker->so->ctx;

    struct SurviveJointKalmanModel s = copy_joint_model(cn_as_const_vector(x_t), x_t->rows);

	if(H_k) {
		cn_set_zero(H_k);
	}

	CN_CREATE_STACK_VEC(h_x, 1);
	FLT *Y = cn_as_vector(y);
	for (int i = 0; i < Z->rows; i++) {
		const LightInfo *info = &tracker->savedLight[tracker->savedLight_idx + i];
		int axis = info->axis;

		assert(ctx->bsd[info->lh].PositionSet);

		const FLT *pt = &so->sensor_locations[info->sensor_idx * 3];
		SurvivePose imu2trackref = so->imu2trackref;
		LinmathPoint3d ptInObj = { 0 };
		gen_scale_sensor_pt(ptInObj, pt, &imu2trackref, so->sensor_scale);

		FLT t = info->timecode / 48000000. - cbctx->tracker->model.t;
		t = 0;
		CnMat H_k_row = {0};
		if(H_k) {
			H_k_row = cnMatView(1, H_k->cols, H_k, i, 0);
		}
		if(cbctx->tracker->lightcap_model.error_state_model && cbctx->tracker->use_error_state) {
			//assert(H_k == 0 || (cbctx->tracker->model.error_state_size + 6) == H_k->cols);
			SurviveJointKalmanErrorModel_LightMeas_jac_x0_with_hx_fns[ctx->lh_version][info->axis](H_k ? &H_k_row : 0,
																							  y ? &h_x : 0, t, &s,
																								   &zero_error_joint_model,
																								   ptInObj);
		} else {
			assert(false);
			//assert(H_k == 0 || cbctx->tracker->model.state_cnt == H_k->cols);
			//SurviveJointKalmanModel_LightMeas_jac_x0_with_hx_fns[ctx->lh_version][info->axis](H_k ? &H_k_row : 0,
			//																			 y ? &h_x : 0, t, &s, ptInObj, &world2lh, &ctx->bsd[info->lh].fcal[axis]);
		}
		if(y) {
			Y[i] = cn_as_const_vector(Z)[i] - h_x.data[0];
			if(tracker->lightcap_max_error > 0) {
				Y[i] = linmath_enforce_range(Y[i], -tracker->lightcap_max_error, tracker->lightcap_max_error);
			}
			SV_DATA_LOG("h_light[%d][%d][%d]", h_x.data, 1, info->lh, info->axis, info->sensor_idx);
			SV_DATA_LOG("Y_light[%d][%d][%d]", Y, 1, info->lh, info->axis, info->sensor_idx);
		}
		SV_DATA_LOG("Z_light[%d][%d][%d]", &info->value, 1, info->lh, info->axis, info->sensor_idx);
	}

	if (H_k && !cn_is_finite(H_k))
		return false;

	survive_recording_write_matrix(tracker->so->ctx->recptr, tracker->so, 100, "light-y", y);

	return true;
}
/**
 * This function reuses the reproject functions to estimate what it thinks the lightcap angle should be based on x_t,
 * and uses that measurement to compare from the actual observed angle. These functions have jacobian functions that
 * correspond to them; see @survive_reproject.c and @survive_reproject_gen2.c
 */
static bool map_light_data(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
						   struct CnMat *H_k) {
	struct map_light_data_ctx *cbctx = (struct map_light_data_ctx *)user;

	SurviveKalmanModel s = copy_model(cn_as_const_vector(x_t), x_t->rows);

	const SurviveKalmanTracker *tracker = cbctx->tracker;

	SurviveObject *so = tracker->so;
	struct SurviveContext *ctx = tracker->so->ctx;
	const survive_reproject_model_t *mdl = survive_reproject_model(ctx);

	if(H_k) {
	    cn_set_zero(H_k);
	}
    SurvivePose obj2world = *(SurvivePose *)cn_as_const_vector(x_t);
    quatnormalize(obj2world.Rot, obj2world.Rot);

	CN_CREATE_STACK_VEC(h_x, 1);
	FLT *Y = cn_as_vector(y);
	for (int i = 0; i < Z->rows; i++) {
		const LightInfo *info = &tracker->savedLight[tracker->savedLight_idx + i];
		int axis = info->axis;

		assert(ctx->bsd[info->lh].PositionSet);

		const SurvivePose world2lh = InvertPoseRtn(survive_get_lighthouse_position(ctx, info->lh));

		const FLT *pt = &so->sensor_locations[info->sensor_idx * 3];
        SurvivePose imu2trackref = so->imu2trackref;
        LinmathPoint3d ptInObj;
        gen_scale_sensor_pt(ptInObj, pt, &imu2trackref, so->sensor_scale);

		FLT t = info->timecode / 48000000. - cbctx->tracker->model.t;
		t = 0;
		CnMat H_k_row = {0};
		if(H_k) {
			H_k_row = cnMatView(1, H_k->cols, H_k, i, 0);
		}
		if(cbctx->tracker->lightcap_model.error_state_model && cbctx->tracker->use_error_state) {
			assert(H_k == 0 || cbctx->tracker->model.error_state_size == H_k->cols);
			SurviveKalmanErrorModel_LightMeas_jac_x0_with_hx_fns[ctx->lh_version][info->axis](H_k ? &H_k_row : 0,
																							  y ? &h_x : 0, t, &s, &zero_error_model, ptInObj, &world2lh,
																							  survive_basestation_cal(ctx, info->lh, axis));
		} else {
			assert(H_k == 0 || cbctx->tracker->model.state_cnt == H_k->cols);
			SurviveKalmanModel_LightMeas_jac_x0_with_hx_fns[ctx->lh_version][info->axis](H_k ? &H_k_row : 0,
																						 y ? &h_x : 0, t, &s, ptInObj, &world2lh,
																						 survive_basestation_cal(ctx, info->lh, axis));
		}
		if(y) {
			Y[i] = cn_as_const_vector(Z)[i] - h_x.data[0];
			if(tracker->lightcap_max_error > 0) {
				Y[i] = linmath_enforce_range(Y[i], -tracker->lightcap_max_error, tracker->lightcap_max_error);
			}
			SV_DATA_LOG("h_light[%d][%d][%d]", h_x.data, 1, info->lh, info->axis, info->sensor_idx);
			SV_DATA_LOG("Y_light[%d][%d][%d]", Y, 1, info->lh, info->axis, info->sensor_idx);
		}
        SV_DATA_LOG("Z_light[%d][%d][%d]", &info->value, 1, info->lh, info->axis, info->sensor_idx);
	}

	if (H_k && !cn_is_finite(H_k))
		return false;

	survive_recording_write_matrix(tracker->so->ctx->recptr, tracker->so, 100, "light-y", y);

	return true;
}

int sort_by_lh_axis_sensor(const void * _info1, const void * _info2) {
	const LightInfo* info1 = _info1;
	const LightInfo* info2 = _info2;
	if(info1->lh != info2->lh)
		return (info1->lh > info2->lh) ? -1 : 1;
	return 0;
}


void survive_kalman_tracker_integrate_saved_light(SurviveKalmanTracker *tracker, PoserData *pd) {
	SurviveContext *ctx = tracker->so->ctx;
	FLT time = pd->timecode / (FLT)tracker->so->timebase_hz;
	if (tracker->use_raw_obs) {
		return;
	}

	// A single light cap measurement has an infinite amount of solutions along a plane; so it only helps if we are
	// already in a good place
	if (tracker->light_threshold_var > 0 &&
		survive_kalman_tracker_position_var2(tracker, 0, 7) > tracker->light_threshold_var) {
		return;
	}

	if (tracker->light_required_obs > tracker->stats.obs_count) {
		return;
	}

	if (tracker->light_var >= 0) {
		for (int i = 0; i < tracker->savedLight_idx; i++) {
			if (!ctx->bsd[tracker->savedLight[i].lh].PositionSet) {
				tracker->savedLight[i] = tracker->savedLight[tracker->savedLight_idx - 1];
				tracker->savedLight_idx--;
				i--;
			}
		}

        if (tracker->savedLight_idx == 0) {
            return;
        }

		bool useJointModel = tracker->joint_lightcap_ratio >= 0;
		if(useJointModel) {
			qsort(tracker->savedLight, tracker->savedLight_idx, sizeof(tracker->savedLight[0]), sort_by_lh_axis_sensor);
		}

		FLT rtn = 0;
		while(tracker->savedLight_idx > 0) {
			int lh = tracker->savedLight[tracker->savedLight_idx-1].lh;
			int cnt = 0;

			if(useJointModel) {
				while(tracker->savedLight_idx > 0) {
					cnt++;
					tracker->savedLight_idx--;
					if(tracker->savedLight_idx > 0 && tracker->savedLight[tracker->savedLight_idx - 1].lh != lh)
						break;
				}
				if (cnt < tracker->joint_min_sensor_cnt) {
					cnt += tracker->savedLight_idx;
					tracker->savedLight_idx = 0;
					useJointModel = false;
					tracker->stats.joint_model_dropped++;
				}
			} else {
				cnt = tracker->savedLight_idx;
				tracker->savedLight_idx = 0;
			}

			if(cnt < tracker->lightcap_min_sensor_cnt) {
				tracker->stats.lightcap_model_dropped++;
				return;
			}

			CN_CREATE_STACK_VEC(Z, cnt);
			for (int i = tracker->savedLight_idx; i < cnt + tracker->savedLight_idx; i++) {
				assert(useJointModel == false || tracker->savedLight[i].lh == lh);
				cnMatrixSet(&Z, i - tracker->savedLight_idx, 0, tracker->savedLight[i].value);
			}

			struct map_light_data_ctx cbctx = {
				.tracker = tracker,
			};

			SurviveObject *so = tracker->so;
			FLT light_var = tracker->light_var;

			SV_DATA_LOG("light_var", &light_var, 1);
			FLT light_vars[32] = {0};
			for (int i = 0; i < 32; i++)
				light_vars[i] = light_var;
			CnMat R = cnVec(Z.rows, light_vars);

			tracker->datalog_tag = "light_data";
			if(time < tracker->model.t)
				time = tracker->model.t;

			FLT obj_trace = cn_trace(&tracker->model.P);
			FLT lh_trace = cn_trace(&ctx->bsd[lh].tracker->model.P);
			FLT ratio = lh_trace / obj_trace;

			tracker->last_light_time = time;

			if(useJointModel && tracker->joint_lightcap_ratio < ratio) {
				tracker->joint_model.ks[0] = &ctx->bsd[lh].tracker->model;
				tracker->joint_model.ks[1] = &ctx->bsd[lh].tracker->bsd_model;
				rtn += cnkalman_meas_model_predict_update(time, &tracker->joint_model, &cbctx, &Z, &R);
				tracker->stats.joint_model_sensor_cnt_sum += cnt;
				survive_kalman_lighthouse_report(ctx->bsd[lh].tracker);
			} else {
				rtn += cnkalman_meas_model_predict_update(time, &tracker->lightcap_model, &cbctx, &Z, &R);
				tracker->stats.lightcap_model_sensor_cnt_sum += cnt;
			}
		}

		tracker->datalog_tag = 0;
		tracker->stats.lightcap_total_error += rtn;

		tracker->light_residuals_all *= .9;
		tracker->light_residuals_all += .1 * rtn;

		SurviveObject * so = tracker->so;
		SV_DATA_LOG("res_error_light_", &rtn, 1);
		SV_DATA_LOG("res_error_light_avg", &tracker->light_residuals_all, 1);
		tracker->stats.lightcap_count++;

		survive_kalman_tracker_report_state(pd, tracker);
	}
}

void survive_kalman_tracker_integrate_light(SurviveKalmanTracker *tracker, PoserDataLight *data) {
	bool isSync = data->hdr.pt == POSERDATA_SYNC || data->hdr.pt == POSERDATA_SYNC_GEN2;
	if (isSync) {
		survive_kalman_tracker_integrate_saved_light(tracker, &data->hdr);
		tracker->savedLight_idx = 0;
	} else {
		LightInfo *info = &tracker->savedLight[tracker->savedLight_idx++];

		info->lh = data->lh;
		info->value = data->angle;
		info->axis = PoserDataLight_axis(data);
		info->sensor_idx = data->sensor_id;
		info->timecode = data->hdr.timecode;

		integrate_variance_tracker(tracker, &tracker->light_variance[info->lh][info->sensor_idx][info->axis], &info->value, 1);
	}

	int batchtrigger = sizeof(tracker->savedLight) / sizeof(tracker->savedLight[0]);
	if (tracker->light_batchsize >= 0) {
		batchtrigger = tracker->light_batchsize;
	}
	if (tracker->savedLight_idx >= batchtrigger) {
		survive_kalman_tracker_integrate_saved_light(tracker, &data->hdr);
		tracker->savedLight_idx = 0;
	}
}

struct map_imu_data_ctx {
	bool use_gyro, use_accel;
	SurviveKalmanTracker *tracker;
};

SURVIVE_EXPORT void survive_kalman_tracker_correct_imu(SurviveKalmanTracker *tracker, LinmathVec3d out, const LinmathVec3d accel) {
    for(int i = 0;i < 3;i++) {
        out[i] = accel[i] / tracker->state.IMUBias.AccScale[i] - tracker->state.IMUBias.AccBias[i];
    }
}
/**
 * The prediction for IMU given x_t is:
 *
 * [Position, Rotation, Velocity, Ang_Velocity, Acc, Gyro_Bias] = x_t
 *
 * acc_predict  = Rotation^-1 * (Acc/9.80665 + [0, 0, 1])
 * gyro_predict = Rotation^-1 * Ang_Velocity + Gyro_Bias
 *
 * The actual code for this is generated from tools/generate_math_functions/imu_functions.py. It isn't done in
 * C natively to allow for the jacobian code to be generated using symengine
 */
bool survive_kalman_tracker_imu_measurement_model(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
						 struct CnMat *H_k) {
	struct map_imu_data_ctx *fn_ctx = user;

	CN_CREATE_STACK_VEC(h_x, 6);
	SurviveKalmanTracker *tracker = (SurviveKalmanTracker *)user;

	SurviveKalmanModel s = copy_model(cn_as_const_vector(x_t), x_t->rows);
	IMUMeasurementModel(&h_x, &s);

    if(H_k) {
        assert(H_k->rows * H_k->cols == H_k->cols * 6);

		bool errorState = fn_ctx->tracker->imu_model.error_state_model;
		if(errorState) {
			IMUMeasurementErrorModel_jac_error_model(H_k, &s, &zero_error_model);
		} else {
			IMUMeasurementModel_jac_model(H_k, &s);
		}

    }

	subnd(cn_as_vector(y), cn_as_const_vector(Z), cn_as_const_vector(&h_x), Z->rows);

	if(fn_ctx) {
		SurviveKalmanTracker * tracker = fn_ctx->tracker;
		SurviveObject * so = fn_ctx->tracker->so;
		SurviveContext *ctx = so->ctx;
		survive_recording_write_matrix(tracker->so->ctx->recptr, tracker->so, 100, "imu-y", y);
		SV_VERBOSE(600, "X     " Point7_format, LINMATH_VEC7_EXPAND(cn_as_const_vector(x_t)))
		SV_VERBOSE(600, "Z     " Point6_format, LINMATH_VEC6_EXPAND(cn_as_const_vector(Z)))

		if(so->ctx->datalogproc) {
			SV_DATA_LOG("imu_prediction", h_x.data, 6);

			LinmathVec3d up = {0, 0, 1};
			FLT q[5];
			LinmathVec3d imuWorld;
			quatrotatevector(imuWorld, tracker->state.Pose.Rot, Z->data);

			quatfrom2vectors(q, imuWorld, up);
			q[4] = norm3d(q + 1);
			quatrotateabout(q, q, tracker->state.Pose.Rot);
			SV_DATA_LOG("perfect_q", q, 5);

			LinmathVec3d perfect_acc;
			quatrotatevector(perfect_acc, q, Z->data);
			perfect_acc[2] -= 1;
			SV_DATA_LOG("perfect_acc", perfect_acc, 3);
		}
	}

	return true;
}

static void tracker_datalog(const cnkalman_state_t* state, const char *desc, const FLT *v, size_t length) {
	SurviveKalmanTracker *tracker = state->datalog_user;
	SurviveObject * so = tracker->so;

	if(tracker->datalog_tag == 0)
		tracker->datalog_tag = "unknown";

	SV_DATA_LOG("%s_%s", v, length, desc, tracker->datalog_tag);
}
static SurviveIMUBiasModel copy_imu_bias_model(const struct CnMat *x0) {
	SurviveIMUBiasModel state = { .IMUCorrection = { 1 }, .AccScale = {1., 1, 1} };
	assert(x0->rows <= sizeof(SurviveIMUBiasModel) / sizeof(FLT));
	memcpy(&state, cn_as_const_vector(x0), x0->rows * sizeof(FLT));
	return state;
}

static void imu_error_state_fn(void *user, const struct CnMat *x0,
							   const struct CnMat *x1, struct CnMat *E,
							   struct CnMat *E_jac_x) {
	SurviveIMUBiasModel state0 = copy_imu_bias_model(x0);
	if(E_jac_x) {
		SurviveIMUBiasModelToErrorModel_jac_x1(E_jac_x, &state0, &state0);
	}

	if(x1 && E) {
		SurviveIMUBiasModel state1 = copy_imu_bias_model(x1);
		SurviveIMUBiasErrorModel error_state = { 0 };
		SurviveIMUBiasModelToErrorModel(&error_state, &state1, &state0);
		memcpy(cn_as_vector(E), &error_state, sizeof(FLT) * E->rows);
	}
}

static void error_state_fn(void *user, const struct CnMat *x0,
						   const struct CnMat *x1, struct CnMat *E,
						   struct CnMat *E_jac_x) {
	SurviveKalmanModel state0 = copy_model(cn_as_const_vector(x0), x0->rows);
	if(E_jac_x) {
		SurviveKalmanModelToErrorModel_jac_x1(E_jac_x, &state0, &state0);
	}

	if(x1 && E) {
		SurviveKalmanModel state1 = copy_model(cn_as_const_vector(x1), x1->rows);
		SurviveKalmanErrorModel error_state = { 0 };
		SurviveKalmanModelToErrorModel(&error_state, &state1, &state0);
		memcpy(cn_as_vector(E), &error_state, sizeof(FLT) * E->rows);
	}
}

static void imu_bias_update_fn(void *user, const struct CnMat *x0, const struct CnMat *E, struct CnMat *x1, struct CnMat* dX_wrt_error_state) {
	SurviveIMUBiasModel state = copy_imu_bias_model(x0);
	const FLT* x0v = cn_as_const_vector(x0);
	if(x1){
		SurviveIMUBiasModel _x1 = { 0 };
		SurviveIMUBiasErrorModel error_state = { 0 };
		memcpy(&error_state, cn_as_const_vector(E), E->rows * sizeof(FLT));
		SurviveIMUBiasModelAddErrorModel(&_x1, &state, &error_state);
		memcpy(cn_as_vector(x1), &_x1, sizeof(FLT) * x1->rows);
	}
	if(dX_wrt_error_state){
		SurviveIMUBiasErrorModel error_model = { 0 };
		SurviveIMUBiasModelAddErrorModel_jac_error_state(dX_wrt_error_state, &state, &error_model);
	}
}

static void state_update_fn(void *user, const struct CnMat *x0, const struct CnMat *E, struct CnMat *x1, struct CnMat* dX_wrt_error_state) {
	SurviveKalmanModel state = copy_model(cn_as_const_vector(x0), x0->rows);
	const FLT* x0v = cn_as_const_vector(x0);
	if(x1){
		SurviveKalmanModel _x1 = { 0 };
		SurviveKalmanErrorModel error_state = copy_error_model(E);
		SurviveKalmanModelAddErrorModel(&_x1, &state, &error_state);
		memcpy(cn_as_vector(x1), &_x1, sizeof(FLT) * x1->rows);
	}
	if(dX_wrt_error_state){
		SurviveKalmanErrorModel error_model = { 0 };
		SurviveKalmanModel state = copy_model(cn_as_const_vector(x0), x0->rows);
		SurviveKalmanModelAddErrorModel_jac_error_state(dX_wrt_error_state, &state, &error_model);
	}
}
/*
static void error_state_fn(void *user, const struct CnMat *x0, struct CnMat *H) {
	cn_set_diag_val(H, 1);
}
static void state_update_fn(void *user, const struct CnMat *x0, struct CnMat *Ky, struct CnMat *x1) {
	cn_elementwise_add(x1, x0, Ky);
}
*/
static bool map_obs_data(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
								   struct CnMat *H_k) {
	SurviveKalmanTracker *tracker = (SurviveKalmanTracker *)user;
	if(y) {
		subnd(cn_as_vector(y), cn_as_const_vector(Z), cn_as_const_vector(x_t), 7);
		survive_recording_write_matrix(tracker->so->ctx->recptr, tracker->so, 100, "obs-y", y);
	}
	if(H_k) {
		bool errorState = tracker->use_error_state && tracker->obs_model.error_state_model;
		if(errorState) {
			state_update_fn(user, x_t, 0, 0, H_k);
		} else {
			cn_set_zero(H_k);
			cn_set_diag_val(H_k, 1);
		}
	}
	return true;
}
static bool map_obs_data_axisangle(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
								   struct CnMat *H_k) {
	SurviveKalmanTracker *tracker = (SurviveKalmanTracker *)user;
	const SurviveKalmanModel *x0 = (const SurviveKalmanModel *)cn_as_const_vector(x_t);

	SurviveAxisAnglePose yp = { 0 };
	SurviveAxisAnglePose predictedPose = *(const SurviveAxisAnglePose *)cn_as_const_vector(Z);
	SurviveObsErrorModelNoFlip(&yp, x0, (const SurviveAxisAnglePose *)cn_as_const_vector(Z));
	scalend((FLT *)&yp, (FLT *)&yp, -1, 6);

	FLT mag = normnd2(yp.AxisAngleRot, 3);
	bool hasFlip = mag > M_PI * M_PI;
	if(hasFlip) {
		mag = sqrt(mag);
		scalend(yp.AxisAngleRot, yp.AxisAngleRot, (mag - 2.*M_PI) / mag, 3);
	}
	assert(norm3d(yp.AxisAngleRot) < M_PI);

	if(y) {
		memcpy(cn_as_vector(y), &yp, y->rows * sizeof(FLT));
	}
    if(H_k) {
		bool errorState = tracker->use_error_state && tracker->obs_model.error_state_model;
		if(errorState) {
			SurviveKalmanErrorModel error_model = { 0 };
			(!hasFlip ?
			 SurviveObsErrorStateErrorModelNoFlip_jac_err :
			 SurviveObsErrorStateErrorModelFlip_jac_err)(H_k, x0, &error_model, &predictedPose);
		} else {
			(!hasFlip ? SurviveObsErrorModelNoFlip_jac_x0 :
			 SurviveObsErrorModelFlip_jac_x0)(H_k, x0, &predictedPose);
		}
    }
    return true;
}

static FLT integrate_pose(SurviveKalmanTracker *tracker, FLT time, const SurvivePose *pose, const struct CnMat *R_q) {
	FLT rtn = 0;

	size_t state_cnt = tracker->model.state_cnt;
	size_t obs_cnt = tracker->obs_axisangle_model ? 6 : 7;

	struct CnMat* Rp = (CnMat*)R_q;
	CN_CREATE_STACK_MAT(R, obs_cnt, obs_cnt);
	CN_CREATE_STACK_VEC(Z, obs_cnt);
	if(tracker->obs_axisangle_model) {
		LinmathAxisAnglePose poseAA = Pose2AAPose(pose);
		if(Rp) {
			survive_covariance_pose2poseAA(&R, pose, R_q);
		}
		memcpy(cn_as_vector(&Z), poseAA.Pos, obs_cnt * sizeof(FLT));
		Rp = &R;
	} else {
		memcpy(cn_as_vector(&Z), pose->Pos, obs_cnt * sizeof(FLT));
	}

	tracker->datalog_tag = "pose_obs";
    rtn = cnkalman_meas_model_predict_update(time, &tracker->obs_model, tracker, &Z, Rp);

	tracker->datalog_tag = 0;
	SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(600, "Resultant state %f (pose) " Point16_format, time,
			   LINMATH_VEC16_EXPAND(cn_as_const_vector(&tracker->model.state)));

	return rtn;
}

void survive_kalman_tracker_integrate_imu(SurviveKalmanTracker *tracker, PoserDataIMU *data) {
	SurviveContext *ctx = tracker->so->ctx;
	SurviveObject *so = tracker->so;

	FLT time = data->hdr.timecode / (FLT)tracker->so->timebase_hz;
	FLT time_diff = time - tracker->model.t;

	FLT norm = norm3d(data->accel);
	SV_DATA_LOG("acc_norm", &norm, 1);

	bool isStationary = SurviveSensorActivations_stationary_time(&tracker->so->activations) > 4800000;

	if (tracker->use_raw_obs) {
		return;
	}

	// Wait til observation is in before reading IMU; gets rid of bad IMU data at the start
	if (tracker->model.t == 0) {
		return;
	}

	if (tracker->stats.obs_count < 16 && tracker->obs_pos_var > -1) {
		return;
	}

	if (time_diff < -.01) {
		// SV_WARN("Processing imu data from the past %fs", time - tracker->rot.t);
		tracker->stats.late_imu_dropped++;
		return;
	}

	if (time_diff > 0.5) {
		SV_WARN("%s is probably dropping IMU packets; %f time reported between %" PRIu64, tracker->so->codename,
				time_diff, data->hdr.timecode);
	}

	FLT rotation_variance[] = {1e5, 1e5, 1e5, 1e5, 1e5, 1e5};

	bool no_light = (time - tracker->last_light_time) > tracker->zvu_no_light_time;
	FLT zvu_var = tracker->zvu_moving_var;
	if(isStationary && tracker->zvu_stationary_var >= 0) zvu_var = linmath_min(tracker->zvu_stationary_var, zvu_var < 0 ? INFINITY : zvu_var);
	if(no_light && tracker->zvu_no_light_var >= 0) zvu_var = linmath_min(tracker->zvu_no_light_var, zvu_var < 0 ? INFINITY : zvu_var);

	bool disable_ang_vel = no_light && !isStationary;
	tracker->stats.no_light_imu_count += no_light;

	if (zvu_var >= 0) {//time - tracker->last_light_time > .1) {//|| isStationary || fabs(1 - norm) < .001 ) {
		// If we stop seeing light data; tank all velocity / acceleration measurements
		size_t row_cnt = linmath_imin(9 - disable_ang_vel * 3, tracker->model.state_cnt - 7);
		CN_CREATE_STACK_MAT(H, row_cnt, tracker->model.state_cnt + tracker->imu_bias_model.state_cnt);
		cn_set_zero(&H);

		int vel_idx = offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT);
		int acc_idx = offsetof(SurviveKalmanModel, Acc) / sizeof(FLT);
		int idx = 0;
		for (idx = 0; idx < 3; idx++) {
			cnMatrixSet(&H, idx, vel_idx + idx, 1);
		}

		if(!disable_ang_vel) {
			for (int i = 0; i < 3; i++) {
				cnMatrixSet(&H, idx + i, vel_idx + 3 + i, 1);
			}
			idx += 3;
		}

		for (int i = 0; i < 3; i++) {
			cnMatrixSet(&H, idx + i, acc_idx + i, 1);
		}

		CN_CREATE_STACK_MAT(R, row_cnt, 1)
		cn_set_constant(&R, zvu_var);
		CN_CREATE_STACK_MAT(Z, row_cnt, 1);
		cn_set_zero(&Z);

		tracker->datalog_tag = "zvu";

		tracker->stats.imu_total_error +=
			cnkalman_meas_model_predict_update(time, &tracker->zvu_model, &H, &Z, &R);

		tracker->datalog_tag = 0;

		CN_FREE_STACK_MAT(Z);
		CN_FREE_STACK_MAT(R);
		CN_FREE_STACK_MAT(H);
	}

	struct map_imu_data_ctx fn_ctx = {.tracker = tracker};
	if (tracker->acc_var >= 0) {
		fn_ctx.use_accel = true;
		for (int i = 0; i < 3; i++) {
			rotation_variance[i] = tracker->acc_var;
			if (tracker->acc_norm_penalty > 0) {
			  FLT ndiff = 1 - norm;
			  rotation_variance[i] += tracker->acc_norm_penalty * ndiff * ndiff;
			}
		}
	}

	if (tracker->gyro_var >= 0) {
		fn_ctx.use_gyro = true;
		for (int i = 0; i < 3; i++)
			rotation_variance[3 + i] = tracker->gyro_var;
	}

	if (fn_ctx.use_gyro || fn_ctx.use_accel) {
		int rows = 6;
		int offset = 0;
		FLT accelgyro[6] = { 0 };
		copy3d(accelgyro, data->accel);
		copy3d(accelgyro+3, data->gyro);

		integrate_variance_tracker(tracker, &tracker->imu_variance, accelgyro, 6);

		CnMat Z = cnMat(rows, 1, accelgyro + offset);

		SV_VERBOSE(600, "Integrating IMU " Point6_format " with cov " Point6_format,
				   LINMATH_VEC6_EXPAND((FLT *)&accelgyro[0]), LINMATH_VEC6_EXPAND(rotation_variance));

		tracker->datalog_tag = "imu_meas";

        CnMat R = cnMat(6, tracker->imu_model.adaptive ? 6 : 1, tracker->imu_model.adaptive ? tracker->IMU_R : rotation_variance);
        FLT err = cnkalman_meas_model_predict_update(time, &tracker->imu_model, &fn_ctx, &Z, &R);
		tracker->datalog_tag = 0;

        SV_DATA_LOG("res_err_imu", &err, 1);
		tracker->stats.imu_total_error += err;
		tracker->imu_residuals *= .9;
		tracker->imu_residuals += .1 * err;

        tracker->stats.acc_norm += norm3d(data->accel);
        if(isStationary) {
            tracker->stats.stationary_acc_norm += norm3d(data->accel);
            tracker->stats.stationary_imu_count++;
        }
		tracker->stats.imu_count++;
		if (tracker->first_imu_time == 0) {
		  tracker->first_imu_time = time;
		}
		
		tracker->last_imu_time = time;

		SV_VERBOSE(600, "%s Resultant state %f (imu %e) " Point26_format, so->codename, time, tracker->imu_residuals,
				   LINMATH_VEC26_EXPAND(cn_as_const_vector(&tracker->model.state)));
	}

	survive_kalman_tracker_report_state(&data->hdr, tracker);
}

void survive_kalman_tracker_predict(const SurviveKalmanTracker *tracker, FLT t, SurvivePose *out) {
	// if (tracker->model.info.P[0] > 100 || tracker->model.info.P[0] > 100 || tracker->model.t == 0)
	//	return;

	if (tracker->model.t == 0)
		return;

	CnMat x1 = cnVec(7, out->Pos);
	cnkalman_extrapolate_state(t, &tracker->model, &x1, 0);
	quatnormalize(out->Rot, out->Rot);

	struct SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(300, "Predict pose %f %f " SurvivePose_format, t, t - tracker->model.t, SURVIVE_POSE_EXPAND(*out))
}

static void survive_kalman_tracker_process_noise_bounce(void *user, FLT t, const CnMat *x, struct CnMat *q_out) {
	struct SurviveKalmanTracker_Params *params = (struct SurviveKalmanTracker_Params *)user;
	survive_kalman_tracker_process_noise(params, false, t, x, q_out);
}

static void survive_kalman_tracker_error_process_noise_bounce(void *user, FLT t, const CnMat *x, struct CnMat *q_out) {
	struct SurviveKalmanTracker_Params *params = (struct SurviveKalmanTracker_Params *)user;
	survive_kalman_tracker_process_noise(params, true, t, x, q_out);
}

void survive_kalman_tracker_process_noise(const struct SurviveKalmanTracker_Params *params, bool errorState, FLT t, const CnMat *x, struct CnMat *q_out) {
	/*
	 * Due to the rotational terms in the model, the process noise covariance is complicated. It mixes a XYZ third order
	 * positional model with a second order rotational model with tuning parameters
	 */

	FLT t2 = t * t;
	FLT t3 = t2 * t;
	FLT t4 = t3 * t;
	FLT t5 = t4 * t;
	FLT t6 = t5 * t;
	FLT t7 = t6 * t;
	/* ================== Positional ============================== */
	// Estimation with Applications to Tracking and Navigation: Theory Algorithms and Software Ch 6
	// http://wiki.dmdevelopment.ru/wiki/Download/Books/Digitalimageprocessing/%D0%9D%D0%BE%D0%B2%D0%B0%D1%8F%20%D0%BF%D0%BE%D0%B4%D0%B1%D0%BE%D1%80%D0%BA%D0%B0%20%D0%BA%D0%BD%D0%B8%D0%B3%20%D0%BF%D0%BE%20%D1%86%D0%B8%D1%84%D1%80%D0%BE%D0%B2%D0%BE%D0%B9%20%D0%BE%D0%B1%D1%80%D0%B0%D0%B1%D0%BE%D1%82%D0%BA%D0%B5%20%D1%81%D0%B8%D0%B3%D0%BD%D0%B0%D0%BB%D0%BE%D0%B2/Estimation%20with%20Applications%20to%20Tracking%20and%20Navigation/booktext@id89013302placeboie.pdf

	// We mix three order models here based on tuning variables.
	FLT Q_jerk[] = {
		t7 / 252.,
		t6 / 72., t5 /20.,
		t5 / 30, t4 / 8., t3 / 3.
	};
	FLT Q_acc[] = {
		t5 / 20.,
		t4 / 8.,      t3 / 3.,
		t3 / 6.,      t2 / 2.,       t
	};
	FLT Q_vel[] = {
		t3 / 3.,
		t2 / 2.,       t,
	};

	FLT p_p = params->process_weight_jerk * Q_jerk[0] + params->process_weight_acc * Q_acc[0] + params->process_weight_vel * Q_vel[0] + params->process_weight_pos * t2;
	FLT p_v = params->process_weight_jerk * Q_jerk[1] + params->process_weight_acc * Q_acc[1] + params->process_weight_vel * Q_vel[1];
	FLT p_a = params->process_weight_jerk * Q_jerk[3] + params->process_weight_acc * Q_acc[3];

	FLT v_v = params->process_weight_jerk * Q_jerk[2] + params->process_weight_acc * Q_acc[2] + params->process_weight_vel * Q_vel[2];
	FLT v_a = params->process_weight_jerk * Q_jerk[4] + params->process_weight_acc * Q_acc[4];
	FLT a_a = params->process_weight_jerk * Q_jerk[5] + params->process_weight_acc * Q_acc[5];


	/* ================== Rotational ==============================
	 * 	https://www.ucalgary.ca/engo_webdocs/GL/96.20096.JSchleppe.pdf
	 *      !!! NOTE: This document uses x,y,z,w quaternions !!!
	  This is a rework using the same methodology. Some helper output functions are in the tools/generate_math_functions
	  code.
	 */
	FLT s_w = params->process_weight_ang_velocity;
	FLT rv =   params->process_weight_ang_velocity * Q_vel[0] + params->process_weight_rotation * t;
	FLT r_av = params->process_weight_ang_velocity * Q_vel[1];

	/* The gyro bias is expected to change, but slowly through time */
	FLT ga = params->process_weight_acc_bias * t;
	FLT gb = params->process_weight_gyro_bias * t;

	if(!errorState) {
		FLT s_f = s_w / 12. * t3;
		FLT s_s = s_w / 4. * t2;

		size_t state_cnt = x->rows;
		SurviveKalmanModel state = copy_model(cn_as_const_vector(x), state_cnt);

		FLT qw = state.Pose.Rot[0], qx = state.Pose.Rot[1], qy = state.Pose.Rot[2], qz = state.Pose.Rot[3];
		FLT qws = qw * qw, qxs = qx * qx, qys = qy * qy, qzs = qz * qz;
		FLT qs = qws + qxs + qys + qzs;

		FLT Q_POSE_BLOCK[] = {
	//       x        y        z                 qw                 qx                 qy                 qz         vx       vy       vz          avx      avy      avz       ax       ay      az
		  p_p,       0,       0,                 0,                 0,                 0,                 0,       p_v,       0,       0,           0,       0,       0,     p_a,       0,       0,  // x
			0,     p_p,       0,                 0,                 0,                 0,                 0,         0,     p_v,       0,           0,       0,       0,       0,     p_a,       0,  // y
			0,       0,     p_p,                 0,                 0,                 0,                 0,         0,       0,     p_v,           0,       0,       0,       0,       0,     p_a,  // z

			0,       0,       0,   rv+s_f*(qs-qws),      s_f*(-qw*qx),      s_f*(-qw*qy),      s_f*(-qw*qz),         0,       0,       0,     -s_s*qx, -s_s*qy, -s_s*qz,       0,       0,       0,  // qw
			0,       0,       0,      s_f*(-qw*qx),   rv+s_f*(qs-qxs),      s_f*(-qx*qy),      s_f*(-qx*qz),         0,       0,       0,      s_s*qw, -s_s*qz,  s_s*qy,       0,       0,       0,  // qx
			0,       0,       0,      s_f*(-qw*qy),      s_f*(-qx*qy),   rv+s_f*(qs-qys),      s_f*(-qy*qz),         0,       0,       0,      s_s*qz,  s_s*qw, -s_s*qx,       0,       0,       0,  // qy
			0,       0,       0,      s_f*(-qw*qz),      s_f*(-qx*qz),      s_f*(-qy*qz),   rv+s_f*(qs-qzs),         0,       0,       0,     -s_s*qy,  s_s*qx,  s_s*qw,       0,       0,       0,  // qz

		  p_v,       0,       0,                 0,                 0,                 0,                 0,       v_v,       0,       0,           0,       0,       0,     v_a,       0,       0,  // vx
			0,     p_v,       0,                 0,                 0,                 0,                 0,         0,     v_v,       0,           0,       0,       0,       0,     v_a,       0,  // vy
			0,       0,     p_v,                 0,                 0,                 0,                 0,         0,       0,     v_v,           0,       0,       0,       0,       0,     v_a,  // vz

			0,       0,       0,           -s_s*qx,            s_s*qw,            s_s*qz,           -s_s*qy,         0,       0,       0,     s_w * t,       0,       0,       0,       0,       0,  // avx
			0,       0,       0,           -s_s*qy,           -s_s*qz,            s_s*qw,            s_s*qx,         0,       0,       0,           0, s_w * t,       0,       0,       0,       0,  // avy
			0,       0,       0,           -s_s*qz,            s_s*qy,           -s_s*qx,            s_s*qw,         0,       0,       0,           0,       0, s_w * t,       0,       0,       0,  // avz

		  p_a,       0,       0,                 0,                 0,                 0,                 0,       v_a,       0,       0,           0,       0,       0,     a_a,       0,       0,  // ax
			0,     p_a,       0,                 0,                 0,                 0,                 0,         0,     v_a,       0,           0,       0,       0,       0,     a_a,       0,  // ay
			0,       0,     p_a,                 0,                 0,                 0,                 0,         0,       0,     v_a,           0,       0,       0,       0,       0,     a_a,  // az

		};
		cn_copy_in_row_major_roi(q_out, Q_POSE_BLOCK, 16, 0, 0, 16, 16);
	} else {

		FLT Q_POSE_BLOCK[] = {
     //       x        y        z           qx         qy       qz              vx       vy       vz          avx      avy      avz       ax       ay      az
			p_p,       0,       0,           0,         0,      0,            p_v,       0,       0,           0,       0,       0,     p_a,       0,       0,  // x
			0,       p_p,       0,           0,         0,      0,              0,     p_v,       0,           0,       0,       0,       0,     p_a,       0,  // y
			0,         0,     p_p,           0,         0,      0,              0,       0,     p_v,           0,       0,       0,       0,       0,     p_a,  // z

			0,         0,       0,          rv,         0,      0,              0,       0,       0,           r_av,    0,       0,       0,       0,       0,  // qx
			0,         0,       0,           0,        rv,      0,              0,       0,       0,           0,       r_av,    0,       0,       0,       0,  // qy
			0,         0,       0,           0,         0,     rv,              0,       0,       0,           0,       0,    r_av,       0,       0,       0,  // qz

			p_v,       0,       0,           0,         0,      0,            v_v,       0,       0,           0,       0,       0,     v_a,       0,       0,  // vx
			0,       p_v,       0,           0,         0,      0,              0,     v_v,       0,           0,       0,       0,       0,     v_a,       0,  // vy
			0,         0,     p_v,           0,         0,      0,              0,       0,     v_v,           0,       0,       0,       0,       0,     v_a,  // vz

			0,         0,       0,           r_av,      0,      0,              0,       0,       0,     s_w * t,        0,       0,       0,       0,       0,  // avx
			0,         0,       0,           0,      r_av,      0,              0,       0,       0,           0,  s_w * t,       0,       0,       0,       0,  // avy
			0,         0,       0,           0,         0,   r_av,              0,       0,       0,           0,        0, s_w * t,       0,       0,       0,  // avz

			p_a,       0,       0,           0,         0,      0,            v_a,       0,       0,           0,        0,       0,     a_a,       0,       0,  // ax
			0,       p_a,       0,           0,         0,      0,              0,     v_a,       0,           0,        0,       0,       0,     a_a,       0,  // ay
			0,         0,     p_a,           0,         0,      0,              0,       0,     v_a,           0,        0,       0,       0,       0,     a_a,  // az

		};
		cn_copy_in_row_major_roi(q_out, Q_POSE_BLOCK, 15, 0, 0, 15, 15);
	}

	assert(cn_is_symmetrical(q_out));

	for(int i = 0;i < 3;i++) {
        int accBiasIdx = (int)(errorState ? offsetof(SurviveKalmanErrorModel , IMUBias.AccBias) : offsetof(SurviveKalmanModel, IMUBias.AccBias))/sizeof(FLT) + i;
        if(accBiasIdx < q_out->rows) cnMatrixSet(q_out, accBiasIdx, accBiasIdx, ga);

        int gyroBiasIdx = (int)(errorState ? offsetof(SurviveKalmanErrorModel, IMUBias.GyroBias) : offsetof(SurviveKalmanModel, IMUBias.GyroBias))/sizeof(FLT) + i;
		if(gyroBiasIdx < q_out->rows) cnMatrixSet(q_out, gyroBiasIdx, gyroBiasIdx, gb);
	}

}

/**
 * The prediction model and associated F matrix use generated code to simplifiy the jacobian. This might not be strictly
 * necessary but allows for quicker development.
 */
void survive_kalman_tracker_predict_jac(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0, struct CnMat *x1, struct CnMat *f) {
	SurviveKalmanModel s_in = copy_model(cn_as_const_vector(x0), x0->rows);

	if(x1) {
		SurviveKalmanModel s_out = {0};

		struct SurviveKalmanTracker_Params *params = (struct SurviveKalmanTracker_Params *)k->user;
		if(params->process_weight_acc == 0) {
			scale3d(s_in.Acc, s_in.Acc, 0);
		}
		if(params->process_weight_vel == 0) {
			scalend(s_in.Velocity.Pos, s_in.Velocity.Pos, 0, 6);
		}
		quatnormalize(s_in.Pose.Rot, s_in.Pose.Rot);
		SurviveKalmanModelPredict(&s_out, dt, &s_in);
		quatnormalize(s_out.Pose.Rot, s_out.Pose.Rot);

		memcpy(cn_as_vector(x1), s_out.Pose.Pos, x1->rows * sizeof(FLT));
	}

	if(f) {
		size_t state_cnt = x0->rows;
		if (dt == 0) {
			cn_eye(f, 0);
		} else {
			SurviveKalmanModelPredict_jac_kalman_model(f, dt, &s_in);
		}
	}
}

void survive_kalman_error_tracker_predict_jac(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0, struct CnMat *x1, struct CnMat *f) {
	SurviveKalmanModel s_in = copy_model(cn_as_const_vector(x0), x0->rows);
	SurviveKalmanErrorModel errorModel = { 0 };

	if(x1) {
		SurviveKalmanModel s_out = {0};

		struct SurviveKalmanTracker_Params *params = (struct SurviveKalmanTracker_Params *)k->user;

		quatnormalize(s_in.Pose.Rot, s_in.Pose.Rot);
		SurviveKalmanModelPredict(&s_out, dt, &s_in);
		quatnormalize(s_out.Pose.Rot, s_out.Pose.Rot);

		memcpy(cn_as_vector(x1), s_out.Pose.Pos, x1->rows * sizeof(FLT));
	}

	if(f) {
		size_t state_cnt = x0->rows;
		if (dt == 0) {
			cn_eye(f, 0);
		} else {
			SurviveKalmanModelErrorPredict_jac_error_model(f, dt, &s_in, &errorModel);
		}
	}
}

void survive_show_covariance(SurviveObject *so, const SurvivePose *pose, const struct CnMat *Ri, FLT s, FLT stddev) {
    SurviveContext *ctx = so->ctx;
    CN_CREATE_STACK_MAT(R, 7, 7);
    cnCopy(Ri, &R, 0);
    CN_CREATE_STACK_MAT(RL, 7, 7);
    CN_CREATE_STACK_MAT(X, 7, 1);
    CN_CREATE_STACK_MAT(Xs, 7, 1);
    cnSqRootSymmetric(&R, &RL);
    for(int i = 0;i < 25;i++) {
        cnRand(&X, 0, stddev);
        cnGEMM(&RL, &X, 1, 0, 0, &Xs, 0);
        addnd(_Xs, _Xs, pose->Pos, 7);
		quatnormalize(&_Xs[3], &_Xs[3]);
        char external_name[16] = {0};
        sprintf(external_name, "%s-sample_%d", so->codename, i);
        SurvivePose head2world = *pose;
        ApplyPoseToPose(&head2world, (SurvivePose *)_Xs, &so->head2imu);

        survive_recording_write_to_output(ctx->recptr, "AXIS %s_sample_%i_%f %f " Point7_format "\n",
                                          so->codename, i, s, s, SURVIVE_POSE_EXPAND(head2world));
    }
}

void survive_kalman_tracker_integrate_observation(PoserData *pd, SurviveKalmanTracker *tracker, const SurvivePose *pose,
												  const struct CnMat *Ri) {
	SurviveObject *so = tracker->so;
    SurviveContext *ctx = so->ctx;

	integrate_variance_tracker(tracker, &tracker->pose_variance, (FLT*)pose->Pos, 7);

    if (tracker->show_raw_obs) {
        static int report_in_imu = -1;
        if (report_in_imu == -1) {
            report_in_imu = survive_configi(so->ctx, "report-in-imu", SC_GET, 0);
        }

        char external_name[16] = {0};
		sprintf(external_name, "%s-raw-obs", so->codename);
		SurvivePose head2world = *pose;
		if(!report_in_imu) {
            ApplyPoseToPose(&head2world, pose, &so->head2imu);
		}
        SURVIVE_INVOKE_HOOK(external_pose, ctx, external_name, &head2world);
	}

	if (tracker->use_raw_obs) {
		SURVIVE_INVOKE_HOOK_SO(imupose, so, pd->timecode, pose);
		return;
	}

	survive_long_timecode timecode = pd->timecode;

	FLT time = timecode / (FLT)tracker->so->timebase_hz;
	if (tracker->model.t == 0) {
		tracker->model.t = time;
	}

	/*
	FLT R[] = {tracker->obs_pos_var, tracker->obs_pos_var, tracker->obs_pos_var, tracker->obs_rot_var,
			   tracker->obs_rot_var, tracker->obs_rot_var, tracker->obs_rot_var};
	if (oR) {
		addnd(R, R, oR, 7);
	}
*/
	if (time - tracker->model.t < 0) {
		if (time - tracker->model.t > -.1) {
			FLT tdiff = tracker->model.t - time;

			// Scale up the covariance
			FLT pS = 10, rS = 1;
			FLT Raug[] = {pS * tdiff, pS * tdiff, pS * tdiff, rS * tdiff, rS * tdiff, rS * tdiff, rS * tdiff};
			// addnd(R, R, Raug, 7);

			time = tracker->model.t;
		} else {
			// CN_WARN("Processing light data from the past %fs", time - tracker->model.t );
			tracker->stats.late_light_dropped++;
			return;
		}
	}

	tracker->last_light_time = time;

	if (tracker->obs_pos_var >= 0 && tracker->obs_rot_var >= 0) {
        CN_CREATE_STACK_MAT(R, 7, 7);
        if(Ri) {
            cnScale(&R, Ri, tracker->obs_cov_scale);
        }
        FLT augR[] = {tracker->obs_pos_var, tracker->obs_pos_var, tracker->obs_pos_var, tracker->obs_rot_var,
                      tracker->obs_rot_var, tracker->obs_rot_var, tracker->obs_rot_var};
        for(int i =0;i < 7;i++)
            cnMatrixSet(&R, i, i, cnMatrixGet(&R, i, i) + augR[i]);

        if(tracker->report_covariance_cnt > 0 && Ri && Ri->rows == Ri->cols && (tracker->stats.obs_count % tracker->report_covariance_cnt) == 0) {
            survive_recording_write_to_output(ctx->recptr, "%s' FULL_COVARIANCE ", so->codename);
            for (int i = 0; i < R.cols * R.cols; i++) {
                survive_recording_write_to_output_nopreamble(ctx->recptr, "%f ", R.data[i]);
            }
            survive_recording_write_to_output_nopreamble(ctx->recptr, "\n");

            if(tracker->report_sampled_cloud > 0) {
                survive_show_covariance(so, pose, Ri, .05, tracker->report_sampled_cloud);
            }

        }

        FLT obs_error = integrate_pose(tracker, time, pose, tracker->obs_model.adaptive ? 0 : &R);
		tracker->stats.obs_total_error += obs_error;
		tracker->stats.obs_count++;

		SurviveObject *so = tracker->so;
		SV_DATA_LOG("res_err_obs", &obs_error, 1);

		survive_kalman_tracker_report_state(pd, tracker);
	}
}

typedef void (*survive_attach_detach_fn)(SurviveContext *ctx, const char *tag, FLT *var);

void survive_kalman_tracker_reinit(SurviveKalmanTracker *tracker) {
	memset(&tracker->stats, 0, sizeof(tracker->stats));

	tracker->report_ignore_start_cnt = 0;
	tracker->last_light_time = 0;
	tracker->light_residuals_all = 0;

	memset(&tracker->state, 0, sizeof(tracker->state));
	tracker->state.Pose.Rot[0] = 1;
	tracker->state.IMUBias.IMUCorrection[0] = 1;
	for(int i = 0;i < 3;i++)
		tracker->state.IMUBias.AccScale[i] = 1.;

	cnkalman_state_reset(&tracker->model);
	for (int i = 0; i < 6; i++) {
		cnMatrixSet(&tracker->model.P, i, i, cnMatrixGet(&tracker->model.P, i, i) + 1e5);
	}

	cnkalman_state_reset(&tracker->imu_bias_model);

	SurviveIMUBiasErrorModel initial_variance = {
		.AccScale = {tracker->params.initial_acc_scale_variance, tracker->params.initial_acc_scale_variance, tracker->params.initial_acc_scale_variance},
		.GyroBias = {
			tracker->params.initial_gyro_variance, tracker->params.initial_gyro_variance, tracker->params.initial_gyro_variance
		},
		.AccBias = {
			tracker->params.initial_acc_bias_variance, tracker->params.initial_acc_bias_variance, tracker->params.initial_acc_bias_variance
		},
		.IMUCorrection = {
			tracker->params.initial_variance_imu_correction, tracker->params.initial_variance_imu_correction, tracker->params.initial_variance_imu_correction
		}
	};
	cn_set_diag(&tracker->imu_bias_model.P, (const FLT *)&initial_variance);

	size_t state_cnt = tracker->model.state_cnt;

	FLT Rrs = tracker->obs_rot_var;
	FLT Rps = tracker->obs_pos_var;
	FLT Rr[] = {Rrs, Rrs, Rrs, Rrs, Rps, Rps, Rps};
	struct CnMat ObsR = cnMat(7, 7, tracker->Obs_R);
	cn_set_diag(&ObsR, Rr);

	FLT Rimu[] = {tracker->acc_var,	 tracker->acc_var,	tracker->acc_var,
				  tracker->gyro_var, tracker->gyro_var, tracker->gyro_var};
	struct CnMat IMU_R = cnMat(6, 6, tracker->IMU_R);
	cn_set_diag(&IMU_R, Rimu);

	FLT var_diag[SURVIVE_MODEL_MAX_STATE_CNT] = {0};
	FLT p_threshold = survive_kalman_tracker_position_var2(tracker, var_diag, tracker->model.error_state_size);
	SurviveObject * so = tracker->so;
	SV_DATA_LOG("tracker_P", var_diag, tracker->model.state_cnt);
}

void survive_kalman_tracker_init(SurviveKalmanTracker *tracker, SurviveObject *so) {
	memset(tracker, 0, sizeof(*tracker));

	tracker->so = so;

	struct SurviveContext *ctx = tracker->so->ctx;
	SV_VERBOSE(110, "Initializing Filter:");
	// These are relatively high numbers to seed with; we are essentially saying
	// origin has a variance of 10m; and the quat can be varied by 4 -- which is
	// more than any actual normalized quat could be off by.

	SurviveKalmanTracker_attach_config(tracker->so->ctx, tracker);

	bool use_imu = (bool)survive_configi(ctx, "use-imu", SC_GET, 1);
	if (!use_imu) {
		tracker->gyro_var = tracker->acc_var = -1;
	}

	bool use_kalman = (bool)survive_configi(ctx, "use-kalman", SC_GET, 1);
	tracker->use_raw_obs = !use_kalman;

	size_t imu_state_cnt = (sizeof(SurviveKalmanModel) - offsetof(SurviveKalmanModel, IMUBias.AccScale)) / sizeof(FLT);
	size_t state_cnt = (sizeof(SurviveKalmanModel) ) / sizeof(FLT) - imu_state_cnt;

	switch(tracker->minimize_state_space) {
	    case 1:
			if(tracker->params.process_weight_gyro_bias) break;
		imu_state_cnt -= 3;
            if(tracker->params.process_weight_acc_bias) break;
		imu_state_cnt -= 3;
            if(tracker->params.initial_variance_imu_correction) break;
		imu_state_cnt -= 4;
            if(tracker->params.initial_acc_scale_variance) break;
		imu_state_cnt -= 1;
            if(tracker->params.process_weight_acc || tracker->params.process_weight_jerk) break;
            state_cnt -= 3;
            if(tracker->params.process_weight_ang_velocity) break;
            state_cnt -= 3;
            if(tracker->params.process_weight_vel) break;
            state_cnt -= 3;
            break;
	    default:
	        break;
	}

	if(imu_state_cnt) {
		cnkalman_error_state_init(&tracker->imu_bias_model, imu_state_cnt, imu_state_cnt - 1, 0, 0,
								  imu_error_state_fn, &tracker->params, (FLT*)&tracker->state.IMUBias);
		tracker->imu_bias_model.Update_fn = imu_bias_update_fn;
		tracker->model.state_variance_per_second = cnVec(imu_state_cnt, tracker->process_variance.IMUBias.AccScale);
	}

	if(tracker->use_error_state) {
		cnkalman_error_state_init(&tracker->model, state_cnt, state_cnt - 1, survive_kalman_error_tracker_predict_jac,
								  tracker->noise_model == 0 ? survive_kalman_tracker_error_process_noise_bounce : 0,
								  error_state_fn, &tracker->params, (FLT *)&tracker->state);
		tracker->model.Update_fn = state_update_fn;
		tracker->model.error_state_transition = true;
	} else {
		cnkalman_state_init(&tracker->model, state_cnt, survive_kalman_tracker_predict_jac,
								  tracker->noise_model == 0 ? survive_kalman_tracker_process_noise_bounce : 0,
								  &tracker->params, (FLT *)&tracker->state);
	}
	if(tracker->noise_model == 1) {
		if(tracker->use_error_state) {
			SurviveKalmanErrorModel* pv = (SurviveKalmanErrorModel *)&tracker->process_variance;
			for(int i = 0;i < 3;i++) {
					pv->Pose.Pos[i] = tracker->params.process_weight_pos;
					pv->Pose.AxisAngleRot[i] = tracker->params.process_weight_rotation;
					pv->Velocity.Pos[i] = tracker->params.process_weight_vel;
					pv->Velocity.AxisAngleRot[i] = tracker->params.process_weight_ang_velocity;
					pv->Acc[i] = tracker->params.process_weight_acc;
					pv->IMUBias.AccBias[i] = tracker->params.process_weight_acc_bias;
				}
		} else {
			SurviveKalmanModel* pv = &tracker->process_variance;
			for(int i = 0;i < 3;i++) {
				pv->Pose.Pos[i] = tracker->params.process_weight_pos;
				pv->Pose.Rot[i] = tracker->params.process_weight_rotation;
				pv->Velocity.Pos[i] = tracker->params.process_weight_vel;
				pv->Velocity.AxisAngleRot[i] = tracker->params.process_weight_ang_velocity;
				pv->Acc[i] = tracker->params.process_weight_acc;
				pv->IMUBias.AccBias[i] = tracker->params.process_weight_acc_bias;
			}
			pv->Pose.Rot[3] = tracker->params.process_weight_rotation;
		}
		tracker->model.state_variance_per_second = cnVec(tracker->model.error_state_size, tracker->process_variance.Pose.Pos);
	}
	//tracker->model.transition_jacobian_mode = cnkalman_jacobian_mode_debug;
	if (ctx) {
		cnkalman_set_logging_level(&tracker->model, ctx->log_level);
	}
    tracker->model.normalize_fn = kalman_model_normalize;

	tracker->model.datalog_user = tracker;
	tracker->model.datalog = tracker_datalog;

	cnkalman_state_t* states[] = {0, 0, &tracker->model};
	cnkalman_meas_model_multi_init(states, 3, "joint lightcap", &tracker->joint_model, map_joint_light_data);
	cnkalman_meas_model_t_joint_lightcap_attach_config(ctx, &tracker->joint_model);
	tracker->joint_model.error_state_model = true;
	if(tracker->joint_model.term_criteria.max_iterations == -1)
		tracker->joint_model.term_criteria.max_iterations = 10;

	cnkalman_state_t* models[] = { &tracker->model, &tracker->imu_bias_model };
    cnkalman_meas_model_multi_init(models, 2, "imu", &tracker->imu_model, survive_kalman_tracker_imu_measurement_model);
	cnkalman_meas_model_t_obj_imu_attach_config(ctx, &tracker->imu_model);

	cnkalman_meas_model_multi_init(models, 2, "zvu", &tracker->zvu_model, 0);

    cnkalman_meas_model_init(&tracker->model, "lightcap", &tracker->lightcap_model, map_light_data);
	cnkalman_meas_model_t_obj_lightcap_attach_config(ctx, &tracker->lightcap_model);
	//tracker->lightcap_model.error_state_model = false;
	tracker->lightcap_model.term_criteria.max_iterations = 10;

    cnkalman_meas_model_init(&tracker->model, "obs", &tracker->obs_model, tracker->obs_axisangle_model ? map_obs_data_axisangle : map_obs_data);
	cnkalman_meas_model_t_obj_obs_attach_config(ctx, &tracker->obs_model);
	tracker->obs_model.term_criteria.max_iterations = 10;

	survive_kalman_tracker_reinit(tracker);

	SV_VERBOSE(10, "Tracker config for %s (%d state count)", survive_colorize_codename(tracker->so), (int)state_cnt);
}

SurviveVelocity survive_kalman_tracker_velocity(const SurviveKalmanTracker *tracker) {
	SurviveKalmanModel mdl = {0};
	struct CnMat x1 = cnVec(13, (FLT *)&mdl);
	cnkalman_extrapolate_state(0, &tracker->model, &x1, 0);
	return mdl.Velocity;
}

static void print_kalman_stats(SurviveContext* ctx, const cnkalman_meas_model_t * model) {
    const struct cnkalman_update_extended_total_stats_t* total_stats = &model->stats;
    if(total_stats->total_runs == 0) return;

    SV_VERBOSE(5, "\t%s Kalman statistics:", model->name);
    FLT t = (FLT)total_stats->total_runs;
    SV_VERBOSE(5, "\t\t%-32s %6d %7.3f%%", "failures", total_stats->total_failures, 100 * total_stats->total_failures / t);
    SV_VERBOSE(5, "\t\t%-32s %7.7f / %7.7f / %7.7f", "avg bestnorm", total_stats->bestnorm_acc / t, total_stats->bestnorm_meas_acc / t, total_stats->bestnorm_delta_acc / t);
    SV_VERBOSE(5, "\t\t%-32s %7.7f / %7.7f", "avg orignorm", total_stats->orignorm_acc / t, total_stats->orignorm_meas_acc / t);
    SV_VERBOSE(5, "\t\t%-32s %7.7f", "avg step", total_stats->step_acc / (FLT)total_stats->step_cnt);
    SV_VERBOSE(5, "\t\t%-32s %6d (%3.2f)", "iterations", total_stats->total_iterations, total_stats->total_iterations / (FLT)total_stats->total_runs);
    SV_VERBOSE(5, "\t\t%-32s %6d", "runs", total_stats->total_runs);
    SV_VERBOSE(5, "\t\t%-32s %6d / %6d", "fevals", total_stats->total_fevals, total_stats->total_hevals);
    SV_VERBOSE(5, "\t\t%-32s", "exit reasons");
    for(int i = 1;i < cnkalman_update_extended_termination_reason_MAX;i++) {
        SV_VERBOSE(5, "\t\t    %-28s %6u", cnkalman_update_extended_termination_reason_to_str(i), (int)total_stats->stop_reason_counts[i]);
    }
}

void survive_kalman_tracker_stats(SurviveKalmanTracker *tracker) {
	FLT report_runtime = tracker->last_report_time - tracker->first_report_time;
	FLT imu_runtime = tracker->last_imu_time - tracker->first_imu_time;
    SurviveContext *ctx = tracker->so->ctx;

	SV_VERBOSE(5, "IMU %s tracker statistics:", tracker->so->codename);
	SV_VERBOSE(5, "\t%-32s %4u %4u", "state_cnt", tracker->model.state_cnt, tracker->model.error_state_size);

	LinmathQuat q;
	quatnormalize(q, tracker->state.IMUBias.IMUCorrection);
	SV_VERBOSE(5, "\t%-32s (" Point3_format ") " Point4_format, "IMU Correction", LINMATH_VEC3_EXPAND(tracker->state.IMUBias.AccScale), LINMATH_VEC4_EXPAND(q));
	SV_VERBOSE(5, "\t%-32s " Point3_format, "Acc Bias", LINMATH_VEC3_EXPAND(tracker->state.IMUBias.AccBias));
	SV_VERBOSE(5, "\t%-32s " Point3_format, "Gyro Bias", LINMATH_VEC3_EXPAND(tracker->state.IMUBias.GyroBias));

	SV_VERBOSE(5, "\t%-32s %f", "avg hz", tracker->stats.reported_poses / report_runtime);

	SV_VERBOSE(5, "\t%-32s %u", "late imu", tracker->stats.late_imu_dropped);
	SV_VERBOSE(5, "\t%-32s %u", "late light", tracker->stats.late_light_dropped);
	//joint_model_sensor_cnt_sum
	SV_VERBOSE(5, "\t%-32s %7.7f avg cnt %8d dropped", "joint model", tracker->stats.joint_model_sensor_cnt_sum / (FLT) tracker->joint_model.stats.total_runs,
			   tracker->stats.joint_model_dropped);
	SV_VERBOSE(5, "\t%-32s %7.7f avg cnt %8d dropped", "lightcap model", tracker->stats.lightcap_model_sensor_cnt_sum / (FLT) tracker->lightcap_model.stats.total_runs,
			   tracker->stats.lightcap_model_dropped);

	SV_VERBOSE(5, "\t%-32s %u of %u (%2.2f%%)", "Dropped poses", (unsigned)tracker->stats.dropped_poses,
			   (unsigned)(tracker->stats.reported_poses + tracker->stats.dropped_poses),
			   100. * tracker->stats.dropped_poses /
				   (FLT)(tracker->stats.reported_poses + tracker->stats.dropped_poses))

	FLT var[SURVIVE_MODEL_MAX_STATE_CNT] = {0};
	scalend(var, tracker->stats.reported_var, 1. / tracker->stats.reported_poses, SURVIVE_MODEL_MAX_STATE_CNT);
	SV_VERBOSE(5, "\t%-32s " Point19_format, "Mean reported variance", LINMATH_VEC19_EXPAND(var));
	scalend(var, tracker->stats.dropped_var, 1. / tracker->stats.reported_poses, SURVIVE_MODEL_MAX_STATE_CNT);
	SV_VERBOSE(5, "\t%-32s " Point19_format, "Mean dropped variance", LINMATH_VEC19_EXPAND(var));
	FLT integration_variance[16] = {0};
	variance_tracker_calc(&tracker->pose_variance, integration_variance);
	SV_VERBOSE(5, "\t%-32s %e (%7u integrations, %7.3fhz) " Point7_format, "Obs error",
			   tracker->stats.obs_total_error / (FLT)tracker->stats.obs_count, (unsigned)tracker->stats.obs_count,
			   (unsigned)tracker->stats.obs_count / report_runtime, LINMATH_VEC7_EXPAND(integration_variance));

	variance_tracker_calc(&tracker->imu_variance, integration_variance);
	SV_VERBOSE(5, "\t%-32s %e (%7u integrations, %7.3fhz) " Point6_format, "IMU error",
			   tracker->stats.imu_total_error / (FLT)tracker->stats.imu_count, (unsigned)tracker->stats.imu_count,
			   (unsigned)tracker->stats.imu_count / imu_runtime, LINMATH_VEC6_EXPAND(integration_variance));
	SV_VERBOSE(5, "\t%-32s " FLT_format " " FLT_format " (%7u)", "IMU acc avg norm",
		   tracker->stats.acc_norm / (FLT)tracker->stats.imu_count,  (FLT)tracker->stats.imu_count / tracker->stats.acc_norm,
			   (unsigned)tracker->stats.imu_count);
    SV_VERBOSE(5, "\t%-32s " FLT_format " " FLT_format " (%7u)", "Stationary IMU acc avg norm",
               tracker->stats.stationary_acc_norm / (FLT)tracker->stats.stationary_imu_count,  (FLT)tracker->stats.stationary_imu_count / tracker->stats.stationary_acc_norm,
               (unsigned)tracker->stats.stationary_imu_count);
	SV_VERBOSE(5, "\t%-32s  %7u", "No light IMU count",
			   (unsigned)tracker->stats.no_light_imu_count);

	var[0] = 0;
	for(int lh = 0;lh < NUM_GEN2_LIGHTHOUSES;lh++) {
		for(int sidx = 0; sidx < tracker->so->sensor_ct;sidx++) {
			for(int axis = 0;axis < 2;axis++) {
				if((FLT)tracker->light_variance[lh][sidx][axis].counts) {
					var[0] += tracker->light_variance[lh][sidx][axis].variances[0] /
							  (FLT)tracker->light_variance[lh][sidx][axis].counts;
				}
			}
		}
	}
	SV_VERBOSE(5, "\t%-32s %e (%7u integrations, %7.3fhz) " FLT_format, "Lightcap error",
			   tracker->stats.lightcap_total_error / (FLT)tracker->stats.lightcap_count,
			   (unsigned)tracker->stats.lightcap_count, (unsigned)tracker->stats.lightcap_count / report_runtime, var[0]);

	SV_VERBOSE(5, " ");
	SV_VERBOSE(5, "\t%-32s " Point3_format, "gyro bias", LINMATH_VEC3_EXPAND(tracker->state.IMUBias.GyroBias));
    SV_VERBOSE(5, "\t%-32s " FLT_format, "Lightcap R", tracker->light_var);
	for (int i = 0; i < 6; i++) {
		SV_VERBOSE(5, "\t%-32s " Point6_format, i == 0 ? "Gyro R" : "", LINMATH_VEC6_EXPAND(tracker->IMU_R + 6 * i));
	}
	for (int i = 0; i < 7; i++) {
		SV_VERBOSE(5, "\t%-32s " Point7_format, i == 0 ? "Observation R" : "",
				   LINMATH_VEC7_EXPAND(tracker->Obs_R + 7 * i));
	}

	FLT* state_variance = (FLT*)&tracker->reported_state_variance;
    scalend(state_variance, state_variance, 1. / (FLT)tracker->state_variance_count, tracker->model.state_cnt);
    SV_VERBOSE(5, "\t%-32s " Point26_format, "Observed state variance", LINMATH_VEC26_EXPAND(state_variance));

    for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (tracker->stats.lightcap_count_by_lh[i]) {
			SV_VERBOSE(5, "\tLighthouse %d", i);
			SV_VERBOSE(5, "\t\t%-32s %e", "Avg error",
					   tracker->stats.lightcap_error_by_lh[i] / tracker->stats.lightcap_count_by_lh[i]);
			SV_VERBOSE(5, "\t\t%-32s %u", "Count", (unsigned)tracker->stats.lightcap_count_by_lh[i]);
			SV_VERBOSE(5, "\t\t%-32s %e", "Current error", tracker->light_residuals[i]);
		}

		for (int j = 0; j < SENSORS_PER_OBJECT; j++) {
			for (int z = 0; z < 2; z++) {
				if (tracker->so->activations.hits[j][i][z]) {
					SV_VERBOSE(5, "\t\t %02d.%d %5d %f", j, z, (int)tracker->so->activations.hits[j][i][z],
							   tracker->so->activations.hits[j][i][z] / report_runtime);
				}
			}
		}
	}
    print_kalman_stats(ctx, &tracker->imu_model);
    print_kalman_stats(ctx, &tracker->lightcap_model);
    print_kalman_stats(ctx, &tracker->obs_model);
	print_kalman_stats(ctx, &tracker->zvu_model);
	print_kalman_stats(ctx, &tracker->joint_model);

	memset(&tracker->stats, 0, sizeof(tracker->stats));
	tracker->first_report_time = tracker->last_report_time = 0;

	SV_VERBOSE(5, " ");
}
void survive_kalman_tracker_free(SurviveKalmanTracker *tracker) {
	SurviveContext *ctx = tracker->so->ctx;

	survive_kalman_tracker_stats(tracker);

	cnkalman_state_free(&tracker->model);
	cnkalman_state_free(&tracker->imu_bias_model);

	cnkalman_meas_model_t_joint_lightcap_detach_config(tracker->so->ctx, &tracker->joint_model);
	cnkalman_meas_model_t_obj_imu_detach_config(tracker->so->ctx, &tracker->imu_model);
	cnkalman_meas_model_t_obj_obs_detach_config(tracker->so->ctx, &tracker->obs_model);
	cnkalman_meas_model_t_obj_lightcap_detach_config(tracker->so->ctx, &tracker->lightcap_model);

	SurviveKalmanTracker_detach_config(tracker->so->ctx, tracker);
}

void survive_kalman_tracker_lost_tracking(SurviveKalmanTracker *tracker, bool allowLHReset) {
	if (tracker == 0)
		return;

	SurviveContext *ctx = tracker->so->ctx;
	SV_WARN("Too many failures for %s at %f; reseting calibration %e (%7.4f stationary)", survive_colorize_codename(tracker->so),
			survive_run_time(ctx),
			tracker->light_residuals_all,
			SurviveSensorActivations_stationary_time(&tracker->so->activations) / 48000000.);
	tracker->light_residuals_all = 0;
	{
		tracker->so->OutPoseIMU = (SurvivePose){0};
		tracker->so->poseConfidence = 0;
		survive_kalman_tracker_reinit(tracker);
		memset(&tracker->so->OutPoseIMU, 0, sizeof(SurvivePose));
		memset(&tracker->so->OutPose, 0, sizeof(SurvivePose));
	}

	if (!allowLHReset)
		return;

	bool objectsAreValid = false;
	for (int i = 0; i < ctx->objs_ct && !objectsAreValid; i++) {
		objectsAreValid |= !quatiszero(ctx->objs[i]->OutPoseIMU.Rot);
	}

	if (!objectsAreValid) {
	  ctx->floor_offset = 0;
		for (int lh = 0; lh < ctx->activeLighthouses; lh++) {
			ctx->bsd[lh].PositionSet = 0;
			SV_WARN("Lost tracking for LH%d %f", lh, tracker->light_residuals[lh]);
		}
	}
}

bool survive_kalman_tracker_check_valid(SurviveKalmanTracker *tracker) {
	bool isValid =
		tracker->light_error_threshold <= 0 || tracker->light_residuals_all < tracker->light_error_threshold ||
		(SurviveSensorActivations_stationary_time(&tracker->so->activations) < tracker->so->timebase_hz / 10);

	for (int i = 0; i < 3; i++) {
		isValid &= fabsf((float)tracker->state.Pose.Pos[i]) < 20;
	}

	if (!isValid) {
		survive_kalman_tracker_lost_tracking(tracker, tracker->use_error_for_lh_pos);
		return false;
	}
	return true;
}

void survive_kalman_tracker_report_state(PoserData *pd, SurviveKalmanTracker *tracker) {
	SurvivePose pose = {0};
	normalize_model(tracker);

	FLT t = pd->timecode / (FLT)tracker->so->timebase_hz;

	if (t < tracker->model.t) {
		assert(tracker->model.t - t < 1);
		t = tracker->model.t;
	}

	SurviveObject *so = tracker->so;
	assert(fabs(1 - quatmagnitude(tracker->state.Pose.Rot)) < 1e-4);
	SV_DATA_LOG("model_state", (const FLT *)&tracker->state, tracker->model.state_cnt);

	if (tracker->so->conf == 0) {
		return;
	}

	SurviveContext *ctx = tracker->so->ctx;
	if (tracker->min_report_time < 0) {
		tracker->min_report_time = 1. / tracker->so->imu_freq;
		SV_VERBOSE(10, "Setting min report time for %s to %f ms", survive_colorize(tracker->so->codename),
				   tracker->min_report_time * 1000.);
	}

	if (t - tracker->last_report_time < tracker->min_report_time) {
		return;
	}

	if (!survive_kalman_tracker_check_valid(tracker)) {
		tracker->stats.dropped_poses++;
		return;
	}

	survive_kalman_tracker_predict(tracker, t, &pose);
	SV_DATA_LOG("model_predict", (FLT *)pose.Pos, sizeof(pose) / sizeof(FLT));

	size_t state_cnt = tracker->model.state_cnt;
	FLT var_diag[SURVIVE_MODEL_MAX_STATE_CNT] = {0};
	FLT p_threshold = survive_kalman_tracker_position_var2(tracker, var_diag, tracker->model.error_state_size);
	SV_DATA_LOG("tracker_P", var_diag, tracker->model.state_cnt);

	if ((tracker->report_threshold_var > 0 && p_threshold >= tracker->report_threshold_var) ||
		(tracker->report_ignore_start > tracker->report_ignore_start_cnt)) {
		tracker->stats.dropped_poses++;
		addnd(tracker->stats.dropped_var, var_diag, tracker->stats.dropped_var, state_cnt);
		tracker->report_ignore_start_cnt++;

		so->OutPoseIMU = pose;
		return;
	}

	addnd(tracker->stats.reported_var, var_diag, tracker->stats.reported_var, state_cnt);

	SV_VERBOSE(600, "Tracker variance %s " Point16_format, tracker->so->codename, LINMATH_VEC16_EXPAND(var_diag));
	SV_VERBOSE(600, "Tracker Bias %s     " Point3_format, tracker->so->codename,
			   LINMATH_VEC3_EXPAND(tracker->state.IMUBias.GyroBias));
	SV_VERBOSE(600, "%f Tracker report %s   " SurvivePose_format, survive_run_time(ctx), tracker->so->codename,
			   SURVIVE_POSE_EXPAND(pose));

	tracker->stats.reported_poses++;

	SurviveVelocity velocity = survive_kalman_tracker_velocity(tracker);

	if (tracker->first_report_time == 0) {
		tracker->first_report_time = t;
	}

	tracker->so->poseConfidence = 1. / p_threshold;

    if(tracker->last_report_time > 0) {
        FLT dt = t - tracker->last_report_time;
        struct survive_kalman_model_t diff = { 0 };
        subnd((FLT *) &diff, (FLT *) &tracker->state, (FLT *) &tracker->previous_state, state_cnt);
        scalend((FLT *) &diff, (FLT *) &diff, 1. / dt, state_cnt);
        mulnd((FLT *) &diff, (FLT *) &diff, (FLT *) &diff, state_cnt);
        addnd((FLT *) &tracker->reported_state_variance, (FLT *) &tracker->reported_state_variance, (FLT *) &diff, state_cnt);
        tracker->state_variance_count++;
    }

    tracker->last_report_time = t;

    if(tracker->report_covariance_cnt > 0 && tracker->stats.reported_poses % tracker->report_covariance_cnt == 0) {
		survive_recording_write_to_output(ctx->recptr, "%s FULL_STATE " Point27_format "\n",
                                          so->codename, LINMATH_VEC27_EXPAND((FLT*)&tracker->state));
        survive_recording_write_to_output(ctx->recptr, "%s FULL_COVARIANCE ", so->codename);
        for (int i = 0; i < tracker->model.P.rows * tracker->model.P.cols; i++) {
            survive_recording_write_to_output_nopreamble(ctx->recptr, "%f ", tracker->model.P.data[i]);
        }
        survive_recording_write_to_output_nopreamble(ctx->recptr, "\n");
		survive_recording_write_matrix(ctx->recptr, so, 5, "imu", &tracker->imu_bias_model.P);

		if(tracker->report_sampled_cloud > 0) {
			survive_show_covariance(so, &pose, &tracker->model.P, .1, tracker->report_sampled_cloud);
		}
		{
			FLT v[16] = { 0 };
			memcpy(v, &tracker->state.IMUBias.AccScale, sizeof(FLT) * tracker->imu_bias_model.state_cnt);
			CnMat meta = cnVec(tracker->imu_bias_model.state_cnt, v);
			for(int i = 0;i < 4;i++)
				v[i] = v[i] - 1;

			if(cn_norm2(&meta) != 0) {
				survive_recording_write_matrix(ctx->recptr, so, 10, "meta", &meta);
			}
		}
    }

    tracker->previous_state = tracker->state;
    copy3d(so->acceleration, tracker->state.Acc);
	SV_VERBOSE(110, "%s confidence %7.7f", survive_colorize_codename(so), 1. / p_threshold);
	if (so->OutPose_timecode < pd->timecode) {
		SURVIVE_INVOKE_HOOK_SO(imupose, so, pd->timecode, &pose);
	}
	if(tracker->stats.imu_count > 100) {
        SURVIVE_INVOKE_HOOK_SO(velocity, so, pd->timecode, &velocity);
    }
}
