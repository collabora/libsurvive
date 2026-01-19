#include "survive_kalman_lighthouses.h"
#include "survive_recording.h"
#include "survive_reproject.h"
#include "survive_reproject_gen2.h"

#include "generated/kalman_kinematics.gen.h"
#include "generated/lighthouse_model.gen.h"
//#include "generated/survive_imu.generated.h"
#include "generated/survive_reproject.aux.generated.h"

// clang-format off
STRUCT_CONFIG_SECTION(SurviveKalmanLighthouse)
	STRUCT_EXISTING_CONFIG_ITEM("report-covariance", t->report_covariance_cnt);
	STRUCT_CONFIG_ITEM("kalman-lighthouse-up-variance", "", -1, t->up_variance);


	STRUCT_CONFIG_ITEM("kalman-lighthouse-initial-pos-variance", "", 1e-1, t->initial_pos_var);
	STRUCT_CONFIG_ITEM("kalman-lighthouse-initial-rot-variance", "", 1e-2, t->initial_rot_var);
	STRUCT_CONFIG_ITEM("kalman-pos-variance-per-sec", "", 0, t->variance_per_sec.Lighthouse.Pos[0]);
	STRUCT_CONFIG_ITEM("kalman-rot-variance-per-sec", "", 0, t->variance_per_sec.Lighthouse.AxisAngleRot[0]);

	STRUCT_CONFIG_ITEM("kalman-bsd-variance", "", 0, t->base_variance);
	STRUCT_CONFIG_ITEM("kalman-bsd-phase-variance", "", 0, t->initial_variance.phase);
	STRUCT_CONFIG_ITEM("kalman-bsd-curve-variance", "", 0, t->initial_variance.curve);
	STRUCT_CONFIG_ITEM("kalman-bsd-tilt-variance", "", 0, t->initial_variance.tilt);
	STRUCT_CONFIG_ITEM("kalman-bsd-ogeephase-variance", "", -1e2, t->initial_variance.ogeephase);
	STRUCT_CONFIG_ITEM("kalman-bsd-ogeemag-variance", "", -1e2, t->initial_variance.ogeemag);
	STRUCT_CONFIG_ITEM("kalman-bsd-gibpha-variance", "", 0, t->initial_variance.gibpha);
	STRUCT_CONFIG_ITEM("kalman-bsd-gibmag-variance", "", 0, t->initial_variance.gibmag);
END_STRUCT_CONFIG_SECTION(SurviveKalmanLighthouse)
// clang-format on

MEAS_MDL_CONFIG(lighthouse, imu, 10, 1e-1)
MEAS_MDL_CONFIG(lighthouse, obs, 10, 0)

//#define TRACK_IN_WORLD2LH

FLT baseline_var = 1e-4;

#ifdef TRACK_IN_WORLD2LH
static SurvivePose survive_kalman_lighthouse_lh2world(SurviveKalmanLighthouse *tracker) {
	return InvertPoseRtn(&tracker->state);
}
static SurvivePose survive_kalman_lighthouse_world2lh(SurviveKalmanLighthouse *tracker) { return tracker->state; }
#else
static SurvivePose survive_kalman_lighthouse_lh2world(SurviveKalmanLighthouse *tracker) {
	return tracker->state.Lighthouse;
}
#endif

bool lighthouse_integrate_imu_hfn(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
								  struct CnMat *H_k) {
	CN_CREATE_STACK_VEC(up, 3);
	if (y) {
		LighthouseIMUPrediction(&up, (const SurvivePose *)cn_as_const_vector(x_t));
		cn_elementwise_subtract(y, Z, &up);
	}
	if (H_k) {
		LinmathAxisAnglePose error_state = {0};
		LighthouseErrorIMUPrediction_jac_error_state(H_k, (const SurvivePose *)cn_as_const_vector(x_t), &error_state);
	}
	return true;
}

void integrate_imu(SurviveKalmanLighthouse *tracker) {
	LinmathVec3d up;
	BaseStationData *bsd = &tracker->ctx->bsd[tracker->lh];
	normalize3d(up, bsd->accel);
	if (isnan(up[0]))
		return;

	FLT v = tracker->up_variance;
	if (v >= 0) {
		FLT v_diag[3] = {v, v, v};
		CnMat R = cnVec(3, v_diag);
		CnMat Z = cnVec(3, up);

		struct cnkalman_update_extended_stats_t stats = {0};
		cnkalman_meas_model_predict_update_stats(0, &tracker->imu_model, 0, &Z, &R, &stats);
		SurviveContext *ctx = tracker->ctx;
		SV_VERBOSE(100, "IMU norm: %7.7f / %7.7f", stats.bestnorm, stats.orignorm);
	}
}

void survive_kalman_lighthouse_update_position(SurviveKalmanLighthouse *tracker, const SurvivePose *pose) {
	if (tracker->updating == false) {
		tracker->state.Lighthouse = *(pose);

		SurviveContext *ctx = tracker->ctx;
		if (normnd(ctx->bsd[tracker->lh].variance.Pos, 6) > 0) {
			cn_set_diag(&tracker->model.P, (const FLT *)ctx->bsd[tracker->lh].variance.Pos);
			cnCopy(&tracker->model.P, &tracker->push_cov, 0);

			SurvivePose min_error = {.Pos = {1e-5, 1e-5, 1e-5}, .Rot = {1e-6, 1e-6, 1e-6, 1e-6}};
			CnMat minError = cnVec(6, min_error.Pos);
			cn_add_diag(&tracker->push_cov, &minError, 5);

			tracker->state.Lighthouse = ctx->bsd[tracker->lh].Pose;
			tracker->push_state = tracker->state;
		} else {
			FLT pv = tracker->initial_pos_var, rv = tracker->initial_rot_var;
			SurviveLighthouseKalmanErrorModel baseline = {
				.Lighthouse = {.Pos = {pv, pv, pv}, .AxisAngleRot = {rv, rv, rv}},
				.BSD0 = tracker->initial_variance,
				.BSD1 = tracker->initial_variance};

			cnSetZero(&tracker->model.P);
			cn_set_diag(&tracker->model.P, (const FLT *)&baseline);
			integrate_imu(tracker);
		}
	}
}

void survive_kalman_lighthouse_report(SurviveKalmanLighthouse *tracker) {
	quatnormalize(tracker->state.Lighthouse.Rot, tracker->state.Lighthouse.Rot);
	SurvivePose lighthouse2world = survive_kalman_lighthouse_lh2world(tracker);
	FLT var_diag[3] = {0};
	for (int i = 0; i < 3; i++) {
		var_diag[i] = cnMatrixGet(&tracker->model.P, i, i);
	}

	tracker->ctx->bsd[tracker->lh].confidence = 1. / norm3d(var_diag);
	assert(cn_is_finite(&tracker->model.state));

	cn_get_diag(&tracker->model.P, (FLT *)&tracker->ctx->bsd[tracker->lh].variance, 6);
	SurviveContext *ctx = tracker->ctx;
	SV_VERBOSE(100, "LH%d %s " Point6_format, tracker->lh, survive_colorize("variance"),
			   LINMATH_VEC6_EXPAND(((FLT *)&tracker->ctx->bsd[tracker->lh].variance)));
	SV_VERBOSE(100, "LH%d %s " Point7_format, tracker->lh, survive_colorize("pose    "),
			   LINMATH_VEC7_EXPAND(((FLT *)&tracker->state.Lighthouse)));

	FLT diff_p[7] = {0};
	subnd(diff_p, lighthouse2world.Pos, survive_get_lighthouse_position(tracker->ctx, tracker->lh)->Pos, 7);
	FLT diff = normnd2(diff_p, 7);
	if (diff > 1e-4 || !tracker->ctx->bsd[tracker->lh].PositionSet) {
		tracker->updating = true;
		SURVIVE_INVOKE_HOOK(raw_lighthouse_pose, tracker->ctx, tracker->lh, &lighthouse2world);
		tracker->updating = false;
	}

	if (tracker->report_covariance_cnt > 0 && ++tracker->stats.reported_poses % tracker->report_covariance_cnt == 0) {
		SurviveContext *ctx = tracker->ctx;
		survive_recording_write_to_output(ctx->recptr, "LH%d FULL_COVARIANCE ", ctx->bsd[tracker->lh].mode);
		for (int i = 0; i < tracker->model.P.rows * tracker->model.P.cols; i++) {
			survive_recording_write_to_output_nopreamble(ctx->recptr, "%f ", tracker->model.P.data[i]);
		}
		survive_recording_write_to_output_nopreamble(ctx->recptr, "\n");

		survive_recording_write_matrix(ctx->recptr, 0, 5, tracker->lh == 0 ? "bsd0" : "bsd1", &tracker->bsd_model.P);

		CN_CREATE_STACK_MAT(BSD, 2, sizeof(BaseStationCal) / sizeof(FLT));
		memcpy(BSD.data, (FLT *)&tracker->state.BSD0, 2 * sizeof(BaseStationCal));
		CnMat tempBSD = cnMat(2, sizeof(BaseStationCal) / sizeof(FLT), (FLT *)&ctx->bsd[tracker->lh].fcal);
		cn_elementwise_subtract(&BSD, &BSD, &tempBSD);
		survive_recording_write_matrix(ctx->recptr, 0, 5, tracker->lh == 0 ? "LH0" : "LH1", &BSD);
	}
}

struct map_light_data_ctx {
	SurviveKalmanLighthouse *tracker;
	SurviveObject *so;

	LightInfo savedLight[32];
	uint32_t savedLight_idx;
	SurvivePose objectPose;
};

static inline int get_axis(const struct PoserDataLight *pdl) {
	switch (pdl->hdr.pt) {
	case POSERDATA_LIGHT:
		return (((PoserDataLightGen1 *)pdl)->acode & 1);
		break;
	case POSERDATA_LIGHT_GEN2:
		return ((PoserDataLightGen2 *)pdl)->plane;
		break;
	default:
		assert(0);
	}
	return 0;
}

static SurviveLighthouseKalmanModel copy_model(const FLT *src, size_t state_size) {
	SurviveLighthouseKalmanModel rtn = {0};
	memcpy(&rtn, src, sizeof(FLT) * state_size);
	return rtn;
}
static void state_update_fn(void *user, const struct CnMat *x0, const struct CnMat *E, struct CnMat *x1,
							struct CnMat *dX_wrt_error_state) {
	assert(x0->rows == 7);
	SurvivePose state = *(SurvivePose *)cn_as_const_vector(x0);
	if (x1) {
		SurvivePose _x1 = {0};
		SurviveAxisAnglePose error_state = *(const SurviveAxisAnglePose *)cn_as_const_vector(E);
		SurvivePoseAddErrorModel(&_x1, &state, &error_state);

		quatnormalize(_x1.Rot, _x1.Rot);

		memcpy(cn_as_vector(x1), &_x1, sizeof(FLT) * x1->rows);
	}
	if (dX_wrt_error_state) {
		SurviveAxisAnglePose error_model = {0};
		SurvivePoseAddErrorModel_jac_error_state(dX_wrt_error_state, &state, &error_model);
		assert(cn_is_finite(dX_wrt_error_state));
	}
	assert(x1 == 0 || cn_is_finite(x1));
}
void error_fn(void *user, const struct CnMat *x0, const struct CnMat *x1, struct CnMat *E, struct CnMat *E_jac_x) {
	SurviveLighthouseKalmanModel state0 = copy_model(cn_as_const_vector(x0), x0->rows);
	if (E_jac_x) {
		SurviveLighthouseKalmanModelToErrorModel_jac_x1(E_jac_x, &state0, &state0);
	}

	if (x1 && E) {
		SurviveLighthouseKalmanModel state1 = copy_model(cn_as_const_vector(x1), x1->rows);
		SurviveLighthouseKalmanErrorModel error_state = {0};
		SurviveLighthouseKalmanModelToErrorModel(&error_state, &state1, &state0);
		memcpy(cn_as_vector(E), &error_state, sizeof(FLT) * E->rows);
	}
}
void survive_kalman_lighthouse_ootx(SurviveKalmanLighthouse *tracker) {
	if (tracker == NULL) return;
	tracker->state.BSD0 = tracker->ctx->bsd[tracker->lh].fcal[0];
	tracker->state.BSD1 = tracker->ctx->bsd[tracker->lh].fcal[1];
}
void minimize_error_state_model_fn(void *user, const struct CnMat *x0, const struct CnMat *x1,
								   struct CnMat *error_state, struct CnMat *E_jac_x1) {
	if (error_state) {
		CnMat x1v = cnMatConstView(error_state->rows, 1, x1, 0, 0);
		CnMat x0v = cnMatConstView(error_state->rows, 1, x0, 0, 0);
		cnSub(error_state, &x1v, &x0v);
	}
	if (E_jac_x1) {
		cn_eye(E_jac_x1, 0);
	}
}
void minimize_integrate_update_fn(void *user, const struct CnMat *x0, const struct CnMat *error_state, struct CnMat *x1,
								  struct CnMat *dX_wrt_error_state) {
	if (x1) {
		CnMat x1v = cnMatView(error_state->rows, 1, x1, 0, 0);
		CnMat x0v = cnMatConstView(error_state->rows, 1, x0, 0, 0);
		cnAddScaled(&x1v, error_state, 1, &x0v, 1);
	}
	if (dX_wrt_error_state) {
		cn_eye(dX_wrt_error_state, 0);
	}
}
void survive_kalman_lighthouse_init(SurviveKalmanLighthouse *tracker, SurviveContext *ctx, int lh) {
	memset(tracker, 0, sizeof(*tracker));
	tracker->ctx = ctx;
	tracker->lh = lh;
	SurviveKalmanLighthouse_attach_config(ctx, tracker);

	FLT *f = (FLT *)&tracker->initial_variance;
	for (int i = 0; i < sizeof(tracker->initial_variance) / sizeof(FLT); i++) {
		f[i] = fmax(0, f[i] + tracker->base_variance);
	}

	tracker->state.BSD0 = ctx->bsd[lh].fcal[0];
	tracker->state.BSD1 = ctx->bsd[lh].fcal[1];
	tracker->state.Lighthouse.Rot[0] = 1;

	// cnkalman_state_init(&tracker->model, 7, 0, 0, tracker, (FLT*)&tracker->state);
	SurviveLighthouseKalmanErrorModel baseline = {
		.Lighthouse = {.Pos = {1e5, 1e5, 1e5}, .AxisAngleRot = {1e5, 1e5, 1e5}},
		.BSD0 = tracker->initial_variance,
		.BSD1 = tracker->initial_variance};

	int bsd_state_cnt = sizeof(tracker->state.BSD0) * 2 / sizeof(FLT);
	while (bsd_state_cnt && ((FLT *)(&baseline.BSD0))[bsd_state_cnt - 1] == 0)
		bsd_state_cnt--;

	cnkalman_error_state_init(&tracker->bsd_model, sizeof(tracker->state.BSD0) * 2 / sizeof(FLT),
							  bsd_state_cnt / sizeof(FLT), 0, 0, minimize_error_state_model_fn, tracker,
							  (FLT *)&tracker->state.BSD0);
	tracker->bsd_model.Update_fn = minimize_integrate_update_fn;

	cnkalman_error_state_init(&tracker->model, sizeof tracker->state.Lighthouse / sizeof(FLT),
							  sizeof tracker->state.Lighthouse / sizeof(FLT) - 1, 0, 0, error_fn, tracker,
							  (FLT *)&tracker->state);
	tracker->model.Update_fn = state_update_fn;

	for (int i = 1; i < 3; i++) {
		tracker->variance_per_sec.Lighthouse.Pos[i] = tracker->variance_per_sec.Lighthouse.Pos[0];
		tracker->variance_per_sec.Lighthouse.AxisAngleRot[i] = tracker->variance_per_sec.Lighthouse.AxisAngleRot[0];
	}
	tracker->model.state_variance_per_second = cnVec(6, (FLT *)&tracker->variance_per_sec);

	tracker->bsd_model.state_variance_per_second = cnVec(bsd_state_cnt, (FLT *)&tracker->variance_per_sec.BSD0);

	cnkalman_meas_model_init(&tracker->model, "IMU", &tracker->imu_model, lighthouse_integrate_imu_hfn);
	cnkalman_meas_model_t_lighthouse_imu_attach_config(ctx, &tracker->imu_model);
	tracker->imu_model.error_state_model = true;
	if (tracker->imu_model.term_criteria.max_iterations == -1)
		tracker->imu_model.term_criteria.max_iterations = 10;

	cnkalman_meas_model_init(&tracker->model, "obs", &tracker->obs_model, 0);
	cnkalman_meas_model_t_lighthouse_obs_attach_config(ctx, &tracker->obs_model);
	tracker->obs_model.error_state_model = false;
	if (tracker->obs_model.term_criteria.max_iterations == -1)
		tracker->obs_model.term_criteria.max_iterations = 10;

	tracker->push_cov = cnMat(tracker->model.error_state_size, tracker->model.error_state_size, tracker->push_cov_data);

	cn_set_diag(&tracker->model.P, (const FLT *)&baseline);
	cn_set_diag(&tracker->bsd_model.P, (const FLT *)&baseline.BSD0);
}

void survive_kalman_lighthouse_reset(SurviveKalmanLighthouse *tracker) {
	SurviveLighthouseKalmanErrorModel baseline = {
		.Lighthouse = {.Pos = {1e5, 1e5, 1e5}, .AxisAngleRot = {1e5, 1e5, 1e5}},
		.BSD0 = tracker->initial_variance,
		.BSD1 = tracker->initial_variance};

	cn_set_zero(&tracker->push_cov);
	cn_set_diag(&tracker->model.P, (const FLT *)&baseline);
	cn_set_diag(&tracker->bsd_model.P, (const FLT *)&baseline.BSD0);
}

SURVIVE_EXPORT void survive_kalman_lighthouse_integrate_observation(SurviveKalmanLighthouse *tracker,
																	const SurvivePose *pose, const CnMat *Rlh) {
	if (tracker == 0)
		return;

	CN_CREATE_STACK_MAT(H, 7, tracker->model.state_cnt);
	CN_CREATE_STACK_MAT(R, 7, 7);
	FLT v = cn_trace(&tracker->model.P);
	bool trustAbsolutely = v > 1e2 || Rlh == 0 || cn_trace(&tracker->push_cov) == 0;

	if (Rlh) {
		cn_matrix_copy(&R, Rlh);
	}

	if (!trustAbsolutely) {
#ifdef TRACK_IN_WORLD2LH
		SurvivePose Z = InvertPoseRtn(pose);
#else
		SurvivePose Z = *pose;
#endif
		cn_set_diag_val(&H, 1);

		if (cn_trace(&tracker->push_cov) > 0) {
			cnCopy(&tracker->push_cov, &tracker->model.P, 0);
			tracker->state = tracker->push_state;
		}

		FLT v[7] = {0};
		cn_get_diag(&R, v, 7);

		SurvivePose min_error = {.Pos = {1e-5, 1e-5, 1e-5}, .Rot = {1e-6, 1e-6, 1e-6, 1e-6}};
		CnMat minError = cnVec(7, min_error.Pos);
		cn_add_diag(&R, &minError, 1);

		CnMat Zp = cnMat(7, 1, (void *)&Z);
		cnkalman_meas_model_predict_update(0, &tracker->obs_model, &H, &Zp, &R);
		SurviveContext * ctx = tracker->ctx;

		SV_VERBOSE(10, "Observation for LH %d(ID: %08x) " SurvivePose_format " R " SurvivePose_format, tracker->lh,
				   (unsigned)ctx->bsd[tracker->lh].BaseStationID, SURVIVE_POSE_EXPAND(Z), LINMATH_VEC7_EXPAND(v));

	} else {
		tracker->state.Lighthouse = *pose;
		survive_covariance_pose2poseAA(&tracker->model.P, pose, &R);
	}
	integrate_imu(tracker);
	survive_kalman_lighthouse_report(tracker);
}
void survive_kalman_lighthouse_free(SurviveKalmanLighthouse *tracker) {
	SurviveKalmanLighthouse_detach_config(tracker->ctx, tracker);
	cnkalman_meas_model_t_lighthouse_imu_detach_config(tracker->ctx, &tracker->imu_model);
	cnkalman_meas_model_t_lighthouse_obs_detach_config(tracker->ctx, &tracker->obs_model);
	cnkalman_state_free(&tracker->model);
	cnkalman_state_free(&tracker->bsd_model);
	free(tracker);
}
