#include <cnkalman/kalman.h>
#include <cnmatrix/cn_matrix.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <gtest/gtest.h>
#include "test_utils.h"

// Generates the transition matrix F
void transition(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0,
											 struct CnMat *x1, struct CnMat *f_out) {
	if(f_out) {
		cn_set_zero(f_out);
		cn_set_diag_val(f_out, 1);
	}
	if(x1) {
		cnCopy(x0, x1, 0);
	}
}

typedef FLT LinmathPoint2d[2];

static void process_noise(void *user, FLT dt, const struct CnMat *x, struct CnMat *Q_out) {
	cn_set_zero(Q_out);
	cn_set_diag_val(Q_out, .1);
}

static FLT measurement_model(const LinmathPoint2d sensor_pos, const LinmathPoint2d state) {
	return atan2(state[1] - sensor_pos[1], state[0] - sensor_pos[0]);
	// return atan2(state[0] - sensor_pos[0], state[1] - sensor_pos[1]);
}

static bool toy_measurement_model(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
								  struct CnMat *H_k) {
	auto sensors = (const LinmathPoint2d *)user;

	for (int i = 0; i < Z->rows; i++) {
		if (y) {
			y->data[i] = Z->data[i] - measurement_model(sensors[i], x_t->data);
		}
		if (H_k) {
			FLT dx = x_t->data[0] - sensors[i][0];
			FLT dy = x_t->data[1] - sensors[i][1];
			FLT scale = 1. / ((dx * dx) + (dy * dy));
			cnMatrixSet(H_k, i, 0, scale * -dy);
			cnMatrixSet(H_k, i, 1, scale * dx);
		}
	}

	return true;
}

void run_standard_experiment(LinmathPoint2d X_out, FLT *P, const term_criteria_t *termCriteria, int time_steps) {
	LinmathPoint2d true_state = {1.5, 1.5};
	FLT X[2] = {0.5, 0.1};
	LinmathPoint2d sensors[2] = {{0, 0}, {1.5, 0}};
	cnkalman_state_t state = {};
	cnkalman_state_init(&state, 2, transition, process_noise, 0, X);
	//    state.debug_jacobian = true;
	//state.log_level = 101;
	cn_set_diag_val(&state.P, .1);

	CN_CREATE_STACK_MAT(Z, 2, 1);
	FLT Rv = M_PI * M_PI * 1e-5;
	FLT _R[] = {Rv, Rv};

	for (int i = 0; i < 2; i++) {
		Z.data[i] = measurement_model(sensors[i], true_state);
	}

	struct cnkalman_meas_model meas_model = {
		.ks = {&state},
		.ks_cnt = 1,
		.Hfn = toy_measurement_model,
		.term_criteria = *termCriteria,
	};

	CnMat R = cnVec(2, _R);

	for (int i = 0; i < time_steps; i++) {
		cnkalman_meas_model_predict_update(1, &meas_model, sensors, &Z, &R);
		printf("%3d: %7.6f %7.6f\n", i, X[0], X[1]);
	}

	memcpy(P, state.P.data, sizeof(FLT) * 4);
	memcpy(X_out, X, sizeof(FLT) * 2);
	cnkalman_state_free(&state);
}

// https://www.diva-portal.org/smash/get/diva2:844060/FULLTEXT01.pdf

TEST(Kalman, EKFTest) {
	/**
	 * These values are not the ones shown in the chart in the paper, but it's unclear if that chart is with noise or
	 * without noise in the measurement. Results reproduced in octave:

	  x0 = [.5, .1]
	  PQv = .1 * .1
	  P0 = [PQv 0; 0 PQv]
	  Q = [ 0 0; 0 0]
	  rv = pi*pi * 10^-5
	  R = [rv 0; 0 rv]

	  P0_1 = P0 + Q
	  x_t = [1.5 1.5]
	  S0 = [ 0, 0]
	  S1 = [ 1.5, 0]
	  y0 = atan2(x_t(2) - S0(2), x_t(1) - S0(1))
	  y1 = atan2(x_t(2) - S1(2), x_t(1) - S1(1))
	  y = [y0; y1]
	  h0 = atan2(x0(2) - S0(2), x0(1) - S0(1))
	  h1 = atan2(x0(2) - S1(2), x0(1) - S1(1))
	  hx = [h0;h1]
	  H0 = [-(x0(2) - S0(2)) (x0(1) - S0(1))] / ( (x0(2) - S0(2))^2 + (x0(1) - S0(1))^2)
	  H1 = [-(x0(2) - S1(2)) (x0(1) - S1(1))] / ( (x0(2) - S1(2))^2 + (x0(1) - S1(1))^2)
	  H = [H0; H1]

	  K0 = P0_1 * H' * (H * P0_1 * H' + R)^-1

	  x1 = x0' + K0 * (y - hx)
	  P1 = (eye(2) - K0 * H) * P0_1

	 */
	FLT expected_X[2] = {4.4049048331227372, 1.1884307714294169};
	FLT expected_P[4] = {0.0014050624776344668, 0.00019267084936229752, 0.0001926708493623336, 4.751355804986091e-05};

	FLT X[2];
	FLT P[4];
	term_criteria_t termCriteria = {};

	run_standard_experiment(X, P, &termCriteria, 1);

	LinmathPoint2d true_state = {1.5, 1.5};
	CnMat Xm = cnVec(2, X);
	CnMat Tm = cnVec(2, true_state);
	FLT error = cnDistance(&Xm, &Tm);

	EXPECT_ARRAY_NEAR(2, X, expected_X, 1e-5);
	EXPECT_ARRAY_NEAR(4, P, expected_P, 1e-5);
}

TEST(Kalman, IEKFTest) {
	FLT X[2];
	FLT P[4];
	term_criteria_t termCriteria = {.max_iterations = 10};
	run_standard_experiment(X, P, &termCriteria, 1);

	LinmathPoint2d true_state = {1.5, 1.5};
	CnMat Xm = cnVec(2, X);
	CnMat Tm = cnVec(2, true_state);

	FLT error = cnDistance(&Xm, &Tm);
	ASSERT_GT(.17, error);
}

static void process_noise2(void *user, FLT dt, const struct CnMat *x, struct CnMat *Q_out) { cn_set_zero(Q_out); }

static bool toy_measurement_model2(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y,
								   struct CnMat *H_k) {
	if (y) {
		y->data[0] = Z->data[0] - -(x_t->data[0] * x_t->data[0]);
	}
	if (H_k) {
		cnMatrixSet(H_k, 0, 0, -2 * x_t->data[0]);
	}

	return true;
}

void run_standard_experiment2(LinmathPoint2d X_out, FLT *P, const term_criteria_t *termCriteria, int time_steps,
							  struct cnkalman_update_extended_stats_t *stats) {
	FLT true_state[] = {1};
	FLT X[2] = {0.1};

	cnkalman_state_t state = {};
	cnkalman_state_init(&state, 1, transition, process_noise2, 0, X);
	//state.log_level = 1001;
	cn_set_diag_val(&state.P, 1);

	CN_CREATE_STACK_MAT(Z, 1, 1);
	FLT Rv = .1;
	Z.data[0] = -true_state[0] * true_state[0];

	cnkalman_meas_model_t measModel = {
		.ks = {&state},
		.ks_cnt = 1,
		.Hfn = toy_measurement_model2,
		.term_criteria = *termCriteria,
	};

	CN_CREATE_STACK_MAT(Rm, 1, 1);
	Rm.data[0] = Rv;

	for (int i = 0; i < time_steps; i++) {
		FLT error = cnkalman_meas_model_predict_update_stats(1, &measModel, 0, &Z, &Rm, stats);
		printf("%3d: %7.6f %7.6f\n", i, X[0], error);
	}

	memcpy(P, state.P.data, sizeof(FLT) * 1);
	memcpy(X_out, X, sizeof(FLT) * 1);
	cnkalman_state_free(&state);
}

TEST(Kalman, EKFTest2) {
	FLT expected_X = 1.5142857142857145;
	FLT expected_P = 0.71428571428571419;

	FLT X;
	FLT P;
	term_criteria_t termCriteria = {};

	run_standard_experiment2(&X, &P, &termCriteria, 1, 0);

	EXPECT_NEAR(X, expected_X, 1e-5);
	EXPECT_NEAR(P, expected_P, 1e-5);
}

TEST(Kalman, IEKFTest2) {
	FLT expected_X = 0.977300604309293;
	FLT expected_P = 0.025506798237172168;

	FLT X;
	FLT P;
	term_criteria_t termCriteria = {.max_iterations = 5};
	struct cnkalman_update_extended_stats_t stats = {};
	run_standard_experiment2(&X, &P, &termCriteria, 1, &stats);

	EXPECT_NEAR(X, expected_X, 1e-5);
	EXPECT_NEAR(P, expected_P, 1e-5);
	EXPECT_NEAR(stats.orignorm, 4.9005, 1e-5);
}
