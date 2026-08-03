#pragma once

#include "cnmatrix/cn_matrix.h"
#include "numerical_diff.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * This file contains a generic kalman implementation.
 *
 * This implementation tries to use the same nomenclature as:
 * https://en.wikipedia.org/wiki/Kalman_filter#Underlying_dynamical_system_model and
 * https://en.wikipedia.org/wiki/Extended_Kalman_filter.
 *
 * This implementation supports both nonlinear prediction models and nonlinear measurement models. Each phase
 * incorporates a time delta to approximate a continous model.
 *
 * Adaptive functionality:
 *
 * https://arxiv.org/pdf/1702.00884.pdf
 *
 * The R matrix should be initialized to reasonable values on the first all and then is updated based on the residual
 * error -- higher error generates higher variance values:
 *
 * R_k = a * R_k-1 + (1 - a) * (e*e^t + H * P_k-1 * H^t)
 *
 * a is set to .3 for this implementation.

 */

struct cnkalman_state_s;

typedef void (*kalman_normalize_fn_t)(void *user, struct CnMat *x);

// Given state x0 and time delta; gives the new state x1. For a linear model, this is just x1 = F * x0 and f_out is a
// constant / time dependent
typedef void (*kalman_transition_model_fn_t)(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0,
                                             struct CnMat *x1, struct CnMat *f_out);

// Given time and current state, generate the process noise Q_k.
typedef void (*kalman_process_noise_fn_t)(void *user, FLT dt, const struct CnMat *x, struct CnMat *Q_out);

// Given a measurement Z, and state X_t, generates both the y difference term and the H jacobian term.
typedef bool (*kalman_measurement_model_fn_t)(void *user, const struct CnMat *Z, const struct CnMat *x_t,
											  struct CnMat *y, struct CnMat *H_k);

/***
 * Given a state x0 and a model update Ky, generate the new state. Typically this is ~ x1 = x0 + error_state. Also
 * optionally generate the jacobian of the operation; d(x1)/d(error_state).
 */
typedef void (*kalman_integrate_update_fn_t)(void *user, const struct CnMat *x0, const struct CnMat *error_state,
										 struct CnMat *x1, struct CnMat * dX_wrt_error_state);

//typedef void (*kalman_error_state_model_fn_t)(void *user, const struct CnMat *x_t, struct CnMat *X_jac_E);
/***
 * Given states x0 and x1, generate the error state. Typically this is ~ error_state = x1 - x0. Also optionally generate
 * the jacobian of the operation; d(error_state) / d(x1)
 * @param x1
 * @param x0 Only provided if error_state is required; otherwise both are null
 * @param Fill in or the parameter state
 * @param X_jac_e Jacobian of the model state with respect to the error state
 */
typedef void (*kalman_error_state_model_fn_t)(void *user, const struct CnMat *x0, const struct CnMat *x1,
											  struct CnMat *error_state, struct CnMat *E_jac_x1);

typedef struct term_criteria_t {
	int max_iterations;

	// Absolute step size tolerance
	FLT minimum_step;

	// Minimum difference in errors
	FLT xtol;

	FLT mtol;

	FLT max_error;
} term_criteria_t;

enum cnkalman_update_extended_termination_reason {
	cnkalman_update_extended_termination_reason_none = 0,
	cnkalman_update_extended_termination_reason_invalid_jacobian,
	cnkalman_update_extended_termination_reason_too_high_error,
	cnkalman_update_extended_termination_reason_maxiter,
	cnkalman_update_extended_termination_reason_xtol,
	cnkalman_update_extended_termination_reason_step,
	cnkalman_update_extended_termination_reason_mtol,
	cnkalman_update_extended_termination_reason_MAX
};
CN_EXPORT_FUNCTION const char * cnkalman_update_extended_termination_reason_to_str(enum cnkalman_update_extended_termination_reason reason);

typedef struct cnkalman_update_extended_total_stats_t {
	FLT bestnorm_acc, orignorm_acc, bestnorm_meas_acc, bestnorm_delta_acc, orignorm_meas_acc;
	int total_iterations, total_fevals, total_hevals;
	int total_runs;
	int total_failures;
	FLT step_acc;
	int step_cnt;
	size_t stop_reason_counts[cnkalman_update_extended_termination_reason_MAX];
} cnkalman_update_extended_total_stats_t;

struct cnkalman_update_extended_stats_t {
	FLT bestnorm, bestnorm_meas, bestnorm_delta;
	FLT orignorm, orignorm_meas;
	FLT origerror, besterror;
	int iterations;
	int fevals, hevals;
	enum cnkalman_update_extended_termination_reason stop_reason;

	cnkalman_update_extended_total_stats_t *total_stats;
};

/**
 * This scheme heavily borrowed from mpfit
 */
enum cnkalman_jacobian_mode {
    cnkalman_jacobian_mode_user_fn = 0,
    cnkalman_jacobian_mode_two_sided = cnkalman_numerical_differentiate_mode_two_sided,
    cnkalman_jacobian_mode_one_sided_plus = cnkalman_numerical_differentiate_mode_one_sided_plus,
    cnkalman_jacobian_mode_one_sided_minus = cnkalman_numerical_differentiate_mode_one_sided_minus,
    cnkalman_jacobian_mode_debug = -1,
};

typedef struct cnkalman_state_s {
	// The number of states stored. For instance, something that tracked position and velocity would have 6 states --
	// [x, y, z, vx, vy, vz]
	int state_cnt;

	void *user;

    enum cnkalman_jacobian_mode transition_jacobian_mode;

	bool error_state_transition;
	kalman_transition_model_fn_t Transition_fn;

	struct CnMat state_variance_per_second;

	kalman_process_noise_fn_t Q_fn;
	kalman_normalize_fn_t normalize_fn;
	kalman_integrate_update_fn_t Update_fn;
	kalman_error_state_model_fn_t ErrorState_fn;
	int error_state_size;

	// Store the current covariance matrix (state_cnt x state_cnt)
	struct CnMat P;

	// Actual state matrix and whether its stored on the heap. Make no assumptions about how this matrix is organized.
	// it is always size of state_cnt*sizeof(FLT) though.
	bool State_is_heap;
	struct CnMat state;

	// Current time
	FLT t;

	int log_level;
	void *datalog_user;
	void (*datalog)(const struct cnkalman_state_s *state, const char *name, const FLT *v, size_t length);
} cnkalman_state_t;

#define CNKALMAN_STATES_PER_MODEL 8
typedef struct cnkalman_meas_model {
	cnkalman_state_t* ks[CNKALMAN_STATES_PER_MODEL];
	size_t ks_cnt;
	//cnkalman_state_t *k;
    enum cnkalman_jacobian_mode meas_jacobian_mode;
	size_t numeric_misses, numeric_calcs;
	FLT numeric_step_size;

	const char *name;
	kalman_measurement_model_fn_t Hfn;
	bool error_state_model;
	bool adaptive;

	struct term_criteria_t term_criteria;
	cnkalman_update_extended_total_stats_t stats;
} cnkalman_meas_model_t;

CN_EXPORT_FUNCTION FLT cnkalman_meas_model_predict_update_stats(FLT t, cnkalman_meas_model_t *mk, void *user,
																  const struct CnMat *Z, CnMat *R,
																  struct cnkalman_update_extended_stats_t *stats);
CN_EXPORT_FUNCTION FLT cnkalman_meas_model_predict_update(FLT t, cnkalman_meas_model_t *mk, void *user,
															const struct CnMat *Z, CnMat *R);

/**
 * Predict the state at a given delta; doesn't update the covariance matrix
 * @param t delta time
 * @param k kalman state info
 * @param index Which state vector to pull out
 * @param out Pre allocated output buffer.
 */
CN_EXPORT_FUNCTION void cnkalman_extrapolate_state(FLT t, const cnkalman_state_t *k, CnMat*x1, CnMat* P);

CN_EXPORT_FUNCTION void cnkalman_predict_state(FLT t, cnkalman_state_t *k);

/**
 * Run predict and update, updating the state matrix. This is for purely linear measurement models.
 *
 * @param t absolute time
 * @param k kalman state info
 * @param z measurement -- CnMat of n x 1
 * @param H Input observation model -- CnMat of n x state_cnt
 * @param R Observation noise -- The diagonal of the measurement covariance matrix; length n
 * @param adapative Whether or not R is an adaptive matrix. When true, R should be a full n x n matrix.
 *
 */
CN_EXPORT_FUNCTION FLT cnkalman_predict_update_state(FLT t, cnkalman_state_t *k, const struct CnMat *Z,
													   const struct CnMat *H, CnMat *R, bool adaptive);

/**
 * Run predict and update, updating the state matrix. This is for non-linear measurement models.
 *
 * @param t absolute time
 * @param k kalman state info
 * @param z measurement -- CnMat of n x 1
 * @param R Observation noise -- The diagonal of the measurement covariance matrix; length n
 * @param extended_params parameters for the non linear update
 * @param stats store stats if requested
 *
 * @returns Returns the average residual error
 */
/*
CN_EXPORT_FUNCTION FLT
cnkalman_predict_update_state_extended(FLT t, cnkalman_state_t *k, const struct CnMat *Z, CnMat* R,
											 const cnkalman_update_extended_params_t *extended_params,
											 struct cnkalman_update_extended_stats_t *stats);
*/
/**
 * Initialize a kalman state object
 * @param k object to initialize
 * @param state_cnt Length of state vector
 * @param F Transition function
 * @param q_fn Noise function
 * @param user pointer to give to user functions
 * @param state Optional state. Pass 0 to malloc one. Otherwise should point to a vector of at least state_cnt FLTs.
 *
 * @returns Returns the average residual error
 */
CN_EXPORT_FUNCTION void cnkalman_state_init(cnkalman_state_t *k, size_t state_cnt,
											kalman_transition_model_fn_t F, kalman_process_noise_fn_t q_fn,
											void *user, FLT *state);

CN_EXPORT_FUNCTION void cnkalman_error_state_init(cnkalman_state_t *k, size_t state_cnt, size_t error_state_cnt,
												  kalman_transition_model_fn_t F,
												  kalman_process_noise_fn_t q_fn,
												  kalman_error_state_model_fn_t Err_F,
												  void *user, FLT *state);

CN_EXPORT_FUNCTION void cnkalman_meas_model_init(cnkalman_state_t *k, const char *name,
												   cnkalman_meas_model_t *mk, kalman_measurement_model_fn_t Hfn);
CN_EXPORT_FUNCTION void cnkalman_meas_model_multi_init(cnkalman_state_t **k, size_t k_cnt, const char *name,
                                                 cnkalman_meas_model_t *mk, kalman_measurement_model_fn_t Hfn);

CN_EXPORT_FUNCTION void cnkalman_state_reset(cnkalman_state_t *k);

CN_EXPORT_FUNCTION void cnkalman_state_free(cnkalman_state_t *k);
CN_EXPORT_FUNCTION void cnkalman_set_P(cnkalman_state_t *k, const FLT *d);
CN_EXPORT_FUNCTION void cnkalman_set_logging_level(cnkalman_state_t *k, int verbosity);

CN_EXPORT_FUNCTION void cnkalman_linear_update(struct CnMat *F, const struct CnMat *x0, struct CnMat *x1);

CN_EXPORT_FUNCTION void cnkalman_linear_transition_fn(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0, struct CnMat *x1, struct CnMat *f_out);

#ifdef __cplusplus
}
#endif
