#include "cnkalman/kalman.h"
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif
#include <cnkalman/kalman.h>
#include <cnkalman/numerical_diff.h>
#include <cnmatrix/cn_matrix.h>

#include <memory.h>
#include <stdio.h>

#include "cnkalman_internal.h"

#include "math.h"

#define KALMAN_LOG_LEVEL 1000

#define CN_KALMAN_VERBOSE(lvl, fmt, ...)                                                                               \
	{                                                                                                                  \
		if (k->log_level >= lvl) {                                                                                     \
			fprintf(stdout, fmt "\n", __VA_ARGS__);                                                                    \
		}                                                                                                              \
	}

void cnkalman_set_logging_level(cnkalman_state_t *k, int v) { k->log_level = v; }

void cnkalman_linear_update(struct CnMat *F, const struct CnMat *x0, struct CnMat *x1) {
	cnGEMM(F, x0, 1, 0, 0, x1, 0);
}

CN_EXPORT_FUNCTION void cnkalman_linear_transition_fn(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0, struct CnMat *x1, struct CnMat *f_out) {
    const CnMat* given_F = k->user;
    if(f_out) cnCopy(given_F, f_out, 0);
    if(x1) cnGEMM(given_F, x0, 1, 0, 0, x1, (enum cnGEMMFlags)0);
}
void kalman_print_mat_v(const cnkalman_state_t *k, int ll, const char *name, const CnMat *M, bool newlines) {
	if (k->log_level < ll) {
		return;
	}
	char term = newlines ? '\n' : ' ';
	if (!M) {
		fprintf(stdout, "null%c", term);
		return;
	}
	fprintf(stdout, "%8s %2d x %2d:%c", name, M->rows, M->cols, term);
	FLT scale = cn_sum(M);
	for (unsigned i = 0; i < M->rows; i++) {
		for (unsigned j = 0; j < M->cols; j++) {
			FLT v = cnMatrixGet(M, i, j);
			if (v == 0)
				fprintf(stdout, "             0, ");
			else
				fprintf(stdout, "%+7.7e,", v);
		}
		if (newlines && M->cols > 1)
			fprintf(stdout, "\n");
	}
	fprintf(stdout, "\n");
}
static void kalman_print_mat(const cnkalman_state_t *k, const char *name, const CnMat *M, bool newlines) {
	kalman_print_mat_v(k, KALMAN_LOG_LEVEL, name, M, newlines);
}

void user_is_q(void *user, FLT t, const struct CnMat *x, CnMat *Q_out) {
	const CnMat *q = (const CnMat *)user;
	cnScale(Q_out, q, t);
}

CN_EXPORT_FUNCTION void cnkalman_state_reset(cnkalman_state_t *k) {
	k->t = 0;
	cn_set_zero(&k->P);

	cnkalman_predict_state(10, k);
	k->t = 0;

	kalman_print_mat(k, "initial Pk_k", &k->P, true);
}

CN_EXPORT_FUNCTION void cnkalman_meas_model_multi_init(cnkalman_state_t **k, size_t cnt, const char *name,
                                                       cnkalman_meas_model_t *mk, kalman_measurement_model_fn_t Hfn) {
    memset(mk, 0, sizeof(*mk));
    assert(cnt < CNKALMAN_STATES_PER_MODEL);
    for(int i = 0;i < cnt;i++) {
        mk->ks[i] = k[i];
    }
    mk->ks_cnt = cnt;
    mk->name = name;
    mk->Hfn = Hfn;
    mk->term_criteria = (struct term_criteria_t){.max_iterations = 0, .xtol = 1e-2, .mtol = 1e-8, .minimum_step = .05};
}

CN_EXPORT_FUNCTION void cnkalman_meas_model_init(cnkalman_state_t *k, const char *name,
												   cnkalman_meas_model_t *mk, kalman_measurement_model_fn_t Hfn) {
	memset(mk, 0, sizeof(*mk));
	mk->ks[0] = k;
	mk->ks_cnt++;
	mk->name = name;
	mk->Hfn = Hfn;
	mk->term_criteria = (struct term_criteria_t){.max_iterations = 0, .xtol = 1e-2, .mtol = 1e-8, .minimum_step = .05};
}

void cnkalman_error_state_init(cnkalman_state_t *k, size_t state_cnt, size_t error_state_cnt,
							   kalman_transition_model_fn_t F, kalman_process_noise_fn_t q_fn,
							   kalman_error_state_model_fn_t Err_F, void *user, FLT *state) {
	memset(k, 0, sizeof(*k));

	k->state_cnt = (int)state_cnt;
	k->error_state_size = error_state_cnt;
	k->ErrorState_fn = Err_F;

	k->Q_fn = q_fn;

	size_t p_size = error_state_cnt;
	k->P = cnMatCalloc(p_size, p_size);

	k->Transition_fn = F;
	k->user = user;

	if (!state) {
		k->State_is_heap = true;
		state = (FLT*)calloc(1, sizeof(FLT) * k->state_cnt);
	}

	k->state = cnMat(k->state_cnt, 1, state);
}

void cnkalman_state_init(cnkalman_state_t *k, size_t state_cnt, kalman_transition_model_fn_t F,
							   kalman_process_noise_fn_t q_fn, void *user, FLT *state) {
	cnkalman_error_state_init(k, state_cnt, state_cnt, F, q_fn, 0, user, state);
}

void cnkalman_state_free(cnkalman_state_t *k) {
	free(k->P.data);
	k->P.data = 0;

	if (k->State_is_heap)
		free(CN_FLT_PTR(&k->state));
	k->state.data = 0;
}

void cnkalman_predict_covariance1(FLT dt, cnkalman_state_t *k, const CnMat *F, const CnMat *x,  CnMat *Pk1_k11) {
	int dims = k->state_cnt;

	CnMat *Pk1_k1 = &k->P;
	kalman_print_mat(k, "Pk-1_k-1", Pk1_k1, 1);

	// k->P = F * k->P * F^T + Q
	if(k->state_variance_per_second.rows > 0) {
		cn_add_diag(Pk1_k1, &k->state_variance_per_second, dt);
		assert(sane_covariance(Pk1_k1));
	}

	struct CnMat* Qp = 0;
	CN_CREATE_STACK_MAT(Q, k->error_state_size, k->error_state_size);
	if(k->Q_fn) {
		k->Q_fn(k->user, dt, x, &Q);
		Qp = &Q;
		assert(sane_covariance(&Q));
	}
	cn_ABAt_add(Pk1_k1, F, Pk1_k1, Qp);
	assert(sane_covariance(Pk1_k1));

	// printf("!!!! %f\n", cnDet(Pk1_k1));
	// assert(cnDet(Pk1_k1) >= 0);
	if (k->log_level >= KALMAN_LOG_LEVEL) {
		CN_KALMAN_VERBOSE(110, "T: %f", dt);
		kalman_print_mat(k, "Q", Qp, 1);
		kalman_print_mat(k, "F", F, 1);
		kalman_print_mat(k, "Pk1_k-1", Pk1_k1, 1);
	}
	CN_FREE_STACK_MAT(Q);
}

static inline void cnkalman_predict_covariance(FLT dt, const CnMat *F, const CnMat *x, const cnkalman_state_t *k, CnMat *P) {


	CnMat Pk1_k1;
	if(P->rows == P->cols && P->cols == k->P.cols) {
		Pk1_k1 = *P;
	} else {
		FLT *Pk1_k1_data = (FLT*)alloca((k->P.rows) * (k->P.cols) * sizeof(FLT));
		Pk1_k1 = cnMat(k->P.rows, k->P.cols, Pk1_k1_data);
	}

	if(Pk1_k1.data != k->P.data) {
		cnCopy(&k->P, &Pk1_k1, 0);
	}

	kalman_print_mat(k, "Pk-1_k-1", &Pk1_k1, 1);

	if(dt != 0) {
		// k->P = F * k->P * F^T + Q
		if (k->state_variance_per_second.rows > 0) {
			cn_add_diag(&Pk1_k1, &k->state_variance_per_second, fabs(dt));
			assert(sane_covariance(&Pk1_k1));
		}

		struct CnMat *Qp = 0;
		CN_CREATE_STACK_MAT(Q, k->error_state_size, k->error_state_size);
		if (k->Q_fn) {
			k->Q_fn(k->user, fabs(dt), x, &Q);
			Qp = &Q;
			assert(sane_covariance(&Q));
		}
		cn_ABAt_add(&Pk1_k1, F, &Pk1_k1, Qp);
		assert(sane_covariance(&Pk1_k1));

		// printf("!!!! %f\n", cnDet(Pk1_k1));
		// assert(cnDet(Pk1_k1) >= 0);
		if (k->log_level >= KALMAN_LOG_LEVEL) {
			CN_KALMAN_VERBOSE(110, "T: %f", dt);
			kalman_print_mat(k, "Q", Qp, 1);
			kalman_print_mat(k, "F", F, 1);
			kalman_print_mat(k, "Pk1_k-1", &Pk1_k1, 1);
		}
		CN_FREE_STACK_MAT(Q);
	}

	if(Pk1_k1.data != P->data) {
		if(P->cols == 1) {
			cn_get_diag(&Pk1_k1, cn_as_vector(P), P->rows);
		} else {
			CnMat Pk1_k1View = cnMatView(P->rows, P->cols, &Pk1_k1, 0, 0);
			cnCopy(&Pk1_k1View, P, 0);
		}
	}
}
void cnkalman_find_s(const cnkalman_state_t *k, cnkalman_gain_matrix *S, const CnMat *Pk_k1Ht, const struct CnMat *H) {
    kalman_print_mat(k, "H", H, 1);

    cnGEMM(H, Pk_k1Ht, 1, S, 1, S, 0);
    assert(cn_is_finite(S));

    kalman_print_mat(k, "S", S, 1);
}
void cnkalman_find_k_from_s(cnkalman_gain_matrix *K, const CnMat *Pk_k1Ht,
                            const CnMat *S) {

    CN_CREATE_STACK_MAT(iS, S->rows, S->rows);
    FLT diag = 0, non_diag = 0;
#define CHECK_DIAG
#ifdef CHECK_DIAG
    FLT* _S = S->data;
    for (int i = 0; i < S->rows; i++) {
        for (int j = 0; j < S->rows; j++) {
            if (i == j) {
                diag += fabs(_S[i + j * S->rows]);
                _iS[i + j * S->rows] = 1. / _S[i + j * S->rows];
            } else {
                non_diag += fabs(_S[i + j * S->rows]);
                _iS[i + j * S->rows] = 0;
            }
        }
    }
#endif
    if (diag == 0 || non_diag / diag > 1e-5) {
        cnInvert(S, &iS, CN_INVERT_METHOD_SVD);
    }
    assert(cn_is_finite(&iS));

    // K = Pk_k1Ht * iS
    cnGEMM(Pk_k1Ht, &iS, 1, 0, 0, K, 0);
}

int cnkalman_model_state_count(const cnkalman_meas_model_t *mk) {
    int rtn = 0;
    for(int i = 0;i < mk->ks_cnt;i++) {
        rtn += mk->ks[i]->state_cnt;
    }
    return rtn;
}

int cnkalman_model_filter_count(const cnkalman_meas_model_t *mk) {
    int rtn = 0;
    for(int i = 0;i < mk->ks_cnt;i++) {
        rtn += mk->ks[i]->error_state_size;
    }
    return rtn;
}

void cnkalman_find_k(const cnkalman_meas_model_t *mk, cnkalman_gain_matrix *K, const struct CnMat *H,
								  const CnMat *R) {
    int state_cnt = cnkalman_model_state_count(mk);
    int filter_cnt = cnkalman_model_filter_count(mk);

	CN_CREATE_STACK_MAT(Pk_k1Ht, filter_cnt, H->rows);
    CN_CREATE_STACK_MAT(S, H->rows, H->rows);
    CN_CREATE_STACK_MAT(tmpS, H->rows, H->rows);

    // S = H * P_k|k-1 * H^T + R
    if (R->cols == 1) {
        cn_set_diag(&S, cn_as_const_vector(R));
    } else {
        cnCopy(R, &S, 0);
    }

    for(int i = 0, state_idx = 0, filter_idx = 0;i < mk->ks_cnt;i++) {
	    cnkalman_state_t* k = mk->ks[i];
        const CnMat *Pk_k = &k->P;
        CnMat Pk_k1HtView = cnMatView(Pk_k->rows, H->rows, &Pk_k1Ht, filter_idx, 0);
        const CnMat HView = cnMatConstView(H->rows, k->error_state_size, H, 0, filter_idx);
        // Pk_k1Ht = P_k|k-1 * H^T
		if(Pk_k->rows != 0) {
			cnGEMM(Pk_k, &HView, 1, 0, 0, &Pk_k1HtView, CN_GEMM_FLAG_B_T);
			cnkalman_find_s(k, &S, &Pk_k1HtView, &HView);
		}

        state_idx += k->state_cnt;
        filter_idx += k->error_state_size;
    }

    cnkalman_find_k_from_s(K, &Pk_k1Ht, &S);
}

static void cnkalman_update_covariance(cnkalman_state_t *k, const cnkalman_gain_matrix *K,
											 const struct CnMat *H, const struct CnMat *R) {
	int filter_cnt = k->P.rows;
	CN_CREATE_STACK_MAT(eye, filter_cnt, filter_cnt);
	cn_set_diag_val(&eye, 1);

	CN_CREATE_STACK_MAT(ikh, filter_cnt, filter_cnt);

	// ikh = (I - K * H)
	cnGEMM(K, H, -1, &eye, 1, &ikh, 0);

	// cvGEMM does not like the same addresses for src and destination...
	CnMat *Pk_k = &k->P;
	CN_CREATE_STACK_MAT(tmp, filter_cnt, filter_cnt);
	cnCopy(Pk_k, &tmp, 0);

	CN_CREATE_STACK_MAT(kRkt, filter_cnt, filter_cnt);
	bool use_joseph_form = R->rows == R->cols;
	if (use_joseph_form) {
	  cn_ABAt_add(&kRkt, K, R, 0);
		for (int i = 0; i < kRkt.rows; i++) {
			FLT v = cnMatrixGet(&kRkt, i, i);
			cnMatrixSet(&kRkt, i, i, v > 0 ? v : 0);
		}
		assert(sane_covariance(&kRkt));
		cn_ABAt_add(Pk_k, &ikh, &tmp, &kRkt);
	} else {
		// P_k|k = (I - K * H) * P_k|k-1
		cnGEMM(&ikh, &tmp, 1, 0, 0, Pk_k, 0);
	}
	for (int i = 0; i < k->P.rows; i++) {
		cnMatrixSet(&k->P, i, i, fabs(cnMatrixGet(&k->P, i, i)));
		for (int j = i + 1; j < k->P.rows; j++) {
			FLT v1 = cnMatrixGet(&k->P, i, j);
			FLT v2 = cnMatrixGet(&k->P, j, i);
			FLT v = (v1 + v2) / 2.;
			if (fabs(v) < 1e-10)
				v = 0;
			cnMatrixSet(&k->P, i, j, v);
			cnMatrixSet(&k->P, j, i, v);
		}
	}
	// printf("!!!? %7.7f\n", cnDet(Pk_k));
	assert(sane_covariance(Pk_k));
	assert(cn_is_finite(Pk_k));
	if (k->log_level >= KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO gain\t");
		kalman_print_mat(k, "K", K, true);

		kalman_print_mat(k, "ikh", &ikh, true);

		fprintf(stdout, "INFO new Pk_k\t");
		kalman_print_mat(k, "Pk_k", Pk_k, true);
	}
	CN_FREE_STACK_MAT(tmp);
	CN_FREE_STACK_MAT(ikh);
	CN_FREE_STACK_MAT(eye);
}

static inline void cnkalman_predict(FLT t, const cnkalman_state_t *k, const CnMat *x_t0_t0, CnMat *x_t0_t1, CnMat* F) {
	// X_k|k-1 = Predict(X_K-1|k-1)
	if (k->log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_predict from ");
		kalman_print_mat(k, "x_t0_t0", x_t0_t0, false);
	}
	assert(cn_as_const_vector(x_t0_t0) != cn_as_const_vector(x_t0_t1));
	if (t == k->t || k->Transition_fn == 0) {
		cnCopy(x_t0_t0, x_t0_t1, 0);
		if(F) {
			cn_eye(F, 0);
		}
	} else {
		//assert(t > k->t);
		k->Transition_fn(t - k->t, k, x_t0_t0, x_t0_t1, F);
	}
	if (k->log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_predict to   ");
		kalman_print_mat(k, "x_t0_t1", x_t0_t1, false);
	}
	if (k->datalog) {
		CN_CREATE_STACK_MAT(tmp, x_t0_t0->rows, x_t0_t1->cols);
		cn_elementwise_subtract(&tmp, x_t0_t1, x_t0_t0);
		k->datalog(k, "predict_diff", cn_as_const_vector(&tmp), tmp.rows * tmp.cols);
	}
}

typedef struct numeric_jacobian_predict_fn_ctx {
    FLT dt;
    const cnkalman_state_t *k;
} numeric_jacobian_predict_fn_ctx;
static bool numeric_jacobian_predict_fn(void * user, const struct CnMat *x, struct CnMat *y) {
    numeric_jacobian_predict_fn_ctx* ctx = user;
    ctx->k->Transition_fn(ctx->dt, ctx->k, x, y, 0);
    return true;
}

static bool numeric_jacobian_predict(const cnkalman_state_t *k, enum cnkalman_jacobian_mode mode, FLT dt, const struct CnMat *x, CnMat *H) {
    numeric_jacobian_predict_fn_ctx ctx = {
            .dt = dt,
            .k = k
    };
    return cnkalman_numerical_differentiate(&ctx, mode == cnkalman_jacobian_mode_debug ?
    (enum cnkalman_numerical_differentiate_mode) cnkalman_jacobian_mode_two_sided : mode, numeric_jacobian_predict_fn, x, H);
}

typedef struct numeric_jacobian_meas_fn_ctx {
	cnkalman_meas_model_t *mk;
    void *user;
    const struct CnMat *Z, *x;
} numeric_jacobian_meas_fn_ctx;

static bool numeric_jacobian_meas_fn(void * user, const struct CnMat *x, struct CnMat *y) {
    numeric_jacobian_meas_fn_ctx* ctx = user;

	CN_CREATE_STACK_VEC(x1, cnkalman_model_state_count(ctx->mk));
	for(int i = 0, state_idx = 0, filter_idx = 0;i < ctx->mk->ks_cnt;i++) {
		cnkalman_state_t *k = ctx->mk->ks[i];

		CnMat x1_view = cnMatView(k->state_cnt, 1, &x1, state_idx, 0);

		if (k->ErrorState_fn && ctx->mk->error_state_model) {
			const CnMat ctx_x_view = cnMatConstView(k->state_cnt, 1, ctx->x, state_idx, 0);
			const CnMat x_view = cnMatConstView(k->error_state_size, 1, x, filter_idx, 0);
			k->Update_fn(user, &ctx_x_view, &x_view, &x1_view, 0);
		} else {
			const CnMat x_view = cnMatConstView(k->state_cnt, 1, x, state_idx, 0);
			cnCopy(&x_view, &x1_view, 0);
		}

		state_idx += k->state_cnt;
		filter_idx += k->error_state_size;
	}

	if (ctx->mk->Hfn(ctx->user, ctx->Z, &x1, y, 0) == 0)
		return false;

    // Hfn gives jacobian of measurement estimation E, y returns the residual (Z - E). So we invert it and its the form
    // we want going forward
    cnScale(y, y, -1);
    return true;
}

static inline bool compare_jacobs(const char* label, const CnMat *H, const CnMat *H_calc, const CnMat *y, const CnMat *Z) {
	bool needsPrint = true;

    for (int j = 0; j < H->cols; j++) {
		if(!needsPrint) {
			fprintf(stderr, "FJAC COLUMN %d\n", j);
		}
        for (int i = 0; i < H->rows; i++) {

            FLT deriv_u = cnMatrixGet(H, i, j);
            FLT deriv_n = cnMatrixGet(H_calc, i, j);
            FLT diff_abs = fabs(deriv_n - deriv_u);
            FLT diff_rel = diff_abs / (deriv_n + deriv_u + 1e-10);

            if ((diff_abs > 1e-2 && diff_rel > 1e-2)) {
				if(needsPrint) {
					fprintf(stderr, "FJAC DEBUG BEGIN %s %2dx%2d\n", label, H->rows, H->cols);
					fprintf(stderr, "FJAC COLUMN %d\n", j);

					needsPrint = false;
				}

                fprintf(stderr, "%2d %+7.7f %+7.7f %+7.7f %+7.7f %+7.7f %+7.7f \n", i, cn_as_const_vector(Z)[i],
						cn_as_const_vector(y)[i], deriv_u,
                        deriv_n, diff_abs, diff_rel);
            }
        }
    }
	if(!needsPrint) {
		fprintf(stderr, "FJAC DEBUG END\n");
	}
	return needsPrint;
}

CnMat *cnkalman_find_residual(cnkalman_meas_model_t *mk, void *user, const struct CnMat *Z,
										   const struct CnMat *x, CnMat *y, CnMat *H) {
	kalman_measurement_model_fn_t Hfn = mk->Hfn;
	int state_cnt = cnkalman_model_state_count(mk);
	int filter_cnt = cnkalman_model_filter_count(mk);

	if (H) {
		cn_set_constant(H, INFINITY);
	}

	CnMat *rtn = 0;
	CN_CREATE_STACK_MAT(HFullState, Z->rows, state_cnt);

    bool hasErrorState = state_cnt != filter_cnt;

    if (Hfn) {
		CnMat * Hfn_arg = (hasErrorState && !mk->error_state_model) ? &HFullState : H;
		bool needsJacobian = H && (mk->meas_jacobian_mode == cnkalman_jacobian_mode_user_fn || mk->meas_jacobian_mode == cnkalman_jacobian_mode_debug);
        bool okay = Hfn(user, Z, x, y, needsJacobian ? Hfn_arg : 0);
		if (okay == false) {
			return 0;
		}
		if(needsJacobian) {
			assert(cn_is_finite(Hfn_arg));
		}

		if (mk->meas_jacobian_mode != cnkalman_jacobian_mode_user_fn && H) {
			CN_CREATE_STACK_MAT(H_calc, Hfn_arg->rows, Hfn_arg->cols);

			CN_CREATE_STACK_VEC(xe, mk->error_state_model ? filter_cnt : state_cnt);

			if(!hasErrorState || !mk->error_state_model) {
				cnCopy(x, &xe, 0);
			}
			numeric_jacobian_meas_fn_ctx ctx = {
				.mk = mk,
				.user = user,
				.Z = Z,
				.x = x
			};
			cnkalman_numerical_differentiate_step_size(&ctx,
											 mk->meas_jacobian_mode == cnkalman_jacobian_mode_debug
												 ? cnkalman_numerical_differentiate_mode_two_sided : mk->meas_jacobian_mode,
													   mk->numeric_step_size, numeric_jacobian_meas_fn, &xe, &H_calc);

			if(mk->meas_jacobian_mode == cnkalman_jacobian_mode_debug) {
				mk->numeric_calcs++;
                if(!compare_jacobs(mk->name, Hfn_arg, &H_calc, y, Z)) {
					mk->numeric_misses++;
					fprintf(stderr, "User H: \n");
					cn_print_mat(Hfn_arg);
					fprintf(stderr, "Calculated H: \n");
					cn_print_mat(&H_calc);

					fprintf(stderr, "For state: ");
					cn_print_mat(x);
					fprintf(stderr, "For Z:     ");
					fprintf(stderr, "Numeric %f%% miss rate", 100. * (FLT)mk->numeric_misses / (FLT)mk->numeric_calcs++);
					cn_print_mat(Z);
				}
            }

            cn_matrix_copy(Hfn_arg, &H_calc);
		}
		rtn = H;
	} else {
		rtn = (struct CnMat *)user;
		cnGEMM(rtn, x, -1, Z, 1, y, 0);
	}

    if(!mk->error_state_model && hasErrorState && H) {
		for(int i = 0, state_idx = 0, filter_idx = 0;i < mk->ks_cnt;i++) {
			cnkalman_state_t* k = mk->ks[i];

			CN_CREATE_STACK_MAT(Hxsx, k->state_cnt, k->error_state_size);
			CnMat *HFn = Hfn ? &HFullState : rtn;
			CnMat HFn_view = cnMatView(HFn->rows, k->state_cnt, HFn, 0, state_idx);
			CnMat H_view = cnMatView(H->rows, k->error_state_size, H, 0, filter_idx);
			if (k->Update_fn && rtn && H && !mk->error_state_model) {
				const CnMat x_view = cnMatConstView(k->state_cnt, 1, x, state_idx, 0);
				k->Update_fn(user, &x_view, 0, 0, &Hxsx);
				cnGEMM(&HFn_view, &Hxsx, 1, 0, 0, &H_view, 0);
			} else {
				cnCopy(&HFn_view, &H_view, 0);
			}
			state_idx += k->state_cnt;
			filter_idx += k->error_state_size;
		}
		rtn = H;
    }

	assert(!rtn || cn_is_finite(rtn));
	assert(cn_is_finite(y));

	return rtn;
}

CN_EXPORT_FUNCTION void cnkalman_extrapolate_state(FLT t, const cnkalman_state_t *k, CnMat*x1, CnMat* P) {
	FLT dt = t - k->t;
	if(t == 0 || fabs(dt) < 1e-4){
		dt = 0;
	}

	int state_cnt = k->state_cnt;

	bool FInErrorState = k->error_state_transition || k->Transition_fn == 0;
	size_t f_size = (FInErrorState) ? k->error_state_size : state_cnt;
	CN_CREATE_STACK_MAT(F, f_size, f_size);
	CN_CREATE_STACK_MAT(FHxsx, k->error_state_size, k->error_state_size);
	cn_set_constant(&F, NAN);
	CnMat* FP = &F;

	CnMat x_k_k;
	if(x1->rows == k->state.rows) {
		x_k_k = *x1;
	} else {
		FLT *x_k_k_data = (FLT*)alloca((state_cnt) * sizeof(FLT));
		x_k_k = cnVec(k->state_cnt, x_k_k_data);
	}

	if (dt != 0) {
		CN_CREATE_STACK_MAT(x_k1_k1, state_cnt, 1);
		cn_matrix_copy(&x_k1_k1, &k->state);

		cnkalman_predict(t, k, &x_k1_k1, &x_k_k, &F);

		assert(cn_is_finite(&F));

		if(k->Update_fn && FInErrorState == false) {
			CN_CREATE_STACK_MAT(OldXOlde, state_cnt, k->error_state_size);
			k->Update_fn(k->user, &x_k1_k1, 0, 0, &OldXOlde);
			CN_CREATE_STACK_MAT(NewENewX, k->error_state_size, state_cnt);
			k->ErrorState_fn(k->user, &x_k_k, 0, 0, &NewENewX);

			CN_CREATE_STACK_MAT(FOldXOldE, F.rows, OldXOlde.cols);
			cnGEMM(&F, &OldXOlde, 1, 0, 0, &FOldXOldE, 0);
			cnGEMM(&NewENewX, &FOldXOldE, 1, 0, 0, &FHxsx, 0);
			FP = &FHxsx;
		}

		if(k->transition_jacobian_mode != cnkalman_jacobian_mode_user_fn) {
			CN_CREATE_STACK_MAT(F_calc, F.rows, F.cols);

			numeric_jacobian_predict(k, k->transition_jacobian_mode, dt, &x_k1_k1, &F_calc);

			if(k->transition_jacobian_mode == cnkalman_jacobian_mode_debug) {
				compare_jacobs("predict", &F, &F_calc, &x_k1_k1, &x_k_k);
			}

			cn_matrix_copy(&F, &F_calc);
		}
	} else {
		cn_matrix_copy(&x_k_k, &k->state);
	}

	if(P) {
		cnkalman_predict_covariance(dt, FP, &x_k_k, k, P);
	}

	if(x_k_k.data != x1->data) {
		memcpy(cn_as_vector(x1), cn_as_const_vector(&x_k_k), sizeof(FLT) * x1->rows);
	}

	CN_FREE_STACK_MAT(F);
}

void cnkalman_predict_state(FLT t, cnkalman_state_t *k) {
	cnkalman_extrapolate_state(t, k, &k->state, &k->P);
    k->t = t;
}

// https://arxiv.org/pdf/1702.00884.pdf
static void
calculate_adaptive_covariance(cnkalman_meas_model_t *mk, void *user, const struct CnMat *Z, CnMat *R,
                              CnMat *Pm, const struct CnMat *H) {
	assert(mk->ks_cnt == 1);
    const cnkalman_state_t *k = mk->ks[0];
    const CnMat *x_k_k = &k->state;
    int state_cnt = k->state_cnt;

    CN_CREATE_STACK_MAT(y, Z->rows, Z->cols);
    CN_CREATE_STACK_MAT(scaled_eTeHPkHt, Z->rows, Z->rows);
    CN_CREATE_STACK_MAT(yyt, Z->rows, Z->rows);

    cnkalman_find_residual(mk, user, Z, x_k_k, &y, 0);
    cnMulTransposed(&y, &yyt, false, 0, 1);

    CN_CREATE_STACK_MAT(Pk_k1Ht, state_cnt, H->rows);

    FLT a = .3;
    FLT b = 1 - a;
    cnGEMM(Pm, H, 1, 0, 0, &Pk_k1Ht, CN_GEMM_FLAG_B_T);
    cnGEMM(H, &Pk_k1Ht, b, &yyt, b, &scaled_eTeHPkHt, 0);

    kalman_print_mat_v(k, 200, "PkHt", &Pk_k1Ht, true);
    kalman_print_mat_v(k, 200, "yyt", &yyt, true);

    cnAddScaled(R, R, a, &scaled_eTeHPkHt, 1);

    kalman_print_mat_v(k, 200, "Adaptive R", R, true);

    CN_FREE_STACK_MAT(Pk_k1Ht);CN_FREE_STACK_MAT(yyt);CN_FREE_STACK_MAT(scaled_eTeHPkHt);
}


static FLT cnkalman_predict_update_state_extended_adaptive_internal(
	FLT t, void *user, const struct CnMat *Z, CnMat *R, cnkalman_meas_model_t *mk,
	struct cnkalman_update_extended_stats_t *stats) {

	assert(R->rows == Z->rows && (R->cols == 1 || R->cols == R->rows));
	assert(Z->cols == 1);

	if (R->cols == R->rows) {
		assert(sane_covariance(R));
	}
	kalman_measurement_model_fn_t Hfn = mk->Hfn;
	bool adaptive = mk->adaptive;

    int state_cnt = cnkalman_model_state_count(mk);
	int filter_cnt = cnkalman_model_filter_count(mk);

    FLT result = 0;

    // Setup the R matrix.
	if (adaptive && R->rows != R->cols) {
		assert(false);
		adaptive = false;
	}

    CnMat x_k_k = { 0 };
    CN_CREATE_STACK_VEC(x_k_k1, state_cnt);
    if(mk->ks_cnt == 1) {
        x_k_k = mk->ks[0]->state;
    } else {
        x_k_k = cnMat(state_cnt, 1, alloca(state_cnt * sizeof(FLT)));
    }

    // Run prediction steps -- gets new state, and covariance matrix based on time delta
    int state_idx = 0;
	for(int ki = 0;ki < mk->ks_cnt;ki++) {
        cnkalman_state_t *k = mk->ks[ki];
        cnkalman_predict_state(t, k);
        memcpy(cn_as_vector(&x_k_k1) + state_idx, cn_as_vector(&k->state), sizeof(FLT) * k->state_cnt);
        state_idx += k->state_cnt;
	}

    // Adaptive update happens on the covariance matrix prior; so save it.
    CN_CREATE_STACK_MAT(Pm, state_cnt, state_cnt);
    if (adaptive) {
        assert(mk->ks_cnt == 1);
        cn_matrix_copy(&Pm, &mk->ks[0]->P);
    }

	if (mk->ks[0]->log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_predict_update_state_extended t=%f", t);
		kalman_print_mat(mk->ks[0], "Z", Z, false);
		fprintf(stdout, "\n");
	}

	CN_CREATE_STACK_MAT(K, filter_cnt, Z->rows);
	CN_CREATE_STACK_MAT(HStorage, Z->rows, filter_cnt);
	struct CnMat *H = &HStorage;

	if (mk->term_criteria.max_iterations > 0) {
		result = cnkalman_run_iterations(mk, Z, R, user, &x_k_k1, &K, H, &x_k_k, stats);
		if (result < 0)
			return result;
	} else {
        CN_CREATE_STACK_MAT(y, Z->rows, Z->cols);
		H = cnkalman_find_residual(mk, user, Z, &x_k_k1, &y, H);

		if (H == 0) {
			return -1;
		}

        cnkalman_find_k(mk, &K, H, R);

		// Calculate the next state
		CN_CREATE_STACK_VEC(Ky, filter_cnt);
		cnGEMM(&K, &y, 1, 0, 0, &Ky, 0);

		int filter_idx = 0;
        for(int ki = 0, state_idx = 0;ki < mk->ks_cnt;ki++) {
            CnMat x_k_k1_view = cnMatView(mk->ks[ki]->state_cnt, 1, &x_k_k1, state_idx, 0);
            CnMat x_k_k_view = cnMatView(mk->ks[ki]->state_cnt, 1, &x_k_k, state_idx, 0);
            CnMat ky_view = cnMatView(mk->ks[ki]->error_state_size, 1, &Ky, filter_idx, 0);
            cnkalman_update_state(user, mk->ks[ki], &x_k_k1_view, 1, &ky_view, &x_k_k_view);
            result += cnNorm2(&y);
            state_idx += mk->ks[ki]->state_cnt;
            filter_idx += mk->ks[ki]->error_state_size;
        }

		if(stats) {
            CN_CREATE_STACK_MAT(yp, Z->rows, Z->cols);
            cnkalman_find_residual(mk, user, Z, &x_k_k, &yp, 0);
		    stats->origerror = sqrt(result);
            stats->besterror = cnNorm(&yp);
			if(stats->total_stats) {
				stats->total_stats->orignorm_acc += stats->origerror;
				stats->total_stats->bestnorm_acc += stats->besterror;

				stats->total_stats->total_runs++;
				stats->total_stats->total_hevals++;
				stats->total_stats->total_fevals++;
			}
		}
	}

	assert(cn_is_finite(H));
	assert(cn_is_finite(&K));

	if (mk->ks[0]->log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_update to    ");
		kalman_print_mat(mk->ks[0], "x1", &x_k_k, false);
	}

	if(mk->ks_cnt > 1 || x_k_k.data != mk->ks[0]->state.data) {
        for(int ki = 0, state_idx = 0;ki < mk->ks_cnt;ki++) {
            CnMat x_k_k_view = cnMatView(mk->ks[ki]->state_cnt, 1, &x_k_k, state_idx, 0);
            cnCopy(&x_k_k_view, &mk->ks[ki]->state, 0);
            state_idx += mk->ks[ki]->state_cnt;
        }
	}

    for(int ki = 0, state_idx = 0;ki < mk->ks_cnt;ki++) {
        CnMat kv = cnMatView(mk->ks[ki]->error_state_size, Z->rows, &K, state_idx, 0);
        CnMat hv = cnMatView(Z->rows, mk->ks[ki]->error_state_size, H, 0, state_idx);
        cnkalman_update_covariance(mk->ks[ki], &kv, &hv, R);
        state_idx += mk->ks[ki]->error_state_size;

        mk->ks[ki]->t = t;
        assert(sane_covariance(&mk->ks[ki]->P));
    }

	if (adaptive) {
        calculate_adaptive_covariance(mk, user, Z, R, &Pm, H);
    }

	CN_FREE_STACK_MAT(K);
	CN_FREE_STACK_MAT(HStorage);
	CN_FREE_STACK_MAT(x_k_k1);
	CN_FREE_STACK_MAT(Pm);

	return result;
}

FLT cnkalman_meas_model_predict_update_stats(FLT t, struct cnkalman_meas_model *mk, void *user,
												   const struct CnMat *Z, CnMat *R,
												   struct cnkalman_update_extended_stats_t *stats) {
	return cnkalman_predict_update_state_extended_adaptive_internal(t, user, Z, R, mk, stats);
}

FLT cnkalman_meas_model_predict_update(FLT t, struct cnkalman_meas_model *mk, void *user,
											 const struct CnMat *Z, CnMat *R) {
	struct cnkalman_update_extended_stats_t stats = {.total_stats = &mk->stats};
	return cnkalman_predict_update_state_extended_adaptive_internal(t, user, Z, R, mk, &stats);
}

FLT cnkalman_predict_update_state(FLT t, cnkalman_state_t *k, const struct CnMat *Z, const struct CnMat *H,
										CnMat *R, bool adaptive) {
	cnkalman_meas_model_t mk = {.adaptive = adaptive, .ks = {k}, .ks_cnt = 1, .term_criteria = { .max_iterations = 10}};
	return cnkalman_meas_model_predict_update(t, &mk, (void *)H, Z, R);
}

void cnkalman_set_P(cnkalman_state_t *k, const FLT *p) { cn_set_diag(&k->P, p); }
