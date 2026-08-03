#include <cnkalman/kalman.h>
#include <cnmatrix/cn_matrix.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <gtest/gtest.h>
#include "test_utils.h"

void F(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0, struct CnMat *x1, struct CnMat *f_out) {
    if(x1) cn_matrix_copy(x1, x0);
    if(f_out) cn_set_diag_val(f_out, 1);
}

void Hfn(void *user, const struct CnMat *x_t, struct CnMat *hx, struct CnMat *Hx) {
    const CnMat* H = static_cast<const CnMat *>(user);

    if(Hx) cnCopy(H, Hx, 0);
    if(hx) cnGEMM(H, x_t, 1, 0, 0, hx, (enum cnGEMMFlags)0);
}
bool Hx(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y, struct CnMat *H_k) {
    CN_CREATE_STACK_VEC(Hx, Z->rows);
    Hfn(user, x_t, &Hx, H_k);
    cn_elementwise_subtract(y, Z, &Hx);
    return true;
}

void random_cov(CnMat* dst, FLT sig) {
    CN_CREATE_STACK_VEC(D, dst->rows);
    cnRand(&D, 0, sig);

    CN_CREATE_STACK_MAT(S, dst->rows, dst->rows);
    cnRand(&S, 0, sig);
    cnGEMM(&S, &S, 1, 0, 1, dst, CN_GEMM_FLAG_B_T);

    for(int i = 0;i < dst->rows;i++) cnMatrixSet(dst, i, i, cnMatrixGet(dst, i, i) + fabs(cn_as_vector(&D)[i]));

}

cnkalman_state_t combine_states(cnkalman_state_t** ks, int ks_cnt) {
    cnkalman_state_t rtn = { 0 };
    int state_cnt = 0, error_state_cnt = 0;
    for(int i = 0;i < ks_cnt;i++) {
        cnkalman_state_t* k = ks[i];
        state_cnt += k->state_cnt;
        error_state_cnt += k->state_cnt;
    }

    cnkalman_state_init(&rtn, state_cnt, ks[0]->Transition_fn, 0, 0, 0);
    for(int i = 0, state_idx = 0, error_state_idx = 0;i < ks_cnt;i++) {
        cnkalman_state_t* k = ks[i];
        CnMat xView = cnMatView(k->state_cnt, 1, &rtn.state, state_idx, 0);
        CnMat pView = cnMatView(k->error_state_size, k->error_state_size, &rtn.P, state_idx, state_idx);
        cnCopy(&ks[i]->P, &pView, 0);
        cnCopy(&ks[i]->state, &xView, 0);
        state_idx += k->state_cnt;
        error_state_idx += k->state_cnt;
    }

    return rtn;
}


TEST(Kalman, MultiTest) {
    CN_CREATE_STACK_MAT(H, 20, 10);
    cnRand(&H, 1, .1);
    //cn_set_diag_val(&H, 11);

    cnkalman_state_t m2 = { 0 }, m3 = { 0 };
    //cnkalman_state_init(&m1, 10, F, 0, 0, 0);
    cnkalman_state_init(&m2, 5, F, 0, 0, 0);
    cnkalman_state_init(&m3, 5, F, 0, 0, 0);

    //random_cov(&m1.P, 100);
    random_cov(&m2.P, 1);
    random_cov(&m3.P, 1);

    cnkalman_state_t *mdls[] = {&m2, &m3};
    cnkalman_state_t m1 = combine_states(mdls, 2);

    CN_CREATE_STACK_VEC(x0, 10);
    cnRand(&x0, 0, .5);

    CN_CREATE_STACK_VEC(Z, 20);
    CN_CREATE_STACK_VEC(n, 20);
    cnRand(&n, 0, 1e-4);
    Hfn(&H, &x0, &Z, 0);
    cn_elementwise_add(&Z, &Z, &n);

    CN_CREATE_STACK_VEC(R, 20);
    cn_set_constant(&R, 1e-3);

    cn_print_mat(&m1.P);
    cn_print_mat(&m2.P);
    cn_print_mat(&m3.P);

    cn_print_mat(&R);
    {
        cnkalman_meas_model meas = {0};
        cnkalman_state_t *mdls[] = {&m1};
        cnkalman_meas_model_multi_init(mdls, 1, "meas", &meas, Hx);
        meas.term_criteria.max_iterations = 10;
        cnkalman_meas_model_predict_update(1, &meas, &H, &Z, &R);
    }
    {
        cnkalman_meas_model meas = {0};
        cnkalman_state_t *mdls[] = {&m2, &m3};
        cnkalman_meas_model_multi_init(mdls, 2, "meas", &meas, Hx);
        meas.term_criteria.max_iterations = 10;
        cnkalman_meas_model_predict_update(1, &meas, &H, &Z, &R);
    }
    cn_print_mat(&m1.P);
    cn_print_mat(&x0);

    CnMat m1_2 = cnMatView(5, 1, &m1.state, 0, 0);
    CnMat p1_2 = cnMatView(5, 5, &m1.P, 0, 0);
    CnMat m1_3 = cnMatView(5, 1, &m1.state, 5, 0);
    CnMat p1_3 = cnMatView(5, 5, &m1.P, 5, 5);
    cn_print_mat(&m1.state);
    cn_print_mat(&m2.state);
    cn_print_mat(&m3.state);

    EXPECT_TRUE(is_mat_near(&m1_2, &m2.state, 1e-7));
    EXPECT_TRUE(is_mat_near(&m1_3, &m3.state, 1e-7));
    EXPECT_TRUE(is_mat_near(&p1_2, &m2.P, 1e-7));
    EXPECT_TRUE(is_mat_near(&p1_3, &m3.P, 1e-7));

    cn_print_mat(&m1.P);
    cn_print_mat(&m2.P);
    cn_print_mat(&m3.P);

    cnkalman_state_free(&m1);
    cnkalman_state_free(&m2);
    cnkalman_state_free(&m3);
}