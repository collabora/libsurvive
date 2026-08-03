#include <cnkalman/kalman.h>
#include <cnmatrix/cn_matrix.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <gtest/gtest.h>
#include "test_utils.h"

TEST(Kalman, Predict) {
    FLT _F[] = {
            0, 0, 0, 0, 1,
            0, 0, 0, 1, 0,
            0, 0, 1, 0, 0,
            0, 1, 0, 0, 0,
            1, 0, 0, 0, 0,
    };
    CnMat F = cnMat(5, 5, _F);
    FLT X[5] = { 1, 2, 3, 4, 5};

    cnkalman_state_t m = { 0 };
    cnkalman_state_init(&m, 5, cnkalman_linear_transition_fn, 0, &F, X);

    FLT X0[5] = { 1, 2, 3, 4, 5};
    ASSERT_DOUBLE_ARRAY_EQ(5, X, X0);
    cnkalman_predict_state(1, &m);
    cnkalman_state_free(&m);
}

void error_fn(void *user, const struct CnMat *x0, const struct CnMat *x1, struct CnMat *error_state, struct CnMat *E_jac_x1) {
    CN_CREATE_STACK_MAT(xd, x0->rows, 1);
    if(error_state) {
        cn_elementwise_subtract(&xd, x1, x0);
        for (int i = 0; i < x0->rows; i++) {
            cn_as_vector(error_state)[i] = 10. * cn_as_vector(&xd)[i];
        }
    }
    if(E_jac_x1) {
        cn_set_diag_val(E_jac_x1, 10);
    }
}
