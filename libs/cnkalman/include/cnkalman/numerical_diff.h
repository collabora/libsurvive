#pragma once

#include <cnmatrix/cn_matrix.h>

#ifdef __cplusplus
extern "C" {
#endif
    enum cnkalman_numerical_differentiate_mode {
        cnkalman_numerical_differentiate_mode_two_sided = 1,
        cnkalman_numerical_differentiate_mode_one_sided_plus = 2,
        cnkalman_numerical_differentiate_mode_one_sided_minus = 3
    };

typedef bool (*cnkalman_eval_fn_t)(void * user, const struct CnMat *x, struct CnMat *y);

bool cnkalman_numerical_differentiate(void * user, enum cnkalman_numerical_differentiate_mode m, cnkalman_eval_fn_t fn, const CnMat* x, CnMat* H);
bool cnkalman_numerical_differentiate_step_size(void * user, enum cnkalman_numerical_differentiate_mode m, FLT step_size, cnkalman_eval_fn_t fn, const CnMat* x, CnMat* H);
#ifdef __cplusplus
}
#endif
