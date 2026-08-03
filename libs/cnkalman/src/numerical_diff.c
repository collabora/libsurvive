#include <cnkalman/numerical_diff.h>

bool cnkalman_numerical_differentiate(void * user, enum cnkalman_numerical_differentiate_mode mode, cnkalman_eval_fn_t fn, const CnMat* cx, CnMat* H) {
	return cnkalman_numerical_differentiate_step_size(user, mode, -1, fn, cx, H);
}

bool cnkalman_numerical_differentiate_step_size(void * user, enum cnkalman_numerical_differentiate_mode mode, FLT step_size, cnkalman_eval_fn_t fn, const CnMat* cx, CnMat* H) {
	CN_CREATE_STACK_VEC(y1, H->rows);
	CN_CREATE_STACK_VEC(y2, H->rows);

	CN_CREATE_STACK_VEC(x, cx->rows);
	cn_matrix_copy(&x, cx);

	if(mode != cnkalman_numerical_differentiate_mode_two_sided) {
		if(fn(user, &x, &y2) == false) return false;
	}

	FLT* xa = cn_as_vector(&x);
	for(int i = 0;i < cx->rows;i++) {
		FLT s = step_size;
		if(step_size <= 0) {
			s = xa[i] == 0 ? 1e-7 : fmax(1e-9, fabs(xa[i] * 1e-8));
		}

		if(mode == cnkalman_numerical_differentiate_mode_two_sided) {
			xa[i] += s;
			if (fn(user, &x, &y1) == false) return false;
			xa[i] -= 2 * s;
			if (fn(user, &x, &y2) == false) return false;
			xa[i] += s;
			s = 2 * s;
		} else {
			s *= mode == cnkalman_numerical_differentiate_mode_one_sided_minus ? -1 : 1;
			xa[i] += s;
			if (fn(user, &x, &y1) == false) return false;
			xa[i] -= s;
		}

		for(int j = 0;j < H->rows;j++) {
			cnMatrixSet(H, j, i, (_y1[j] - _y2[j])/s);
		}
	}

	return true;
}