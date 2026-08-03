/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>

// clang-format off
static inline void predict_meas(CnMat* out, const FLT* x) {
	const FLT x0 = x[0];
	const FLT x1 = x[1];
	cnMatrixOptionalSet(out, 0, 0, sin((5 * (x1 * x1)) + (5 * (x0 * x0))));
	cnMatrixOptionalSet(out, 1, 0, sin(101 * x0) * sin(101 * x1));
	cnMatrixOptionalSet(out, 2, 0, cos(11 * x1) * sin(11 * x0));
}

static inline void predict_meas_jac_x(CnMat* Hx, const FLT* x) {
	const FLT x0 = x[0];
	const FLT x1 = x[1];
	const FLT x2 = 10 * cos((5 * (x1 * x1)) + (5 * (x0 * x0)));
	const FLT x3 = 101 * x1;
	const FLT x4 = 101 * x0;
	const FLT x5 = 11 * x1;
	const FLT x6 = 11 * x0;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, 0, x0 * x2);
	cnMatrixOptionalSet(Hx, 0, 1, x2 * x1);
	cnMatrixOptionalSet(Hx, 1, 0, 101 * sin(x3) * cos(x4));
	cnMatrixOptionalSet(Hx, 1, 1, 101 * sin(x4) * cos(x3));
	cnMatrixOptionalSet(Hx, 2, 0, 11 * cos(x6) * cos(x5));
	cnMatrixOptionalSet(Hx, 2, 1, -11 * sin(x6) * sin(x5));
}


    static inline void predict_meas_jac_x_with_hx(CnMat* Hx, CnMat* hx, const FLT* x) {
        if(hx != 0) { 
            predict_meas(hx, x);
        }
        if(Hx != 0) { 
            predict_meas_jac_x(Hx, x);
        }
    }
