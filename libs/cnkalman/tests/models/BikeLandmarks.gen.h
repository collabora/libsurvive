/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>

// clang-format off
static inline void predict_function(CnMat* out, const FLT dt, const FLT wheelbase, const FLT* state, const FLT* u) {
	const FLT state0 = state[0];
	const FLT state1 = state[1];
	const FLT state2 = state[2];
	const FLT u0 = u[0];
	const FLT u1 = u[1];
	const FLT x0 = tan(u1);
	const FLT x1 = state2 + (u0 * x0 * dt * (1. / wheelbase));
	const FLT x2 = (1. / x0) * wheelbase;
	cnMatrixOptionalSet(out, 0, 0, (-1 * x2 * sin(state2)) + state0 + (x2 * sin(x1)));
	cnMatrixOptionalSet(out, 1, 0, (x2 * cos(state2)) + state1 + (-1 * x2 * cos(x1)));
	cnMatrixOptionalSet(out, 2, 0, x1);
}

static inline void predict_function_jac_dt(CnMat* Hx, const FLT dt, const FLT wheelbase, const FLT* state, const FLT* u) {
	const FLT state2 = state[2];
	const FLT u0 = u[0];
	const FLT u1 = u[1];
	const FLT x0 = u0 * tan(u1) * (1. / wheelbase);
	const FLT x1 = state2 + (x0 * dt);
	cnMatrixOptionalSet(Hx, 0, 0, u0 * cos(x1));
	cnMatrixOptionalSet(Hx, 1, 0, u0 * sin(x1));
	cnMatrixOptionalSet(Hx, 2, 0, x0);
}


    static inline void predict_function_jac_dt_with_hx(CnMat* Hx, CnMat* hx, const FLT dt, const FLT wheelbase, const FLT* state, const FLT* u) {
        if(hx != 0) { 
            predict_function(hx, dt, wheelbase, state, u);
        }
        if(Hx != 0) { 
            predict_function_jac_dt(Hx, dt, wheelbase, state, u);
        }
    }
static inline void predict_function_jac_wheelbase(CnMat* Hx, const FLT dt, const FLT wheelbase, const FLT* state, const FLT* u) {
	const FLT state2 = state[2];
	const FLT u0 = u[0];
	const FLT u1 = u[1];
	const FLT x0 = tan(u1);
	const FLT x1 = 1. / x0;
	const FLT x2 = u0 * dt;
	const FLT x3 = x2 * (1. / wheelbase);
	const FLT x4 = state2 + (x0 * x3);
	const FLT x5 = cos(x4);
	const FLT x6 = sin(x4);
	cnMatrixOptionalSet(Hx, 0, 0, (x1 * x6) + (-1 * x1 * sin(state2)) + (-1 * x3 * x5));
	cnMatrixOptionalSet(Hx, 1, 0, (-1 * x1 * x5) + (x1 * cos(state2)) + (-1 * x3 * x6));
	cnMatrixOptionalSet(Hx, 2, 0, -1 * x0 * x2 * (1. / (wheelbase * wheelbase)));
}


    static inline void predict_function_jac_wheelbase_with_hx(CnMat* Hx, CnMat* hx, const FLT dt, const FLT wheelbase, const FLT* state, const FLT* u) {
        if(hx != 0) { 
            predict_function(hx, dt, wheelbase, state, u);
        }
        if(Hx != 0) { 
            predict_function_jac_wheelbase(Hx, dt, wheelbase, state, u);
        }
    }
static inline void predict_function_jac_state(CnMat* Hx, const FLT dt, const FLT wheelbase, const FLT* state, const FLT* u) {
	const FLT state2 = state[2];
	const FLT u0 = u[0];
	const FLT u1 = u[1];
	const FLT x0 = tan(u1);
	const FLT x1 = (1. / x0) * wheelbase;
	const FLT x2 = state2 + (u0 * x0 * dt * (1. / wheelbase));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, 0, 1);
	cnMatrixOptionalSet(Hx, 0, 2, (x1 * cos(x2)) + (-1 * x1 * cos(state2)));
	cnMatrixOptionalSet(Hx, 1, 1, 1);
	cnMatrixOptionalSet(Hx, 1, 2, (x1 * sin(x2)) + (-1 * x1 * sin(state2)));
	cnMatrixOptionalSet(Hx, 2, 2, 1);
}


    static inline void predict_function_jac_state_with_hx(CnMat* Hx, CnMat* hx, const FLT dt, const FLT wheelbase, const FLT* state, const FLT* u) {
        if(hx != 0) { 
            predict_function(hx, dt, wheelbase, state, u);
        }
        if(Hx != 0) { 
            predict_function_jac_state(Hx, dt, wheelbase, state, u);
        }
    }
static inline void predict_function_jac_u(CnMat* Hx, const FLT dt, const FLT wheelbase, const FLT* state, const FLT* u) {
	const FLT state2 = state[2];
	const FLT u0 = u[0];
	const FLT u1 = u[1];
	const FLT x0 = tan(u1);
	const FLT x1 = dt * (1. / wheelbase);
	const FLT x2 = x0 * x1;
	const FLT x3 = state2 + (u0 * x2);
	const FLT x4 = cos(x3);
	const FLT x5 = x4 * dt;
	const FLT x6 = x0 * x0;
	const FLT x7 = 1 + x6;
	const FLT x8 = (1. / x6) * x7 * wheelbase;
	const FLT x9 = u0 * x7;
	const FLT x10 = (1. / x0) * x9;
	const FLT x11 = sin(x3);
	const FLT x12 = dt * x11;
	cnMatrixOptionalSet(Hx, 0, 0, x5);
	cnMatrixOptionalSet(Hx, 0, 1, (-1 * x8 * x11) + (x8 * sin(state2)) + (x5 * x10));
	cnMatrixOptionalSet(Hx, 1, 0, x12);
	cnMatrixOptionalSet(Hx, 1, 1, (x4 * x8) + (-1 * x8 * cos(state2)) + (x12 * x10));
	cnMatrixOptionalSet(Hx, 2, 0, x2);
	cnMatrixOptionalSet(Hx, 2, 1, x1 * x9);
}


    static inline void predict_function_jac_u_with_hx(CnMat* Hx, CnMat* hx, const FLT dt, const FLT wheelbase, const FLT* state, const FLT* u) {
        if(hx != 0) { 
            predict_function(hx, dt, wheelbase, state, u);
        }
        if(Hx != 0) { 
            predict_function_jac_u(Hx, dt, wheelbase, state, u);
        }
    }
static inline void meas_function(CnMat* out, const FLT* state, const FLT* landmark) {
	const FLT state0 = state[0];
	const FLT state1 = state[1];
	const FLT state2 = state[2];
	const FLT landmark0 = landmark[0];
	const FLT landmark1 = landmark[1];
	const FLT x0 = landmark1 + (-1 * state1);
	const FLT x1 = landmark0 + (-1 * state0);
	cnMatrixOptionalSet(out, 0, 0, sqrt((x1 * x1) + (x0 * x0)));
	cnMatrixOptionalSet(out, 1, 0, atan2(x0, x1) + (-1 * state2));
}

static inline void meas_function_jac_state(CnMat* Hx, const FLT* state, const FLT* landmark) {
	const FLT state0 = state[0];
	const FLT state1 = state[1];
	const FLT landmark0 = landmark[0];
	const FLT landmark1 = landmark[1];
	const FLT x0 = landmark1 + (-1 * state1);
	const FLT x1 = landmark0 + (-1 * state0);
	const FLT x2 = (x1 * x1) + (x0 * x0);
	const FLT x3 = 1. / sqrt(x2);
	const FLT x4 = 1. / x2;
	cnMatrixOptionalSet(Hx, 0, 0, -1 * x1 * x3);
	cnMatrixOptionalSet(Hx, 0, 1, -1 * x0 * x3);
	cnMatrixOptionalSet(Hx, 0, 2, 0);
	cnMatrixOptionalSet(Hx, 1, 0, x0 * x4);
	cnMatrixOptionalSet(Hx, 1, 1, -1 * x1 * x4);
	cnMatrixOptionalSet(Hx, 1, 2, -1);
}


    static inline void meas_function_jac_state_with_hx(CnMat* Hx, CnMat* hx, const FLT* state, const FLT* landmark) {
        if(hx != 0) { 
            meas_function(hx, state, landmark);
        }
        if(Hx != 0) { 
            meas_function_jac_state(Hx, state, landmark);
        }
    }
static inline void meas_function_jac_landmark(CnMat* Hx, const FLT* state, const FLT* landmark) {
	const FLT state0 = state[0];
	const FLT state1 = state[1];
	const FLT landmark0 = landmark[0];
	const FLT landmark1 = landmark[1];
	const FLT x0 = landmark1 + (-1 * state1);
	const FLT x1 = landmark0 + (-1 * state0);
	const FLT x2 = (x1 * x1) + (x0 * x0);
	const FLT x3 = 1. / sqrt(x2);
	const FLT x4 = 1. / x2;
	cnMatrixOptionalSet(Hx, 0, 0, x1 * x3);
	cnMatrixOptionalSet(Hx, 0, 1, x0 * x3);
	cnMatrixOptionalSet(Hx, 1, 0, -1 * x0 * x4);
	cnMatrixOptionalSet(Hx, 1, 1, x1 * x4);
}


    static inline void meas_function_jac_landmark_with_hx(CnMat* Hx, CnMat* hx, const FLT* state, const FLT* landmark) {
        if(hx != 0) { 
            meas_function(hx, state, landmark);
        }
        if(Hx != 0) { 
            meas_function_jac_landmark(Hx, state, landmark);
        }
    }
