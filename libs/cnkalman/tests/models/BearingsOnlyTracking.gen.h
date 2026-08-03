/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>

// clang-format off
static inline FLT bearings_meas_function(const FLT* state, const FLT* landmark) {
	const FLT state0 = state[0];
	const FLT state1 = state[1];
	const FLT landmark0 = landmark[0];
	const FLT landmark1 = landmark[1];
	return atan2(state1 + (-1 * landmark1), state0 + (-1 * landmark0));
}

static inline void bearings_meas_function_jac_state(CnMat* Hx, const FLT* state, const FLT* landmark) {
	const FLT state0 = state[0];
	const FLT state1 = state[1];
	const FLT landmark0 = landmark[0];
	const FLT landmark1 = landmark[1];
	const FLT x0 = state1 + (-1 * landmark1);
	const FLT x1 = state0 + (-1 * landmark0);
	const FLT x2 = 1. / ((x1 * x1) + (x0 * x0));
	cnMatrixOptionalSet(Hx, 0, 0, -1 * x0 * x2);
	cnMatrixOptionalSet(Hx, 0, 1, x2 * x1);
}


    static inline void bearings_meas_function_jac_state_with_hx(CnMat* Hx, CnMat* hx, const FLT* state, const FLT* landmark) {
        if(hx != 0) { 
            hx->data[0] = bearings_meas_function(state, landmark);
        }
        if(Hx != 0) { 
            bearings_meas_function_jac_state(Hx, state, landmark);
        }
    }
static inline void bearings_meas_function_jac_landmark(CnMat* Hx, const FLT* state, const FLT* landmark) {
	const FLT state0 = state[0];
	const FLT state1 = state[1];
	const FLT landmark0 = landmark[0];
	const FLT landmark1 = landmark[1];
	const FLT x0 = state1 + (-1 * landmark1);
	const FLT x1 = state0 + (-1 * landmark0);
	const FLT x2 = 1. / ((x1 * x1) + (x0 * x0));
	cnMatrixOptionalSet(Hx, 0, 0, x0 * x2);
	cnMatrixOptionalSet(Hx, 0, 1, -1 * x2 * x1);
}


    static inline void bearings_meas_function_jac_landmark_with_hx(CnMat* Hx, CnMat* hx, const FLT* state, const FLT* landmark) {
        if(hx != 0) { 
            hx->data[0] = bearings_meas_function(state, landmark);
        }
        if(Hx != 0) { 
            bearings_meas_function_jac_landmark(Hx, state, landmark);
        }
    }
