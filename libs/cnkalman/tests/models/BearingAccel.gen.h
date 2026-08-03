/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>

// clang-format off
static inline void BearingAccelModel_predict(BearingAccelModel* out, const FLT t, const BearingAccelModel* model) {
	const FLT x0 = 1.0/2.0 * (t * t);
	out->Position.Pos[0]=(t * (*model).Velocity.Pos[0]) + (x0 * (*model).Accel[0]) + (*model).Position.Pos[0];
	out->Position.Pos[1]=(t * (*model).Velocity.Pos[1]) + (x0 * (*model).Accel[1]) + (*model).Position.Pos[1];
	out->Position.Theta=(*model).Position.Theta + (t * (*model).Velocity.Theta);
	out->Velocity.Pos[0]=(*model).Velocity.Pos[0] + (t * (*model).Accel[0]);
	out->Velocity.Pos[1]=(*model).Velocity.Pos[1] + (t * (*model).Accel[1]);
	out->Velocity.Theta=(*model).Velocity.Theta;
	out->Accel[0]=(*model).Accel[0];
	out->Accel[1]=(*model).Accel[1];
}

static inline void BearingAccelModel_predict_jac_t(CnMat* Hx, const FLT t, const BearingAccelModel* model) {
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Position.Pos[0])/sizeof(FLT), 0, (t * (*model).Accel[0]) + (*model).Velocity.Pos[0]);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Position.Pos[1])/sizeof(FLT), 0, (t * (*model).Accel[1]) + (*model).Velocity.Pos[1]);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Position.Theta)/sizeof(FLT), 0, (*model).Velocity.Theta);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Velocity.Pos[0])/sizeof(FLT), 0, (*model).Accel[0]);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Velocity.Pos[1])/sizeof(FLT), 0, (*model).Accel[1]);
}

static inline void BearingAccelModel_predict_jac_model(CnMat* Hx, const FLT t, const BearingAccelModel* model) {
	const FLT x0 = 1.0/2.0 * (t * t);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Position.Pos[0])/sizeof(FLT), offsetof(BearingAccelModel, Accel[0])/sizeof(FLT), x0);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Position.Pos[0])/sizeof(FLT), offsetof(BearingAccelModel, Position.Pos[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Position.Pos[0])/sizeof(FLT), offsetof(BearingAccelModel, Velocity.Pos[0])/sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Position.Pos[1])/sizeof(FLT), offsetof(BearingAccelModel, Accel[1])/sizeof(FLT), x0);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Position.Pos[1])/sizeof(FLT), offsetof(BearingAccelModel, Position.Pos[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Position.Pos[1])/sizeof(FLT), offsetof(BearingAccelModel, Velocity.Pos[1])/sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Position.Theta)/sizeof(FLT), offsetof(BearingAccelModel, Position.Theta)/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Position.Theta)/sizeof(FLT), offsetof(BearingAccelModel, Velocity.Theta)/sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Velocity.Pos[0])/sizeof(FLT), offsetof(BearingAccelModel, Accel[0])/sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Velocity.Pos[0])/sizeof(FLT), offsetof(BearingAccelModel, Velocity.Pos[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Velocity.Pos[1])/sizeof(FLT), offsetof(BearingAccelModel, Accel[1])/sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Velocity.Pos[1])/sizeof(FLT), offsetof(BearingAccelModel, Velocity.Pos[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Velocity.Theta)/sizeof(FLT), offsetof(BearingAccelModel, Velocity.Theta)/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Accel[0])/sizeof(FLT), offsetof(BearingAccelModel, Accel[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(BearingAccelModel, Accel[1])/sizeof(FLT), offsetof(BearingAccelModel, Accel[1])/sizeof(FLT), 1);
}

static inline void BearingAccelModel_imu_predict(CnMat* out, const BearingAccelModel* model) {
	const FLT x0 = cos((*model).Position.Theta);
	const FLT x1 = sin((*model).Position.Theta);
	cnMatrixOptionalSet(out, 0, 0, (*model).Velocity.Theta);
	cnMatrixOptionalSet(out, 1, 0, x0 * (*model).Accel[0]);
	cnMatrixOptionalSet(out, 2, 0, x1 * (*model).Accel[1]);
	cnMatrixOptionalSet(out, 3, 0, x0);
	cnMatrixOptionalSet(out, 4, 0, x1);
}

static inline void BearingAccelModel_imu_predict_jac_model(CnMat* Hx, const BearingAccelModel* model) {
	const FLT x0 = cos((*model).Position.Theta);
	const FLT x1 = sin((*model).Position.Theta);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(BearingAccelModel, Velocity.Theta)/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, 1, offsetof(BearingAccelModel, Accel[0])/sizeof(FLT), x0);
	cnMatrixOptionalSet(Hx, 1, offsetof(BearingAccelModel, Position.Theta)/sizeof(FLT), -1 * x1 * (*model).Accel[0]);
	cnMatrixOptionalSet(Hx, 2, offsetof(BearingAccelModel, Accel[1])/sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, 2, offsetof(BearingAccelModel, Position.Theta)/sizeof(FLT), x0 * (*model).Accel[1]);
	cnMatrixOptionalSet(Hx, 3, offsetof(BearingAccelModel, Position.Theta)/sizeof(FLT), -1 * x1);
	cnMatrixOptionalSet(Hx, 4, offsetof(BearingAccelModel, Position.Theta)/sizeof(FLT), x0);
}


    static inline void BearingAccelModel_imu_predict_jac_model_with_hx(CnMat* Hx, CnMat* hx, const BearingAccelModel* model) {
        if(hx != 0) { 
            BearingAccelModel_imu_predict(hx, model);
        }
        if(Hx != 0) { 
            BearingAccelModel_imu_predict_jac_model(Hx, model);
        }
    }
static inline FLT BearingAccelModel_tdoa_predict(const BearingAccelLandmark* A, const BearingAccelLandmark* B, const BearingAccelModel* model) {
	const FLT x0 = -1 * (*model).Position.Pos[1];
	const FLT x1 = -1 * (*model).Position.Pos[0];
	return (3.33564095198152 * sqrt(1e-05 + (((*A).Position[0] + x1) * ((*A).Position[0] + x1)) + (((*A).Position[1] + x0) * ((*A).Position[1] + x0)))) + (-3.33564095198152 * sqrt(1e-05 + (((*B).Position[0] + x1) * ((*B).Position[0] + x1)) + (((*B).Position[1] + x0) * ((*B).Position[1] + x0))));
}

static inline void BearingAccelModel_tdoa_predict_jac_A(CnMat* Hx, const BearingAccelLandmark* A, const BearingAccelLandmark* B, const BearingAccelModel* model) {
	const FLT x0 = (*A).Position[0] + (-1 * (*model).Position.Pos[0]);
	const FLT x1 = (*A).Position[1] + (-1 * (*model).Position.Pos[1]);
	const FLT x2 = 3.33564095198152 * (1. / sqrt(1e-05 + (x0 * x0) + (x1 * x1)));
	cnMatrixOptionalSet(Hx, 0, offsetof(BearingAccelLandmark, Position[0])/sizeof(FLT), x0 * x2);
	cnMatrixOptionalSet(Hx, 0, offsetof(BearingAccelLandmark, Position[1])/sizeof(FLT), x2 * x1);
}


    static inline void BearingAccelModel_tdoa_predict_jac_A_with_hx(CnMat* Hx, CnMat* hx, const BearingAccelLandmark* A, const BearingAccelLandmark* B, const BearingAccelModel* model) {
        if(hx != 0) { 
            hx->data[0] = BearingAccelModel_tdoa_predict(A, B, model);
        }
        if(Hx != 0) { 
            BearingAccelModel_tdoa_predict_jac_A(Hx, A, B, model);
        }
    }
static inline void BearingAccelModel_tdoa_predict_jac_B(CnMat* Hx, const BearingAccelLandmark* A, const BearingAccelLandmark* B, const BearingAccelModel* model) {
	const FLT x0 = (*B).Position[0] + (-1 * (*model).Position.Pos[0]);
	const FLT x1 = (*B).Position[1] + (-1 * (*model).Position.Pos[1]);
	const FLT x2 = 3.33564095198152 * (1. / sqrt(1e-05 + (x0 * x0) + (x1 * x1)));
	cnMatrixOptionalSet(Hx, 0, offsetof(BearingAccelLandmark, Position[0])/sizeof(FLT), -1 * x0 * x2);
	cnMatrixOptionalSet(Hx, 0, offsetof(BearingAccelLandmark, Position[1])/sizeof(FLT), -1 * x2 * x1);
}


    static inline void BearingAccelModel_tdoa_predict_jac_B_with_hx(CnMat* Hx, CnMat* hx, const BearingAccelLandmark* A, const BearingAccelLandmark* B, const BearingAccelModel* model) {
        if(hx != 0) { 
            hx->data[0] = BearingAccelModel_tdoa_predict(A, B, model);
        }
        if(Hx != 0) { 
            BearingAccelModel_tdoa_predict_jac_B(Hx, A, B, model);
        }
    }
static inline void BearingAccelModel_tdoa_predict_jac_model(CnMat* Hx, const BearingAccelLandmark* A, const BearingAccelLandmark* B, const BearingAccelModel* model) {
	const FLT x0 = -1 * (*model).Position.Pos[0];
	const FLT x1 = (*A).Position[0] + x0;
	const FLT x2 = -1 * (*model).Position.Pos[1];
	const FLT x3 = (*A).Position[1] + x2;
	const FLT x4 = 3.33564095198152 * (1. / sqrt(1e-05 + (x1 * x1) + (x3 * x3)));
	const FLT x5 = (*B).Position[0] + x0;
	const FLT x6 = (*B).Position[1] + x2;
	const FLT x7 = 3.33564095198152 * (1. / sqrt(1e-05 + (x5 * x5) + (x6 * x6)));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(BearingAccelModel, Position.Pos[0])/sizeof(FLT), (x5 * x7) + (-1 * x1 * x4));
	cnMatrixOptionalSet(Hx, 0, offsetof(BearingAccelModel, Position.Pos[1])/sizeof(FLT), (x6 * x7) + (-1 * x4 * x3));
}


    static inline void BearingAccelModel_tdoa_predict_jac_model_with_hx(CnMat* Hx, CnMat* hx, const BearingAccelLandmark* A, const BearingAccelLandmark* B, const BearingAccelModel* model) {
        if(hx != 0) { 
            hx->data[0] = BearingAccelModel_tdoa_predict(A, B, model);
        }
        if(Hx != 0) { 
            BearingAccelModel_tdoa_predict_jac_model(Hx, A, B, model);
        }
    }
