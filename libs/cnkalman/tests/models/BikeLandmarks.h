#include <cnkalman/model.h>
#include "BikeLandmarks.gen.h"

struct LandmarkMeasurement : public cnkalman::KalmanMeasurementModel {
    FLT px, py;

    virtual cnmatrix::Matrix default_R() {
        auto R = cnmatrix::Matrix(2, 2);
        cnMatrixSet(R, 0, 0, .3*.3);
        cnMatrixSet(R, 1, 1, .1*.1);
        return R;
    }

    LandmarkMeasurement(cnkalman::KalmanModel *kalmanModel, double x, double y)
            : KalmanMeasurementModel(kalmanModel, "Landmark", 2), px(x), py(y) {

    }

    static inline FLT wrapAngle( FLT angle )
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    bool residual(const CnMat &Z, const CnMat &x, CnMat *y, CnMat *h) override {
        auto rtn = KalmanMeasurementModel::residual(Z, x, y, h);
        y->data[1] = wrapAngle(y->data[1]);
        return rtn;
    }

    bool predict_measurement(const CnMat &x_t, CnMat *pz, CnMat *h) override {
        FLT landmark[] = { px, py };
        meas_function_jac_state_with_hx(h, pz, x_t.data, landmark);

        return h == nullptr || cn_is_finite(h);
    }
};

struct BikeLandmarks : public cnkalman::KalmanModel {
    FLT wheelbase = 0.5;
    FLT v = 1.1, alpha = .01;
    FLT v_std = .1, alpha_std = .015;
    BikeLandmarks() : cnkalman::KalmanModel("BikeLandmarks", 3) {
        this->measurementModels.emplace_back(std::make_unique<LandmarkMeasurement>(this, 5, 10));
        this->measurementModels.emplace_back(std::make_unique<LandmarkMeasurement>(this, 10, 5));
        this->measurementModels.emplace_back(std::make_unique<LandmarkMeasurement>(this, 15, 15));

        state[0] = 2; state[1] = 6; state[2] = .3;
        cn_set_diag_val(&kalman_state.P, .1);
    }

    void process_noise(FLT dt, const struct CnMat &x, struct CnMat &Q_out) override {
        FLT U[2] = { v, alpha};
        if(fabs(U[1]) < 1e-7) {
            U[1] = 1e-7;
        }
        CN_CREATE_STACK_MAT(V, 3, 2);
        predict_function_jac_u(&V, dt, wheelbase, state, U);

        FLT _M[] = {
                v_std*v_std, 0,
                0, alpha_std*alpha_std
        };
        auto M = cnMat(2, 2, _M);
        cn_ABAt_add(&Q_out, &V, &M, 0);
    }

    void predict(FLT dt, const struct CnMat &x0, struct CnMat *x1, CnMat* F) override {
        FLT U[2] = { v, alpha};
        if(fabs(U[1]) < 1e-7) {
            U[1] = 1e-7;
        }

        predict_function_jac_state_with_hx(F, x1, dt, wheelbase, x0.data, U);
    }

};
