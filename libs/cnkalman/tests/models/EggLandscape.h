#pragma once

#include <cnkalman/model.h>

struct EggLandscapeMeas : public cnkalman::KalmanMeasurementModel {
    EggLandscapeMeas(cnkalman::KalmanModel *kalmanModel, const std::string &name)
            : KalmanMeasurementModel(kalmanModel, name, 3) {

    }

    cnmatrix::Matrix default_R() override{
        auto rtn = cnmatrix::Matrix(meas_cnt, meas_cnt);
        cn_set_diag_val(rtn, 5e-7);
        return rtn;
    }

    FLT coeff_dist = 5;
    FLT coeff_a = 101, coeff_b = 11;
    bool predict_measurement(const CnMat &x_t, CnMat *pz, CnMat *h) override {
        FLT x = x_t.data[0]; FLT y = x_t.data[1];
        if(pz) {
            pz->data[0] = sin((x * x + y * y) * coeff_dist);
            //pz->data[0] = (x * x + y * y) * coeff_dist - 1;
            pz->data[1] = sin(x * coeff_a) * sin(y * coeff_a);
            pz->data[2] = sin(x * coeff_b) * cos(y * coeff_b);
        }
        if(h) {
            FLT c = cos(x * x + y * y);
            FLT jac[] = {
                    2 * x * coeff_dist * c, 2 * y * coeff_dist * c, 0, 0,
                    //2 * x * coeff_dist, 2 * y * coeff_dist, 0, 0,
                    coeff_a * cos(coeff_a * x) * sin(y * coeff_a), coeff_a * cos(coeff_a * y) * sin(x * coeff_a), 0, 0,
                    coeff_b * cos(coeff_b * x) * cos(y * coeff_b), -coeff_b * sin(coeff_b * y) * sin(x * coeff_b), 0, 0,
            };
            cn_copy_data_in(h, 1, jac);
        }

        return true;
    }
};
struct EggLandscapeProblem : public cnkalman::KalmanLinearPredictionModel {
    FLT _F[16] = {
            1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1,
    };
    CnMat Fm = cnMat(4, 4, _F);
    virtual const CnMat& F() const { return Fm; }

    EggLandscapeProblem() : cnkalman::KalmanLinearPredictionModel("EggLandscape", 4) {
        state_cnt = 4;
        measurementModels.emplace_back(std::make_unique<EggLandscapeMeas>(this, "meas"));
        cnkalman_state_reset(&kalman_state);

        state[0] = .1; state[1] = .1;
    }

    void process_noise(FLT dt, const CnMat &x, CnMat &Q_out) override {
        cn_set_zero(&Q_out);
        cnMatrixSet(&Q_out, 2, 2, .0001);
        cnMatrixSet(&Q_out, 3, 3, .0001);
    }

    void sample_state(FLT dt, const CnMat &x0, CnMat &x1, const struct CnMat* iQ) override {
        KalmanModel::sample_state(dt, x0, x1, iQ);

        FLT wall = .4;
        if(x1.data[0] > +wall && x1.data[2] > 0) x1.data[2] *= .9;
        if(x1.data[0] < -wall && x1.data[2] < 0) x1.data[2] *= .9;
        if(x1.data[1] > +wall && x1.data[3] > 0) x1.data[3] *= .9;
        if(x1.data[1] < -wall && x1.data[3] < 0) x1.data[3] *= .9;
    }
};