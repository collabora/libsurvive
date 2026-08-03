#pragma once

#include "cnkalman/kalman.h"
#ifdef __cplusplus
#include <vector>
#include <memory>
#include <string>

namespace cnkalman {
    struct ModelPlot;

    struct KalmanModel;
    struct CN_EXPORT_CLASS KalmanMeasurementModel {
        size_t meas_cnt;
        cnkalman_meas_model meas_mdl = {};

        KalmanMeasurementModel(KalmanModel* kalmanModel, const std::string& name, size_t meas_cnt);
        KalmanMeasurementModel(KalmanModel* kalmanModel, size_t meas_cnt) : KalmanMeasurementModel(kalmanModel, "meas", meas_cnt) {}
        virtual ~KalmanMeasurementModel() = default;

        /***
         * @param x current state
         * @param z measurement prediction
         * @param h measurement jacobian wrt x
         * @return Whether the residual / jacobian is valid
         */
        virtual bool predict_measurement(const CnMat& x, CnMat* z, CnMat* h) = 0;

        /***
         * @param Z observed measurement
         * @param x current state
         * @param z measurement prediction
         * @param h measurement jacobian wrt x
         * @return Whether the residual / jacobian is valid
         */
        virtual bool residual(const CnMat& Z, const CnMat& x, CnMat* y, CnMat* h);

        cnkalman_update_extended_stats_t update(FLT t, const struct CnMat& Z, CnMat& R);

        virtual std::ostream& write(std::ostream&) const;
        /***
         * Given an assumed state x and measurement variance R, generate a plausible Z
         */
        virtual void sample_measurement(const CnMat& x, struct CnMat& Z, const CnMat& R);
        virtual cnmatrix::Matrix default_R() {
            auto rtn = cnmatrix::Matrix(meas_cnt, meas_cnt);
            cn_set_diag_val(rtn, .1 * .1);
            return rtn;
        }

        virtual void draw(ModelPlot& p) {}
    };

    struct CN_EXPORT_CLASS KalmanModel {
        std::string name;

        cnkalman_state_t kalman_state = {};
        size_t state_cnt;
        FLT* state;
        CnMat* stateM;
        std::vector<std::shared_ptr<struct KalmanMeasurementModel>> measurementModels;

        virtual std::ostream& write(std::ostream&) const;

        KalmanModel(const std::string& name, size_t state_cnt);
        KalmanModel(size_t state_cnt) : KalmanModel("mdl", state_cnt) {}
        virtual void reset();
        virtual ~KalmanModel();

		virtual void predict(FLT dt, const CnMat& x0, CnMat* x1, CnMat* cF) = 0;
		void predict(FLT dt, const CnMat& x0, CnMat* x1) { return predict(dt, x0, x1, 0); }

        virtual void process_noise(FLT dt, const struct CnMat &x, struct CnMat &Q_out) = 0;

        virtual void sample_state(FLT dt, const struct CnMat &x0, struct CnMat &x1, const struct CnMat* Q = 0);

        void update(FLT t);

        void bulk_update(FLT t, const std::vector<CnMat>& Zs, const std::vector<cnmatrix::Matrix>& Rs);
        virtual void draw(ModelPlot& p) {
            for(auto& m : measurementModels) m->draw(p);
        }
    };

    struct CN_EXPORT_CLASS KalmanLinearPredictionModel : public KalmanModel {
        virtual const CnMat& F() const = 0;
        KalmanLinearPredictionModel(const std::string &name, size_t stateCnt);
        KalmanLinearPredictionModel(size_t stateCnt) : KalmanLinearPredictionModel("mdl", stateCnt) {}
        void predict(FLT dt, const CnMat& x0, CnMat* x1, CnMat* cF) override;
    };

    struct CN_EXPORT_CLASS KalmanLinearMeasurementModel : public KalmanMeasurementModel {
        CnMat H;

        KalmanLinearMeasurementModel(KalmanModel* kalmanModel, const std::string& name, const CnMat& H);
        KalmanLinearMeasurementModel(KalmanModel* kalmanModel, const CnMat& H) : KalmanLinearMeasurementModel(kalmanModel, "meas", H) {}
        ~KalmanLinearMeasurementModel() override;
        bool predict_measurement(const CnMat &x, CnMat *z, CnMat *h) override;
    };
}

#endif