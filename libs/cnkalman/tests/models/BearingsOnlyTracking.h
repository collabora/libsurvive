#include <cnkalman/model.h>
#include "BearingsOnlyTracking.gen.h"
#include <cnkalman/ModelPlot.h>
#ifdef HAS_SCIPLOT
#include <sciplot/sciplot.hpp>
#endif

struct BearingsOnlyTracking : public cnkalman::KalmanModel {
    struct LandmarkMeasurement : public cnkalman::KalmanMeasurementModel {
        FLT px, py;

#ifdef HAS_SCIPLOT
        void draw(cnkalman::ModelPlot& p) override {
            std::stringstream ss;
            static int idx = 1000;
            p.include_point_in_range(px, py);
            ss << "set obj " << idx++ << " ellipse fc rgb \"green\" fs transparent solid 1 center "
               << px << "," << py << " size " << .05 << "," << .05 << " angle " << 0 << " front\n";
            p.map.gnuplot(ss.str());
        }
#endif
        virtual cnmatrix::Matrix default_R() {
            auto R = cnmatrix::Matrix(meas_cnt, meas_cnt);
            const FLT r = M_PI * M_PI * 1e-5;
            cn_set_diag_val(R, r);
            return R;
        }

        LandmarkMeasurement(cnkalman::KalmanModel *kalmanModel, double x, double y)
                : KalmanMeasurementModel(kalmanModel, "Landmark", 1), px(x), py(y) {

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
            y->data[0] = wrapAngle(y->data[0]);
            return rtn;
        }

        bool predict_measurement(const CnMat &x_t, CnMat *pz, CnMat *h) override {
            FLT landmark[] = { px, py };
            bearings_meas_function_jac_state_with_hx(h, pz, x_t.data, landmark);

            return h == nullptr || cn_is_finite(h);
        }
    };

    BearingsOnlyTracking() : cnkalman::KalmanModel("BearingsOnlyTracking", 2) {
        this->measurementModels.emplace_back(std::make_unique<LandmarkMeasurement>(this, 0, 0));
        this->measurementModels.emplace_back(std::make_unique<LandmarkMeasurement>(this, 1.5, 0));
        reset();
    }
    void reset() override {
        KalmanModel::reset();
        state[0] = 1.5; state[1] = 1.5;
        cn_set_diag_val(&kalman_state.P, .1);
    }

    void process_noise(FLT dt, const struct CnMat &x, struct CnMat &Q_out) override {
        cn_set_diag_val(&Q_out, 0.1);
    }

    void predict(FLT dt, const struct CnMat &x0, struct CnMat *x1, CnMat* F) override {
        if(F) cn_set_diag_val(F, 1);
        if(x1) cnCopy(&x0, x1, 0);
    }

};
