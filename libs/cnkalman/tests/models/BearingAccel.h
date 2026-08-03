#include <cnkalman/model.h>
#include "cnkalman/ModelPlot.h"
#include <sstream>

struct BearingAccelLandmark {
    FLT Position[2];
};

struct BearingAccelPose {
    FLT Pos[2];
    FLT Theta;
};

struct BearingAccelModel {
    BearingAccelPose Position;
    BearingAccelPose Velocity;
    FLT Accel[2];
};

#include "BearingAccel.gen.h"
#ifdef HAS_SCIPLOT
#include <sciplot/sciplot.hpp>
#endif

struct BearingAccel : public cnkalman::KalmanModel {
    struct IMUMeasurement : public cnkalman::KalmanMeasurementModel {
        IMUMeasurement(KalmanModel *kalmanModel) : KalmanMeasurementModel(
                kalmanModel, "IMU", 5) {}

        bool predict_measurement(const CnMat &x, CnMat *z, CnMat *h) override {
            auto* mdl = reinterpret_cast<BearingAccelModel *>(x.data);
            BearingAccelModel_imu_predict_jac_model_with_hx(h, z, mdl);
            return h == nullptr || cn_is_finite(h);
        }


        cnmatrix::Matrix default_R() override {
            auto R = cnmatrix::Matrix(meas_cnt, meas_cnt);
            const FLT r = 1e-5;
            cn_set_diag_val(R, r);
            return R;
        }
    };
	struct LandmarkMeasurement : public cnkalman::KalmanMeasurementModel {
        BearingAccelLandmark a, b;

		cnmatrix::Matrix default_R() override {
			auto R = cnmatrix::Matrix(meas_cnt, meas_cnt);
			const FLT r = 1;
			cn_set_diag_val(R, r);
			return R;
		}

		LandmarkMeasurement(cnkalman::KalmanModel *kalmanModel, const BearingAccelLandmark& a, const BearingAccelLandmark& b)
		: KalmanMeasurementModel(kalmanModel, "Landmark", 1), a(a), b(b) {
            //this->meas_mdl.meas_jacobian_mode = cnkalman_jacobian_mode_debug;
		}

		bool predict_measurement(const CnMat &x_t, CnMat *pz, CnMat *h) override {
            if(pz) {
                pz->data[0] = BearingAccelModel_tdoa_predict(&a, &b, (BearingAccelModel *) x_t.data);
            }
            if(h) {
                BearingAccelModel_tdoa_predict_jac_model(h, &a, &b, (BearingAccelModel *) x_t.data);
            }
			return h == nullptr || cn_is_finite(h);
		}
	};
	const FLT spacing = 10;
	std::vector<BearingAccelLandmark> landmarks = {
            {spacing, -spacing},
            {-spacing, spacing},
	        {-spacing, -spacing},
		{spacing,spacing},

        //{-0,0}
	};

	static std::string get_name(bool useIMU = true, bool useLandmarks = true) {
	    if(useIMU && useLandmarks) return "BearingAccel";
        if(!useIMU) return "BearingAccelNoIMU";
        return "IMU only";
	}

	BearingAccel(bool useIMU = true, bool useLandmarks = true) : cnkalman::KalmanModel(get_name(useIMU, useLandmarks), sizeof(BearingAccelModel)/sizeof(FLT)) {
	    if(useLandmarks) {
            for (size_t i = 0; i < landmarks.size(); i++) {
                for (size_t j = i + 1; j < landmarks.size(); j++) {
                    this->measurementModels.emplace_back(
                            std::make_unique<LandmarkMeasurement>(this, landmarks[i], landmarks[j]));
                }
            }
        }
	    if(useIMU) {
            this->measurementModels.emplace_back(std::make_unique<IMUMeasurement>(this));
        }
		reset();
	}
	void reset() override {
		KalmanModel::reset();
		cn_set_diag_val(&kalman_state.P, .1);
        cn_set_zero(&kalman_state.state);
	}

	void process_noise(FLT dt, const struct CnMat &x, struct CnMat &Q_out) override {
	    BearingAccelModel v = { };
        v.Velocity.Pos[0] = v.Velocity.Pos[1] = 1e-2;
        v.Velocity.Theta = 1e-1;
        v.Accel[0] = v.Accel[1] = .01;

        cn_set_diag(&Q_out, reinterpret_cast<const FLT *>(&v));
	}

	void predict(FLT dt, const struct CnMat &x0, struct CnMat *x1, CnMat* F) override {
	    if(F) {
            BearingAccelModel_predict_jac_model(F, dt, reinterpret_cast<const BearingAccelModel *>(x0.data));
	    }
        if(x1) {
            BearingAccelModel_predict(reinterpret_cast<BearingAccelModel *>(x1->data), dt,
                                          reinterpret_cast<const BearingAccelModel *>(x0.data));
	    }
	}

    void sample_state(FLT dt, const CnMat &x0, CnMat &x1, const struct CnMat *Q) override {
        BearingAccelModel mdl = *reinterpret_cast<BearingAccelModel *>(x1.data);
        FLT G = 8. / landmarks.size();
        mdl.Accel[0] = mdl.Accel[1] = 0;
        for(auto& l : landmarks) {
            FLT dx = l.Position[0] - mdl.Position.Pos[0];
            FLT dy = l.Position[1] - mdl.Position.Pos[1];
            FLT dist = sqrt(dx*dx + dy*dy);
            if(dist < 1) dist = 1;
            mdl.Accel[0] += dx / dist / dist * G;
            mdl.Accel[1] += dy / dist / dist * G;
        }

        mdl.Velocity.Theta *= .5;

        auto x1_5 = cnMat(x1.rows, x1.cols, (FLT*)&mdl);
        KalmanModel::sample_state(dt, x1_5, x1, Q);
    }

    void draw(cnkalman::ModelPlot &p) override {
        KalmanModel::draw(p);
        for(auto& l : landmarks) {
            std::stringstream ss;
            static int idx = 1000;
            p.include_point_in_range(l.Position);
            ss << "set obj " << idx++ << " ellipse fc rgb \"green\" fs transparent solid 1 center "
               << l.Position[0] << "," << l.Position[1] << " size " << .1 << "," << .1 << " angle " << 0 << " front\n";
#ifdef HAS_SCIPLOT
            p.map.gnuplot(ss.str());
#endif
        }
    }

};
