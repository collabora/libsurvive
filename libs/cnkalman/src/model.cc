#include <iostream>
#include "cnkalman/model.h"
namespace cnkalman {

    bool KalmanMeasurementModel::residual(const CnMat& Z, const CnMat& x_t, CnMat* y, CnMat* H_k) {
        CN_CREATE_STACK_VEC(pz, Z.rows);
        auto rtn = predict_measurement(x_t, &pz, H_k);
        //cn_elementwise_subtract(y, &pz, &Z);
        cn_elementwise_subtract(y, &Z, &pz);
        return rtn;
    }

    static bool kalman_measurement(void *user, const struct CnMat *Z, const struct CnMat *x_t, struct CnMat *y, struct CnMat *H_k) {
        auto *self = static_cast<cnkalman::KalmanMeasurementModel *>(user);

        return self->residual(*Z, *x_t, y, H_k);
    }

    KalmanMeasurementModel::KalmanMeasurementModel(KalmanModel* kalmanModel, const std::string &name, size_t meas_cnt) : meas_cnt(meas_cnt) {
        cnkalman_meas_model_init(&kalmanModel->kalman_state, "meas", &meas_mdl, kalman_measurement);
    }

    cnkalman_update_extended_stats_t KalmanMeasurementModel::update(FLT t, const CnMat &Z, CnMat &R) {
        struct cnkalman_update_extended_stats_t stats = {};
        cnkalman_meas_model_predict_update_stats(t, &meas_mdl, this, &Z, &R, &stats);
        return stats;
    }

    void KalmanMeasurementModel::sample_measurement(const CnMat &x, CnMat &Z, const CnMat &R) {
        predict_measurement(x, &Z, 0);

        CN_CREATE_STACK_MAT(RL, meas_cnt, meas_cnt);
        cnSqRootSymmetric(&R, &RL);
        CN_CREATE_STACK_MAT(X, meas_cnt, 1);
        CN_CREATE_STACK_MAT(Xs, meas_cnt, 1);
        cnRand(&X, 0, 1);
        cnGEMM(&RL, &X, 1, 0, 0, &Xs, (cnGEMMFlags)0);
        cn_elementwise_add(&Z, &Z, &Xs);
    }

    std::ostream &KalmanMeasurementModel::write(std::ostream & os) const {
        os << "{" << std::endl;
        os << "\t" << R"("name": ")" << this->meas_mdl.name << "\"," << std::endl;
        os << "\t" << R"("meas_cnt": )" << meas_cnt << std::endl;
        os << "}";
        return os;
    }

    static void transition_bounce(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0, struct CnMat *x1, CnMat* F) {
        auto *self = static_cast<cnkalman::KalmanModel *>(k->user);
        self->predict(dt, *x0, x1, F);
    }

    static void kalman_process_noise_fn(void *user, FLT dt, const struct CnMat *x, struct CnMat *Q_out) {
        auto *self = static_cast<cnkalman::KalmanModel *>(user);
        self->process_noise(dt, *x, *Q_out);
    }

    KalmanModel::KalmanModel(const std::string& name, size_t state_cnt) : name(name), state_cnt(state_cnt) {
        cnkalman_state_init(&kalman_state, state_cnt, transition_bounce, kalman_process_noise_fn, this, 0);
        state = kalman_state.state.data;
        stateM = &kalman_state.state;
    }

    void KalmanModel::sample_state(FLT dt, const CnMat &x0, CnMat &x1, const struct CnMat* iQ) {
        CN_CREATE_STACK_MAT(X, state_cnt, 1);
        CN_CREATE_STACK_MAT(Xs, state_cnt, 1);
        CN_CREATE_STACK_MAT(Q, state_cnt, state_cnt);

        CN_CREATE_STACK_MAT(QL, state_cnt, state_cnt);

        predict(dt, x0, &x1);

        if(!iQ) {
            process_noise(dt, x1, Q);
        }

        cnSqRootSymmetric(iQ ? iQ : &Q, &QL);
        cnRand(&X, 0, 1);
        cnGEMM(&QL, &X, 1, 0, 0, &Xs, (cnGEMMFlags)0);
        cn_elementwise_add(&x1, &x1, &Xs);
    }

    void KalmanModel::update(FLT t) {
        cnkalman_predict_state(t, &kalman_state);
    }

    bool KalmanLinearMeasurementModel::predict_measurement(const CnMat &x, CnMat *z, CnMat *h) {
        if(z) cnGEMM(&H, &x, 1, 0, 0, z, cnGEMMFlags(0));
        if(h) cnCopy(&H, h, 0);
        return true;
    }

    KalmanLinearMeasurementModel::KalmanLinearMeasurementModel(KalmanModel* kalmanModel, const std::string& name,
                                                               const CnMat& H) : KalmanMeasurementModel(kalmanModel, name, H.rows) {
        this->H = cnMatCalloc(H.rows, H.cols);
        cnCopy(&H, &this->H, 0);
    }

    KalmanLinearMeasurementModel::~KalmanLinearMeasurementModel() {
        free(this->H.data);
    }

    std::ostream& KalmanModel::write(std::ostream& os) const {
        os << "\"model\": {" << std::endl;
        os << "\t" << R"("name": ")" << name << "\"," << std::endl;
        os << "\t" << R"("state_cnt": )" << state_cnt << "," << std::endl;
        os << "\t" << R"("measurement_models": [)" << std::endl;
        bool needsComma = false;
        for(auto& meas : measurementModels) {
            if(needsComma) os << ", ";
            needsComma = true;
            meas->write(os);
        }
        os << "]" << std::endl;
        os << "}" << std::endl;
        return os;
    }

    KalmanModel::~KalmanModel() {
        cnkalman_state_free(&this->kalman_state);
    }

    /*
    void KalmanModel::predict(double dt, const CnMat &x0, CnMat *x1, CnMat* F) {
        int state_cnt = this->state_cnt;
        CN_CREATE_STACK_MAT(F, state_cnt, state_cnt);
        CN_CREATE_STACK_VEC(x1_, state_cnt);

        predict(dt, F, x0);

        // X_k|k-1 = F * X_k-1|k-1
        cnGEMM(&F, &x0, 1, 0, 0, &x1_, (cnGEMMFlags)0);
        cnCopy(&x1_, &x1, 0);
        CN_FREE_STACK_MAT(x1_);
        CN_FREE_STACK_MAT(F);
    }
*/

    static inline bool bulk_update_fn(void *user, const struct CnMat *Zs, const struct CnMat *x_t, struct CnMat *ys, struct CnMat *H_ks) {
        auto *self = static_cast<cnkalman::KalmanModel *>(user);
        size_t meas_cnt = 0, max_cnt = 0;
        for(auto& meas : self->measurementModels) {
            meas_cnt += meas->meas_cnt;
            if(max_cnt < meas->meas_cnt) max_cnt= meas->meas_cnt;
        }

        FLT* HStorage = (FLT*)alloca(max_cnt * self->state_cnt * sizeof(FLT));
        int meas_idx = 0;

        if(H_ks) {
            cnSetZero(H_ks);
        }

        for(auto & meas : self->measurementModels) {
            auto Z = cnVec(meas->meas_cnt, Zs->data + meas_idx);
            auto y = cnVec(meas->meas_cnt, ys ? ys->data + meas_idx : 0);
            auto H = cnMat(meas->meas_cnt, self->state_cnt, HStorage);
            cnSetZero(&H);

            bool okay = meas->residual(Z, *x_t, ys ? &y : 0, H_ks ? &H : 0);
            if(!okay)
                return false;

            if(H_ks) {
                cnCopyROI(&H, H_ks, meas_idx, 0);
            }
            meas_idx += (int)meas->meas_cnt;
        }

        return true;
    }
    void KalmanModel::bulk_update(FLT t, const std::vector<CnMat> &Zs, const std::vector<cnmatrix::Matrix> &Rs) {
        assert(Zs.size() == Rs.size());
        assert(Zs.size() == measurementModels.size());
        size_t meas_cnt = 0;
        int min_iterations = 0;
        bool debugJac = false;
        for(auto& meas : measurementModels) {
            meas_cnt += meas->meas_cnt;
            if(meas->meas_mdl.term_criteria.max_iterations > min_iterations)
                min_iterations = meas->meas_mdl.term_criteria.max_iterations;
            if(meas->meas_mdl.meas_jacobian_mode == cnkalman_jacobian_mode_debug)
                debugJac = true;
        }

        CN_CREATE_STACK_VEC(Z, meas_cnt);
        CN_CREATE_STACK_MAT(R, meas_cnt, meas_cnt);
        int meas_idx = 0;
        for(int i = 0;i < (int)Zs.size();i++) {
            cnCopyROI(&Zs[i], &Z, meas_idx, 0);
            cnCopyROI(Rs[i], &R, meas_idx, meas_idx);
            meas_idx += (int)measurementModels[i]->meas_cnt;
        }

        cnkalman_meas_model bulk = { };
        cnkalman_meas_model_init(&kalman_state, "bulk", &bulk, bulk_update_fn);
        bulk.term_criteria.max_iterations = min_iterations;
        if(debugJac)
            bulk.meas_jacobian_mode = cnkalman_jacobian_mode_debug;
        cnkalman_meas_model_predict_update(t, &bulk, this, &Z, &R);
    }

    void KalmanModel::reset() {
        cnkalman_state_reset(&kalman_state);
    }

    KalmanLinearPredictionModel::KalmanLinearPredictionModel(const std::string &name, size_t stateCnt)
            : KalmanModel(name, stateCnt) {
    }


    void KalmanLinearPredictionModel::predict(FLT dt, const CnMat &x0, CnMat *x1, CnMat *cF) {
        if(cF) {
            cnCopy(&this->F(), cF, 0);
        }
        if(x1) {
            CN_CREATE_STACK_VEC(x1_, state_cnt);

            // X_k|k-1 = F * X_k-1|k-1
            cnGEMM(&F(), &x0, 1, 0, 0, &x1_, (cnGEMMFlags)0);
            cnCopy(&x1_, x1, 0);
            CN_FREE_STACK_MAT(x1_);
        }
    }
}