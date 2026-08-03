#include <vector>
#include <cmath>

#include "cnkalman/kalman.h"
#include "cnkalman/model.h"
#include "ModelRunner.h"


#include <iostream>
#include <cnmatrix/cn_matrix.h>
#include <fstream>

#if HAS_SCIPLOT
#include <sciplot/sciplot.hpp>

namespace sciplot::internal {
    template<>
    auto escapeIfNeeded<std::tuple<double, double>>(const std::tuple<double, double> &val) {
        return internal::str(std::get<0>(val)) + " " + internal::str(std::get<1>(val));
    }
}

#endif

static inline FLT linmath_enforce_range(FLT v, FLT mn, FLT mx) {
    if (v < mn)
        return mn;
    if (v > mx)
        return mx;
    return v;
}

static inline uint32_t create_rgb(const FLT* rgb) {
    FLT _r = rgb[0], _g = rgb[1], _b = rgb[2];
    uint8_t r = linmath_enforce_range(_r * 127 + 127, 0, 255);
    uint8_t g = linmath_enforce_range(_g * 127 + 127, 0, 255);
    uint8_t b = linmath_enforce_range(_b * 127 + 127, 0, 255);
    return 0xff << 24 | b << 16 | g << 8 | r;
}

static inline std::ostream& write_matrix(std::ostream& os, const char* name, const std::vector<std::tuple<FLT, FLT>>& m) {
    os << "\"" << name << "\": [" << std::endl;
    bool needsCommaA = false;
    for(auto& row : m) {
        if(needsCommaA) os << ",";
        needsCommaA = true;

        os << "\t[" << std::get<0>(row) << ", " << std::get<1>(row) << "]" << std::endl;
    }
    os << "]" << std::endl;
    return os;
}
static inline std::ostream& write_matrix(std::ostream& os, const std::vector<std::vector<FLT>>& m) {
    os << "[" << std::endl;
    bool needsCommaA = false;
    for(auto& row : m) {
        if(needsCommaA) os << ",";
        needsCommaA = true;

        bool needsCommaB = false;
        os << "\t[ ";
        for(auto& col : row) {
            if(needsCommaB) os << ",";
            needsCommaB = true;

            os << col;
        }
        os << "]" << std::endl;
    }
    os << "]" << std::endl;
    return os;
}
static inline std::ostream& write_matrix(std::ostream& os, const char* name, const std::vector<std::vector<FLT>>& m) {
    os << "\"" << name << "\":";
    return write_matrix(os, m);
}
static inline std::ostream& write_matrix(std::ostream& os, const char* name, const std::vector<std::vector<std::vector<FLT>>>& m) {
    os << "\"" << name << "\": [" << std::endl;
    bool needsCommaA = false;
    for(auto& matrix : m) {
        if(needsCommaA) os << ",";
        needsCommaA = true;
        write_matrix(os, matrix);
    }
    os << "]" << std::endl;
    return os;
}
static inline std::ostream& write_matrix(std::ostream& os, const CnMat& m) {
    os << "[" << std::endl;
    for(int i = 0;i < m.rows;i++) {
        if(i != 0) os << ", ";
        os << "\t[";
        for(int j = 0;j < m.cols;j++) {
            if(j != 0) os << ", ";
            os << m(i, j) << " ";
        }
        os << "]" << std::endl;
    }
    os << "]";
    return os;
}
template <typename M>
static inline std::ostream& write_matrix(std::ostream& os, const char* name, const std::vector<M>& m) {
    os << "\"" << name << "\": [" << std::endl;
    bool needsCommaA = false;
    for(auto& matrix : m) {
        if(needsCommaA) os << ",";
        needsCommaA = true;

        write_matrix(os, matrix);
    }
    os << "]" << std::endl;
    return os;
}
static inline std::ostream& write_matrix(std::ostream & os, const char* name, const CnMat* m) {
    os << "\"" << name << "\":" << std::endl;
    write_matrix(os, *m);
    return os;
}
static inline std::ostream& write_matrix(std::ostream & os, const char* name, const cnmatrix::Matrix& m) {
    return write_matrix(os, name, (const CnMat*)m);
}

void ModelRunner::Run(cnkalman::KalmanModel &model, bool show, const CnMat* iX, const CnMat *Q, std::vector<cnmatrix::Matrix> mRs) {
    cnkalman::ModelPlot plotter(model.name);
    plotter.show = show;
    draw_gt = true;
    Run(plotter, model, iX, Q, mRs);
}
void ModelRunner::Run(cnkalman::ModelPlot& plotter, cnkalman::KalmanModel &model, const CnMat* iX, const CnMat *Q, std::vector<cnmatrix::Matrix> mRs, bool drawMap) {
    model.reset();

    std::vector<std::vector<std::vector<FLT>>> observations;
    std::vector<cnmatrix::Matrix> Xs;
    std::vector<std::vector<FLT>> Xfs;
    std::vector<std::tuple<double, double>> pts_GT;
    std::vector<std::tuple<double, double>> pts_Filtered;

    std::vector<double> origerrors, besterrors, error;

    FLT deviations = 2;

    std::vector<cnmatrix::Matrix> Rs;
    for (auto &measModel : model.measurementModels) {
        auto &R = Rs.emplace_back(measModel->default_R());
        measModel->meas_mdl.term_criteria.max_iterations = max_iterations;
        //measModel->meas_mdl.meas_jacobian_mode = cnkalman_jacobian_mode_debug;
    }

    //model.kalman_state.transition_jacobian_mode = cnkalman_jacobian_mode_debug;

    std::vector<std::vector<std::vector<FLT>>> covs;
    std::vector<double> meas_error;

    CN_CREATE_STACK_VEC(X, model.state_cnt);
    cnCopy(model.stateM, &X, 0);

    if(iX) {
        cnCopy(iX, model.stateM, 0);
    }

    FLT t = 1 - dt;
    plotter.include_point_in_range(model.state);
    plotter.include_point_in_range(_X);

    covs.emplace_back(cnMatToVectorVector(model.kalman_state.P));
    pts_Filtered.emplace_back(model.state[0], model.state[1]);
    pts_GT.emplace_back(X.data[0], X.data[1]);
    Xfs.push_back(cnMatToVector(*model.stateM));

    srand(42);
    for (int i = 0; i <= iterations; i++) {
        t += dt;
        model.sample_state(dt, X, X, Q);
        Xs.push_back(X);
    }

    t = 1 - dt;
    for (int i = 0; i <= iterations; i++) {
        t += dt;

        X = Xs[i];

        plotter.include_point_in_range(X.data);

        pts_GT.emplace_back(X.data[0], X.data[1]);

        if (i % run_every != 0)
            continue;

        model.update(t);

        if (i % ellipse_step == 0) {
            //plotter.plot_cov(model, deviations, "blue");
        }

        FLT error = 0, oerror = 0;

        std::vector<std::vector<FLT>> obs;
        std::vector<CnMat> Zs;
        for (int j = 0; j < (int) model.measurementModels.size(); j++) {
            auto &measModel = model.measurementModels[j];
            auto Z = cnMatCalloc(measModel->meas_cnt, 1);
            measModel->sample_measurement(X, Z, mRs.size() ? mRs[j] : Rs[j]);
            Zs.push_back(Z);
            obs.push_back(cnMatToVector(Z));
        }

        if (bulk_update) {
            model.bulk_update(t, Zs, Rs);
        } else {
            for (int j = 0; j < (int) model.measurementModels.size(); j++) {
                auto &measModel = model.measurementModels[j];
                auto Z = Zs[j];

                if(run_every_per_meas.empty() || iterations % run_every_per_meas[j] == 0) {
                    auto stats = measModel->update(t, Z, Rs[j]);
                    error += stats.besterror;
                    oerror += stats.origerror;
                }
            }
        }

        for (int j = 0; j < (int) model.measurementModels.size(); j++) {
            auto &Z = Zs[j];
            free(Z.data);
        }

        observations.push_back(obs);
        plotter.include_point_in_range(model.state);

        Xfs.push_back(cnMatToVector(*model.stateM));
        pts_Filtered.emplace_back(model.state[0], model.state[1]);

        besterrors.push_back(error);
        origerrors.push_back(oerror);

        meas_error.emplace_back(cnDistance(&X, model.stateM));

        if (i % ellipse_step == 0) {
            plotter.plot_cov(model, deviations);
        }
        covs.emplace_back(cnMatToVectorVector(model.kalman_state.P));
    }

    std::string suffix = "";
    if(!settingsName.empty()) suffix += "." + settingsName;
    std::ofstream f(model.name + suffix + ".kf");
    f << "{" << std::endl;
    model.write(f);
    f << ", " << std::endl;
    write_matrix(f, "Z", observations) << ", ";
    write_matrix(f, "X", Xs) << ", ";

    if(Q) {
        write_matrix(f, "Q", Q) << ", ";
    }

    write_matrix(f, "Xf", Xfs) << ", ";
    write_matrix(f, "Ps", covs) << ", ";
    write_matrix(f, "Rs", Rs) << ", ";
    f << "\"run_every\":" << run_every << ", " << std::endl;
    f << "\"dt\":" << dt << ", " << std::endl;
    f << "\"ellipse_step\":" << ellipse_step << "" << std::endl;
    f << "}";

    model.draw(plotter);
#ifdef HAS_SCIPLOT

    //plotter.plot.drawWithVecs("lines", origerrors).label(settingsName + " Orig");
    plotter.cnt++;
    plotter.plot.drawWithVecs("lines", besterrors).label(settingsName + " Best").dashType(plotter.cnt % 12).lineStyle(plotter.cnt % 16);
    plotter.plot.drawWithVecs("lines", meas_error).label(settingsName + " GT Error").dashType(plotter.cnt % 12).lineStyle(plotter.cnt % 16);

    if (!model.measurementModels.empty() && drawMap ) {
        cnmatrix::Matrix map(250, 250 * 3);
        cnmatrix::Matrix map2(250, 250);

        FLT x,y,w,h;
        plotter.get_view(x, y, w, h);
        CN_CREATE_STACK_VEC(S, model.state_cnt);
        int color_offset = 0;
        FLT min_z[4] = {INFINITY,INFINITY,INFINITY, INFINITY}, max_z[4] = {-INFINITY, -INFINITY, -INFINITY, -INFINITY};

        for (int j = 0; j < 250; j++) {
            for (int i = 0; i < 250; i++) {
                FLT sx = i / 250. * w + x - w / 2;
                FLT sy = j / 250. * w + y - w / 2;
                S.data[0] = sx;
                S.data[1] = sy;
                color_offset = 0;
                for(size_t z = 0;z < model.measurementModels.size();z++) {
                    auto &meas = model.measurementModels[z];
                    auto Z = cnmatrix::Matrix(meas->meas_cnt);
                    meas->predict_measurement(S, Z, 0);
                    auto iR = cnmatrix::Matrix::Like(Rs[z]);
                    cnInvert(Rs[z], iR, CN_INVERT_METHOD_SVD);

                    auto rZ = cnmatrix::Matrix::Like(Z);
                    cnGEMM(iR, Z, 1, 0, 0, rZ, (enum cnGEMMFlags)0);

                    for(size_t zc = 0;zc < meas->meas_cnt;zc++) {
                        map(j, i * 3 + (color_offset++) % 3) += fabs(rZ(zc));
                    }
                }
            }
        }
        for (int j = 0; j < 250; j++) {
            for (int i = 0; i < 250; i++) {
                for(int k = 0;k < 3;k++) {
                    auto v = map(j, i * 3 + k);
                    min_z[k] = fmin(min_z[k], v);
                    max_z[k] = fmax(max_z[k], v);
                    map2(j, i) += v;
                }
                min_z[3] = fmin(min_z[3], map2(j,i));
                max_z[3] = fmax(max_z[3], map2(j,i));
            }
        }

        FILE *f = fopen("map.rgb", "w");
        FILE *f2 = fopen("map2.rgb", "w");
        for (int j = 0; j < 250; j++) {
            for (int i = 0; i < 250; i++) {
                for(int k = 0;k < 3;k++) {
                    auto v = (map(j, i * 3 + k) - min_z[k]) / (max_z[k] - min_z[k] + 1e-10);
                    uint8_t b = fmax(0, fmin(255, v * 255));
                    fwrite(&b, 1, 1, f);
                }
                uint8_t b = 0xff;

                auto v = (map2(j, i) - min_z[3]) / (max_z[3] - min_z[3] + 1e-10);
                b = fmax(0, fmin(255, v * 255));
                uint8_t alpha = 0;

                FLT div = .1;
                FLT fv = fmod(v, div);
                if(fv > div/2) fv = div - fv;
                FLT pa = .01, pb = .005;
                auto fvm = fmax(0, fmin(1, (fv - pb) / (pa - pb)));
                alpha = 80 * fvm + (1-fvm) * 0;

                for(int zz = 0;zz < 3;zz++) fwrite(&b, 1, 1, f2);
                fwrite(&alpha, 1, 1, f);
                fwrite(&alpha, 1, 1, f2);
            }
        }
        fclose(f2);
        fclose(f);

        {
            std::stringstream ss;
            ss << "'map.rgb' binary array=(250,250) center=(" << x << ", " << y << ") dx=" << (w / 250.)
               << " format='%uchar' ";
            plotter.map.draw(ss.str(), "", "rgbalpha").labelNone();
        }
        if(false)
        {
            std::stringstream ss;
            ss << "'map2.rgb' binary array=(250,250) center=(" << x << ", " << y << ") dx=" << (w / 250.)
               << " format='%uchar' ";
            plotter.map.draw(ss.str(), "", "rgbalpha").labelNone();
        }
    }

    if(draw_gt)
        plotter.map.drawWithVecs("lines", pts_GT).label(settingsName + " GT").lineWidth(3);
    draw_gt = false;
    plotter.cnt++;
    plotter.map.drawWithVecs("lines", pts_Filtered).label(settingsName + " Filter").dashType(plotter.cnt % 12).lineStyle(plotter.cnt % 16);

#endif

}

ModelRunner::ModelRunner(const std::string &settingsName, double dt, int iterations, int max_iterations, int runEvery, int ellipseStep,
                         bool bulkUpdate) : settingsName(settingsName), dt(dt), iterations(iterations), max_iterations(max_iterations),
                                            run_every(runEvery), ellipse_step(ellipseStep), bulk_update(bulkUpdate) {}
