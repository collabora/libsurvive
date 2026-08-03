#pragma once
#include <string>
#include "cnmatrix/cn_matrix.h"
#include "cnkalman/model.h"
#if HAS_SCIPLOT
#include <sciplot/sciplot.hpp>
#endif
namespace cnkalman {
    struct ModelPlot {
        FLT range[4] = {INFINITY, -INFINITY, INFINITY, -INFINITY};
        bool show = false;
        bool lock_range = false;
        std::string name;
        int cnt = 0;
#ifdef HAS_SCIPLOT
        sciplot::Plot2D plot;
        sciplot::Plot2D map;
#endif
        ModelPlot(const std::string &name = "plot", bool show = false);

        void plot_cov(const cnkalman::KalmanModel &model, FLT deviations, const std::string &color = "red");

        void include_point_in_range(const FLT *X);

        void get_view(FLT &x, FLT &y, FLT &w, FLT &h) const;

        void include_point_in_range(FLT x, FLT y);

        ~ModelPlot();
    };
}
