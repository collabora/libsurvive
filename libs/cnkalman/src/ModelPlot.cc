#include "cnkalman/ModelPlot.h"
#include <algorithm>

namespace cnkalman {
  ModelPlot::ModelPlot(const std::string &name, bool show) : show(show), name(name) {
#ifdef HAS_SCIPLOT
        plot.gnuplot("set title \"" + name + "\"");
        map.gnuplot("set title \"" + name + "\"");
        map.palette("paired");

        map.size(1600, 1600);
        map.gnuplot("set size square");
        map.border().none();

        plot.size(1600, 1200);
#endif
    }

    void ModelPlot::include_point_in_range(const FLT *X) {
        if(lock_range) return;

        for (int x = 0; x < 2; x++) {
            range[x * 2] = std::min(range[x * 2], X[x]);
            range[x * 2 + 1] = std::max(range[x * 2 + 1], X[x]);
        }
    }

    void ModelPlot::include_point_in_range(FLT x, FLT y) {
        FLT Xs[] = {x, y};
        include_point_in_range(Xs);
    }

    void ModelPlot::get_view(FLT &x, FLT &y, FLT &w, FLT &h) const {
        FLT dx = fmax(1, range[1] - range[0]);
        FLT dy = fmax(1, range[3] - range[2]);
        FLT R0 = range[0] - .1 * dx;
        FLT R1 = range[1] + .1 * dx;
        FLT R2 = range[2] - .1 * dy;
        FLT R3 = range[3] + .1 * dy;
        w = R1 - R0, h = R3 - R2;
        x = R1 - w / 2, y = R3 - h / 2;
        w = fmax(w, h);
        h = w;
    }

    ModelPlot::~ModelPlot() {

        FLT dx = fmax(1e-3, range[1] - range[0]);
        FLT dy = fmax(1e-3, range[3] - range[2]);
        range[0] -= .1 * dx;
        range[1] += .1 * dx;
        range[2] -= .1 * dy;
        range[3] += .1 * dy;
        FLT w = range[1] - range[0], h = range[3] - range[2];
        FLT x = range[1] - w / 2, y = range[3] - h / 2;
        w = fmax(w, h);
        range[0] = x - w / 2;
        range[1] = x + w / 2.;
        range[2] = y - w / 2;
        range[3] = y + w / 2.;
#ifdef HAS_SCIPLOT
        map.xrange(range[0], range[1]);
        map.yrange(range[2], range[3]);

        if (show) {
            sciplot::Figure figs = {{plot, map}};
            sciplot::Canvas canvas {{figs}};
            canvas.show();
        }

        sciplot::Figure fig_plot = {{plot}};
        sciplot::Canvas canvas_plot = {{fig_plot}};
        canvas_plot.save(name + "-plot.svg");

        sciplot::Figure fig_map = {{map}};
        sciplot::Canvas canvas_map = {{fig_map}};
        canvas_plot.save(name + ".svg");
        canvas_plot.save(name + ".png");
#endif
    }

    void ModelPlot::plot_cov(const cnkalman::KalmanModel &model, FLT deviations, const std::string &color) {
#ifdef HAS_SCIPLOT
        CN_CREATE_STACK_MAT(Pp, 2, 2);
        CN_CREATE_STACK_MAT(Evec, 2, 2);
        CN_CREATE_STACK_VEC(Eval, 2);
        static int idx = 1;

        cnCopy(&model.kalman_state.P, &Pp, 0);
        cnSVD(&Pp, &Eval, &Evec, 0, (enum cnSVDFlags)0);
        FLT angle = atan2(_Evec[2], _Evec[0]) *57.2958;
        FLT v1 = deviations*2*sqrt(_Eval[0]), v2 = deviations*2*sqrt(_Eval[1]);
        std::stringstream ss;
        ss << "set obj " << idx++ << " ellipse fc rgb \"" << color << "\" fs transparent solid .5 center "
           << (model.state[0]) << "," <<
           (model.state[1]) << " size "
           << v1 << "," << v2 <<
           " angle " << angle << " front\n";
        map.gnuplot(ss.str());
#endif
    }
}
