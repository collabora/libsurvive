#include <cnkalman/model.h>
#include "cnkalman/ModelPlot.h"

struct ModelRunner {
    std::string settingsName = "";
    FLT dt = 0.1;
    int iterations = 200;
    int max_iterations = 0;
    int run_every = 1;
    int ellipse_step = 20;
    bool bulk_update = false;
    bool draw_gt = true;
    std::vector<int> run_every_per_meas;

    ModelRunner(const std::string &settingsName = "", double dt = .1, int iterations = 200, int max_iterations = 0, int runEvery = 1, int ellipseStep = 20,
                bool bulkUpdate = false);

    void Run(cnkalman::KalmanModel& model, bool show = false, const CnMat* X = 0, const CnMat* Q = 0, std::vector<cnmatrix::Matrix> mRs = {});
    void Run(cnkalman::ModelPlot& plotter, cnkalman::KalmanModel& model, const CnMat* X = 0, const CnMat* Q = 0, std::vector<cnmatrix::Matrix> mRs = {}, bool drawMap = true);
};
