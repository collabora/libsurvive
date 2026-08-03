#include <models/BearingsOnlyTracking.h>
#include "models/LinearPoseVel.h"
#include "models/EggLandscape.h"
#include "models/BikeLandmarks.h"
#include "models/BearingAccel.h"

#include "ModelRunner.h"

int main(int argc, char** argv) {
    bool show = argc > 1 && strcmp(argv[1], "--show") == 0;

    ModelRunner defaultRunner;
    ModelRunner defaultRunnerEvery10;
    defaultRunnerEvery10.run_every = 10;

    BikeLandmarks bikeModel;
    EggLandscapeProblem eggModel;
    LinearPoseVel linearModel;
	BearingAccel bearingAccel;
    BearingAccel bearingAccelNoIMU(false);
    BearingAccel bearingAccelNoLandmark(true, false);

	BearingsOnlyTracking bearingsOnlyTracking;

    defaultRunnerEvery10.Run(bikeModel, show);
    defaultRunner.Run(eggModel, show);
    defaultRunner.Run(linearModel, show);
    defaultRunner.Run(bearingsOnlyTracking, show);
    {
        cnkalman::ModelPlot iekfPlot("TDOA", show);

        ModelRunner iekfRunner1000("iekf", .01, 5000, 10, 1, 500, false);
        iekfRunner1000.run_every_per_meas = { 10, 10, 10, 10, 10, 10, 1};
        ModelRunner runner1000("ekf", .01, 5000, 0, 1, 500, false);
        runner1000.run_every_per_meas = { 10, 10, 10, 10, 10, 10, 1};

        iekfRunner1000.Run(iekfPlot, bearingAccel, 0, 0, {}, true);
        iekfRunner1000.settingsName = "iekf(No IMU)";
        iekfRunner1000.Run(iekfPlot, bearingAccelNoIMU, 0, 0, {}, false);
        runner1000.draw_gt = false;
        runner1000.Run(iekfPlot, bearingAccel, 0, 0, {}, false);
    }

    CN_CREATE_STACK_MAT(Q, 2, 2);
    std::vector<cnmatrix::Matrix> Rs;
    Rs.emplace_back(1, 1);
    Rs.emplace_back(1, 1);
    Rs.emplace_back(1, 1);

    CN_CREATE_STACK_VEC(initial_x, 2);
    initial_x.data[0] = .5; initial_x.data[1] = .1;

    cnkalman::ModelPlot iekfPlot("IEKF vs not");
    iekfPlot.show = show;
    ModelRunner testRunner("small-iteration", .1, 5, 0, 1, 1, false);
    testRunner.Run(iekfPlot, bearingsOnlyTracking, &initial_x, &Q, Rs);

    ModelRunner testRunnerIEKF("small-iteration-iekf", .1, 5, 5, 1, 1, false);
    testRunnerIEKF.Run(iekfPlot, bearingsOnlyTracking, &initial_x, &Q, Rs);

    return 0;
}
