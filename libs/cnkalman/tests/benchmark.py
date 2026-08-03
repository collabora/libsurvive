import math
import sys

import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.stats import plot_covariance

import json

from matplotlib import pyplot as plt

from models.EggLandscape import EggLandscape
from models.RobotEKF import RobotEKF
from models.BearingsOnlyTracking import BearingsOnlyTracking

kp = json.load(open(sys.argv[1]))
model = kp['model']

def create_filter(model):
    if model['name'] == "BikeLandmarks":
        return RobotEKF(1, .5, [1.1, .01], .1, .015)
    elif model['name'] == "EggLandscape":
        return EggLandscape()
    elif model['name'] == "BearingsOnlyTracking":
        return BearingsOnlyTracking()
    elif model['name'] == "LinearToy":
        f = KalmanFilter(model['state_cnt'], model['measurement_models'][0]['meas_cnt'])
        f.Q = np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0.01, 0],
            [0, 0, 0, 0.01],
        ])
        f.H = np.array([
            [-1, 0, 0, 0],
            [0, 1, 0, 0],
            [-.5, .5, 0, 0]
        ])
        f.F = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        return f


matrices = {k: np.array(kp[k]) for k in kp.keys() if k != 'model'}
Xf = matrices['Xf']
GT = matrices['X']
Ps = matrices['Ps']

def run_model(kp, f):
    ellipse_step = kp['ellipse_step']
    run_every = kp['run_every']

    Xf = matrices['Xf']
    GT = matrices['X']
    Ps = matrices['Ps']

    Zs = matrices['Z']

    f.x = np.array([Xf[0]]).transpose()
    f.P = Ps[0]

    Rs = matrices['Rs']

    Xs = []

    ax = plt.gca()  # you first need to get the axis handle
    ax.set_aspect(1)  # sets the height to width ratio to 1.5.

    dZs = []
    z_idx = 0
    Xs.append(f.x)
    pError = 0

    for i in range(0, GT.shape[0]):
        if i % run_every != 0:
            continue

        f.predict()

        if i % ellipse_step == 0 and False:
            plot_covariance(
                (f.x[0], f.x[1]), cov=f.P[0:2, 0:2],
                std=1, facecolor='k', alpha=0.3)

        z = Zs[z_idx]
        z_idx += 1

        if len(model['measurement_models']) == 1:
            R = Rs[0]
            f.update(z[0].reshape(-1, 1), Rs[0])
        else:
            for meas_idx in range(len(model['measurement_models'])):
                R = Rs[meas_idx]
                if hasattr(f, 'predict_meas'):
                    ZGt = f.predict_meas(GT[i].reshape(-1, 1), meas_idx)
                    dZ = z[meas_idx].reshape(-1) - ZGt.reshape(-1)
                    dZs.append(dZ)
                f.update(z[meas_idx].reshape(-1, 1), Rs[meas_idx], meas_idx)

        dP = f.P - Ps[z_idx]
        pError = np.linalg.norm(dP) ** 2

        Xs.append(f.x)

        if i % ellipse_step == 0:
            plot_covariance(
                (f.x[0], f.x[1]), cov=f.P[0:2, 0:2],
                std=1, facecolor='g', alpha=0.8)

    Xs = np.array(Xs)
    err = np.linalg.norm(Xf - Xs.reshape(Xf.shape)) / Xf.shape[0]
    print(err, math.sqrt(pError) / Xf.shape[0])

    has_error = math.sqrt(pError) / Xf.shape[0] > 1e-3 or err > 1e-3


    return Xs

f = create_filter(model)
Xs = run_model(kp, f)

plt.plot(Xs[:, 0], Xs[:, 1], label="pyfilter")
plt.plot(Xf[:, 0], Xf[:, 1], '--', label="cnkalman")
plt.plot(GT[:, 0], GT[:, 1], '-.', label="GT")
plt.legend()

if '--show' in sys.argv:
    plt.show()
