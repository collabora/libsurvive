from sympy import cos, sin, tan, atan2

import numpy as np
import sympy
from filterpy.kalman import ExtendedKalmanFilter as EKF
from numpy import array, sqrt
from sympy import symbols, Matrix
import cnkalman.codegen as cg

@cg.generate_code(x=4)
def predict_meas(x):
    var_a = 101
    var_b = 11
    coeff_dist = 5

    return [
            sin((x[0] * x[0] + x[1] * x[1]) * coeff_dist),
            sin(x[0] * var_a) * sin(x[1] * var_a),
            sin(x[0] * var_b) * cos(x[1] * var_b)
        ]

class EggLandscape(EKF):
    def __init__(self):
        EKF.__init__(self, 4, 3)
        self.Q = np.array([
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, .0001, 0],
            [0, 0, 0, 0.0001],
        ])
        self.F = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
            ])

    def update(self, z, R):
        EKF.update(self, z,
                   HJacobian=lambda x: predict_meas.jacobians.x(x.reshape(-1)),
                   Hx=lambda x: np.array(predict_meas(x.reshape(-1)), dtype=np.float64).reshape(-1, 1), R=R)