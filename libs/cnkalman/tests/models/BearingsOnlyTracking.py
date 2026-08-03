from symengine import atan2, asin, cos, sin, tan, sqrt
import cnkalman.codegen as cg
from filterpy.kalman import ExtendedKalmanFilter as EKF
import numpy as np

@cg.generate_code(state = 2, landmark = 2)
def bearings_meas_function(state, landmark):
    x, y = state
    px, py = landmark
    return atan2(y - py, x - px)

class BearingsOnlyTracking(EKF):
    def __init__(self):
        EKF.__init__(self, 2, 1)
        self.Q = np.eye(2) * .1
        self.F = np.eye(2)
        self.landmarks = [[0, 0], [1.5, 0]]

    def update(self, z, R, idx):
        def residual(a, b):
            """ compute residual (a-b) between measurements containing
            [range, bearing]. Bearing is normalized to [-pi, pi)"""
            y = a - b
            y[0] = y[0] % (2 * np.pi)    # force in range [0, 2 pi)
            if y[0] > np.pi:             # move to [-pi, pi)
                y[0] -= 2 * np.pi
            return y

        EKF.update(self, z,
                   HJacobian=lambda x: bearings_meas_function.jacobians.state(x.reshape(-1), self.landmarks[idx]), residual=residual,
                   Hx=lambda x: np.array(bearings_meas_function(x.reshape(-1), self.landmarks[idx]), dtype=np.float64).reshape(-1, 1), R=R)
