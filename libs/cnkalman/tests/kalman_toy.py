import math
import unittest

import numpy as np

from cnkalman import filter

class StandardModel(filter.Model):
    def __init__(self):
        super().__init__("standard_model", 2)

    def predict(self, dt, x0):
        return x0

    def predict_jac(self, dt, x0):
        return np.eye(2)

    def process_noise(self, dt, x):
        return np.eye(2) * .1

def measurement_model(sensor_pos, state):
    return math.atan2(state[1] - sensor_pos[1], state[0] - sensor_pos[0])

class StandardMeasModel(filter.MeasurementModel):
    def __init__(self, mdl, sensors):
        self.sensors = sensors
        super().__init__(mdl, 1)
        self.meas_mdl.meas_jacobian_mode = filter.jacobian_mode.debug

    def predict_measurement(self, x):
        return np.array([measurement_model(sensor, x) for sensor in self.sensors])

    def predict_measurement_jac(self, x):
        H = np.zeros((len(self.sensors), 2))
        for idx, sensor in enumerate(self.sensors):
            dx = x[0] - sensor[0]
            dy = x[1] - sensor[1]
            scale = 1. / ((dx*dx) + (dy*dy)) if dx != 0 or dy != 0 else 0
            H[idx, 0] = scale * -dy
            H[idx, 1] = scale * dx
        return H
#  2 x  1: +5.0000000e-01, +1.0000000e-01,
#
# -3.8461538e-01, +1.9230769e+00,
# -9.9009901e-02, -9.9009901e-01,
def run_standard_experiment(termCriteria, time_steps):
    true_state = [1.5, 1.5]
    X = [.5, .1]
    sensors = np.array([
        [0, 0],
        [1.5, 0]
    ])

    mdl = StandardModel()
    mdl.kalman_state.X = np.array(X)
    mdl.kalman_state.P = np.eye(2) * .1

    Rv = math.pi * math.pi * 1e-5
    R = np.array([Rv, Rv])

    Z = np.array([measurement_model(sensor, true_state) for sensor in sensors])

    meas_model = StandardMeasModel(mdl, sensors)
    meas_model.meas_mdl.term_criteria = termCriteria

    for i in range(time_steps):
        meas_model.update(1., Z, R)
        print(f"{np.array(mdl.kalman_state.X)}")

    assert meas_model.jacobian_debug_misses() < .1
    return mdl.kalman_state.P, mdl.kalman_state.X

class KalmanTestCase(unittest.TestCase):
    def test_EKF(self):
        expected_X = [4.4049048331227372, 1.1884307714294169]
        expected_P = np.array([0.0014050624776344668, 0.00019267084936229752, 0.0001926708493623336, 4.751355804986091e-05]).reshape(2,2)
        termCriteria = filter.term_criteria_t()
        P, X = run_standard_experiment(termCriteria, 1)
        np.testing.assert_allclose(X, expected_X, rtol=1e-5, atol=1e-5)
        np.testing.assert_allclose(P, expected_P, rtol=1e-5, atol=1e-5)

    def test_IEKF(self):
        termCriteria = filter.term_criteria_t(max_iterations=10)
        P, X = run_standard_experiment(termCriteria, 1)

        error = np.linalg.norm(X - np.array([1.5,1.5]))
        assert .01 > error

def main():
    termCriteria = filter.term_criteria_t()
    print(run_standard_experiment(termCriteria, 1))

if __name__ == '__main__':
    main()
