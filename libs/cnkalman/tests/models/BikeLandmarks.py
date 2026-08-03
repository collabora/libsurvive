from symengine import atan2, asin, cos, sin, tan, sqrt
import cnkalman.codegen as cg

@cg.generate_code(state = 3, u = 2)
def predict_function(dt, wheelbase, state, u):
    x, y, theta = state
    v, alpha = u
    d = v * dt
    R = wheelbase/tan(alpha)
    beta = d / wheelbase * tan(alpha)

    return [x + -R * sin(theta) + R * sin(theta + beta),
            y + R * cos(theta) - R * cos(theta + beta),
            theta + beta]

@cg.generate_code(state = 3, landmark = 2)
def meas_function(state, landmark):
    x, y, theta = state
    px, py = landmark

    hyp = (px-x)**2 + (py-y)**2
    dist = sqrt(hyp)

    return [dist, atan2(py - y, px - x) - theta]
