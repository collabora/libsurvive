# CNKalman [![Build and Test](https://github.com/cntools/cnkalman/actions/workflows/cmake.yml/badge.svg)](https://github.com/cntools/cnkalman/actions/workflows/cmake.yml)

This is a relatively low level implementation of a kalman filter; with support for extended and iterative extended
kalman filters. The goals of the project are to provide a numerically stable, robust EKF implementation which is both
fast and portable. 

The main logic is written in C and only needs the associated matrix library to work; and there are C++ wrappers provided 
for convenience.

## Tutorial on Kalman Filter Theory

There was originally going to be a more in depth discussion of the theoretical side but there isn't much one could do to 
improve on the very in depth [tutorial by Roger Labbe](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/).

## Features

- Support for [extended kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter), [linear kalman filters](https://en.wikipedia.org/wiki/Kalman_filter), and [Iterate Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter#Iterated_extended_Kalman_filter) ([paper](https://www.diva-portal.org/smash/get/diva2:844060/FULLTEXT01.pdf))
- Support for [adaptive measurement covariance](https://arxiv.org/pdf/1702.00884.pdf)
- Support for [error-state kalman filter](http://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf)  
- Built-in support for numerical-based jacobians, and an option to debug user provided jacobians by using 
  the numerical results
- Minimal heap allocations  
- Supports multiple measurement models per filter, which can be integrated at varying frequencies
- C++ bindings for objected oriented applications
- Automatic code generation for analytical jacobians

## Quick start

[kalman_example.c](https://github.com/cntools/cnkalman/blob/develop/tests/kalman_example.c)
```C
#include <cnkalman/kalman.h>
#include <stdio.h>

static inline void kalman_transition_model_fn(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0,
struct CnMat *x1, struct CnMat *F) {
    // Logic to fill in the next state x1 and the associated transition matrix F
}

static inline void kalman_process_noise_fn(void *user, FLT dt, const struct CnMat *x, struct CnMat *Q) {
    // Logic to fill in the process covariance Q
}

static inline bool kalman_measurement_model_fn(void *user, const struct CnMat *Z, const struct CnMat *x_t,
struct CnMat *y, struct CnMat *H_k) {
    // Logic to fill in the residuals `y`, and the jacobian of the predicted measurement function `h`
    return false; // This should return true if the jacobian and evaluation were valid.
}

int main() {
    int state_cnt = 1;
    cnkalman_state_t kalman_state = { 0 };
    cnkalman_state_init(&kalman_state, state_cnt, kalman_transition_model_fn, kalman_process_noise_fn, 0, 0);
    // Uncomment the next line if you want to use numerical jacobians for the transition matrix
    //kalman_state.transition_jacobian_mode = cnkalman_jacobian_mode_two_sided; 
    
    cnkalman_meas_model_t kalman_meas_model = { 0 };
    cnkalman_meas_model_init(&kalman_state, "Example Measurement", &kalman_meas_model, kalman_measurement_model_fn);
    // Uncomment the next line if you want to use numerical jacobians for this measurement
    // kalman_meas_model.meas_jacobian_mode = cnkalman_jacobian_mode_two_sided; 
    
    CnMat Z, R;
    // Logic to fill in measurement matrix Z and measurement covariance matrix R
    cnkalman_meas_model_predict_update(1, &kalman_meas_model, 0, &Z, &R);
    
    printf("Output:%f\n", cn_as_vector(&kalman_state.state)[0]);
    return 0;
}
```

# Code Generation

One of the more difficult things about extended kalman filters is the fact that calculating the jacobian for even a simple
measurement or prediction function can be tedious and error prone. An optional portion of cnkalman is easy integration 
of symengine in such a way that you can write the objective function in python and it'll generate the C implementation 
of both the function itself as well as it's jacobian with each of it's inputs. 

[BikeLandmarks.py](https://github.com/cntools/cnkalman/blob/develop/tests/models/BikeLandmarks.py)
```python
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
```

There are limitations in what type of logic is permissible here -- it must be something that is analytically tractable --
but most objective functions themselves are not too complicated to write. 

Notice that for array-type input like `state` and `u` above, you must specify a size hint.

When ran, this python script generates the companion `BikeLandmarks.gen.h` which has the generated code, and callouts
such as:
```c
static inline void gen_predict_function(CnMat* out, const FLT dt, const FLT wheelbase, const FLT* state, const FLT* u);
static inline void gen_predict_function_jac_state(CnMat* Hx, const FLT dt, const FLT wheelbase, const FLT* state, const FLT* u);
```

If you include this project in as a cmake project, a cmake function `cnkalman_generate_code` is available that makes this
an optional part of your build process. 

