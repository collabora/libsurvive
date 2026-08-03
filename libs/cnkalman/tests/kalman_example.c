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