from KalmanFilter import KalmanFilter

def main():
    """
    Example: Independent Measurement
    
    Implementing https://zhuanlan.zhihu.com/p/64539108

    In this example:
    main update equation
    # Xk = Kk Zk + (1 - Kk) X{k-1}
    # X_k^{\hat} = K_k * Z_k + (1 - K_k) * X_{k-1}^{\hat}
    assumption + prediction
    # Xk = A X{k-1} + B uk + w{k-1}
    # zk = H xk + vk
    Prediction
    Xk- = X{k-1}
    Pk- = P{k-1} + Q
    Update
    Kk = Pk- HT(H Pk- HT + R)^-1
    xk = (1 - Kk H ) Xk + Kk zk
    Pk = (1-Kk H) Pk-

    wk is related to Q: Q = var(wk)
    vk is related to R: R = var(vk)

    Q = process_variance = 0
    R = estimated_measurement_variance = 0.1
    xk = posteri_estimate = 0.0 initially
    Pk =  posteri_error_estimate = 1.0 initially

    """
    measurement_list = [0.39, 0.50, 0.48, 0.29, 0.25, 0.32, 0.34, 0.48, 0.41, 0.45]
    process_variance = 0
    estimated_measurement_variance = 0.1
    posteri_estimate = 0.0
    posteri_error_estimate = 1.0
    for measurement in measurement_list:
        kalman_filter = KalmanFilter(
            process_variance,
            estimated_measurement_variance,
            posteri_estimate,
            posteri_error_estimate,
        )
        kalman_filter.input_latest_noisy_measurement(measurement)
        posteri_estimate = kalman_filter.get_latest_estimated_measurement()
        posteri_error_estimate = kalman_filter.get_latest_estimated_error()
        print("#" * 20)


if __name__ == "__main__":
    main()
