from KalmanFilter import KalmanFilter

def main():
    """
    Example: Self driving example

    From https://www.zhihu.com/question/23971601/answer/2757161188

    IMU: prior
    GNSS: measurement

    In this example:
    main update equation
    # Xk = Kk Zk + (1 - Kk) X{k-1}
    # X_k^{\hat} = K_k * Z_k + (1 - K_k) * X_{k-1}^{\hat}
    assumption + prediction
    # Xk = A X{k-1} + B uk + w{k-1}
    # zk = H xk + vk
    Prediction
    Xk- = A X{k-1} + B uk
    Pk- = A P{k-1} AT + Q
    Update
    Kk = Pk- HT(H Pk- HT + R)^-1
    xk = (1 - Kk H ) Xk + Kk zk
    Pk = (1-Kk H) Pk-

    wk is related to Q: Q = var(wk)
    vk is related to R: R = var(vk)
    Q = process_variance = 0.12
    R = estimated_measurement_variance = 0.12
    xk = posteri_estimate = 2 initially
    Pk =  posteri_error_estimate = 0.22 initially
    """
    measurement_list = [13]
    process_variance = 0.1**2
    estimated_measurement_variance = 0.4**2
    posteri_estimate = 12
    posteri_error_estimate = 0.22**2 - 0.1**2
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
