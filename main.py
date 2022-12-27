from KalmanFilter import KalmanFilter

def main():
    """Implementing https://zhuanlan.zhihu.com/p/64539108"""
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
        posteri_error_estimate = kalman_filter.get_latest_estiamted_error()
        print("#" * 20)

if __name__ == "__main__":
    main()
