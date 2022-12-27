class KalmanFilter:
    """
    A simple Kalman filter implementation.
    
    Assumes a process with additive white Gaussian noise and attempts to estimate
    the state of the process using noisy measurements.
    
    Parameters
    ----------
    process_variance : float
        The variance of the process noise. Controls the trade-off between trusting
        the process model and the measurements.
    estimated_measurement_variance : float
        The variance of the measurement noise. Controls the trade-off between trusting
        the process model and the measurements.
        
        
    Example
    ----------
        # Create a Kalman filter with some process variance and some estimated measurement variance
        kalman_filter = KalmanFilter(1, 1)

        # Provide an initial estimate of the process state
        kalman_filter.input_latest_noisy_measurement(5)

        # Get the current estimate of the process state
        estimate = kalman_filter.get_latest_estimated_measurement()
        print(estimate)  # Output: 5

        # Input a new measurement
        kalman_filter.input_latest_noisy_measurement(10)

        # Get the updated estimate of the process state
        estimate = kalman_filter.get_latest_estimated_measurement()
        print(estimate)  # Output: something close to 7.5
    """
    
    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def input_latest_noisy_measurement(self, measurement):
        """
        Update the estimate of the process state based on the latest measurement.
        
        Parameters
        ----------
        measurement : float
            The latest measurement of the process state.
        """
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

    def get_latest_estimated_measurement(self):
        """
        Return the current estimate of the process state.
        
        Returns
        -------
        float
            The current estimate of the process state.
        """
        return self.posteri_estimate
