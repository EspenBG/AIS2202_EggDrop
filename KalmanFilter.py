import numpy as np


class KalmanFilter:
    import numpy as np
    """Kalman filter"""
    def __init__(self, G, variance, H, start_value):
        self.A_ = None  # Can not be made before delta_t is given 3x3 vector
        self.x_priori_ = start_value  # Set initial start value 1x3 vector

        self.H_ = H  # 3x1 vector
        self.x_Var_ = variance
        self.R_ = None
        self.Q_ = None

        self.x_ = 0
        pass

    def getNewSensorValue(self, delta_t, measurement, H=None):
        if H is not None:
            self._H_ = H
        x = np.array([0, 0, 0])

        return x
        pass

    def _pred_x_(self):
        pass

    def _pred_P_(self):
        pass

    def _correct_K_(self):
        pass

    def _correct_x_(self):
        pass

    def _correct_P_(self):
        pass


a = KalmanFilter()