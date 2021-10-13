import numpy as np


class KalmanFilter:
    import numpy as np
    """Kalman filter, 
    """
    def __init__(self, A, start_value, variance):
        size = A.size()

        self.A_ = A  # Can not be made before delta_t is given 3x3 vector
        self.x_priori_ = start_value  # Set initial start value 1x3 vector

        self.H_ = np.zeros(size[0])  # 3x1 vector
        self.x_Var_ = variance
        self.R_ = None

        # Q matrix represent G*G^(-1)*(expected variance)
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