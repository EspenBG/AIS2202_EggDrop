import numpy as np


class KalmanFilter:
    import numpy as np

    def __init__(self, time_usage, Var):
        self._A_ = None
        self._x_priori_ = 0
        self._H_ = None
        self._x_Var_ = Var
        self._Q_ = None
        self._R_ = None
        self._Q_ = None

        self._x_ = 0
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