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
        self.R_ = None
        self.P_
        self.P_priori_
        self.K_

        self.x_ = 0
        pass
    def setH(self, H):
        self.H_ = H

    def setVarians(self, varians):
        self.x_Var_ = varians

    def getNewSensorValue(self, delta_t, measurement):
        x = np.array([0, 0, 0])

        return x
        pass

   def calc_Q_(self):
       pass

    def predict(self):
        self.x_priori_ = self.A_@self.x_
        self.P_priori_ = self.A_@self.P_@np.transpose(self.A_)+self.Q_

    def correct(self):
        tmpHP = self.P_priori@np.transpose(self.H_)
        self.K_ = tmpHP*(self.H_@tmpHP+self.x_Var_)

        self.x_=self.x_priori_+self.K_*(z-self.H_@self.x_priori_)

        self.P_=(1-self.K_@self.H_)@self.P_priori


a = KalmanFilter()