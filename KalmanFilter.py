import numpy as np


class KalmanFilter:
    import numpy as np
    """Kalman filter, 
    """
    def __init__(self, A, G,  set_variance, white_variance=0, start_value=np.array([0, 0, 0]), H=None):
        size = A(0).size()

        self.A_ = A  # Can not be made before delta_t is given 3x3 vector
        self.G_ = G
        self.x_priori_ = start_value  # Set initial start value 1x3 vector
        self.x_ = start_value

        if H is None:
            self.H_ = H
        else:
            self.H_ = np.zeroes(size[0])  # 3x1 vector

        self.x_Var_ = set_variance
        self.R_ = white_variance
        self.P_ = np.zeros(size)
        self.P_priori_ = None
        self.K_ = np.zeros(size[0])

        # Q matrix represent G*G^(-1)*(expected variance)
        self.Q_ = None



    def setH(self, H):
        self.H_ = H

    def setHviteVarians(self, variance):
        self.R_ = variance

    def getNewSensorValue(self, delta_t, measurement):
        self.calc_Q_(delta_t)
        self.predict(delta_t)
        self.correct(measurement)

    def get_x(self):
        return self.x_.copy()

    def calc_Q_(self, delta_t):
        self.Q_ = self.G_(delta_t)@np.transpose(self.G_(delta_t))*self.x_Var_

    def predict(self, delta_t):
        self.x_priori_ = self.A_(delta_t)@self.x_ # + w_(k-1)  // dette er produkt stoy
        self.P_priori_ = self.A_(delta_t)@self.P_@np.transpose(self.A_(delta_t))+self.Q_

    def correct(self, measurement):
        tmpHP = self.P_priori@np.transpose(self.H_)
        self.K_ = tmpHP*(self.H_@tmpHP+self.hvite_var)

        self.x_=self.x_priori_+self.K_*(measurement-self.H_@self.x_priori_)

        self.P_=(1-self.K_@self.H_)@self.P_priori


