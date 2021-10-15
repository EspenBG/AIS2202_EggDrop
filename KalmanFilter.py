import numpy as np


class KalmanFilter:
    import numpy as np
    """Kalman filter, 
    """
    def __init__(self, A, G,  set_variance, white_variance=0, start_value=np.array([0, 0, 0]), H=None):
        size = A(0).shape

        self.A_ = A  # Can not be made before delta_t is given 3x3 vector
        self.G_ = G
        self.x_priori_ = start_value  # Set initial start value 1x3 vector
        self.x_ = start_value

        if H is not None:
            self.H_ = H
        else:
            self.H_ = np.zeros((1, size[0]))  # 3x1 vector

        self.x_Var_ = set_variance
        self.R_ = white_variance
        self.P_ = np.zeros(size)
        self.P_priori_ = None
        self.K_ = np.zeros((size[0], 1))

        # Q matrix represent G*G^(-1)*(expected variance)
        self.Q_ = None
        self.I = np.eye(size[1])

    def setR(self, R):
        self.R_ = R

    def setH(self, H):
        self.H_ = H

    def set_hvite_varians(self, variance):
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
        A = self.A_(delta_t)
        self.x_priori_ = A@self.x_  # + w_(k-1)  // dette er produkt stoy
        self.calc_Q_(delta_t)
        A_trans = A.T
        self.P_priori_ = A@self.P_@A_trans+self.Q_

    def correct(self, measurement):
        H_trans = self.H_.T
        tmpPH = self.P_priori_@H_trans
        self.K_ = tmpPH*np.linalg.inv(self.H_@tmpPH+self.R_)

        self.x_=self.x_priori_+self.K_*(measurement-self.H_@self.x_priori_)

        self.P_=(self.I-self.K_@self.H_)@self.P_priori_


