import numpy as np


class KalmanFilter:
    import numpy as np
    """Kalman filter, 
    """
    def __init__(self, A, G, H, P,  start_value, hvite_variance, set_variance):
        size = A.size()

        self.A_ = A  # Can not be made before delta_t is given 3x3 vector
        self.G_ = G
        self.x_priori_ = start_value  # Set initial start value 1x3 vector
        self.x_ = start_value

        self.H_ = H  # 3x1 vector
        self.x_Var_ = set_variance
        self.hvite_var = hvite_variance
        self.R_ = None
        self.P_ = P
        self.P_priori_ = None
        self.K_ = np.zeros(size[0])

        # Q matrix represent G*G^(-1)*(expected variance)
        self.Q_ = None



    def setH(self, H):
        self.H_ = H

    def setVarians(self, varians):
        self.x_Var_ = varians

    def getNewSensorValue(self, delta_t, measurement):
        calc_Q_(delta_t)
        predict(delta_t)
        correct(measurement)

    def get_x(self):
        return self.x_.copy()

    def calc_Q_(self, delta_t):
        self.Q_ = self.G_(delta_t)@np.transpose(G_(deta_t))*self.x_Var_

    def predict(self, delta_t):
        self.x_priori_ = self.A_(delta_t)@self.x_ # + w_(k-1)  // dette er produkt stoy
        self.P_priori_ = self.A_(delta_t)@self.P_@np.transpose(self.A_(delta_t))+self.Q_

    def correct(self, measurement):
        tmpHP = self.P_priori@np.transpose(self.H_)
        self.K_ = tmpHP*(self.H_@tmpHP+self.hvite_var)

        self.x_=self.x_priori_+self.K_*(measurement-self.H_@self.x_priori_)

        self.P_=(1-self.K_@self.H_)@self.P_priori


