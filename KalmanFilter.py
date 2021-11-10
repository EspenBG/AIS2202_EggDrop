##
# Class for a generic kalman filter
##

import numpy as np

class KalmanFilter:
    """"
    Class for a generic kalman filter
    """
    import numpy as np

    def __init__(self, A, G,  set_variance, white_variance=0, start_value=np.array([0, 0, 0]), H=None):
        """
        :param A: Give the function for making the A matrix
        :param G: Give the function for making the G matrix
        :param set_variance: Set the variance for the filter, this is the allowed variance of the system
        :param white_variance: The R matrix for the filter
        :param start_value: The start value for the filter
        :param H: Set the H matrix (measurement selection)
        """
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

    # setters
    def setR(self, R):
        self.R_ = R

    def setH(self, H):
        self.H_ = H

    def set_white_variance(self, variance):
        self.R_ = variance

    def getNewSensorValue(self, delta_t, measurement):
        self.calc_Q_(delta_t)
        self.predict(delta_t)
        self.correct(measurement)

    # Getters
    def get_x(self):
        return self.x_.copy()

    def calc_Q_(self, delta_t):
        self.Q_ = self.G_(delta_t)@np.transpose(self.G_(delta_t))*self.x_Var_

    # Methods
    def predict(self, delta_t):
        """
        The predict step for the calculation of the kalman filter
        :param delta_t: Give the timestep to use
        """
        # Calculate the A matrix
        A = self.A_(delta_t)
        # set the new priori estimates
        self.x_priori_ = A@self.x_  # + w_(k-1)  // This is the process noice
        # Calculate the Q matrix
        self.calc_Q_(delta_t)
        A_trans = A.T
        # Calculate the new P priori matrix
        self.P_priori_ = A@self.P_@A_trans+self.Q_

    def correct(self, measurement):
        """
        Calculate the estimate for the
        :param measurement:
        """
        H_trans = self.H_.T
        tmpPH = self.P_priori_@H_trans
        # Set the new K matrix
        self.K_ = tmpPH*np.linalg.inv(self.H_@tmpPH+self.R_)
        # Se the new estimate values
        self.x_=self.x_priori_+self.K_*(measurement-self.H_@self.x_priori_)
        # Set the new P matrix for next iteration
        self.P_=(self.I-self.K_@self.H_)@self.P_priori_


