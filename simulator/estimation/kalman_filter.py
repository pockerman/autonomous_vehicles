"""
Linear Kalman Filter implementation. See
An Introduction to the Kalman Filter, TR 95-041
by
Greg Welch1 and Gary Bishop2

"""

import numpy as np
from numpy.linalg import inv
import collections

class KalmanFilter(object):
    """
    Implements the Linear Kalman Filter algorithm. The following algorithm is
    implemented
    x_k = A_kx_{k-1} + B_k u_k + w_{k-1}
    y_k = H_k x_k + v_k

    where w_k and v_k  represent process and measurement noise respectively. They are assumed
    independent and normally distributed:

    p(w) ~ N(0,Q)
    p(v) ~ N(0,R)

    The following matrices dimensions are assumed:

    A n x n
    B n x l
    u l x 1
    H m x n
    y m x 1
    x n x 1

    The Kalman Filter algorithm is a predictor-corrector process:

    Prediction step
    ---------------

    X_{k}^{-} = A_{k-1}X_{k-1} + B_k U_k
    P_{k}^{-} = A_{k-1}P_{k-1}A_{k-1}^T + Q_{k-1} // Process covariance prediction

    Update/correct step
    ---------------

    K_k = P_{k}^{-}H_{k}^T(H_kP_k{-}H_{k}^T + R_k)^{-1}// Gain matrix it says how much the predictions should be corrected
    \hat{X}_k = X_{k}^{-} + K_k(y_k - H_kX_{k}^{-})
    \hat{P}_k = (I - K_kH_k)P_{k}^{-}

    where

    K n x m


    """

    def __init__(self):
        self.__data = dict()
        self.__data['Q'] = None
        self.__data['R'] = None
        self.__data['x'] = None
        self.__data['P'] = None
        self.__data['A'] = None
        self.__data['B'] = None
        self.__data['K'] = None
        self.__data['H'] = None


    def __setitem__(self, key, value):
        self.__data[key] = value

    def __getitem__(self, item):
        return self.__data[item]

    def _assert_prediction_matrices(self):

        if not self['x'].any():
            raise ValueError("State vector x has not been pecified.")

        if not self['A'].any():
            raise ValueError("Matrix A has not been pecified.")

        if not self['B'].any():
            raise ValueError("Matrix B has not been pecified.")

        if not self['Q'].any():
            raise ValueError("Matrix Q has not been pecified.")

    def _assert_update_matrices(self):

        if not self['H'].any():
            raise ValueError("Matrix H has not been pecified.")

        if not self['P'].any():
            raise ValueError("Matrix P has not been pecified.")

    def predict(self, u):

        """
        Predicts the state and the process covariance matrix using
        X_{k}^{-} = A_{k-1}X_{k-1} + B_k U_k
        P_{k}^{-} = A_{k-1}P_{k-1}A_{k-1}^T + Q_{k-1}
        :param u: The control input
        """

        # make sure that all needed matrices are here
        self._assert_prediction_matrices()

        self['x'] = np.dot(self['A'], self['x']) + np.dot(self['B'], u)
        self['P'] = np.dot(self['A'], np.dot(self['P'], self['A'].T)) + self['Q']



    def update(self, measurement):
        """
        Updates the state and covariance matrices using the measurement

        K_k = P_{k}^{-}H_{k}^T(H_kP_k{-}H_{k}^T + R_k)^{-1}
        \hat{X}_k = X_{k}^{-} + K_k(y_k - H_kX_{k}^{-})
        \hat{P}_k = (I - K_kH_k)P_{k}^{-}
        """

        self._assert_update_matrices()

        S = np.dot(self['H'], np.dot(self['P'], self['H'].T)) + self['R']

        if isinstance(S, collections.Sequence):
            S_inv = inv(S)
        else:
            # assume it is a scalar
            S_inv = np.reciprocal(S)

        self['K'] = np.dot(np.dot(self['P'], self['H'].T), S_inv)

        # expect that self['x'] is a numpy.array and thus take the traspose to have
        # correct dimensions
        innovation = measurement - np.dot(self['H'], self['x'].T)
        self['x'] = self['x'] + np.dot(self['K'], innovation)

        I = np.identity(self['K'].shape[0])
        self['P'] = np.dot(I - np.dot(self['K'], self['H']), self['P'])

    def filter(self, u, measurement):
        self.predict(u)
        self.update(measurement=measurement)

    def iterate(self, u, measurement):
        self.filter(u=u, measurement=measurement)





