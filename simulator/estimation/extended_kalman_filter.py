"""
Implementation of Extended Kalman Filter algorithm
References:

"""


import numpy as np
from numpy.linalg import inv
import collections
from .kalman_filter import KalmanFilter


class ExtendedKalmanFilter(KalmanFilter):
    """

    Implements the Extended Kalman filter algorithm. This class inherits from KalmanFilter class
    and overrides the predict and update functions. In terms of implementation, the following differences
    are noted

    Prediction step
    ---------------

    1. The predicted state vector \check{x} is computed via an application defined function specified by the 'f' keyword
    2. The computation of predicted process covariance matrix \check{P} is augmented with the L matrix as follows
            P_{k}^{-} = A_{k-1}P_{k-1}A_{k-1}^T + L_{k-1}Q_{k-1}L_{k-1}^T
    The linear case does not involve L


    Update/correct step
    ---------------

    1. The computation of the Kalman gain matrix K, is augmented with an M matrix as follows

            K_k = P_{k}^{-} x H_{k}^T x (H_k x P_k{-} x H_{k}^T + M_k x R_k x M_k^T)^{-1}
        The application should set this matrix using the M keyword

    2. The computation of the innovation uses an application defined function specified by the 'h' keyword.


    """

    def __init__(self):
        KalmanFilter.__init__(self)
        self['L'] = None
        self['M'] = None
        self['f'] = None
        self['h'] = None

    def predict(self, u):

        """
        Predicts the state vector x and the process covariance matrix P using
        the given input control u accroding to the following rules


        X_{k}^{-} = f(A_{k-1}, X_{k-1}, B_k,  U_k)
        P_{k}^{-} = A_{k-1}P_{k-1}A_{k-1}^T + L_{k-1}Q_{k-1}L_{k-1}^T

        """

        self._assert_prediction_matrices()

        if not self['f']:
            raise ValueError("Dynamics function f has not been specified. This should be a callable of the form f(A, x, B, u) ")

        if not self['L'].any():
            raise ValueError("Matrix L has not been pecified.")


        self['x'] = self['f'](self['A'], self['x'], self['B'], u)
        self['P'] = np.dot(self['A'], np.dot(self['P'], self['A'].T)) + np.dot(self['L'], np.dot(self['Q'], self['L'].T))

    def update(self, measurement):
        """
        Updates the gain matrix K, the  state vector x and covariance matrix P using the given measurement y_k
        according to the following update rules

        K_k = P_{k}^{-} x H_{k}^T x (H_k x P_k{-} x H_{k}^T + M_k x R_k x M_k^T)^{-1}
        \hat{X}_k = X_{k}^{-} + K_k(y_k - h( X_{k}^{-}, 0)
        \hat{P}_k = (I - K_kH_k)P_{k}^{-}

        """

        self._assert_update_matrices()

        if not self['h']:
            raise ValueError("Output mapping function h has not been specified. This should be a callable of the form h(x, v) ")

        if isinstance(self['M'], collections.Sequence):
            S = np.dot(self['H'], np.dot(self['P'], self['H'].T)) + np.dot(self['M'], np.dot(self['R'], self['M'].T))
        else:
            S = np.dot(self['H'], np.dot(self['P'], self['H'].T)) + np.dot(self['M'], np.dot(self['R'], self['M']))

        if isinstance(S, collections.Sequence):
            S_inv = inv(S)
        else:
            # assume it is a scalar
            S_inv = np.reciprocal(S)

        self['K'] = np.dot(np.dot(self['P'], self['H'].T), S_inv)

        innovation = measurement - self['h'](self['x'], 0.0)
        self['x'] = self['x'] + np.dot(self['K'], innovation)
        I = np.identity(self['K'].shape[0])
        self['P'] = np.dot(I - np.dot(self['K'], self['H']), self['P'])





