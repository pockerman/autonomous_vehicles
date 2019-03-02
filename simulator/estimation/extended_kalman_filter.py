import numpy as np
from numpy.linalg import inv
import collections
from .kalman_filter import KalmanFilter

class ExtendedKalmanFilter(KalmanFilter):

    def __init__(self):
        KalmanFilter.__init__(self)
        self['L'] = None
        self['M'] = None
        self['f'] = None
        self['h'] = None


    def predict(self, u):

        """
        Predicts the state and the process covariance matrix using
        X_{k}^{-} = f(A_{k-1}, X_{k-1}, B_k,  U_k)
        P_{k}^{-} = A_{k-1}P_{k-1}A_{k-1}^T + L_{k-1}Q_{k-1}L_{k-1}^T
        :param u: The control input
        """
        self['x'] = self['f'](self['A'], self['x'], self['B'], u)
        self['P'] = np.dot(self['A'], np.dot(self['P'], self['A'].T)) + np.dot(self['L'], np.dot(self['Q'], self['L'].T))

    def update(self, measurement):
        """
        Updates the state and covariance matrices using the measurement

        K_k = P_{k}^{-}H_{k}^T(H_kP_k{-}H_{k}^T + M_kR_kM_k^T)^{-1}
        \hat{X}_k = X_{k}^{-} + K_k(y_k - H_kX_{k}^{-})
        \hat{P}_k = (I - K_kH_k)P_{k}^{-}
        """

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