import numpy as np
from estimation.extended_kalman_filter import ExtendedKalmanFilter

dt = 0.5
def f(A, x, B, u ):
    return np.dot(A, x) + np.dot(B, u)

def h(x, v):
    D = 40.0
    S = 20.0
    return np.arctan(S/(D-x)) + v

if __name__ == '__main__':

    ekf = ExtendedKalmanFilter()

    var_q = 0.1
    D = 40.0
    S = 20.0

    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([0.0, dt])
    H = np.array([S/((D-2.5)**2), 0.0])
    Q = var_q*np.identity(2)
    R = 0.05
    x = np.array([0.0, 5.0])
    L = np.identity(2)

    # initial estimate for the state covariance
    P = np.array([[0.01, 0.0], [0.0, 1.0]])

    # prepare the filter
    ekf['A'] = A
    ekf['B'] = B
    ekf['Q'] = Q
    ekf['R'] = R
    ekf['H'] = H
    ekf['x'] = x
    ekf['P'] = P
    ekf['f'] = f
    ekf['h'] = h
    ekf['L'] = L
    ekf['M'] = 1.0

    ekf.predict(u = -2.0)

    # expect x = [2.5, 4.0]
    print("Predicted state: ", ekf['x'])

    # expect P = [[0.36, 0.5], [0.5, 1.1]]
    print("Predicted P: ", ekf['P'])

    ekf.update(measurement = 30.0)

    # expect K = [0.88, 1.22]
    print("Gain matrix: ", ekf['K'])

    # expect x = [2.24, 3.63]
    print("State: ", ekf['x'])

    # expect P = [[0.04 0.06], [0.06 0.49]]
    print("Covariance: ", ekf['P'])