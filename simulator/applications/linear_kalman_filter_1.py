import numpy as np
from estimation.kalman_filter import KalmanFilter


def simulate():
    pass

if __name__ == '__main__':

    kf = KalmanFilter()
    dt = 0.5
    var_q = 0.1

    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([0.0, dt])
    H = np.array([1.0, 0.0])
    Q = var_q*np.identity(2)
    R = 0.05
    x = np.array([0.0, 5.0])
    P = np.array([[0.01, 0.0], [0.0, 1.0]])

    # prepare the filter
    kf['A'] = A
    kf['B'] = B
    kf['Q'] = Q
    kf['R'] = R
    kf['H'] = H
    kf['x'] = x
    kf['P'] = P

    kf.predict(u = -2.0)

    # expect x = [2.5, 4.0]
    print("Predicted state: ", kf['x'])

    # expect P = [[0.36, 0.5], [0.5, 1.1]]
    print("Predicted P: ", kf['P'])

    kf.update(measurement = 2.2)

    # expect K = [0.88, 1.22]
    print("Gain matrix: ", kf['K'])

    # expect x = [2.24, 3.63]
    print("State: ", kf['x'])

    # expect P = [[0.04 0.06], [0.06 0.49]]
    print("Covariance: ", kf['P'])