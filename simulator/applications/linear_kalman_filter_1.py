import numpy as np
from estimation.kalman_filter import KalmanFilter

if __name__ == '__main__':

    kf = KalmanFilter()
    dt = 0.5
    var_q = 0.1

    A = np.matrix([[1.0, dt], [0.0, 1.0]])
    B = np.matrix([0.0, dt])
    H = np.matrix([1.0, 0.0])
    Q = var_q*np.identity(2)
    R = 0.05
    x = np.array([0.0, 5.0])
    P = np.matrix([[0.01, 0.0], [0.0, 1.0]])

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
    print("Expected predicted state: [2.5, 4.0]")
    print("Predicted state: ", kf['x'])
    print("==================================================================")

    # expect P = [[0.36, 0.5], [0.5, 1.1]]
    print("Expected predicted covariance P: [[0.36, 0.5], [0.5, 1.1]]")
    print("Predicted P: ", kf['P'])
    print("==================================================================")

    kf.update(measurement = 2.2)

    # expect K = [0.88, 1.22]
    print("Expected gain matrix K: [0.88, 1.22]")
    print("Gain matrix: ", kf['K'])
    print("==================================================================")

    # expect x = [2.24, 3.63]
    print("Expected update state: [2.24, 3.63]")
    print("State: ", kf['x'])
    print("==================================================================")

    # expect P = [[0.04 0.06], [0.06 0.49]]
    print("Expected update covariance P: [[0.04, 0.06], [0.06, 0.49]]")
    print("Covariance: ", kf['P'])
    print("==================================================================")