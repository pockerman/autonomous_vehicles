import numpy as np
from estimation.extended_kalman_filter import ExtendedKalmanFilter


def f(A, x, B, u ):
    return np.dot(A, x) + np.dot(B, u)

def h(x, v):
    D = 40.0
    S = 20.0

    # convert the result to degrees 1 rad is 57.29 degrees
    return (np.arctan(S/(D-x)) + v)*57.29

if __name__ == '__main__':

    ekf = ExtendedKalmanFilter()

    dt = 0.5
    var_q = 0.1
    D = 40.0
    S = 20.0

    A = np.array([[1.0, dt], [0.0, 1.0]])
    B = np.array([0.0, dt])
    H = np.array([S/((D-2.5)**2 + S**2), 0.0])
    Q = var_q*np.identity(2)
    R = 0.01
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
    print("Expected predicted state: [2.5, 4.0]")
    print("Predicted state: ", ekf['x'])
    print("==================================================================")

    # expect P = [[0.36, 0.5], [0.5, 1.1]]
    print("Expected predicted covariance P: [[0.36, 0.5], [0.5, 1.1]]" )
    print("Predicted P: ", ekf['P'])
    print("==================================================================")

    # update step
    ekf.update(measurement = 30.0)

    # expect K = [0.88, 1.22]
    print("Expected gain matrix K: [0.39, 0.55]")
    print("Gain matrix: ", ekf['K'])
    print("==================================================================")

    # expect x = [3.24, 5.05]
    print("Expected update state: [3.24, 5.05]")
    print("State: ", ekf['x'])
    print("==================================================================")

    # expect P = [[0.36, 0.5], [0.5, 1.1]]
    print("Expected update covariance P: [[0.36, 0.5], [0.5, 1.1]]")
    print("Covariance: ", ekf['P'])
    print("==================================================================")

