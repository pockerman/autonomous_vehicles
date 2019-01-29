"""
Plots the direction field for a first order ODE
"""
import matplotlib.pyplot as plt
from scipy import *
from scipy import integrate
from scipy.integrate import ode
import numpy as np


class DirFieldPlotter(object):

    def __init__(self):
        self.__y_vals =[]



    def plot(self, vf, integrator_type, method, dt, time_interval, ic, color):
        r = ode(vf).set_integrator(integrator_type, method=method, max_step=dt)

        t_start = time_interval[0]
        t_end = time_interval[1]
        r.set_initial_value(ic, t_start).set_f_params()

        while r.successful() and r.t + dt < t_end:
            r.integrate(r.t + dt)
            self.__y_vals.append(r.y)

        S = np.array(np.real(self.__y_vals))

        fig = plt.figure(num=1)
        ax = fig.add_subplot(111)
        ax.plot(S[:, 0], S[:, 1], color=color, lw=1.25)

        X, Y = np.meshgrid(np.linspace(-5, 5, 20), np.linspace(-10, 10, 20))
        U = 1
        V = X ** 2 - X - 2
        # Normalize arrows
        N = np.sqrt(U ** 2 + V ** 2)
        U2, V2 = U / N, V / N
        ax.quiver(X, Y, U2, V2)

        plt.xlim([-5, 5])
        plt.ylim([-10, 10])
        plt.xlabel(r"$x$")
        plt.ylabel(r"$y$")
        plt.show()


def vf(t,x):
  dx=np.zeros(2)
  dx[0]=1
  dx[1]=x[0]**2-x[0]-2
  return dx

if __name__ == '__main__':

    dt = 0.01
    time_interval = (0.0, 10.0)
    ic = (0.0, -10.0)
    dir_field_plotter = DirFieldPlotter()
    dir_field_plotter.plot(vf=vf, integrator_type='vode', method='bdf', dt=dt, time_interval=time_interval, ic=ic, color='r')
