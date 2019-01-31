"""
Plots the direction field for a first order ODE
"""
import matplotlib.pyplot as plt
import numpy as np
from .two_d_plotter_base import TwoDPlotterBase

class DirFieldPlotter(TwoDPlotterBase):

    def __init__(self, **kwargs):
        TwoDPlotterBase.__init__(self, **kwargs)

    def plot(self, f, x_interval, y_interval,  color):

        Y1, Y2 = np.meshgrid(x_interval, y_interval)
        u, v = np.zeros(Y1.shape), np.zeros(Y2.shape)

        NI, NJ = Y1.shape
        t = 0
        for i in range(NI):
            for j in range(NJ):
                x = Y1[i, j]
                y = Y2[i, j]
                yprime = f([x, y], t)
                u[i, j] = yprime[0]
                v[i, j] = yprime[1]

        Q = plt.quiver(Y1, Y2, u, v, color=color)

