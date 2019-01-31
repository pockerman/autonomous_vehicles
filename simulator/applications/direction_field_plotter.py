import numpy as np
import matplotlib.pyplot as plt

from plot.direction_field_plotter import DirFieldPlotter

def f(Y, t):
    y1, y2 = Y
    return [y2, -np.sin(y1)]


field_plotter = DirFieldPlotter(xlabel='t', ylabel='y', plot_title='Direction Field')

t = np.linspace(-2.0, 8.0, 20)
y = np.linspace(-2.0, 2.0, 20)

field_plotter.plot(f=f, x_interval = t, y_interval=y, color='r')

kwrags= {'axis_limits' : [[2, 8], [-4, 4]]}
field_plotter.show_plots(**kwrags)
