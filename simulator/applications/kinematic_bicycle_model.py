"""

See the application_notebooks/kinematic_bicycle_model.ipynb
notebook for documentation of the driver

"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from vehicle.vehicle_base import VehicleBase
from dynamics.dynamics_base import DynamicsBase
from systems.system_state import SysState
from ode_integrators.forward_euler import ODEScalarFWDEuler
from physics.physics import Physics
from plot.two_d_plotter import TwoDPlotter

class VehicleDynamics(DynamicsBase):

    def __init__(self, sample_time, init_condition):
        DynamicsBase.__init__(self)
        self.__state = SysState(n_entries=4)
        self.__state.add_state("x", 0)
        self.__state.add_state("y", 1)
        self.__state.add_state("theta", 2)
        self.__state.add_state("delta", 3)

        # integrator for the velocity
        self.set_integrator("x", integrator=ODEScalarFWDEuler(init_condition=init_condition['x'], step_size=sample_time, rhs_func=self.__integrate_x))
        self.set_integrator("y", integrator=ODEScalarFWDEuler(init_condition=init_condition['y'], step_size=sample_time, rhs_func=self.__integrate_y))
        self.set_integrator("theta", integrator=ODEScalarFWDEuler(init_condition=init_condition['theta'], step_size=sample_time, rhs_func=self.__integrate_theta))
        self.set_integrator("delta", integrator=ODEScalarFWDEuler(init_condition=init_condition['delta'], step_size=sample_time, rhs_func=self.__integrate_delta))

    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, value):
        self.__state = value

    def __integrate_x(self, **kwargs):
        """
        The rhs for integrating the x-component
        """
        theta_old = self.get_old_state("theta", 0)
        beta = kwargs['beta']
        v = kwargs['v']
        return v*math.cos((theta_old + beta))

    def __integrate_y(self, **kwargs):
        """
        The rhs for integrating the y-component
        """
        theta_old = self.get_old_state("theta", 0)
        beta = kwargs['beta']
        v = kwargs['v']
        return v*math.sin((theta_old + beta))

    def __integrate_theta(self, **kwargs):
        """
        The rhs for integrating the theta-component
        """
        beta = kwargs['beta']
        v = kwargs['v']
        L = kwargs['L']
        delta_old = self.get_old_state("delta", 0)
        return (v*math.cos(beta)*math.tan(delta_old))/L

    def __integrate_delta(self, **kwargs):
        """
        The rhs for integrating the delta-component
        """
        return kwargs['omega']

    def execute(self, **kwargs):
        """
        performs one time-step of the velocity dynamics equation
        """
        x_c = self.get_integrator("x").execute(**kwargs)
        self.__state.set_state_value_by_name(name='x', value=x_c)
        y_c = self.get_integrator("y").execute(**kwargs)
        self.__state.set_state_value_by_name(name='y', value=y_c)
        theta = self.get_integrator("theta").execute(**kwargs)
        self.__state.set_state_value_by_name(name='theta', value=theta)

        if 'integrate_delta' in kwargs.keys():

            delta = self.get_integrator("delta").execute(**kwargs)
            self.__state.set_state_value_by_name(name='delta', value=delta)
        else:
            delta = self.get_integrator("delta").get_history(0)
            self.__state.set_state_value_by_name(name='delta', value=delta)




class Vehicle(VehicleBase):
    def __init__(self, properties, sample_time, init_condition):
        VehicleBase.__init__(self, properties)
        self.__dynamics = VehicleDynamics(sample_time=sample_time, init_condition=init_condition)

    @property
    def state(self):
        return self.__dynamics.state

    @state.setter
    def state(self, value):
        self.__dynamics.state = value

    def get_old_state(self, name, idx):
        return self.__dynamics.get_old_state(name=name, idx=idx)

    def set_old_state(self, name, idx, value):
        self.__dynamics.set_old_state(name=name, idx=idx, value=value)

    def execute(self, **kwargs):
        """
        executes one time step of the vehicle dynamics
        """
        kwargs['L'] = self.get_property("L")
        self.__dynamics.execute(**kwargs)


def do_test(t_data, sample_time, use_slip, integrate_delta, angular_velocity_data=None, velocity_data=None):

    #sample_time = 0.01
    #time_end = 20

    #t_data = np.arange(0, time_end, sample_time)
    x_data = np.zeros_like(t_data)
    y_data = np.zeros_like(t_data)

    vehicle_properties = {"L": 2, "lr": 1.2, "w_max": 1.22}
    delta_init  = np.arctan(2 / 10)

    if angular_velocity_data is not None:
        delta_init = 0.0

    init_condition = {"x": 0.0, "y": 0.0, "theta": 0.0, 'delta': delta_init}
    model = Vehicle(properties=vehicle_properties, sample_time=sample_time, init_condition=init_condition)

    kwargs = dict()
    kwargs['integrate_delta'] = integrate_delta
    time = 0.0

    for i in range(t_data.shape[0]):
        print("At time: %f" % time)

        if use_slip:
            delta = model.state.get_state_value_by_name("delta") #get_old_state("delta", 0)
            lr = model.get_property("lr")
            L = model.get_property("L")
            kwargs['beta'] = (lr * math.tan(delta)) / L
        else:
            kwargs['beta'] = 0.0

        if velocity_data is not None:
            kwargs['v'] = velocity_data[i]
        else:
            kwargs['v'] = np.pi

        kwargs['omega'] = 0.0

        if integrate_delta == True:

            if angular_velocity_data is not None:
                kwargs['omega'] = angular_velocity_data[i]
            else:
                delta = model.state.get_state_value_by_name("delta")
                if delta < np.arctan(2/10):
                    kwargs['omega'] = model.get_property("w_max")

        model.execute(**kwargs)
        x = model.state.get_state_value_by_name("x")
        y = model.state.get_state_value_by_name("y")

        x_data[i] = x
        y_data[i] = y
        time += sample_time

    return x_data, y_data

def test_1():

    """
    Assumes that the steering angle is set directly.
    The vehicle then should execute a circular path
    """
    sample_time = 0.01
    time_end = 20
    t_data = np.arange(0, time_end, sample_time)

    x_data_slip, y_data_slip = do_test(t_data=t_data, sample_time=sample_time, use_slip=True, integrate_delta=False)
    x_data_no_slip, y_data_no_slip = do_test(t_data=t_data, sample_time=sample_time, use_slip=False, integrate_delta=False)
    plotter = TwoDPlotter(xlabel="X-coordinate", ylabel="Y-coordinate")

    plotter.plot(x=x_data_slip, y=y_data_slip, label="vehicle position with slip")
    plotter.plot(x=x_data_no_slip, y=y_data_no_slip, label="vehicle position no slip")
    plotter.show_plots(show_grid=True, show_legend=True)


def test_2():
    """
    test by setting the angular velocity instead of the steering angle
    """

    sample_time = 0.01
    time_end = 20
    t_data = np.arange(0, time_end, sample_time)

    x_data, y_data = do_test(t_data=t_data, sample_time=sample_time, use_slip=False, integrate_delta=True)

    plotter = TwoDPlotter(xlabel="X-coordinate", ylabel="Y-coordinate")
    plotter.plot(x=x_data, y=y_data, label="vehicle position with angular velocity")
    plotter.show_plots(show_grid=True, show_legend=True)

def test_square_path():

    sample_time = 0.01
    time_end = 60

    t_data = np.arange(0, time_end, sample_time)
    w_data = np.zeros_like(t_data)

    # ==================================
    #  Square Path: set w at corners only
    # ==================================
    w_data[670:670 + 100] = 0.753
    w_data[670 + 100:670 + 100 * 2] = -0.753
    w_data[2210:2210 + 100] = 0.753
    w_data[2210 + 100:2210 + 100 * 2] = -0.753
    w_data[3670:3670 + 100] =  0.753
    w_data[3670 + 100:3670 + 100 * 2] = -0.753
    w_data[5220:5220 + 100] = 0.753
    w_data[5220 + 100:5220 + 100 * 2] = -0.753

    # maintain velocity at 4 m/s
    v_data = np.zeros_like(t_data)
    v_data[:] = 4

    x_data, y_data = do_test(t_data, sample_time=sample_time, use_slip=False, integrate_delta=True, angular_velocity_data=w_data, velocity_data=v_data)

    plotter = TwoDPlotter(xlabel="X-coordinate", ylabel="Y-coordinate")
    plotter.plot(x=x_data, y=y_data, label="vehicle position with angular velocity")
    plotter.show_plots(show_grid=True, show_legend=True)

def test_square_wave_path():

    sample_time = 0.01
    time_end = 60

    t_data = np.arange(0, time_end, sample_time)
    w_data = np.zeros_like(t_data)

    w_data[:] = 0
    w_data[0:100] = 1
    w_data[100:300] = -1
    w_data[300:500] = 1
    w_data[500:5700] = np.tile(w_data[100:500], 13)
    w_data[5700:] = -1

    # maintain velocity at 4 m/s
    v_data = np.zeros_like(t_data)
    v_data[:] = 4

    x_data, y_data = do_test(t_data, sample_time=sample_time, use_slip=False, integrate_delta=True,
                             angular_velocity_data=w_data, velocity_data=v_data)

    plotter = TwoDPlotter(xlabel="X-coordinate", ylabel="Y-coordinate")
    plotter.plot(x=x_data, y=y_data, label="vehicle position with angular velocity")
    plotter.show_plots(show_grid=True, show_legend=True)

def test_figure_8_trajectory():


    sample_time = 0.01
    time_end = 30

    t_data = np.arange(0, time_end, sample_time)
    v_data = np.zeros_like(t_data)
    w_data = np.zeros_like(t_data)

    # maintain constnat velocity
    v_data[:] = (2.0*np.pi*8.0)/time_end
    w_data[:] = 1.22

    x_data, y_data = do_test(t_data, sample_time=sample_time, use_slip=False, integrate_delta=True,
                             angular_velocity_data=w_data, velocity_data=v_data)

    plotter = TwoDPlotter(xlabel="X-coordinate", ylabel="Y-coordinate")
    plotter.plot(x=x_data, y=y_data, label="vehicle position with angular velocity")
    plotter.show_plots(show_grid=True, show_legend=True)

if __name__ == '__main__':
    # test_1()
    #test_2()
    #test_square_path()
    #test_square_wave_path()
    test_figure_8_trajectory()














