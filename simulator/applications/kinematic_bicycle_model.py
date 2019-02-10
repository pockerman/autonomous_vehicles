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
        #delta = self.get_integrator("delta").execute(**kwargs)




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



def test_1():

    sample_time = 0.01
    time_end = 20

    t_data = np.arange(0, time_end, sample_time)
    x_data = np.zeros_like(t_data)
    y_data = np.zeros_like(t_data)
    v_data = np.zeros_like(t_data)

    vehicle_properties = {"L": 2, "lr": 1.2, "w_max":1.22}

    init_condition = {"x": 0.0, "y": 0.0, "theta": 0.0, 'delta': 0.0}
    model = Vehicle(properties=vehicle_properties, sample_time=sample_time, init_condition=init_condition)

    # set delta directly
    model.delta = np.arctan(2 / 10)

    kwargs = dict()

    # Use this to control whether sideslip is accounted for
    use_slip = True
    time = 0.0
    for i in range(t_data.shape[0]):
        print("At time: %f" % time)

        #x_data[i] = model.xc
        #y_data[i] = model.yc


        #model.step(np.pi, 0)

        if use_slip:
            delta = model.state.get_state_value_by_name("delta")
            lr = model.get_property("lr")
            L = model.get_property("L")
            kwargs['beta'] = (lr*math.tan(delta))/L
        else:
            kwargs['beta'] = 0.0

        kwargs['v'] = np.pi
        kwargs['omega'] = 0.0

        model.execute(**kwargs)


        # model.beta = 0
        time += sample_time
    plotter = TwoDPlotter(xlabel="Time in secs", ylabel="Velocity")
    plotter.plot(x=t_data, y=v_data)
    plotter.show_plots(show_grid=True)

    #plt.axis('equal')
    #plt.plot(x_data, y_data)
    #plt.show()


if __name__ == '__main__':
    test_1()














