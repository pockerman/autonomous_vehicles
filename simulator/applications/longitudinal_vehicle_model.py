"""

See the application_notebooks/longitudinal_vehicle_model.ipynb
notebook for documentation of the driver

"""

import math
import numpy as np

from vehicle.vehicle_base import VehicleBase
from dynamics.dynamics_base import DynamicsBase
from systems.system_state import SysState
from ode_integrators.forward_euler import ODEScalarFWDEuler
from physics.physics import Physics
from plot.two_d_plotter import TwoDPlotter


class EngineDynamics(DynamicsBase):

    def __init__(self, properties, sample_time, init_condition):
        DynamicsBase.__init__(self)
        self.__properties = properties
        self.set_integrator("w_e", integrator=ODEScalarFWDEuler(step_size=sample_time, init_condition=init_condition))
        self.__state = 0.0

    def __compute_t_e(self, **kwargs):
        # get the previous velocity
        w_e_old = self.get_old_state("w_e", 0)
        throttle = kwargs['throttle']
        return throttle * (self.__properties['a_0'] + self.__properties['a_1'] * w_e_old + self.__properties['a_2'] * w_e_old ** 2)

    def __compute_rhs(self, **kwargs):
        T_e = self.__compute_t_e(**kwargs)
        GR = kwargs['GR']
        r_eff = kwargs['r_e']
        F_load = kwargs['F_load']
        return (T_e - GR * r_eff * F_load) / self.__properties['J_e']

    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, value):
        self.__state = value

    def execute(self, **kwargs):
        """
        performs one time-step of the enigne dynamics equation
        """
        kwargs['f'] = self.__compute_rhs
        self.__state = self.get_integrator("w_e").execute(**kwargs)


class VehicleDynamics(DynamicsBase):

    def __init__(self, sample_time, init_condition):
        DynamicsBase.__init__(self)
        self.__state = SysState(n_entries=3)
        self.__state.add_state("x", 0)
        self.__state.add_state("v", 1)
        self.__state.add_state("w_e", 2)

        # integrator for the velocity
        self.set_integrator("velocity",
                            integrator=ODEScalarFWDEuler(init_condition=init_condition, step_size=sample_time))

    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, value):
        self.__state = value

    def execute(self, **kwargs):
        """
        performs one time-step of the velocity dynamics equation
        """
        v_new = self.get_integrator("velocity").execute(**kwargs)
        self.__state.set_state_value_by_name(name="v", value=v_new)

        x = self.__state.get_state_value_by_name('x')
        x += v_new*self.get_integrator("velocity").step_size
        self.__state.set_state_value_by_name(name='x', value=x)


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

    def __get_f_load(self, **kwargs):

        v = self.get_old_state(name="velocity", idx=0)
        F_aero = self.get_property("c_a") * v ** 2
        R_x = self.get_property("c_r1") * v
        F_g = self.get_property("m") * Physics.gravity_constant() * math.sin(kwargs['alpha'])
        return F_aero + R_x + F_g

    def __get_f_x(self, **kwargs):
        w_e = self.get_property("propulsion").get_old_state(name='w_e', idx=0)
        w_w = self.get_property("GR") * w_e
        v = self.get_old_state(name="velocity", idx=0)
        s = (w_w * self.get_property("r_e") - v) / v

        if math.fabs(s) < 1:
            return self.get_property("c") * s

        return self.get_property("F_max")

    def __compute_rhs(self, **kwargs):
        return (self.__get_f_x(**kwargs) - self.__get_f_load(**kwargs))/self.get_property('m')

    def execute(self, **kwargs):
        F_load = self.__get_f_load(**kwargs)
        kwargs['F_load'] = F_load
        kwargs['GR'] = self.get_property("GR")
        kwargs['r_e'] = self.get_property("r_e")
        self.get_property("propulsion").execute(**kwargs)
        kwargs['f'] = self.__compute_rhs
        self.__dynamics.execute(**kwargs)


def constant_throttle():

    ### SIMULATION DRIVER

    sample_time = 0.01
    time_end = 100

    t_data = np.arange(0, time_end, sample_time)
    v_data = np.zeros_like(t_data)

    engine_properties = {"a_0": 400,
                         "a_1": 0.1,
                         "a_2": -0.0002,
                         "J_e": 10}

    init_engine_speed = 100.0  # rad/s
    propulsion = EngineDynamics(properties=engine_properties,
                                sample_time=sample_time,
                                init_condition=init_engine_speed)

    # Gear ratio, effective radius, mass + inertia
    vehicle_properties = {"GR": 0.35,
                          "r_e": 0.3,
                          "J_e": 10,
                          "m": 2000,
                          "c_a": 1.36,
                          "c_r1": 0.01,
                          "c": 10000,
                          "F_max": 10000,
                          "propulsion": propulsion}

    init_velocity = 5.0  # m/s
    model = Vehicle(properties=vehicle_properties,
                    sample_time=sample_time,
                    init_condition=init_velocity)

    # throttle percentage between 0 and 1
    throttle = 0.2

    # incline angle (in radians)
    alpha = 0

    kwargs = {"throttle": throttle, "alpha": alpha}

    # the simulation time
    time = 0.0
    for i in range(t_data.shape[0]):
        print("At time: %f" % time)

        model.execute(**kwargs)
        v_data[i] = model.state.get_state_value_by_name("v")
        print("Vehicle velcoity %f" % model.state.get_state_value_by_name("v"))

        time += sample_time

    plotter = TwoDPlotter(xlabel="Time in secs", ylabel="Velocity")
    plotter.plot(x=t_data, y=v_data)
    plotter.show_plots(show_grid=True)


def calculate_alpha(x):

    if x < 60.0:
        return math.atan(3.0/60.0)

    if x >= 60.0 and x <= 150.0:
        return math.atan(12.0/90.0)

    return 0.0


def calculate_throttle(time):

    if time <= 5.0:

        beta = 0.2
        alpha = 0.3/5.0
        return alpha*time + beta

    if time <= 15.0:
        return 0.5

    return 0.0

def variable_throttle():

        ### SIMULATION DRIVER

        sample_time = 0.01
        time_end = 100

        t_data = np.arange(0, time_end, sample_time)
        v_data = np.zeros_like(t_data)
        throttle_data = np.zeros_like(t_data)
        alpha_data = np.zeros_like(t_data)

        engine_properties = {"a_0": 400,
                             "a_1": 0.1,
                             "a_2": -0.0002,
                             "J_e": 10}

        init_engine_speed = 100.0  # rad/s
        propulsion = EngineDynamics(properties=engine_properties,
                                    sample_time=sample_time,
                                    init_condition=init_engine_speed)

        # Gear ratio, effective radius, mass + inertia
        vehicle_properties = {"GR": 0.35,
                              "r_e": 0.3,
                              "J_e": 10,
                              "m": 2000,
                              "c_a": 1.36,
                              "c_r1": 0.01,
                              "c": 10000,
                              "F_max": 10000,
                              "propulsion": propulsion}

        init_velocity = 5.0  # m/s
        model = Vehicle(properties=vehicle_properties,
                        sample_time=sample_time,
                        init_condition=init_velocity)

        kwargs = dict()

        # the simulation time
        time = 0.0
        for i in range(t_data.shape[0]):
            print("At time: %f" % time)

            pos = model.state.get_state_value_by_name("x")

            # incline angle (in radians)
            alpha = calculate_alpha(pos)
            alpha_data[i] = alpha

            # throttle percentage between 0 and 1
            throttle = calculate_throttle(time)
            throttle_data[i] = throttle

            print("\talpha %f throttle %f"%(alpha, throttle))

            kwargs['alpha'] = alpha
            kwargs["throttle"] = throttle

            model.execute(**kwargs)
            v_data[i] = model.state.get_state_value_by_name("v")
            print("Vehicle velcoity %f" % model.state.get_state_value_by_name("v"))

            time += sample_time


        plotter = TwoDPlotter(xlabel="Time in secs", ylabel="Throttle/alpha")
        plotter.plot(x=t_data, y=throttle_data, label="Throttle")
        plotter.plot(x=t_data, y=alpha_data, label="alpha")
        #plotter.plot(x=t_data, y=v_data)
        plotter.show_plots(show_legend=True, show_grid=True)

if __name__ == "__main__":

    constant_throttle()
    variable_throttle()