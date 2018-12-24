import math

from dynamics.dynamics_base import DynamicsBase
from vehicle.vehicle_base import VehicleBase
from physics.physics import Physics
from control.pid_control import PIDControl
from ode_integrators.forward_euler import ODEFWDEuler
from plot.two_d_plotter import TwoDPlotter


class Dynamics(DynamicsBase):

    def __init__(self):
        DynamicsBase.__init__(self)
        self.__velocity = 0.0
        self.__vel_integrator = ODEFWDEuler()

    @property
    def state(self):
        return self.__velocity

    @state.setter
    def state(self, value):
        self.__velocity = value

    @property
    def old_state(self):
        return self.__vel_integrator.get_history(0)

    @old_state.setter
    def old_state(self, value):
        self.__vel_integrator.update_history(0, value)

    def compute_rhs(self, **kwargs):
        m = kwargs['m']
        a = kwargs['a']
        b = kwargs['b']
        u = kwargs['u']
        d = kwargs['d']
        yn = self.old_state

        return -(a/m)*yn + (b/m)*u - d

    def execute(self, **kwargs):
        kwargs['f'] = self.compute_rhs
        self.state = self.__vel_integrator.execute(**kwargs)


class Vehicle(VehicleBase):

    def __init__(self):
        VehicleBase.__init__(self)
        self.mass = 1000.0 # kg

        # the plant
        self.__dynamics = Dynamics()

    def execute(self, **kwargs):
        self.__dynamics.execute(**kwargs)

    @property
    def state(self):
        return self.__dynamics.state

    @state.setter
    def state(self, value):
        self.__dynamics.state = value

    @property
    def old_state(self):
        return self.__dynamics.old_state

    @old_state.setter
    def old_state(self, value):
        self.__dynamics.old_state = value


if __name__ == '__main__':

    print("Starting simulation....")

    Ki = 5.0
    Kp = 0.1
    Kd = None

    # the controller to use
    controller = PIDControl(Ki = Ki, Kp = Kp, Kd=Kd)

    # number of simulation steps
    n_steps = 30000

    # the time step
    dt = 0.001

    # reference velocity in m/s
    r = 25.0

    # road slope
    alpha = 5.0

    # Ns/m
    a = 600.0

    # kN/rad
    b = 10.0

    vehicle = Vehicle()
    vehicle.old_state = 25.0

    time = 0.0

    kwargs = dict()
    kwargs['a'] = a
    kwargs['b'] = b
    kwargs['d'] = 0.0 # initially no slope
    kwargs['m'] = vehicle.mass


    tarray = []
    yarray = []

    t_ref = []
    y_ref = []

    # time loop
    for step in range(n_steps):

        print("At time %f "%time)

        y = vehicle.state

        print("\tVelocity is %f "%y)

        error = r - y

        u = controller.execute(error)

        kwargs['u'] = u
        vehicle.execute(**kwargs)

        if step == 1000: # at 1 second change reference signal to 30 m/s
            r = 30.0


        if step == 10000: #at t=10 there is a slope at the road
            kwargs['d'] = Physics.gravity_constant()*math.sin(alpha)

        time += dt

        #if step % 100 == 0:
        tarray.append(time)
        yarray.append(vehicle.state)

        t_ref.append(time)
        y_ref.append(r)



    plotter = TwoDPlotter(xlabel = "time (sec)", ylabel="Velocity (m/s)")
    plotter.plot(tarray, yarray)
    plotter.plot(t_ref, y_ref)
    plotter.show_plots()

    print("End simulation....")