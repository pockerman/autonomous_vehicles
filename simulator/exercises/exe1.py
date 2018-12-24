"""
This is the driver code for exercise 1.3 Cruise Controller
from the course Model Based Engineering. The model that is simulated
is given by the ODE:

dv/dt = (-a/m)*v + (b/m)*u

where

v = vehicle velocity (m/s)
u = control input
a = resistance (Ns/m)
b = Throttle gain (N/rad)
m = Vehicle mass (kg)
g = Gravitational constant (m/s^2)

The simulator uses a PI controller to generate the control signal u
given the error (or deviation) of the produced velocity velocity at some given
time t and a given reference signal r. You can alter various options using the
exe1_options.txt file. Concretely you can alter the following:

Integrator: EULER #(or RK4)
dt: 0.001
steps: 1000 #number of steps for the integrator to perform
a: 200
m: 1000 #vehicle mass
g: 9.82 #gravitational acceleration
b: 10000 #Throttle gain
Kp: 0.01 #proportial gain
Ki: 0.01 #integral gain
"""


import math

from dynamics.dynamics_base import DynamicsBase
from vehicle.vehicle_base import VehicleBase
from control.pid_control import PIDControl
from ode_integrators.integrator_factory import IntegratorFactory
from plot.two_d_plotter import TwoDPlotter

class Dynamics(DynamicsBase):

    def __init__(self):
        DynamicsBase.__init__(self)
        self.__velocity = 0.0

    @property
    def state(self):
        return self.__velocity

    @state.setter
    def state(self, value):
        self.__velocity = value

    def compute_rhs(self, **kwargs):
        m = kwargs['m']
        a = kwargs['a']
        b = kwargs['b']
        u = kwargs['u']
        yn = self.get_old_state('V', 0)

        return -(a/m)*yn + (b/m)*u

    def execute(self, **kwargs):
        kwargs['f'] = self.compute_rhs
        self.state = self.get_integrator('V').execute(**kwargs)


class Vehicle(VehicleBase):

    def __init__(self):
        VehicleBase.__init__(self)

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
        return self.__dynamics.get_old_state('V', 0)

    @old_state.setter
    def old_state(self, value):
        self.__dynamics.set_old_state('V', 0, value)

    def set_integrator(self, integrator):
        self.__dynamics.set_integrator('V', integrator=integrator)

if __name__ == '__main__':

    # attempt to read the properties file
    with open('exe1_options.txt', 'r') as options_file:

        properties = dict()

        for line in options_file:
            #line = options_file.readline()

            # remove any comments from the line
            if line.find('#') != -1:

                #split with the comment value and take the first resulting string
                line = line.split('#')[0]

            #find out property and its value
            line_data = line.split('=')

            properties[line_data[0]] = line_data[1].strip()


    print(properties)

    #create the reference signal
    rstr = properties['r']
    rstr = rstr[1:]
    rstr = rstr[:-1]
    rstr = rstr.split(';')

    # collect the refernce signals as given by the user
    rsignals = dict()

    for item in rstr:
        if item != '':

            item = item[1:]
            item = item[:-1]
            item = item.split(',')

            assert len(item) == 2, "Inavlid number of items. %s should be 2"%(len(item))


            if int(item[0]) not in rsignals.keys():
                rsignals[int(item[0])] = float(item[1])
            else:
                assert False, "Inavlid number of items. %s should be 2" % (len(item))


    # the vehicle
    vehicle = Vehicle()
    vehicle.mass = float(properties['mass'])
    vehicle.set_integrator(integrator = IntegratorFactory.create(properties['Integrator'], step_size=float(properties['dt'])))
    vehicle.old_state = float(properties['v0'])

    # the controller to use
    Ki = float(properties['Ki'])
    Kp = float(properties['Kp'])
    controller = PIDControl(Ki=Ki, Kp=Kp, Kd=None)

    dt = float(properties['dt'])
    n_steps = int(properties['steps'])

    kwargs = dict()
    kwargs['a'] = float(properties['a'])
    kwargs['b'] = float(properties['b'])
    kwargs['m'] = vehicle.mass

    # arrays to assemble the current solution
    tarray = []
    yarray = []

    # arrays to assemble the reference signal
    t_ref = []
    y_ref = []

    print("Starting time loop....")

    time=0.0
    r = 0.0
    for step in range(n_steps):
        print("\tAt time %f " % time)

        y = vehicle.state

        print("\t\tVelocity is %f " % y)

        error = 0.0

        if step == 0:
            r = rsignals[0] #by default the reference signal is signal at step 0

        if step in rsignals.keys():
            r = rsignals[step]
            error =  r - y

        u = controller.execute(error)

        kwargs['u'] = u
        vehicle.execute(**kwargs)

        time += dt

        tarray.append(time)
        yarray.append(vehicle.state)

        t_ref.append(time)
        y_ref.append(r)

    plotter = TwoDPlotter(xlabel="time (sec)", ylabel="Velocity (m/s)")
    plotter.plot(tarray, yarray)
    plotter.plot(t_ref, y_ref)
    plotter.show_plots()

    print("End simulation....")

