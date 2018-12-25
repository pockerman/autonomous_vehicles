"""
Implements the 4-step Runge-Kutta method

"""

from .ode_integrator_base import ODEIntegratorBase

class ODERungeKutta4(ODEIntegratorBase):

    @staticmethod
    def create(**kwargs):
        return ODERungeKutta4(**kwargs)

    @staticmethod
    def type():
        return "RK4"

    def __init__(self, **kwargs):

        ODEIntegratorBase.__init__(self, **kwargs)
        self.history_size = 1

        if 'init_condition' in kwargs.keys():
            self.update_history(0, kwargs['init_condition'] )

    def execute(self, **kwargs):

        f = kwargs['f']
        tn = kwargs['tn']
        yn = self.get_history(0)
        kn1 = f(tn=tn, yn=yn)
        kn2 = f(tn=tn + 0.5*self.step_size, yn=yn + 0.5*self.step_size*kn1)
        kn3 = f(tn=tn + 0.5*self.step_size, yn=yn + 0.5*self.step_size*kn2)
        kn4 = f(tn=tn + self.step_size, yn=yn + self.step_size*kn3)

        new = yn + self.step_size*((kn1 + 2*kn2 +2.0*kn3 + kn4)/6.0)
        yn = new
        self.update_history(0, yn)
        return new