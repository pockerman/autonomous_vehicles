from ode_integrators.ode_integrator_base import ODEIntegratorBase


class ODEFWDEuler(ODEIntegratorBase):


    @staticmethod
    def create(**kwargs):
        return ODEFWDEuler(**kwargs)

    @staticmethod
    def type():
        return "FWD_EULER"

    def __init__(self, **kwargs):
        ODEIntegratorBase.__init__(self, **kwargs)
        self.history_size = 1

        if 'init_condition' in kwargs.keys():
            self.update_history(0, kwargs['init_condition'] )

    #def __name__(self):
    #    return "FWD_EULER"

    def execute(self, **kwargs):

        # get the callable to calculate the rhs
        f = kwargs['f']
        yn = self.get_history(0)
        #tn = kwargs['tn']
        new = yn + self.step_size*f(**kwargs)
        yn = new
        self.update_history(0, yn)
        return new
