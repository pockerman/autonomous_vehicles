from ode_integrators.ode_integrator_base import ODEIntegratorBase


class ODEScalarFWDEuler(ODEIntegratorBase):


    @staticmethod
    def create(**kwargs):
        return ODEScalarFWDEuler(**kwargs)

    @staticmethod
    def type():
        return "SCALAR_FWD_EULER"

    def __init__(self, **kwargs):
        ODEIntegratorBase.__init__(self, **kwargs)
        self.history_size = 1

        if 'init_condition' in kwargs.keys():
            self.update_history(0, kwargs['init_condition'] )

    def execute(self, **kwargs):

        # get the callable to calculate the rhs
        f = kwargs['f']
        yn = self.get_history(0)
        new = yn + self.step_size*f(**kwargs)
        yn = new
        self.update_history(0, yn)
        return new


class ODEVectorFWDEuler(ODEIntegratorBase):


    @staticmethod
    def create(**kwargs):
        return ODEVectorFWDEuler(**kwargs)

    @staticmethod
    def type():
        return "VECTOR_FWD_EULER"

    def __init__(self, **kwargs):
        ODEIntegratorBase.__init__(self, **kwargs)
        self.history_size = 1

        if 'init_condition' in kwargs.keys():
            self.update_history(0, kwargs['init_condition'] )

    def execute(self, **kwargs):

        # get the callable to calculate the rhs
        f = kwargs['f']
        yn = self.get_history(0)
        new = yn + self.step_size*f(**kwargs)
        yn = new
        self.update_history(0, yn)
        return new



