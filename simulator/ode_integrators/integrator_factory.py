from .ode_integrator_base import ODEIntegratorBase
from .forward_euler import ODEFWDEuler
from .runge_kutta4 import ODERungeKutta4

class IntegratorFactory(object):

    @staticmethod
    def create(type, **kwargs):
        types = ODEIntegratorBase.__subclasses__()

        for integrator in types:
            if integrator.type() == type:
                return integrator.create(**kwargs)

        assert False, "Type %s not in: "%(type)


