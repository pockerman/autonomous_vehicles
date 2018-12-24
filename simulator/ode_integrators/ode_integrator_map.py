"""

class ODEIntegratorMap. It represents a map to ODE Integrators. This is useful when we want
to aggregate a different type of ODE integrator for a given property

"""

from .ode_integrator_base import ODEIntegratorBase


class ODEIntegratorMap(ODEIntegratorBase):

    @staticmethod
    def create(**kwargs):
        raise ValueError("Should not be called")

    @staticmethod
    def type():
        return "IntegratorMap"

    def __init__(self):
        ODEIntegratorBase.__init__(self)
        self.__integrators = dict()

    def execute(self, **kwargs):

        for integrator in self.__integrators.values():
            integrator.execute(**kwargs)

    def __getitem__(self, item):
        return self.__integrators[item]

    def __setitem__(self, key, value):
        self.__integrators[key] = value


