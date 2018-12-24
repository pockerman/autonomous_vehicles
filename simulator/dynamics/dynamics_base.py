from abc import ABC
from abc import abstractmethod
from ode_integrators.ode_integrator_map import ODEIntegratorMap


class DynamicsBase(ABC):
    """
    Base class for describing dynamics
    """

    def __init__(self):
        self.__integrators = ODEIntegratorMap()

    @property
    @abstractmethod
    def state(self):
        pass

    @state.setter
    @abstractmethod
    def state(self, value):
        pass

    def get_old_state(self, name, idx):
        return self.__integrators[name].get_history(idx)

    def set_old_state(self, name, idx, value):
        self.__integrators[name].update_history(idx, value)

    def get_integrator(self, name):
        return self.__integrators[name]

    def set_integrator(self, name, integrator):
        self.__integrators[name] = integrator


    @abstractmethod
    def execute(self, **kwargs):
        pass