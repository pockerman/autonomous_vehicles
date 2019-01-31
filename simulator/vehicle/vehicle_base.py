from abc import ABC
from abc import abstractmethod


class VehicleBase(ABC):
    """
    Base class for modeling a vehicle
    """

    def __init__(self):
        self.__properties = dict()
        self.__properties['mass'] = 0.0
        self.__propulsion = None

    @property
    def mass(self):
        return self.__properties['mass']

    @mass.setter
    def mass(self, value):
        self.__properties['mass'] = value

    @property
    @abstractmethod
    def state(self):
        pass

    @state.setter
    @abstractmethod
    def state(self, value):
        pass

    @abstractmethod
    def get_old_state(self, name, idx):
        pass

    @abstractmethod
    def set_old_state(self, name, idx, value):
        pass

