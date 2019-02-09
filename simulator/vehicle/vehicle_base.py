from abc import ABC
from abc import abstractmethod


class VehicleBase(ABC):
    """
    Base class for modeling a vehicle
    """

    def __init__(self, properties):

        if properties is None:
            self.__properties = dict()
            self.__properties['mass'] = 0.0
        else:
            self.__properties = properties

    def get_property(self, name):
        return self.__properties[name]

    def set_property(self, name, value):
        self.__properties[name] = value

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

