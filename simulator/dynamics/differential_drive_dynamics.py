"""

Implements Differential Drive dynamics


"""

import numpy as np


from .dynamics_base import DynamicsBase


class DiffDriveDynamics(DynamicsBase):

    def __init__(self):
        DynamicsBase.__init__(self)
        self.__state = np.zeros((1,0))


    @property
    def state(self):
        return self.__state

    @state.setter
    def state(self, value):
        pass


    def execute(self, **kwargs):
        pass

