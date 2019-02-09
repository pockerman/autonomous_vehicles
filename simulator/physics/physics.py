
import numpy as np


class Physics(object):

    @staticmethod
    def gravity():
        return np.array([0., 0., Physics.gravity_constant()])


    @staticmethod
    def gravity_constant():
        """
        Returns the gravitational constant see:
        https://en.wikipedia.org/wiki/Gravitational_constant
        """
        return 9.82