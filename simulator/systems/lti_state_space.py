
import numpy as np
from ode_integrators.ode_integrator_map import ODEIntegratorMap

class LTIStateSpace(object):

    def __init__(self, state, state_integrator):
        self.__state = state
        self.__state_integrator = state_integrator
        self.__A = None
        self.__B = None
        self.__y = None
        self.__C = None
        self.__D = None

    def set_old_state(self, idx, old_state):

        assert old_state.shape == self.__state.shape, "Invalid state shape"
        assert idx < self.__state_integrator.history_size, "Invalid history index"
        self.__state_integrator.update_history(idx, old_state)

    def set_A(self, A):
        self.__A = A

    def set_B(self, B):
        self.__B = B

    def set_C(self, C):
        self.__C = C

    def set_D(self, D):
        self.__D = D

    def execute(self, **kwargs):

        # integrate each variable
        kwargs['f'] = self.rhs
        result = self.__state_integrator.execute(**kwargs)
        self.__state.set_system_state_value(result)
        np.dot(self.__C, self.__state.get_system_state_value(), self.__y)
        return self.__y


    def rhs(self, **kwargs):

        a_dot_c = np.zeros(shape=(self.__state.get_n_states(),1))
        np.dot(self.__A, self.__state.get_system_state_value(), a_dot_c )
        b_dot_u = np.zeros(shape=(self.__B.shape[0],1))

        b_dot_u = self.__B*kwargs['u']
        #np.dot(self.__B, kwargs['u'], b_dot_u)
        return a_dot_c + b_dot_u