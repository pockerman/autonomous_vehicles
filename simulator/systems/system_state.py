
import numpy as np

class SysState(object):

    def __init__(self, n_entries, dtype = np.float64):

        assert n_entries >0, "Invalid number of entries. %s should be >0"%n_entries

        self.__state_data = dict()
        self.__state_value = np.zeros(shape=(n_entries, 1),dtype=dtype)


    @property
    def states_names(self):
        return self.__state_data.keys()

    @property
    def shape(self):
        return self.__state_value.shape

    def add_state(self, state_name, idx):

        assert idx < self.__state_value.shape[0] and idx >=0,"Inavlid state idx"
        assert state_name not in self.__state_data.keys(), "State %s already exists"%state_name
        self.__state_data[state_name] = idx

    def get_n_states(self):
        return self.__state_value.shape[0]

    def get_state_value_by_idx(self, idx):
        return self.__state_value[idx]

    def get_state_value_by_name(self, name):
        idx = self.__state_data[name]
        return self.__state_value[idx]

    def get_system_state(self):
        return self.__state_data, self.__state_value

    def get_system_state_value(self):
        return self.__state_value

    def set_system_state_value(self, value):
        assert value.shape == self.__state_value.shape, "Invalid shape. Shape %s not equal to %s"%(value.shape, self.__state_value.shape)
        self.__state_value = value