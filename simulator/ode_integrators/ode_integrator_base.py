from abc import ABC
from abc import abstractmethod

class ODEIntegratorBase(ABC):


    @staticmethod
    @abstractmethod
    def create(**kwargs):
        pass

    @staticmethod
    @abstractmethod
    def type():
        pass

    def __init__(self, **kwargs):
        ABC.__init__(self)
        self.__history_size = 0
        self.__history = []
        self.__rhs_func = None

        if 'rhs_func' in kwargs.keys():
            self.__rhs_func = kwargs['rhs_func']

        if 'step_size' in kwargs.keys():
            self.__step_size = kwargs['step_size']
        else:
            self.__step_size = 0.1

    @abstractmethod
    def execute(self, **kwargs):
        raise Exception("Invalid function call")

    @property
    def history_size(self):
        return self.__history_size

    @history_size.setter
    def history_size(self, size):
        self.__history_size = size

    @property
    def step_size(self):
        return self.__step_size

    @step_size.setter
    def step_size(self, value):
        self.__step_size = value

    @property
    def rhs_func(self):
        if self.__rhs_func is None:
            raise ValueError("Right hand side has not been set")
        return self.__rhs_func

    @rhs_func.setter
    def rhs_func(self, value):
        self.__rhs_func = value

    def has_rhs_func(self):
        return self.__rhs_func is not None

    def update_history(self, idx, value):

        if idx >= self.__history_size or idx < 0:
            raise ValueError("Invalid hystory index. Index %s not in [0, self.__history_size)"%idx)

        if value is not None:

            if self.__history == []:
                self.__history.append(value)
            else:
                self.__history[idx] = value

    def get_history(self, idx):
        if idx >= self.__history_size or idx < 0:
            raise ValueError("Invalid hystory index. Index %s not in [0, %s)"%(idx, self.__history_size))
        return self.__history[idx]

    def __call__(self, **kwargs):
        self.execute(**kwargs)
