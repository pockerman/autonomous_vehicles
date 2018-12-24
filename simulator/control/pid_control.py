from . controller_base import ControllerBase

class PIDControl(ControllerBase):

    def __init__(self, Kp, Ki, Kd):
        ControllerBase.__init__(self)

        self.__error = 0.0

        if Kp is not None:
            self.set_property('Kp', Kp)

        if Ki is not None:
            self.set_property('Ki', Ki)

        if Kd is not None:
            self.set_property('Kd', Kd)

    def execute(self, error, **kwargs):

        rslt = 0.0

        if self.has_property('Kd'):
            delta_error = error - self.__error
            dt = kwargs['dt']
            rslt += self.get_property('Kd') * (delta_error/dt) #self.__error

        if self.has_property('Kp'):
            rslt += self.get_property('Kp')*error

        # accumulate  the error
        self.__error += error

        if self.has_property('Ki'):
            rslt += self.get_property('Ki') * self.__error

        return rslt

