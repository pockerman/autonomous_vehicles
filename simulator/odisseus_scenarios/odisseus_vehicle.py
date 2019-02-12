"""
class that models the Odisseus vehicle
"""

from vehicle.vehicle_base import VehicleBase
from dynamics.differential_drive_dynamics import DiffDriveDynamics
from sensors.sensor_assembly import SensorAssembly
from sensors.sensor_controller import SensorController
from error_report.error_logger import ErrLogger

class Odisseus(VehicleBase):

    def __init__(self, properties):
        VehicleBase.__init__(self, properties=property)
        self.__error_logger = ErrLogger()
        self.__dynamics = DiffDriveDynamics()
        self.__sensors = SensorAssembly()
        self.__sensor_controller = SensorController(err_logger=self.__error_logger)

    @property
    def state(self):
        pass

    @state.setter
    def state(self, value):
        pass

    def get_old_state(self, name, idx):
        pass

    def set_old_state(self, name, idx, value):
        pass

    def poll_sensors_and_report(self, **kwargs):

        for sensor in self.__sensors:
            self.__sensor_controller.filter(sensor)

    def execute(self, **kwargs):

        self.poll_sensors_and_report(**kwargs)

        self.__dynamics.execute(**kwargs)