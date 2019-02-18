"""
class that models the Odisseus vehicle
"""

from vehicle.vehicle_base import VehicleBase
from dynamics.differential_drive_dynamics import DiffDriveDynamics
from sensors.sensor_controller import SensorController

class Odisseus(VehicleBase):

    def __init__(self, properties):
        VehicleBase.__init__(self, properties=property)
        self.__error_logger = properties['error_logger']
        self.__dynamics = DiffDriveDynamics()
        self.__sensors = properties['sensor_assembly']
        self.__sensor_spec = properties['sensor_specification']
        self.__sensor_controller = SensorController(err_logger=self.__error_logger,sensor_spec=self.__sensor_spec)

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

        sensors = self.__sensors.get_sensors()
        for sensor in sensors:
            self.__sensor_controller.filter(sensor)

    def execute(self, **kwargs):

        self.poll_sensors_and_report(**kwargs)

        self.__dynamics.execute(**kwargs)