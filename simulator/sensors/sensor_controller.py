"""
Generic class to be used as a controller for sensor data.
Frequently, an application has to check the data recorded by a snsor
before sending it to a computational component this class is meant to
serve this purpose
"""

from error_report.error_msg import ErrMsg

class SensorController(object):

    def __init__(self, err_logger, sensor_spec):
        self.__error_logger = err_logger
        self.__sensor_spec = sensor_spec


    def filter(self, sensor):

        result = self.__sensor_spec.check(sensor)

        if result == False:

            # the sensor data is not sane
            err_msg = ErrMsg(sensor.type, "Invalid sensor data")
            self.__error_logger.append_error(err_msg)



