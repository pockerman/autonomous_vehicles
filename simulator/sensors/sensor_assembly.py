"""
Represents an assembly of sensors
"""

class SensorAssembly(object):

    def __init__(self):
        self.__sensors = dict()

    def poll(self):
        return None

    def __next__(self):
        return next(self.__sensors)

    def __getitem__(self, item):
        return self.__sensors[item]

    def __setitem__(self, key, sensor):
        self.__sensors[key] = sensor

    def get_sensors(self):
        return self.__sensors.values()