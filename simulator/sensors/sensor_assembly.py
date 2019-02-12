"""
Represents an assembly of sensors
"""

class SensorAssembly(object):

    def __init__(self):
        self.__sensors = dict()

    def poll(self):
        return None