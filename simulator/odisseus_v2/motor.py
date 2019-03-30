"""
Models an electric motor
"""

class Motor(object):

    def __init__(self, parameters):
        self.__is_on: bool = False
        self.__parameters = parameters
        self.__speed: int = 0

    def turn_off(self)-> None:
        self.__is_on = False

    def turn_on(self)-> None:
        self.__is_on = True

    def set_speed(self, speed: int) -> None:
        self.__speed = speed

    def run(self, dir)-> None:
        pass