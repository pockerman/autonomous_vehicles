"""
Models a motor hat.
"""

from typing import List
from .motor import Motor

class MotorHat(object):

    def __init__(self, addr):
        """
        Initialize the hat by passing the I2C address
        """
        # the I2C address
        self.__addr = addr

        # the motors
        self.__motors: List[Motor] = []

    def create_motor(self, parameters):
        self.__motors.append(Motor(parameters=parameters))

    def get_motor(self, idx: int)-> Motor:

        assert idx < 4 and idx>=0, "Invalid motor index"
        return self.__motors[idx]

    def turn_off_motors(self):
        for motor in self.__motors:
            motor.turn_off()

    def run_all_motors(self, dir):
        for motor in self.__motors:
            motor.turn_off()
