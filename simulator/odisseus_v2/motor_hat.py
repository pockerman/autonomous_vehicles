"""
Models a motor hat.
"""

import time
import atexit


from .conf import ON_RASP_PI
from .conf import I2CAddr

if ON_RASP_PI:

    # if we are on the Pi use the MotorHat
    from Raspi_MotorHAT import Raspi_MotorHAT

    motor_hat = Raspi_MotorHAT(I2CAddr)

    # the front motors
    flm = motor_hat.getMotor(1)
    frm = motor_hat.getMotor(2)

    # the rear motors
    rlm = motor_hat.getMotor(3)
    rrm = motor_hat.getMotor(4)



else:

    # this a testing framework
    from .motor import Motor

    # the front motors
    flm = Motor(1)
    frm = Motor(2)

    # the rear motors
    rlm = Motor(3)
    rrm = Motor(4)

# the motors array
motors = [flm, frm, rlm, rrm]


# given the motors we can now define some basic bahaviors

def turn_off_motors():
    if ON_RASP_PI:

        for motor in motors:
            motor.run(Raspi_MotorHAT.RELEASE)
    else:
        for motor in motors:
            motor.turn_off()

def move_fwd(speed, sleeptime):
    """
    Move forward at the given speed
    """

    if ON_RASP_PI:

        for motor in motors:
            motor.setSpeed(speed)
            motor.run(Raspi_MotorHAT.FORWARD)
    else:
        for motor in motors:
            motor.set_speed(speed=speed)
            motor.foward()

    time.sleep(sleeptime)

# Execute this function before exiting the script
atexit.register(turn_off_motors)



