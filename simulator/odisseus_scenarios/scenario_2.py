"""
Scenario ID: 2
Description: Tests the scenario where the data received from a sensor
are implausible. Thus, an error message should be sent to the ErrorLogger
"""

from odisseus_vehicle import Odisseus


if __name__ == '__main__':

    print("Starting simulation....")

    properties = dict()

    vehicle = Odisseus(properties=properties)

    # number of simulation steps
    n_steps = 10

    # the time step
    dt = 0.001

    time = 0.0

    for step in range(n_steps):
        print("At time %f " % time)

        vehicle.execute()
        time += dt

    print("End simulation....")