"""
Scenario ID: 2
Description: Tests the scenario where the data received from a sensor
are implausible. Thus, an error message should be sent to the ErrorLogger
The ultrasound sensor is used to test this.
"""

from  odisseus_vehicle import Odisseus
from sensors.sensor_assembly import SensorAssembly
from sensors.ultrasound_sensor import UltraSoundSensor
from sensors.sensor_specification import SensorSpecification
from error_report.error_logger import ErrLogger

def test():
    print("Starting simulation....")

    properties = dict()
    properties['error_logger'] = ErrLogger()
    properties['sensor_assembly'] = SensorAssembly()
    properties['sensor_assembly']['US'] = UltraSoundSensor()
    properties['sensor_specification'] = SensorSpecification()

    vehicle = Odisseus(properties=properties)

    # number of simulation steps
    n_steps = 1

    # the time step
    dt = 0.001
    time = 0.0

    for step in range(n_steps):
        print("At time %f " % time)

        vehicle.execute()
        assert properties['error_logger'].n_errors() != 0, "No error was detected"
        assert properties['error_logger'].error_exists_for_type(type='US') == True, "No error was detected for US sensor"
        time += dt

    print("End simulation....")



if __name__ == '__main__':
    test()