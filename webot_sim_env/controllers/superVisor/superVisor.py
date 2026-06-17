"""superVisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor

# https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_set_sf_float
# https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_import_mf_node_from_string
# See tray example provided in env folder

# Once a new Id pops up, create a new proto with that name, set it up to be a telemetry listener

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
