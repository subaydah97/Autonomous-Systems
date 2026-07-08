"""sim_unit controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor, Node

import paho.mqtt.publish as publish
from enum import Enum
import json

# Declarations
class States(Enum):
    traversing_to_reserved_yband = "traversing_to_reserved_yband"
    traversing_x_axis = "looking_for_obstacles_on_x_axis"
    traversing_home_x = "traversing_home_x"
    rest = "rest"

# Variables

reserved_y_band = [-0.50,-0.20]
my_width = 0.20

sim_obstacles = [
    {
        "id":99,
        "payload":
            {"position":
             {
                 "x":1.5,
                 "y":-0.3,
                 "z":0
             }
             }
    }]

# create the Robot instance.
robot = Supervisor()
myID = robot.getName()
self = robot.getSelf()

PRFX = f"sim_unit_{myID}"

positionField = self.getField("translation")
rotationField = self.getField("rotation")
wheelLeftField = self.getField("wheel_position_left")
wheelRightField = self.getField("wheel_position_right")

lp = positionField.getSFVec3f()

# Functions

def send_telemetry():
    telemetry_data = {
        "position": {
                        "x": lp[0],
                        "y": lp[1]
                    }
        }
    publish.single(f"bot/{myID}", json.dumps(telemetry_data), hostname="localhost")

# Move to absolute coordinates
def teleport(p):
    positionField.setSFVec3f([p[0],p[1],p[3]])

# Move relative
def move(p):
    global lp
    positionField.setSFVec3f([p[0] + lp[0],
                              p[1] + lp[1],
                              p[2] + lp[2] 
                              ])
    
def in_reserved_y_band():
    global lp

    returnVal = False
    
    if lp[1] > reserved_y_band[0] and lp[1] < reserved_y_band[1]:
        returnVal = True
    
    return returnVal
# Routines

# Retuns detected obstacle id, or -1 if none
def obstacle_detection_routine():
    global state
    returnval = -1
    for obstacle in sim_obstacles:
        botX = lp[0]
        if abs(botX - obstacle["payload"]["position"]["x"]) < 0.1:
            returnval = obstacle["id"]
            
            publish.single("OR/NEW", json.dumps(obstacle["payload"]), hostname="localhost")
            sim_obstacles.remove(obstacle)
            state = States.traversing_home_x
            break

    return int(returnval)
# Main program

state = States.traversing_to_reserved_yband

# Run this controller 4 times each simulated second
timestep = int(1000/4)
print(PRFX, "Running at a step of:",timestep)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    positionField = self.getField("translation")
    rotationField = self.getField("rotation")
    wheelLeftField = self.getField("wheel_position_left")
    wheelRightField = self.getField("wheel_position_right")

    lp = positionField.getSFVec3f()

    send_telemetry()
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    match state:
        case States.traversing_to_reserved_yband:
            move([0,-0.05,0])
            if in_reserved_y_band():
                state = States.traversing_x_axis
            print(PRFX,"lp:",lp)
        case States.traversing_x_axis:
            move([0.05,0,0])
            obstacle_detection_routine()
        case States.traversing_home_x:
            move([-0.05,0,0])
            if(lp[0] < 0.1):
                state = States.rest
        case States.rest:
            pass
        case _:
            print(PRFX,"State machine doesn't acount for state:",state)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
