"""telemetryListener controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
import paho.mqtt.client as mqtt
import json

# def setSFFloat(self, value):
# def setSFRotation(self, values):
# def setSFVec3f(self, values):

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(PRFX,f"Connected with result code {reason_code}")
    #print(f"my id:'{myID}'")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(f"bot/{myID}")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    #print(msg.topic+" "+str(msg.payload))
    
    # Decode json
    try:
        decodedPayload = json.loads(msg.payload)
    except Exception as e:
        print(PRFX,e)
        print({PRFX,"Ignoring faulty formatted message"})
        return
    #print(json.dumps(decodedPayload))
    
    for key in decodedPayload.keys():
        field = decodedPayload[key]
        #print(f"{key} : {field}")
        
        match key:
            case "position":
                z = positionField.getSFVec3f()[2]
                positionField.setSFVec3f([float(field['x']),float(field['y']) ,z])
            case "rotation":
                rotationField.setSFRotation([0,0,1,field['radians_from_north']])
            case "wheels":
                wheelLeftField.setSFFloat(float(field['radian_position_wheel_left']))
                wheelRightField.setSFFloat(float(field['radian_position_wheel_right']))
            case "laser_turret":
                1 == 1

# Starts a background thread for managing the network loop.
## Prevents slowing the simulation down by multi threading. Chosen for processing speed, at th cost of required computational power.

# create the Robot instance.
robot = Supervisor()

myID = robot.getName()
self = robot.getSelf()
PRFX = f"chariot_{myID}:"

positionField = self.getField("translation")
rotationField = self.getField("rotation")
wheelLeftField = self.getField("wheel_position_left")
wheelRightField = self.getField("wheel_position_right")

mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect("localhost", 1883, 60)

mqttc.loop_start()



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
mqttc.loop_end()