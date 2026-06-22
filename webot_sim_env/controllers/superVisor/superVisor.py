"""superVisor controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
import paho.mqtt.client as mqtt
import json

# All the robots ids in the simmulation
robots = []
controllerPath = "telemetryListener"

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("bot/#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    #print(msg.topic+" "+str(msg.payload))
    botID = extract_botID(msg)
    if (int(botID) > 0) & robots.count(botID) < 1:
        add_chariot(msg)
    
# Extracts the botID as integer
def extract_botID(msg):
    # decode msg payload
    txt = msg.topic
    
    # Locate the first delimiter, "/"
    delim = [txt.find("/")]
    delim.append(txt.find("/", delim[-1]))
    botID = txt[delim[0]+1:]
    # print(f"botID:({botID})")
    # print(f"delim{delim})
    return botID
def extract_properties(msg):
    #Extract properties from JSON, append with \n
    properties = []
    #properties = ["translation -0.2 0.2 0 \n", "rotation 0 0 1 2 \n"]
    
    # JSON parsing here
    decodedPayload = json.loads(msg.payload)
    #print(json.dumps(decodedPayload))
    
    for key in decodedPayload.keys():
        field = decodedPayload[key]
        print(f"{key} : {field}")
        
        match key:
            case "position":
                #print("updating pos")
                properties.append(f"translation {field['x']} {field['y']} 0.1 \n")
            case "rotation":
                properties.append(f"rotation 0 0 1 {field['radians_from_north']} \n")
            case "wheels":
                properties.append(f"wheel_position_left {field['radian_position_wheel_left']} \n")
                properties.append(f"wheel_position_right {field['radian_position_wheel_right']} \n")
            case "laser_turret":
                properties.append(f"turret_position_horizontal {field['radian_position_horizontal']} \n")
                properties.append(f"turret_position_vertical {field['radian_position_vertical']} \n")
    return properties
    
# Handles adding new robots to the simulation
def add_chariot(msg):
    # https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_set_sf_float
    # https://cyberbotics.com/doc/reference/supervisor?tab-language=python#wb_supervisor_field_import_mf_node_from_string
    # See tray example provided in env folder
    print(f"Supervisor adding new robot to payload")
    botID = extract_botID(msg)
    
    properties = extract_properties(msg)
    properties.append(f'controller "{controllerPath}" \n')
    
    propertiesString = "{\n"
    
    for property in properties:
        propertiesString += property
        
    propertiesString += "\n}"
    
    print(propertiesString)
    bot = rootChildrenField.importMFNodeFromString(-1, f"DEF chariot_{botID} chariot {propertiesString}")
    
    robots.append(botID)
    
# create the Robot instance.
supervisor = Supervisor()

rootNode = supervisor.getRoot()  # get root of the scene tree
rootChildrenField = rootNode.getField('children')

# Setup mqtt client, and it's callbacks
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect("localhost", 1883, 60)
# Starts a background thread for managing the network loop.
## Prevents slowing the simulation down by multi threading. Chosen for processing speed, at th cost of required computational power.
mqttc.loop_start()
# Once a new Id pops up, create a new proto with that name, set it up to be a telemetry listener


# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass
mqttc.loop_stop()
# Enter here exit cleanup code.
