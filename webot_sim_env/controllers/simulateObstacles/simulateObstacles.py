"""simulateObstacles controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import json

# global variables

PRFX = "OBST_SIM:"
HOSTNAME = "localhost"

# Filled with nodes
obstacleCache = []

# Functions
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(PRFX,f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("OR/#")
    publish.single("OR/COMMANDS", f"SPIT", hostname=HOSTNAME)


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global obstacleCache
    #print(PRFX,msg.topic+" "+str(msg.payload))

    match msg.topic:
        case "OR/COMMANDS":
                match msg.payload.decode("utf-8").upper():
                    case "CLEAR":
                          remove_all_obstacles()
                          
        case "OR/COMPLETE_REGISTRY":
            
            remove_all_obstacles()

            obstacles = json.loads(msg.payload)

            for obstacle in obstacles:
                print(PRFX,obstacle)
                create_obstacle(obstacle)
                
        case "OR/NEW":
            obstacle = json.loads(msg.payload)
            formattedObstacle = {"id":get_new_id(),"payload":obstacle}
            create_obstacle(formattedObstacle)
        case "OR/REM":
            remove_obstacle(int(msg.payload))
        case _:
            print(PRFX,"Topic not accounted for:",msg.topic+" "+str(msg.payload))

def get_new_id(obstacleCache=obstacleCache):
    newID = -1
    # If the obstacle has no id yet, create it. This should allign with the id an obstacle registry would assign to this obstacle.
    try:
        newID = obstacleCache[-1]["id"]+1
    except:
        newID = 0
    return newID

def create_obstacle(obstacle):
    
    # Create base object
    obstacleString = f"DEF obstacle_{obstacle["id"]} " + " Pose { translation 0 0 0 children [ Shape {geometry Sphere {radius 0.1} appearance PBRAppearance {baseColor 1 0 0 metalness 0} } ] }" 
    selfChildren.importMFNodeFromString(-1, obstacleString)

    # Alter base object according to keys in the obstacle data
    node = robot.getFromDef(f"obstacle_{obstacle["id"]}")
    
    obstacleCache.append({"id":obstacle["id"],"node":node})

    for key in obstacle["payload"].keys():
        field = obstacle["payload"][key]
        match key:
            case "position":
                node.getField("translation").setSFVec3f([field["x"],field["y"],field["z"]])
            case "rotation":
                node.getField("rotation").setSFRotation([field["x"],field["y"],field["z"],field["radians"]])
            case "id":
                # Used to remove confusing prints from the cmd
                1 == 1
            case _:
                print(PRFX,"Field not accounted for:",key)

def remove_obstacle(obstacleID):
                try:
                    for obstacle in obstacleCache:
                        if obstacle["id"] == obstacleID: 
                            print(PRFX,f'removing obstacle:"{obstacle}"')
                            obstacle["node"].remove()
                   

                except Exception as e:
                        print("Obstacle removal error:"+'"'+msg.payload.decode("utf-8")+'"',e)

def remove_all_obstacles():
    global obstacleCache
    if len(obstacleCache) < 1: return

    try:
        for obstacle in obstacleCache:
            remove_obstacle(obstacle["id"])
        obstacleCache.clear()
    except Exception as e:
         print(PRFX,e)

# create the Robot instance.
robot = Supervisor()

selfChildren = robot.getSelf().getField('children')

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Setup mqtt client, and it's callbacks
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect("localhost", 1883, 60)
# Starts a background thread for managing the network loop.
## Prevents slowing the simulation down by multi threading. Chosen for processing speed, at th cost of required computational power.
mqttc.loop_start()

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
mqttc.loop_stop()