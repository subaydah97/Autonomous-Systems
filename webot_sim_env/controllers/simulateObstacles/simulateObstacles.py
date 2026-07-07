"""simulateObstacles controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor, Node
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
                try:
                    create_obstacle(obstacle)
                except AttributeError as e:
                    remove_obstacle(obstacle["id"])
                    print(PRFX,e)
                except Exception as e:
                    print(PRFX,e)
                     

        case "OR/NEW":
            try:
                obstacle = json.loads(msg.payload)
            except Exception as e:
                print(PRFX,"JSON faulty formatting detected, ignoring message.")
                return
            try:
                formattedObstacle = {"id":get_new_id(),"payload":obstacle}
                create_obstacle(formattedObstacle)
            except Exception as exception:
                print(PRFX,msg.topic,exception)
                remove_obstacle(obstacle["id"])                 
        case "OR/MOV":
            try:
                obstacle = json.loads(msg.payload)
                update_obstacle(obstacle)
            except json.decoder.JSONDecodeError as exception:
                print(PRFX,exception)
            except Exception as exception:
                print(PRFX,exception)                   
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

# Obstacle format ought to be {"id":<id>, "payload":<obstacle format outlined in IDD>}
def create_obstacle(obstacle):
    
    if isinstance(obstacle["payload"],str): print(PRFX,"Atribute error caught. Object not created"); return

    # Create base object
    obstacleString = f"DEF obstacle_{obstacle["id"]} " + " Solid { translation 0 0 0 boundingObject Box {size 0.1 0.1 0.1} children [ Shape {geometry Box {size 0.1 0.1 0.1} appearance PBRAppearance {baseColor 1 0 0 metalness 0} } ] }" 
    selfChildren.importMFNodeFromString(-1, obstacleString)
    
    node = robot.getFromDef(f"obstacle_{obstacle["id"]}")
    obstacleCache.append({"id":obstacle["id"],"node":node})


    # Alter base obstacle according to keys in obstacle wrapper
    update_obstacle(obstacle)

def update_obstacle(obstacle):
    #Get webot node of obstacle
    #print(PRFX,"Updating:",obstacle)
    node = None
    
    for obstacleLookup in obstacleCache:
        if obstacleLookup["id"] == obstacle["id"]:
            node = obstacleLookup["node"]
            #print(PRFX,"Found:",obstacleLookup)
            break

    if not isinstance(node,Node):
       print(PRFX,"Updated obstacle doesn't exist:",obstacle["id"])
       print(PRFX,"Creating instead.")
       create_obstacle(obstacle)
       print(PRFX,"Creation complete, terminating update routine")
       return

    # Alter base object according to keys in the obstacle data
    if len(obstacle["payload"].keys()) < 1: raise Exception("Broken payload")
    for key in obstacle["payload"].keys():
        #print(PRFX,"Updating field:"+key)
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
                print(PRFX,"Creating obstacle:",obstacle["id"],"\tField not accounted for:",key)    

def remove_obstacle(obstacleID):
                try:
                    for obstacle in obstacleCache:
                        if obstacle["id"] == obstacleID: 
                            print(PRFX,f'removing obstacle:"{obstacle}"')
                            obstacle["node"].remove()
                except Exception as e:
                        print(PRFX,"Obstacle removal error:"+'"'+msg.payload.decode("utf-8")+'"',e)

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