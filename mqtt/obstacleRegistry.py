# Dependencies
import json
import pickle
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

# Global variables
obstacles = []
    ## Obstacles fields
    ## 'id': assigned at creation. Used to count total obstacles
    ## 'payload': object or byte-string
# The amount of obstacles that have been added to the registry. Resets when the registry is cleared.
obstacleCount = 0
# Used to control program termination
persistFlag = 1

HOSTNAME = "localhost"

# Exceptions
class doesNotExist(Exception):
    1==1

# Functions
def publishRegistry():
    global obstacles
    publish.single("OR/COMPLETE_REGISTRY", json.dumps(obstacles), hostname=HOSTNAME)
def printOR():
    global obstacles
    print(f"obstacleCount:{obstacleCount}")
    for obstacle in obstacles:
        print("\t",obstacle)
 
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"obstacle_registry: Connected with result code {reason_code}")
    client.subscribe("OR/#")
    
    printOR()
    publishRegistry()




# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global obstacleCount, obstacles, persistFlag
    try:
        decoded_payload = msg.payload.decode("utf-8")
        #print(msg.topic+':"'+ decoded_payload +'"')
    except:
         print(msg.topic+':'+"payload unsafe to print to terminal")

    match msg.topic:
            case "OR/COMMANDS":
                match decoded_payload.upper():
                    case "CLEAR":
                        print("CLEAR command received, clearing obstacle registry")
                        obstacles.clear()
                        obstacleCount = 0
                    case "TERM":
                        print("TERM command received, lowering persist flag")
                        persistFlag = 0
                    case "SPIT":
                       printOR()
                       publishRegistry()
                    case _:
                        print("Unacounted for command:",decoded_payload)
            case "OR/NEW":
                # USed to receive new payloads
                id = obstacleCount
                obstacleCount += 1
                try:
                    obstacles.append({"id":id,"payload":json.loads(msg.payload)})
                except Exception as e:
                    print("failed to load obstacle as JSON, storing as bytes instead:",msg.payload)
                    obstacles.append({"id":id,"payload":str(msg.payload.decode("utf-8"))})
                
                print(f'added obstacle:"{obstacles[-1]}"')

            case "OR/MOV":
                try:
                    updatedObstacle = json.loads(msg.payload)
                    obstacleExistsFlag = False
                    for obstacle in obstacles:
                        if obstacle["id"] == updatedObstacle["id"]:
                            obstacle["payload"] = updatedObstacle["payload"]
                            print("Updated obstacle:",obstacle)
                            obstacleExistsFlag = True
                            break
                    if obstacleExistsFlag: return
                    raise doesNotExist("Obstacle doesn't exist")
                except doesNotExist as e:
                    print(e)
                    print("Creating obstacle instead")
                    obstacles.append(updatedObstacle)
                except Exception as e:
                    print("Failed updating obstacle:",e)
                #publishRegistry()

            case "OR/REM":
                try:
                    targID = int(decoded_payload)
                    for obstacle in obstacles:
                        if obstacle["id"] == targID: print(f'removing obstacle:"{obstacle}"'); obstacles.remove(obstacle)
                except Exception as e:
                        print("Obstacle removal error:"+'"'+decoded_payload+'"',e)


# Main
print("attempting to open to obstacle file")
try:
    with open("./mqtt/obstacles.pickle", "rb") as saveFile:
        obstacles = pickle.load(saveFile)
except Exception as e:
    print("attempt failed:",e)
else: 
    print("attempt success.")

try:
    obstacleCount = int(obstacles[-1]["id"]) + 1
except Exception as e:
    print("Recovering obstacle count failed:",e)


mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect(HOSTNAME, 1883, 60)

mqttc.loop_start()

while persistFlag > 0:
    1 == 1

# Cleanup
print(f"exiting with code:{persistFlag}")

print("attempting to save obstacles to file")
try:
    with open("./mqtt/obstacles.pickle", "wb") as saveFile:
        pickle.dump(obstacles,saveFile)
except Exception as e:
    print("attempt failed:",e)
    print("Performing death-cry.")
    publishRegistry()
else: 
    print("attempt success.")

print("disconnecting from mqtt server")
mqttc.loop_stop()