# Dependencies
import json
import pickle
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

# Global variables
obstacles = []
    ## Obstacles fields
    ## 'id': assigned at creation. Used to count total obstacles
    ## 'payload': bytes
# The amount of obstacles that have been added to the registry. Resets when the registry is cleared.
obstacleCount = 0
# Used to control program termination
persistFlag = 1

HOSTNAME = "localhost"

# Functions
def publishRegistry(obstacles):
    publish.single("OR/COMPLETE_REGISTRY", json.dumps(obstacles), hostname=HOSTNAME)
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"obstacle_registry: Connected with result code {reason_code}")
    client.subscribe("OR/#")


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
                        print(f"obstacleCount:{obstacleCount}")
                        for obstacle in obstacles:
                            print(obstacle)
                        publishRegistry(obstacles)
            case "OR/NEW":
                id = obstacleCount
                obstacleCount += 1
                
                obstacles.append({"id":id,"payload":str(msg.payload)})
                print(f'added obstacle:"{obstacles[-1]}"')

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
    publishRegistry(obstacles)
else: 
    print("attempt success.")

print("disconnecting from mqtt server")
mqttc.loop_stop()