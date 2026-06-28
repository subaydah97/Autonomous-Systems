# Dependencies
import json
import pickle
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

# Obstacles fields
# id: assigned at creation. Used to count total obstacles

# Functions
def getObstacleCount():
    #Get the ID of the latest obstacle in the file.
    return 0

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"obstacle_registry: Connected with result code {reason_code}")
    client.subscribe("OR/#")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    try:
        decoded_payload = msg.payload.decode("utf-8")
        #print(msg.topic+':"'+ decoded_payload +'"')
    except:
         print(msg.topic+':'+"payload unsafe to print to terminal")

    match msg.topic:
            case "OR/COMMANDS":
                match decoded_payload.upper():
                    case "TERM":
                        print("TERM command received, lowering persist flag")
                        global persistFlag; persistFlag = 0
                    case "SPIT":
                        print(obstacles)
                        publish.single("OR/COMPLETE_REGISTRY", json.dumps(obstacles), hostname=HOSTNAME)
            case "OR/NEW":
                global obstacleCount
                id = obstacleCount
                obstacleCount += 1
                
                obstacles.append((id,str(msg.payload)))
                print(f'added obstacle:"{obstacles[-1]}"')

            case "OR/REM":
                try:
                    targID = int(decoded_payload)
                    for obstacle in obstacles:
                        if obstacle[0] == targID: print(f'removing obstacle:"{obstacle}"'); obstacles.remove(obstacle)
                except:
                        print("Obstacle removal error:"+'"'+decoded_payload+'"')

# Global variables
obstacles = []
obstacleCount = getObstacleCount()

persistFlag = 1

HOSTNAME = "localhost"

# Main
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect(HOSTNAME, 1883, 60)

mqttc.loop_start()

while persistFlag > 0:
    1 == 1

# Cleanup
print(f"exiting with code:{persistFlag}")

print("disconnecting from mqtt server")
mqttc.loop_stop()

print("attempting to save obstacles to file")
try:
    with open("./mqtt/obstacles.pickle", "wb") as saveFile:
        pickle.dump(obstacles,saveFile)
except Exception as e:
    print("attempt failed:",e)
else: 
    print("attempt success.")

