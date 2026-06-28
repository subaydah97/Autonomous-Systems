import json
import warnings
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

persistFlag = True
HOSTNAME = "localhost"

def on_connect(client, userdata, flags, reason_code, properties):
    print(f"obstacle_registry: Connected with result code {reason_code}")
    client.subscribe("OR/COMPLETE_REGISTRY")

def on_message(client, userdata, msg):
    decoded_payload = msg.payload.decode("utf-8")

    obstacles = json.loads(decoded_payload)
    print(type(obstacles),obstacles)

    for obstacle in obstacles:
        print(f"obstacle id_type:{type(obstacle["id"])}:{obstacle["id"]}\tobstacle payload_type:{type(obstacle["payload"])}:{obstacle["payload"]}")
        
        # Check for obstacled that would be removed in base test.
        if obstacle["id"] == 1 or obstacle["id"] == 3 or obstacle["id"] == 7:
            warnings.warn("Obstacle id detected that would have been removed in testBase.")
        
        global persistFlag; persistFlag = False


mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect(HOSTNAME, 1883, 60)

mqttc.loop_start()

publish.single("OR/COMMANDS", f"SPIT", hostname=HOSTNAME)


while persistFlag:
    1 == 1

mqttc.loop_stop()