import pickle
import paho.mqtt.client as mqtt

# Global variables
obstacles = []
persistFlag = True

# Obstacles fields
# id: assigned at creation. Used to count total obstacles

# Functions
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"obstacle_registry: Connected with result code {reason_code}")
    client.subscribe("OR/#")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    decoded_payload = msg.payload.decode("utf-8")
    
    print(msg.topic+':"'+ decoded_payload +'"')

    if msg.topic == "OR/COMMANDS" and decoded_payload == "KILL":
        persistFlag = False



# Main
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect("localhost", 1883, 60)

mqttc.loop_start()

while persistFlag:
    1 == 1

# Cleanup
mqttc.loop_end()