import pickle
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, reason_code, properties):
    print(f"obstacle_registry: Connected with result code {reason_code}")
    client.subscribe("OR/COMPLETE_REGISTRY")

def on_message(client, userdata, msg):
    decoded_payload = msg.payload.decode("utf-8")
    print(msg.topic+':"'+ decoded_payload +'"')

mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect(HOSTNAME, 1883, 60)

mqttc.loop_start()

persistFlag = True
while persistFlag:
    1 == 1

mqttc.loop_stop()