# Flags
# -s, silent. Use this flag to prevent this program from calling spit
import paho.mqtt.publish as publish
import json
import sys

HOSTNAME = "localhost"

fileWithTestData = open("obstacleCommunicationTestData.json","r")
obstacles = json.load(fileWithTestData)

#print(f"parsedJson_{type(obstacles)}={obstacles}")
print("Clearing OR")

publish.single("OR/COMMANDS", f"CLEAR", hostname=HOSTNAME)

# Load telemetry from file: For testData in parsedJson: 
for obstacle in obstacles:
    payload = obstacle["payload"]
    print(f"Loading payload:{type(payload)}={payload}")
    publish.single("OR/NEW", json.dumps(payload), hostname=HOSTNAME)

if not sys.argv[1:].count("-s"):print("Initiating spit");publish.single("OR/COMMANDS", f"SPIT", hostname=HOSTNAME)