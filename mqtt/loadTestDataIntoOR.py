import paho.mqtt.publish as publish

HOSTNAME = "localhost"


testfile = open("obstacleTestData.json","r")
testText = testfile.read()

publish.single("OR/COMMANDS", f"CLEAR", hostname=HOSTNAME)
# Load telemetry from file: For testData in parsedJson: 
for obstacleData in range(4):
    publish.single("OR/NEW", testText, hostname=HOSTNAME)