import paho.mqtt.publish as publish

HOSTNAME = "localhost"
for obstacleData in range(10):
    publish.single("OR/NEW", f"DUMMY_DATA:{obstacleData}", hostname=HOSTNAME)
for obstacleID in [1,3,7]:
    publish.single("OR/REM", f"{obstacleID}", hostname=HOSTNAME)
publish.single("OR/COMMANDS", f"SPIT", hostname=HOSTNAME)

publish.single("OR/COMMANDS", f"TERM", hostname=HOSTNAME)