# Dependencies
https://pypi.org/project/paho-mqtt/ v:"2.1.0"
# Protos
## Chariot
```proto
    field SFVec3f translation 0 0 0
    field SFRotation rotation 0 0 1 0
 
    field SFString name "defaultName"
    field SFString controller "./topath"
   
    field SFFloat turret_position_horizontal 0
    field SFFloat turret_position_vertical 0
    field SFFloat wheel_position_left 0
    field SFFloat wheel_position_right 0
```
# Controllers
## telemetryListener
This controller subscribes to a localhost mqtt server, and subscribes to "bot/x" where x is the robot's name.
It will receive and parse telemetry data and update itself accordingly.
## SuperVisor
Supervisor with the controller "superVisor" is used to create digital twins of robots connected to the network.
## Telemetry tester
Any robot with the "telemetryTest" controller will use paho.mqtt to connect to a localhost mqtt server. And send the telemetryTestData.json file located in "./webot_sim_env/libraries/telemetryTestData.json"