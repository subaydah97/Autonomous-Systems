import time
import json
import paho.mqtt.publish as publish
import math
import sys

HOSTNAME = "localhost"

#publish.single("OR/COMMANDS", f"CLEAR", hostname=HOSTNAME)

unit = {"id":101}
unit = {  
                        "position": 
                        {
                            "x": 0.2,
                            "y": 0.3
                        },
                        "rotation": 
                        {
                        "radians_from_north": 0.5
                        },
                        "wheels": 
                        {
                        "radian_position_wheel_left": 1.0,
                        "radian_position_wheel_right": 2.0
                        },
                        "laser_turret": 
                        {
                        "radian_position_horizontal": 1.0,
                        "radian_position_vertical": 2.0
                        } 
                       }

publish.single("BOT/767", f"{json.dumps(unit)}", hostname=HOSTNAME)

update_freq = 12

time_step = 999

height_offset = 0.4

while 1 == 1:
    time_step += 1

    publish.single("bot/767", f"{json.dumps(unit)}", hostname=HOSTNAME)

    unit["rotation"]["radians_from_north"] += 1/update_freq

    unit["position"]["z"] = math.sin(1/update_freq * time_step) + height_offset
    unit["position"]["y"] = math.tan(1/update_freq * time_step)/4 
    unit["position"]["x"] = math.cos(2/update_freq * time_step)*5 + 5

    time.sleep(1/update_freq)
    
