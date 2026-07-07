import time
import json
import paho.mqtt.publish as publish
import math
import sys

HOSTNAME = "localhost"

#publish.single("OR/COMMANDS", f"CLEAR", hostname=HOSTNAME)

obstacle = {"id":101}
obstacle["payload"] = {"position":{
                            "x":0,
                            "y":0,
                            "z":0.5
                                   },
                        "rotation":{
                            "x":0,
                            "y":0,
                            "z":1,
                            "radians":0
                        }
                                   }

publish.single("BOT/767", f"{json.dumps(obstacle)}", hostname=HOSTNAME)

update_freq = 12

time_step = 0

height_offset = 0.4

while 1 == 1:
    time_step += 1

    publish.single("OR/MOV", f"{json.dumps(obstacle)}", hostname=HOSTNAME)

    obstacle["payload"]["rotation"]["radians"] += 1/update_freq
    obstacle["payload"]["rotation"]["x"] += 1/update_freq
    obstacle["payload"]["rotation"]["z"] -= 2/update_freq

    obstacle["payload"]["position"]["z"] = math.sin(1/update_freq * time_step)/4 + height_offset
    obstacle["payload"]["position"]["y"] = math.cos(1/update_freq * time_step)/4 
    obstacle["payload"]["position"]["x"] = math.tan(1/4/update_freq * time_step)/4 

    time.sleep(1/update_freq)
    pass
publish.single("OR/REM", f"{obstacle["id"]}", hostname=HOSTNAME)