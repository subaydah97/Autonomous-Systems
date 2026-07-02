import time
import json
import paho.mqtt.publish as publish

HOSTNAME = "localhost"

publish.single("OR/COMMANDS", f"CLEAR", hostname=HOSTNAME)

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

publish.single("OR/MOV", f"{json.dumps(obstacle)}", hostname=HOSTNAME)

update_freq = 8

obstacle["payload"].pop("position")

while 1 == 1:
    publish.single("OR/MOV", f"{json.dumps(obstacle)}", hostname=HOSTNAME)

    obstacle["payload"]["rotation"]["radians"] += 1/update_freq
    obstacle["payload"]["rotation"]["x"] += 1/update_freq
    obstacle["payload"]["rotation"]["z"] -= 2/update_freq
    time.sleep(1/update_freq)
    pass
publish.single("OR/REM", f"{obstacle["id"]}", hostname=HOSTNAME)