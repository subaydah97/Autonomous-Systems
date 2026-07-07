# A. Creating docker container
```bash
docker compose up -d
```
# B. Start the obstacle registry
1. Ensure you have the following dependencies:
    1. python Paho mqtt 2.1.0
    2. python3
2. Enter the mqtt/obstacle_registry directory
3. run ``` python ./obstacleRegistry.py ``` to start the obstacle registry
# C. Start the webot simulation
1. Ensure you have the following dependencies:
    1. python Paho mqtt 2.1.0
    2. python3
2. Open the "road.wbt" in webot_sim_env