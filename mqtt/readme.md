# Dependencies
- Python version as of 06/2026
- paho-mqtt 2.1.0
# Brief
This readme outlines the obstacle registry program.
The program is concerned with:
- Keeping track of obstacles
- Sharing obstacles using JSON.
The program is explicitly not concerned with the following.:
- Processing obstacles
- Ensuring each robot has a full copy of the registry.

## Input, Persistent registry
When gracefully terminated, the program will attempt to store the registry as a pickle file. If such a file is present at the start of runtime, it will attempt to load it.

## Output, JSON
The program can be prompted to output the full registry to topic "OR/COMPLETE_REGISTRY" as an array of JSON objects, encoded in utf-8.

# Adding obstacles
Publish to "OR/NEW"
Payload will be stored as bytes.

# Removing obstacles
Publish to "OR/REM"
The payload should be an integer represented as a utf-8 string. 

# Auxiliary ommands
Publish to "OR/COMMANDS"

## CLEAR
Clears the obstacle registry.

## SPIT
Instructs the obstacle registry to publish the full registry on "OR/COMPLETE_REGISTRY". Usefull for robots with an out of date obstacle cash.

## TERM
Terminates the obstacle registry routine gracefully. It will attempt to store the registry into a file. If file storage fails, it will perform a death cry, and publish the full registry to "OR/COMPLETE_REGISTRY".

# Performing tests
## Setup
1. Ensure the mqtt server is running, and all imports are fullfulled.
2. Run obstacleRegistry.py

## testBase
Tests for all basic functionality. See test behavior below:
1. Instruct the OR to be CLEARed.
2. Adds 10 obstacles with dummy data.
3. Clears obstacles with IDs: 1,3,7
4. Instructs the OR to SPIT.
5. Instructs the OR to terminate
Expected results is that the obstacles list contains obstacle IDs 0 through 10, missing 1,3 and 7. 

## testSpit
Test for receiving COMPLETE_REGISTRY using json. 
1. Connects to the topic "OR/COMPLETE_REGISTRY"
2. Instructs the OR to SPIT.
3. Attempts to swallow the spit data.
4. Expect id_type: int, payload_type:string