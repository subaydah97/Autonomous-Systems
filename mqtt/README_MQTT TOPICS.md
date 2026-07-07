# MQTT topics
# Obstacle registry
### Obstacle payload
When handling obstacle payload, no guarrantee can be expected as to what fields are and aren't present in the payload.

Implemented fields:
- Position 
- Rotation

Example below:
```json
{
    "position": {
        "x": 1.0,
        "y": 2.0,
        "z": 0.5
    },
    "rotation": {
        "x": 0.0,
        "y": 0.0,
        "z": 1.0,
        "radians": 1.2
    },
    "shape":"rectangle 4 2 1"
}
```
### Obstacle wrapper
For communicating obstacles, a identifier is used.
```json
"id":<obstacle_id>,
"payload":<obstacle_payload>
```

## OR/NEW
New obstacle payloads are sent here.

## OR/MOV
Publish an obstacle wrapper with an updated payload here to alter an obstacle.
```json
{
    "id":<id>,
    "payload":<payload>
}
```

## OR/REM
```<obstacle id>```
Remove the obstacle with that id.

## OR/COMPLETE\_REGISTRY
Receives the OR in the format outlined below:

```json
[
  {
    "id": 0,
    "payload": {
      "position": {
        "x": 0.1,
        "y": 0.2,
        "z": 0.5
      },
      "rotation": {
        "x": 0,
        "y": 0,
        "z": 1,
        "radians": 1.2
      },
      "shape": "rectangle 4 2 1"
    }
  },
  {
    "id": 1,
    "payload": {
      "position": {
        "x": 0.5,
        "y": 0.6,
        "z": 0.7
      },
      "rotation": {
        "x": 0.1,
        "y": 0.2,
        "z": 1.3,
        "radians": 1.4
      },
      "shape": "sphere 0.9"
    }
  },
    {
    "id": 2,
    "payload": {
      "position": {
        "x": 0.7,
        "y": 0.6,
        "z": 0.5
      }
    }
  }
]
```

## OR/COMMANDS
```<command_name>```
### Clear
Name:"CLEAR"

Clears the obstacles loaded into the OR.
### Spit
Name:"SPIT"

Instrucst the OR to publish it's loaded obstacles to "OR/COMPLETE_REGISTRY"



# Swarm unit topics
replace <id> with the bots id.
## BOT/<id>
Telemetry data is published here, in the following format.

Telemetry data handling must allow for the independent handling of each field. For example so that a telemetry message only updates the "rotation" 

```json
{
  "position": {
    "x": 0.2,
    "y": 0.3
  },
  "rotation": {
    "radians_from_north": 0.5
  },
  "wheels": {
    "radian_position_wheel_left": 1.0,
    "radian_position_wheel_right": 2.0
  },
  "laser_turret": {
    "radian_position_horizontal": 1.0,
    "radian_position_vertical": 2.0
  }
}
```

## BOT/<id>/COMMANDS
```
<command_name> <arguments, seperated by spaces>
```
### override
Name:"OVERRIDE"

Arguments: ```OVERRIDE <identifier> <new value> <identifier> <new value>...```

Takes any number of arguments. 

```
Identifier  new value   description    
'X'         float       Override the latestX of the robots position.
'Y'         float       Override the latestY..
'Z'         float       Override the latestZ..
'R'         float       Override the latestRot variable.
'T'         int         Override the forward ticks the robot has moved on both motors.
'l'         int         Override the forward ticks the robot has moved on the left motor.
'r'         int         Override the forward ticks the robot has moved on the right motor.
```

### Estop
Name:"ESTOP
Arguments:```ESTOP```

Raises the estop flag.