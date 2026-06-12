# Protos
## Chariot
```proto
    field SFVec3f translation 0 0 0
    field SFRotation rotation 0 0 1 0
 
    field SFString name "defaultName"
    field SFString controller "./topath"
    field SFString controller_arguments ""
   
    field SFFloat turret_position_horizontal 0
    field SFFloat turret_position_vertical 0
    field SFFloat wheel_position_left 0
    field SFFloat wheel_position_right 0
```

# Entities
## Beacon
- Radio emitter "emitter"
## Chariot
Ouputs
- Motors "rm_left" & "rm_right"
- Laser turret motors: "turret horizontal rotation motor" & "turret vertical rotation motor"

Inputs
- Receiver "receiver"
- Compass "compas

## Supervisor
