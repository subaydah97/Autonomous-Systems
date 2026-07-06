"""tray_supervisor controller."""
# Example code provided by 1069459@hr.nl

from controller import Supervisor

robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

root_node = robot.getRoot()
root_list = root_node.getField('children')

#get node definition except for def declaration
trayString = robot.getFromDef("TRAY").exportString()
trayString = "Solid {\n"+trayString[trayString.find("\n"):]

luggageString = robot.getFromDef("LUGGAGE").exportString()
luggageString = "Solid {\n"+luggageString[luggageString.find("\n"):]

num_trays = 0
def spawnTrayWithLuggage(x,y,z):
    global num_trays
    num_trays += 1
    traydef = "TRAY"+str(num_trays)
    lugdef = "LUG"+str(num_trays)
    
    #add solid node with numbered DEF
    root_list.importMFNodeFromString(-1,"DEF "+traydef+" "+trayString)
    #set position
    traypos = robot.getFromDef(traydef).getField("translation")
    traypos.setSFVec3f([x,y,z])
    
    root_list.importMFNodeFromString(-1,"DEF "+lugdef+" "+luggageString)
    contpos = robot.getFromDef(lugdef).getField("translation")
    contpos.setSFVec3f([x,y,z+0.3])
    return num_trays

spawnTrayWithLuggage(5,-2,1.2)


while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
