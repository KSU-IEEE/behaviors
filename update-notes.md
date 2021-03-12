# What each behavior needs updated before the competition: 

## Future Arm subs and pubs:  
subs:
- arm/grabBlock - std_msgs float64
    - will have the arm grab a ghost at an angle in relation to the arms initial position (some function of heading)
    - arm will just go and grab, will have to position bot first
- arm/scan - behaviors polar_coord 
    - will look at the location sent in polar
    - prompts arm/distance
- arm/reset - std_msgs Bool
    - sets arm to starting position

pubs:
- arm/done - std_msgs Bool
    - sent when the arm is done moving
- arm/distance - std_msgs Float64
    - prompted by arm/scan
    - holds the value read by the distance sensor on the arm in inches

## Motor subs and pubs:
subs:
- /bot/move - std_msgs Float32 
- /bot/turnLeft - std_msgs Bool
- /bot/turnRight - std_msgs Bool
- /bot/turn180 - std_msgs Bool

Pubs:
- /bot/doneMove = std_msgs Bool

## Random new Publishers added
- grabbedGhost - std_msgs::Int8
    - sent when a ghost is picked up, need to update a_star to sub to it

## Random Params:
- /arm/length
    - the length of the physical arm
- /arm/baseHeight
    - the height that the arm sits on the bot from the ground
- /arm/baseLength
    - the length of the bottom joint of the arm
- /arm/sensorDist
    - the radius of the sensor on the arm
- starting_x
    - the start position
- starting_y
    - the start position
- distanceTolerance
    - the tolerance for checking the distances on the arm sensor
- botWidth
    - the width of thebot


## init:
- TBD

## return to start
- add path to reset arm