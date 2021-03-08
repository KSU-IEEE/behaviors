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


## attack_ghosts:
- Arm publishers and Subscribers
- Update locations where it is safe to grab the ghosts 
- read in armBaseHeight, sensorDist, and armLength
- add remove_wall publisher

## check_for_ghosts:
- update to just check for locations with the arm's distance sensor

## end:
- it's perfoect

## ground_search:
- Yeah just clean this up, need to consider initial search conditions
- update safe locations
- using arm for search? 

## init:
- yeah this one is fine as well

## reach_search:  
- need to update everything with the arm
    - add publishers and subscribers for arm (if not already added in ground_search)
- update search algorithm
    - don't need to move for it, the arm can pivot
    - move before grabbing, have to add some logic to get the position correct

## return to start
- add path to reset arm