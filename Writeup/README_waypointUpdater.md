## Waypoint Updater Node Overview

The waypoint updater node belongs to the planning system of a self-driving car. It is designed to publish a fixed number of waypoints ahead the vehicle with the correct target velocities for the car to follow. Naturally, the final published waypoints should depend on the status of the car itself and environments around it such as traffic lights and obstacles. As a result, it tells the car when and how to accelerate or deaccelerate depending on all those considerations.

## Input and Output / Subscribers and Publishers

### Input/Subscribers
    * /base_waypoints
    * /traffic_waypoint
    * /current_pose
    * /current_velocity
    
### Output/Publisher
    * /final_waypoints

## Waypoint Updater Node Implementation

### Idea

Suppose the base waypoints of a certain track are uploaded and received by the system and the car's position is successfully detected. The waypoint updating is then split into two sequence steps: waypoints updating regardless of their velocities and velocity updating for the updated waypoints. In case a stop line is set or one wishes the car can loop around the track, we designed a middle layer for adapting purpose.


In detail, if the current waypoints list is empty, then it is updated with the base waypoints that is subscribed from the traffic light detection node. If it is not empty, we find the closest waypoint in the current waypoint list to get the vehicles current position and get it index. If the index is equal to the list of the real lookahead values used for publishing (lookahead_wps), then the value of the lookahead_wps is doubled and the next set of way points are found using the new value and the current way point list is updated. If the index is not equal to zero or the lookahead, we just update the current waypoint list with the present set of waypoints. Next if the car is approaching the stop line we get the index of the final waypoint in the current waypoint list and declare it to a variable called stop_line. Using this, we decide on the velocities of the current way points and decide if the car must accelerate or deaccelerate or stop. If stop_line is less than zero, then it means that we are approaching a green light and instruct the car to accelerate. Else we tell it deaccelerate or stop depending on how far away the car is from the traffic light. If a red or yellow light is detected but if the current position of the car is too near to the stop light then we tell it to accelerate otherwise, the car deaccelerates and stops.


### Flowchart


### Code
