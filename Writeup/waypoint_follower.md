## Waypoint Follower Node Overview

## Subscribers and Publishers 
### Subscribers
* /final_waypoints (styx_msgs/Lane)
* /current_pose (geometry_msgs/PoseStamped: header, pose.position, pose.orientation) 
* /current_velocity (geometry_msgs/TwistStamped: header, twist.linear, twist.angular) 

### Publisher
* /twist_cmd (geometry_msgs/TwistStamped)

## Description of the Work Flow
Assumed that the pose of the vehicle is known and the waypoint data is obtained from the topic /final_waypoints.

    while (ros::ok()) repeat a)-e)

    a) calculate a lookahead distance, which will be decided by current velocity and predefined related ratios. Its range is [6, 10*current_velocity].

    b) according to obtained lookahead distance, search for the next reference waypoint. It is the waypoint in final_waypoints whose distance to current_pose is "almost"  the lookahead distance.

    c) use interpolation method find out an exact waypoint so that its distance to current_pose is exactly the lookahead distance.

    d) publish the velocity /final_waypoints.waypoints[0].twist.twist.linear as velocity and the corrected angular according to the above-calculated info, /final_waypoints.waypoints[0].twist.twist.angular as angular

    e) sleep
    
## Structure
```
pure_pursuit.cpp
    |
    |-- pure_pursuit_core.h + pure_pursuit_core.cpp
           |
           |-- libwaypoint_follower.h + libwaypoint_follower.cpp
```

## Note 

The interesting step is d), which basically always publish /final_waypoints.waypoints[0] info to /twist_cmd. I think the better way is to replace "0" with the nearest waypoint to the current_pose. 
