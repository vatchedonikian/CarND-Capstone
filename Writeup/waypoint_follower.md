
## Structure
```
pure_pursuit.cpp
    |
    |--pure_pursuit_core.h + pure_pursuit_core.cpp
           |
           |- libwaypoint_follower.h + libwaypoint_follower.cpp
```

### waypoint_follower/include/libwaypoint_follower.h

This file mainly introduced a class WayPoints, which will be used to save current waypoints list. It also introduces several related functions, such as setPath, get all kinds of information functions from current waypoints list and isFront etc.
