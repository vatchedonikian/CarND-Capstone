### 1. Settings for testing in 'Test Lot' environment in Simulator in v1.3

```    
    source devel/setup.sh
    #cd launch && ls
    #cat styx.launch
    #cd ..
    cd src/waypoint_loader && ls
    cd launch && ls
    nano waypoint_loader.launch
    ##change 'wp_yaw_const.csv' to 'churchlot_with_cars.csv'
    ##change velocity value to 40 and save
```

### 2. clean logs
```
    rosclean check
    rosclean purge
```

### 3. General Parameters
Copied from Terminal 

    SUMMARY
    ========

    PARAMETERS
    * /dbw_node/accel_limit: 1.0
    * /dbw_node/brake_deadband: 0.2
    * /dbw_node/decel_limit: -5.0
    * /dbw_node/fuel_capacity: 0.0
    * /dbw_node/max_lat_accel: 3.0
    * /dbw_node/max_steer_angle: 8.0
    * /dbw_node/steer_ratio: 14.8
    * /dbw_node/vehicle_mass: 1080.0
    * /dbw_node/wheel_base: 3
    * /dbw_node/wheel_radius: 0.335
    * /pure_pursuit/linear_interpolate_mode: True
    * /rosdistro: kinetic
    * /rosversion: 1.12.7
    * /traffic_light_config: <...>
    * /waypoint_loader/path: /home/student/Des...
    * /waypoint_loader/velocity: 10

    NODES
    /
       dbw_node (twist_controller/dbw_node.py)
       pure_pursuit (waypoint_follower/pure_pursuit)
       styx_server (styx/server.py)
       tl_detector (tl_detector/tl_detector.py)
       unity_simulator (styx/unity_simulator_launcher.sh)
       waypoint_loader (waypoint_loader/waypoint_loader.py)
       waypoint_updater (waypoint_updater/waypoint_updater.py)
