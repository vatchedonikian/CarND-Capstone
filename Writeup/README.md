## 1. Settings for testing in 'Test Lot' environment in Simulator in v1.3

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

## 2. clean logs
```
    rosclean check
    rosclean purge
```

## 3. General Parameters
