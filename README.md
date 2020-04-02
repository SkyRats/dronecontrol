# dronecontrol

:robot: Control Nodes for PX4 compatible drones

### Requirements
* ROS
* MAVROS

### Launching mavros
* via USB telemetry `roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0:57600"`
* simulation `roslaunch mavros px4.launch fcu_url:="udp://:14550@127.0.0.1:14557"`


## Launching swarm simulations
* simulation - spawns two iris drones and runs mavros on both
```
cd src/Firmware 
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

roslaunch dronecontrol test.launch
```

* run box node (to update box position and apply forces to both MAVs)
`rosrun dronecontrol update_box_position.py`

* run control nodes
Example: `rosrun dronecontrol simple_sm.py`