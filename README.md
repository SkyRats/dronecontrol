\\\\\\\\   \\ //   \\ // \\\\\\\   \\\\   \\\\\\\\ \\\\\\\\\
\\         \\//     \//  \\    \   \\\\\\     \\\   \\
\\\\\\\\   \\\\     //   \\\\\\\  \\\   \\\    \\\   \\\\\\\\
      \\   \\ \\   //    \\\\\    \\\\\\\\\\    \\\         \\ 
\\\\\\\\   \\  \\ //     \\  \\  \\\\     \\\\   \\\  \\\\\\\\\

Intelligent drones team of Polytechnic School of the University of Sao Paulo

Welcome to DroneControl package!

This is the package is responsible for controlling the aircraft. It's an easy way
to send/receive ros messages. 
Notice that this package requires MAVROS installed on your computer to work. 
Furthermore, we think you must install Gazebo simulator, because this pack is design to work better with him.

Instructions:

1. Firstly, you must install ROS (we recommend kinetic version). We suggest you follow the official tutorial, disponible on WikiRos.
2. Now, you are ready to install Mavros. Mavros is a ROS package responsible for the integration between ROS and MAVLink messages.
Therefore, you can integrate your ROS packages easily with PX4, Ardupilot or another flight stacks that communicate by mavlink protocol.
In this case, the Skyrats development team uses PX4 Flight Stack to control our aircrafts.
3. Finally, you can install dronecontrol.

Currently version: 2.0.0


# dronecontrol

:robot: Control Nodes for PX4 compatible drones

### Requirements
* ROS
* MAVROS

### Launching mavros
* via USB telemetry `roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0:57600"`
* simulation `roslaunch mavros px4.launch fcu_url:="udp://:14550@127.0.0.1:14557"`




