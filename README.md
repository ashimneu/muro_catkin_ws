# setup

Raspberry Pi running Raspian Jessie

Install ROS - http://wiki.ros.org/indigo/Installation/UbuntuARM

Install Aruco 1.3.0

put Muro_SFTP/muro_catkin_ws/ on the pi

#### Terminal 1 on Pi:
```
rosrun raspicam raspicam_raw_node _framerate:=30 _quality:=100 _width:=640 _height:=480
```
#### Terminal 2 on Pi:
```
rosservice call /camera/start_capture 
rosrun aruco_node aruco_node
```