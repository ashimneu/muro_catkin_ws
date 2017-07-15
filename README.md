# setup

Raspberry Pi running Raspian Jessie

Install ROS - http://wiki.ros.org/indigo/Installation/UbuntuARM

#### Install Aruco 1.3.0
```
wget -O aruco.tgz https://sourceforge.net/projects/aruco/files/OldVersions/aruco-1.3.0.tgz/download &&
tar -xvzf aruco.tgz &&
cd aruco-1.3.0/ &&
mkdir -p build && cd build &&
cmake .. &&
make &&
sudo make install
```

#### Clone and Build
```
git clone https://github.com/ackhoury/muro_catkin_ws.git &&
cd muro_catkin_ws/ &&
catkin clean &&
catkin_make -j4
```

#### Terminal 1 on Pi: Starting Raspicam Node
```
rosrun raspicam raspicam_raw_node _framerate:=30 _quality:=100 _width:=640 _height:=480
```

#### Terminal 2 on Pi: Start Raspicam, Start Aruco Node
```
rosservice call /camera/start_capture 
rosrun aruco_node aruco_node
```

#### Terminal 3 on Pi: Start PWM Node
```
rosrun pi_controller pi_pwm.py
```

#### Terminal on Pi/Computer: Start Motor Handler
```
rosrun pi_controller motor_handler_aruco.py
```
