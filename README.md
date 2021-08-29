# odrive_odometry

Write the contents of motor.ino to the Arduino

- Terminal 1
```bash
rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=115200
```
- Terminal 2
```bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
rosrun odrive_odometry odometry_publisher
```

- Terminal 3
```bash
rosrun odrive_odometry key_teleop.py
```
