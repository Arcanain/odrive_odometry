# odrive_odometry

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
rosrun ros_odrive key_teleop.py
```
