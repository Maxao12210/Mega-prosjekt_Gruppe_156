# Robot colored cube definer

## Assumptions

- Ubuntu 22.04 (Noble) or similar
- UR CB3 (UR3) robot
- ROS 2 Jazzy installed
- Webkamera connected to `/dev/videox`
- An external communication device with `ur_drivers` and `moveit` installed and connected to the ur_robot with corresponding IP-address and gateway 
- Required ROS-packages and tools: 
  - moveit/move_group_interface
  - moveit/planning_scene_interface
  - rclcpp
  - Cmake (should be included with ros2 jazzy)

## Building the project in Ubuntu:
Open a terminal:
```bash
git@github.com:Maxao12210/Mega-prosjekt_Gruppe_156.git
```
Open new terminal:
```bash
cd [path/to/workspace]/Mega-prosjekt_Gruppe_156
colcon build
source install/setup.bash
```

## To run the program 
Connect the robot, external communication device and your PC via LAN and ensure the corresponding IP and gateway are correct.

In new terminal on your PC and the device connected to the robot:
```bash
export ROS_DOMAIN_ID:=<your_desired_id>
```
### On the external communication device:

Open new terminal:
```bash
ros2 launch ros2 launch ur_robot_driver ur_control.launch.py ur_type:=urX robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=false
initial_joint_controller:=scaled_joint_trajectory_controller launch_rviz:=false headless_mode:=false
```
Open new terminal:
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=urX launch_rviz:=true
```

### On your PC
Connect the webcamera to your pc via usb

In a terminal:
```bash
cd [path/to/workspace]/Mega-prosjekt_Gruppe_156
source install/setup.bash
ros2 run robot_moveit robot_moveit
```
In new terminal:

Check your video device:
```bash
ls /dev/video*
```
It should result in something like this:
`/dev/videox  /dev/videoy
`

Run camera node with the video device corresponding to the webcamera:
```bash
ros2 launch camera_node box_detection.launch.py video:=/dev/videox
```

The robot should move to it's home position and then to the camera reference position 1 and 2 until all three cubes are detected. Afterwards the robot should move over each cube position with the first reference position between each of them. 