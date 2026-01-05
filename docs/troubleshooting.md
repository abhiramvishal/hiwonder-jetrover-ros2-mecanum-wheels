# Troubleshooting — JetRover ROS2

This section lists common issues you may encounter when working with ROS2
on the Hiwonder JetRover and how to resolve them.

---

## ROS2 commands not found

Ensure ROS2 is sourced:

```bash
source /opt/ros/humble/setup.bash
````

If using a workspace, also source:

```bash
source ~/jetrover_ws/install/setup.bash
```

---

## No topics visible

Check that robot bring-up or camera launch is running:

```bash
ros2 node list
ros2 topic list
```

If the list is empty, the robot core services are likely not started.

---

## Camera stream not visible

1. Confirm camera launch is running
2. Find camera topics:

```bash
ros2 topic list | grep -i image
```

3. View stream:

```bash
rqt_image_view
```

---

## Robot does not move

* Confirm the correct velocity topic exists (commonly `/cmd_vel`)
* Test stop first, then low-speed forward:

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

* Ensure no emergency stop is active

---

## Commands run but robot does nothing

* Verify you are running commands **on the robot**, not the laptop
* Ensure you are connected via NoMachine
* Check that the correct ROS_DOMAIN_ID is being used (if modified)

---

## Network-related notes

* ROS2 runs fully on the robot
* Laptop is only for remote access
* No ROS2 networking configuration is required unless you later run nodes on the laptop

---

## Tip

If something behaves unexpectedly:

```bash
ros2 node list
ros2 topic list
ros2 topic info <topic_name>
```

These three commands solve most issues.

```