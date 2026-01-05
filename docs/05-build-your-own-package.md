# 05 — Build Your Own ROS2 Package

This section explains how to create a simple ROS2 Python package to control
the JetRover using velocity commands.

The goal is to understand how movement logic is wrapped inside a ROS2 node.

---

## Create a ROS2 Python package

From inside the workspace `src/` directory:

```bash
cd ~/jetrover_ws/src
ros2 pkg create --build-type ament_python jetrover_motion --dependencies rclpy geometry_msgs
````

This creates a basic ROS2 package structure.

---

## What this package will contain

You will implement:

* A ROS2 node that publishes `Twist` messages
* A **move** script (start moving forward)
* A **stop** script (stop the robot)

These scripts publish messages to the velocity topic.

---

## Build the workspace

After adding or modifying code:

```bash
cd ~/jetrover_ws
colcon build
source install/setup.bash
```

---

## Running your package (example)

Once implemented, you should be able to run:

```bash
# ros2 run jetrover_motion move
# ros2 run jetrover_motion stop
```

(Exact script names depend on your implementation.)

---

## Notes

* Start with simple logic before adding automation
* Keep movement speeds low during testing
* This package can later be reused for AI-driven control

```
