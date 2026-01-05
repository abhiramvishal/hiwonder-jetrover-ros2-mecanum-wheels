# 01 — Environment Setup (JetRover + ROS2)

This guide explains how to set up the ROS2 environment on the **Hiwonder JetRover**.
All commands in this section are executed **on the robot**, accessed via **NoMachine**.

---

## Assumptions
- JetRover is powered on and connected to Wi-Fi
- Laptop is on the same Wi-Fi network
- NoMachine connection to the robot is working
- ROS2 (recommended: Humble) is already installed on the robot

---

## Create a ROS2 workspace

Open a terminal **on the robot**:

```bash
mkdir -p ~/jetrover_ws/src
cd ~/jetrover_ws
colcon build
source install/setup.bash
````

---

## Auto-source the workspace (recommended)

So you don’t need to source every time:

```bash
echo "source ~/jetrover_ws/install/setup.bash" >> ~/.bashrc
```

Restart the terminal or run:

```bash
source ~/.bashrc
```

---

## Verify ROS2 is working

```bash
ros2 --version
ros2 topic list
```

If no errors appear, the ROS2 environment is ready.

---

## Notes

* ROS2 runs **entirely on the robot**
* The laptop is used only for remote access (NoMachine)
* Networking configuration is handled by the robot OS

---

Next → `docs/02-ros2-basics.md`

```
