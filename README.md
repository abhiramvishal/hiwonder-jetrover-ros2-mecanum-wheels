# Hiwonder JetRover (Mecanum Wheels) — ROS2 On-Robot Guide

This repository is a **course-style ROS2 guide** for working with the **Hiwonder JetRover with mecanum wheels** (holonomic drive).

All ROS2 code is executed **directly on the robot**, while the laptop connects via **NoMachine** over the same Wi-Fi network.

---

## What’s different with mecanum wheels?
Unlike standard wheels, mecanum wheels support **omnidirectional movement**:
- Forward / backward (x-axis)
- Strafe left / right (y-axis)
- Rotate (z-axis)

This repo adds a clear guide for controlling **sideways movement** using ROS2 `Twist` messages.

---

## What this repository covers
- ROS2 workspace setup on the JetRover
- Camera bring-up and topic inspection
- Mecanum movement control (forward, strafe, rotate, stop)
- Creating custom ROS2 Python packages for motion control
- Foundation for AI-driven behaviors (vision, tracking, choreography)

---

## Setup assumptions
- Robot and laptop are connected to the **same Wi-Fi**
- You connect to the robot using **NoMachine**
- ROS2 runs **on the robot**, not on the laptop

---

## Quick start (run on the robot)

```bash
mkdir -p ~/jetrover_ws/src
cd ~/jetrover_ws
colcon build
source install/setup.bash
```

Check ROS2:
```bash
ros2 topic list
```

---

## Learning path
Follow these files in order:

1. `docs/01-setup.md`
2. `docs/02-ros2-basics.md`
3. `docs/03-camera.md`
4. `docs/04-movement-mecanum.md`
5. `docs/05-build-your-own-package.md`
6. `docs/troubleshooting.md`

---

## License
© Macquarie University — Academic Coursework  
Intended for educational and demonstration purposes.
