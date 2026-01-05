# 02 — ROS2 Basics (Minimum You Need)

This section introduces only the ROS2 concepts required to work with the
Hiwonder JetRover. The goal is practical understanding, not theory.

---

## What is ROS2 used for here?
ROS2 acts as the **communication layer** between:
- Sensors (camera)
- Control logic (your code)
- Actuators (robot movement)

---

## Core concepts

- **Node**  
  A running program (e.g., camera node, movement node)

- **Topic**  
  A named data stream (e.g., images, velocity commands)

- **Publisher**  
  Sends messages to a topic

- **Subscriber**  
  Receives messages from a topic

- **Message**  
  The data type sent over a topic

---

## Common ROS2 commands

List running nodes:
```bash
ros2 node list
````

List available topics:

```bash
ros2 topic list
```

Check topic message type:

```bash
ros2 topic type <topic_name>
```

Inspect topic details:

```bash
ros2 topic info <topic_name>
```

View topic data (use with care):

```bash
ros2 topic echo <topic_name>
```

---

## What matters for JetRover

For this robot, you mainly need to identify:

* **Camera topic** → provides image data
* **Velocity topic** → controls robot movement

These topics will be used in the next sections.

---

## Important note

All ROS2 commands in this repository are run **on the robot**.
The laptop is only used for remote access via NoMachine.

---
