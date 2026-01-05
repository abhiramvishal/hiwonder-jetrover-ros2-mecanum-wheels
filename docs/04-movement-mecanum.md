
# 04 — Mecanum Wheel Movement (Holonomic Drive)

Mecanum wheels support **omnidirectional movement**, meaning the robot can:
- Move forward / backward (x-axis)
- Move left / right (y-axis)  ← this is the key difference
- Rotate in place (z-axis)

All commands below are executed **on the robot**.

---

## 1) Identify the velocity topic

List topics:
```bash
ros2 topic list
````

Look for:

```text
/cmd_vel
```

Confirm the message type:

```bash
ros2 topic type <cmd_vel_topic>
```

Expected:

```text
geometry_msgs/msg/Twist
```

---

## 2) Understanding Twist for mecanum

`Twist` has:

* `linear.x` → forward/backward
* `linear.y` → left/right (strafe) ✅ mecanum feature
* `angular.z` → rotate left/right

---

## 3) Safety first (test stop)

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}"
```

---

## 4) Basic movement commands (CLI)

### Forward

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.10, y: 0.0}, angular: {z: 0.0}}"
```

### Backward

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: -0.10, y: 0.0}, angular: {z: 0.0}}"
```

### Strafe right (move sideways)

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.10}, angular: {z: 0.0}}"
```

### Strafe left (move sideways)

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.10}, angular: {z: 0.0}}"
```

### Rotate left (counter-clockwise)

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.5}}"
```

### Rotate right (clockwise)

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: -0.5}}"
```

### Stop

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}"
```

---

## 5) Combined motion examples

### Forward + rotate (curved path)

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.10, y: 0.0}, angular: {z: 0.3}}"
```

### Strafe + rotate (sideways arc)

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.10}, angular: {z: 0.3}}"
```

### Diagonal move (forward + strafe)

```bash
ros2 topic pub --once <cmd_vel_topic> geometry_msgs/msg/Twist "{linear: {x: 0.08, y: 0.08}, angular: {z: 0.0}}"
```

---

## 6) Notes & calibration

* If strafe direction feels reversed, swap the sign of `linear.y`
* Start with small speeds: **0.05 to 0.10**
* Ensure robot is on flat surface and wheels have traction
* If robot jitters while strafing, reduce speed and check wheel alignment

---

## 7) Next step

After confirming manual movement works, implement a ROS2 package that provides:

* `move_forward.py`
* `strafe_left.py`
* `strafe_right.py`
* `rotate_left.py`
* `rotate_right.py`
* `stop.py`

```
