# 03 — Camera Bring-Up and Viewing

This section explains how to bring up the JetRover camera and view the live
image stream using ROS2.

All commands are executed **on the robot**.

---

## Start the camera

Most JetRover systems provide a launch file for the camera.

Example (your actual command may differ):
```bash
# ros2 launch peripherals depth_camera.launch.py
````

If you are unsure, start the robot bring-up and continue with the steps below.

---

## Find the camera topic

List all topics:

```bash
ros2 topic list
```

Look for topics containing:

* `image`
* `rgb`
* `camera`
* `depth_cam`

A common example:

```text
/depth_cam/rgb/image_raw
```

---

## Check the camera message type

```bash
ros2 topic type <camera_topic_name>
```

Expected type:

```text
sensor_msgs/msg/Image
```

---

## View the camera stream

Recommended method:

```bash
rqt_image_view
```

Select the camera topic from the dropdown menu.

Alternative (command-line check):

```bash
ros2 topic echo <camera_topic_name> --once
```

---

## Notes

* Ensure the camera launch is running before checking topics
* Camera topics may differ based on robot configuration

---
