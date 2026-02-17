# SIMPLE PID CONTROLLER

A very basic controller based on a simple PID and yolov8n.
It just follows the boat detected on the image (it simply tries to center the detected boat on the camera img). If the size of the detected boat's bbox is too large, the controlled boat stop. It only has been tested with the gazebo simumation and it worked.

## Launch

```bash
ros2 run ctrl_pid pid_boat
```

