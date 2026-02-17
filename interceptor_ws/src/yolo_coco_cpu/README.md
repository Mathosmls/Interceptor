# BOAT RECOGNITION THANKS TO YOLO

Lightweight classification meant to run on cpu only devices. It works pretty well if there is just another boat in front of the camera (and no other similar objects).
It has been tested and is configured to work with the camera ouput of the gazebo simulation.
It uses yolov8n (you can find at the root of the ws).

## Launch


```bash
ros2 run yolo_coco_cpu yolo_coco_cpu
```

