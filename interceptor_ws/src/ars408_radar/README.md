# ARS408 ROS2 Minimal Driver

Minimal ROS2 package to handle the **ARS408 radar**.

It can manage:

- Radar state reading  
- Radar configuration  
- Object reading  
- Object filters configuration  
- Clusters reading  
- Clusters filters configuration  

It would be very easy to implement other radar functionalities (you just have to decode the CAN frames in `radar_lib/senders.py` and `radar_lib/decoders.py`), but that was not the objective and it would have taken too much time.

The package also publishes RViz2-compatible messages for visualization purposes.

---

## Launch

```bash
ros2 launch ars408_radar radar_launch.py
```

---

## CAN Setup (Required)

The CAN interface must be up and running at **500000 baud**.

Example with a CANUSB adapter:

```bash
sudo slcand -o -s5 /dev/ttyUSB0
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

---

## Radar Library

The radar library is located in:

```
ars408_radar/radar_lib
```

This library can work on its own if needed. You can:

- Print received messages  
- Send radar configuration  
- Send object filters  
- Send clusters filters  
- Do a simple visualization using matplotlib  

---

## YAML Configuration

You can modify the object filters and radar configuration using the YAML files located in:

```
config/
```

You can change these parameters while the radar is running and apply the modifications with:

```bash
ros2 param set /radar_node cfg_radar_yaml "/absolute_path_to_yaml/radar_cfg.yaml"
ros2 param set /radar_node objects_filters_yaml "/absolute_path_to_yaml/objects_filters.yaml"
```

The node will reload the YAML file and send the updated configuration to the radar.
