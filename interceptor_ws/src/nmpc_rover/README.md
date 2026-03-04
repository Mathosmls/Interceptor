# nmpc_rover — Contrôleur NMPC pour rover ArduPilot (ROS2)

## Architecture

```
ros2_nmpc_ws/
└── src/
    └── nmpc_rover/
        ├── nmpc_rover/
        │   ├── __init__.py
        │   └── nmpc_controller_node.py   ← Nœud principal
        ├── launch/
        │   └── nmpc_rover.launch.py      ← Lance MAVROS + NMPC
        ├── config/
        │   └── nmpc_params.yaml          ← Tous les paramètres
        ├── package.xml
        └── setup.py
```

### Flux de données

```
ArduPilot (FCU)
     │  MAVLink (USB/UART/UDP)
     ▼
 MAVROS Node
     │  /mavros/local_position/odom   (position, vitesse, orientation)
     │  /mavros/state                 (connexion, armement)
     ▼
 NMPC Controller Node
     │  Calcule (v, w) optimal sur horizon N
     │  /mavros/setpoint_velocity/cmd_vel  →  MAVROS
     │  /nmpc/trajectory_markers           →  RViz2
     │  /nmpc/debug                        →  monitoring
     ▼
 ArduPilot (GUIDED mode)
```

---

## Prérequis

### Dépendances système
```bash
# ROS2 Humble/Iron (Ubuntu 22.04)
sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras

# Geographiclib (requis par MAVROS)
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh

# Python
pip3 install scipy numpy
```

---

## Build

```bash
cd ~/ros2_nmpc_ws
colcon build --packages-select nmpc_rover
source install/setup.bash
```

---

## Configuration ArduPilot

Paramètres à régler dans Mission Planner ou via MAVLink :

| Paramètre         | Valeur | Description                          |
|-------------------|--------|--------------------------------------|
| `SYSID_MYGCS`     | 1      | ID GCS autorisée                     |
| `SERIAL0_BAUD`    | 115    | 115200 baud si USB                   |
| `SERIAL1_BAUD`    | 57     | 57600 si UART                        |
| `GUIDED_SPEED_DN` | 150    | Vitesse max cm/s en GUIDED           |
| `WP_SPEED`        | 150    | idem                                 |

Le rover doit être en mode **GUIDED** pour accepter les commandes MAVROS.

---

## Lancement

### Rover réel via USB
```bash
ros2 launch nmpc_rover nmpc_rover.launch.py \
  fcu_url:=serial:///dev/ttyACM0:115200
```

### Rover réel via WiFi/UDP
```bash
ros2 launch nmpc_rover nmpc_rover.launch.py \
  fcu_url:="udp://:14550@192.168.1.1:14555"
```

### Modifier la trajectoire à la volée
```bash
ros2 launch nmpc_rover nmpc_rover.launch.py \
  traj_amplitude:=5.0 \
  traj_wavelength:=15.0 \
  traj_forward_speed:=0.8
```

### Modifier les paramètres NMPC sans recompiler
```bash
# Fichier config/nmpc_params.yaml → relancer le node
ros2 param set /nmpc_controller prediction_horizon 15
ros2 param set /nmpc_controller heading_weight 8.0
```

---

## Monitoring

```bash
# Voir l'état de connexion MAVROS
ros2 topic echo /mavros/state

# Voir la position
ros2 topic echo /mavros/local_position/odom

# Voir les commandes envoyées
ros2 topic echo /mavros/setpoint_velocity/cmd_vel

# Voir le debug NMPC (x, y, tx, ty, v, w, dist, heading_err)
ros2 topic echo /nmpc/debug

# Visualiser dans Rviz2
rviz2  # Ajouter topic /nmpc/trajectory_markers (MarkerArray)
```

---

## Procédure de démarrage sécurisée

1. Démarrer ArduPilot, vérifier dans Mission Planner
2. `ros2 launch nmpc_rover nmpc_rover.launch.py`
3. Vérifier : `ros2 topic echo /mavros/state` → `connected: True`
4. Armer le rover depuis Mission Planner ou :
   ```bash
   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
   ```
5. Passer en mode GUIDED :
   ```bash
   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
   ```
6. Le contrôleur commence à envoyer des commandes automatiquement

---

## Notes importantes

- **Repère local** : MAVROS fournit la position dans le repère `map` (EKF local).  
  Assurez-vous que l'origine de la sinusoïde correspond à la position de départ du rover.
- **Mode GUIDED** : ArduPilot doit être en mode GUIDED pour accepter `setpoint_velocity`.
- **Ajustements** : Les paramètres `linear_drag` et `angular_drag` dans le node  
  sont des estimations — à calibrer sur votre rover réel.
