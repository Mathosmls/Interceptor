# visual_servowing — Contrôleur IBVS

Système d'asservissement visuel 2½D pour l'interception autonome d'une embarcation.  
Le contrôleur ferme la boucle de commande directement sur les mesures extraites de l'image caméra, sans GPS ni reconstruction 3D complète.

---

## Dépendances

- ROS 2 (Humble ou supérieur)
- `rclcpp`, `geometry_msgs`, `vision_msgs`
- `Eigen3`

---

## Build

```bash
cd ~/ros_ws
colcon build --packages-select visual_servowing
source install/setup.bash
```

---

## Lancement

Le contrôleur s'abonne au topic `detections_output` et publie les commandes sur `/mavros/setpoint_velocity/cmd_vel_unstamped`.  
La source de détection (YOLO, couleur, ou autre) doit être lancée séparément et publier sur ce topic.

```bash
ros2 launch visual_servowing visual_servowing.launch.py
```

---

## Paramètres (`config/visual_control.yaml`)

| Paramètre | Valeur par défaut | Description |
|---|---|---|
| `fx` | 642.0 | Focale horizontale de la caméra (pixels) |
| `fy` | 634.0 | Focale verticale de la caméra (pixels) |
| `cx` | 330.0 | Centre optique horizontal (pixels) |
| `cy` | 242.0 | Centre optique vertical (pixels) |
| `height_target` | 0.63 | Hauteur réelle de la cible en mètres. Utilisée pour estimer la profondeur : `Z = fy * height_target / h` |
| `desired_height` | 400.0 | Hauteur désirée du bounding box en pixels. Définit la distance d'interception : plus la valeur est grande, plus le robot s'approche de la cible |
| `lamda` | 0.5 | Gain de la loi de commande. Contrôle la rapidité de convergence. Une valeur trop élevée peut provoquer des oscillations |
| `Vmax` | 2.0 | Vitesse maximale en m/s. Sature les commandes `u_c` et `ω` |
| `filter_alpha` | 0.4 | Coefficient du filtre exponentiel sur les primitives `x` et `h`. Proche de 0 = plus lisse, proche de 1 = plus réactif |
| `timeout_sec` | 5.0 | Durée en secondes sans détection avant l'arrêt d'urgence |
| `control_rate_hz` | 20.0 | Fréquence de la boucle de contrôle en Hz |

---

## Description du contrôleur

Le nœud `control_node` (`visual_control.cpp`) implémente un asservissement visuel **2½D** à 2 degrés de liberté : vitesse linéaire avant `u_c` et vitesse de lacet `ω`.

### Primitives d'image

Deux grandeurs sont extraites du bounding box à chaque détection :

- `x` : position horizontale normalisée de la cible
  ```
  x = (u - cx) / fx
  ```
- `h` : hauteur du bounding box en pixels, utilisée comme proxy de la distance via
  ```
  Z_hat = fy * height_target / h
  ```

### Vecteur d'erreur

```
e1 = x                          → erreur de centrage latéral
e2 = log(h) - log(desired_height)  → erreur de distance
```

Le terme `log(h)` rend la commande moins agressive à grande distance et plus réactive à courte distance.

### Matrice d'interaction

```
J = [ h*x / (fy*H)   -(1 + x²) ]
    [ h   / (fy*H)    -x        ]
```

Cette matrice est recalculée à chaque itération avec les valeurs courantes de `x` et `h`. C'est cette mise à jour en temps réel qui constitue la signature du **2½D visual servoing** : contrairement à un IBVS 2D pur qui utiliserait une profondeur constante, `J` s'adapte à la distance courante.

### Loi de commande

```
[u_c]  =  -lambda * J⁻¹ * e
[ ω ]
```

- `u_c` → vitesse linéaire avant (`cmd_vel.linear.x`)  
- `ω` → vitesse de lacet (`cmd_vel.angular.z`)

La commande est annulée si `|det(J)| < 1e-6` et saturée par `±Vmax`.

### Architecture des callbacks

Deux callbacks sont séparés intentionnellement :

- `control_callback` : déclenché à chaque détection reçue. Applique le filtre exponentiel sur `x` et `h` et met à jour le timestamp de dernière détection.
- `control_timer_callback` : déclenché à fréquence fixe (`control_rate_hz`). Calcule et publie la commande à partir des primitives filtrées courantes.

Cette séparation découple la fréquence de détection (variable) de la fréquence de contrôle (fixe), ce qui garantit une commande régulière indépendamment du framerate du détecteur.

### Comportement en boucle fermée

| Situation | Réponse du contrôleur |
|---|---|
| Cible décalée sur le côté (`e1 ≠ 0`) | `ω` tourne pour recentrer la cible |
| Cible trop loin (`e2 > 0`, `h` trop petit) | `u_c` accélère pour se rapprocher |
| Cible trop proche (`e2 < 0`, `h` trop grand) | `u_c` ralentit ou s'inverse |
| Erreurs nulles (`e1 = 0`, `e2 = 0`) | Commandes nulles, position maintenue |

### Sécurités

| Condition | Action |
|---|---|
| Aucune détection depuis `timeout_sec` secondes | Commande mise à zéro |
| `\|det(J)\| < 1e-6` | Commande annulée |
| Commande hors plage | Saturation par `±Vmax` |
