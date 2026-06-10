# SY31 — Projet 2 : Cartographie guidée dans un labyrinthe

Package ROS2 (Jazzy) réalisant le **sujet 1.2 — Cartographie Guidée**.

Un robot TurtleBot3, piloté manuellement, parcourt un labyrinthe. Pendant le
trajet le système :

1. **maintient une odométrie** à partir des encodeurs et du gyromètre ;
2. **accumule les points LiDAR** dans un repère fixe pour cartographier le
   parcours (matrice de transformation homogène de l'énoncé) ;
3. **détecte des flèches colorées** par la caméra et indique à l'opérateur de
   tourner à **gauche** (rouge) ou à **droite** (bleu) ;
4. **regroupe les flèches en clusters**, chacun représenté par **un point**.

> **Bonus** implémenté : les flèches détectées sont propagées sur la carte
> (`/arrow_map`), posées à la position du robot au moment de la détection.

---

## Architecture

| Nœud | Entrées | Sorties | Rôle |
|------|---------|---------|------|
| `odometry` | `/sensor_state`, `/imu` | `/odom_est`, `/trajectory`, TF | Odométrie encodeurs + gyromètre |
| `mapper` | `/scan`, `/odom_est`, `/direction_cmd` | `/map_points`, `/filtered_scan`, `/arrow_map` | Accumulation LiDAR + clustering + carte |
| `arrow_detector` | `/turtlecam/image_raw/compressed` | `/detections`, `/direction_cmd` | Détection + clustering des flèches |
| `direction_display` | `/direction_cmd` | `/direction_display` | Flèche de consigne pour l'opérateur |

```
/sensor_state ─┐
               ├─► odometry ─► /odom_est ──┐
/imu ──────────┘                           ├─► mapper ─► /map_points
/scan ─────────────────────────────────────┘            /filtered_scan
                                                         /arrow_map (bonus)
/turtlecam/image_raw/compressed ─► arrow_detector ─► /direction_cmd ─► direction_display
                                                  └► /detections
```

### Transformation LiDAR (cœur du sujet)

Chaque point LiDAR exprimé dans le repère robot (L) est replacé dans le repère
fixe (0) par la matrice homogène donnée dans l'énoncé, appliquée dans
`mapper.py` :

```
⎡x'⎤   ⎡cos θ  −sin θ  x⎤   ⎡x⎤
⎢y'⎥ = ⎢sin θ   cos θ  y⎥ · ⎢y⎥
⎣1 ⎦   ⎣  0       0    1⎦   ⎣1⎦
```

où `(x, y, θ)` est la pose du robot estimée par `odometry`.

---

## Prérequis (à installer une seule fois)

```bash
sudo apt-get install -y ros-jazzy-turtlebot3-msgs   # message /sensor_state
sudo apt-get install -y python3-opencv ros-jazzy-cv-bridge
```

ROS2 Jazzy doit être sourcé : `source /opt/ros/jazzy/setup.bash`.

---

## Build

Depuis la racine du workspace (`~/sy31_ws` sous Ubuntu) :

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select projet2 --symlink-install
source install/setup.bash
```

> `--symlink-install` est requis pour que `ros2 run` / `ros2 launch` trouvent
> les exécutables (scripts placés dans `lib/projet2/`).

---

## Lancement

### Terminal 1 — les nœuds + visualisation

```bash
source /opt/ros/jazzy/setup.bash
source ~/sy31_ws/install/setup.bash
ros2 launch projet2 projet2.launch.xml
```

Ouvre les 4 nœuds, RViz2 et rqt_image_view (flux `/detections`).

### Terminal 2 — rejouer le bag

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play ~/sy31_ws/labyrinthe/ --loop
```

### Visualisation

- **rqt_image_view** : `/detections` (flèches détectées + centroïdes des
  clusters) et `/direction_display` (consigne gauche/droite).
- **RViz2** (Fixed Frame = `odom`) — ajouter par topic :
  - `/map_points` (PointCloud2) — carte du labyrinthe
  - `/trajectory` (PointCloud2) — trace du robot
  - `/filtered_scan` (PointCloud2) — scan courant (debug)
  - `/arrow_map` (MarkerArray) — flèches propagées sur la carte (bonus)

---

## Réglages à chaud

```bash
# Seuil de portée LiDAR (m)
ros2 param set /mapper max_range 2.5

# Résolution de la carte (m par voxel)
ros2 param set /mapper voxel_size 0.03

# Aire minimale d'une flèche détectée (px²)
ros2 param set /arrow_detector min_area 800.0
```

---

## Conventions

- **Rouge → tourner à gauche**, **Bleu → tourner à droite**
  (modifiable en tête de `arrow_detector.py`).
- Repère fixe de la carte : `odom` (origine = position de départ du robot).

---

## Topics du bag `labyrinthe`

| Topic | Type | Utilisé par |
|-------|------|-------------|
| `/sensor_state` | turtlebot3_msgs/SensorState | `odometry` |
| `/imu` | sensor_msgs/Imu | `odometry` |
| `/scan` | sensor_msgs/LaserScan (BEST_EFFORT) | `mapper` |
| `/turtlecam/image_raw/compressed` | sensor_msgs/CompressedImage | `arrow_detector` |
| `/odom` | nav_msgs/Odometry | (référence, non utilisé : recalculé) |

---

## Utilisation d'un LLM

Le squelette de ce package a été généré avec l'aide d'un assistant IA, puis
relu et adapté. Les algorithmes (odométrie, matrice homogène, clustering)
reprennent les méthodes vues en TP3/TP4/TP5.
