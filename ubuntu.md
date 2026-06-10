# Tests SY31 — Ubuntu natif (`~/sy31_ws`)

Ce guide suppose que `sy31_ws` est dans le home Ubuntu (`~/sy31_ws`) et que le bag `labyrinthe` est à `~/sy31_ws/labyrinthe/`.

---

## Prérequis — installation unique

```bash
# ROS2 Jazzy (si pas encore sourcé automatiquement)
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Messages TurtleBot3 (nécessaires pour /sensor_state dans tp3, odompose, pipeline)
sudo apt-get install -y ros-jazzy-turtlebot3-msgs

# Conversion euler→quaternion (odompose.py)
sudo apt-get install -y python3-transforms3d

# Traitement image (tp5, detector)
sudo apt-get install -y python3-opencv ros-jazzy-cv-bridge

# Vérification
ros2 --version
```

---

## ⚠️ Prérequis critique WSL — désactiver la mémoire partagée Zenoh

Si l'environnement utilise **Zenoh** (`RMW_IMPLEMENTATION=rmw_zenoh_cpp` dans
`~/.bashrc`), la **mémoire partagée perd les gros messages sous WSL** : `/scan`
(LiDAR) et le flux caméra n'arrivent jamais (écran noir dans rqt, aucune carte).
Les petits messages (`/imu`, `/odom`) passent, ce qui rend le bug trompeur.

### Vérifier si tu es concerné

```bash
# Quel middleware ? (vide = Fast DDS par défaut ; rmw_zenoh_cpp = Zenoh)
echo "$RMW_IMPLEMENTATION"

# La mémoire partagée est-elle activée dans la config Zenoh ?
grep ZENOH_CONFIG_OVERRIDE ~/.bashrc
```

Si la ligne contient `transport/shared_memory/enabled=true`, applique le correctif.

### Correctif (une seule fois)

```bash
# Sauvegarde
cp ~/.bashrc ~/.bashrc.bak

# Retire la portion shared_memory de ZENOH_CONFIG_OVERRIDE
sed -i 's#;transport/shared_memory/enabled=true##g' ~/.bashrc

# Vérifie le résultat
grep ZENOH_CONFIG_OVERRIDE ~/.bashrc
```

La ligne doit devenir :
```bash
export ZENOH_CONFIG_OVERRIDE='transport/unicast/compression/enabled=true'
```

### Appliquer le changement

**Ferme et rouvre TOUS les terminaux** (le routeur `rmw_zenohd` inclus), puis
relance le routeur Zenoh :

```bash
# Tuer un ancien routeur encore actif si besoin
pkill -f rmw_zenohd

# Relancer le routeur (ou via ta fonction `zenohd` si elle est définie)
ros2 run rmw_zenoh_cpp rmw_zenohd &
```

### Vérifier que les gros messages passent

```bash
# Terminal 1 : jouer le bag
ros2 bag play ~/sy31_ws/labyrinthe/ --loop

# Terminal 2 : ces deux topics doivent afficher un débit non nul
ros2 topic hz /scan                              # ~5 Hz attendu
ros2 topic hz /turtlecam/image_raw/compressed    # ~15 Hz attendu
```

Si les débits sont à 0, la mémoire partagée est encore active (terminal pas
rouvert, ou routeur pas relancé).

> Astuce : le routeur `rmw_zenohd` **doit tourner** pour que les nœuds Zenoh
> communiquent. S'il est absent, aucun topic ne circule (même petits messages).

---

## Build du workspace

```bash
cd ~/sy31_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Pour ne builder qu'un seul package :
```bash
colcon build --packages-select tp4 --symlink-install
colcon build --packages-select tp5 --symlink-install
colcon build --packages-select projet --symlink-install
```

> **Note :** `--symlink-install` est indispensable. Sans lui, les scripts atterrissent dans `bin/` au lieu de `lib/<pkg>/` et `ros2 run` / `ros2 launch` ne les trouvent pas.

> **TP3** n'a pas de `setup.py` — il s'exécute directement avec `python3`, pas via `colcon`.

---

## Terminal type — à faire dans chaque nouveau terminal

```bash
source /opt/ros/jazzy/setup.bash
source ~/sy31_ws/install/setup.bash
```

---

## TP3 — Odométrie (`odom2pose.py`)

### Contexte
Estime la pose du robot depuis 3 sources : encodeurs, gyroscope, magnétomètre.
Publie `/pose_enco`, `/pose_gyro`, `/pose_magn`.

### Terminal 1 — lancer le nœud

```bash
source /opt/ros/jazzy/setup.bash
cd ~/sy31_ws/src/tp3/tp3
python3 odom2pose.py
```

### Terminal 2 — jouer le bag de test TP3

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play ~/sy31_ws/src/tp3/tp3/odometry/ --loop
```

### Terminal 3 — vérifier les sorties

```bash
source /opt/ros/jazzy/setup.bash

# Voir les topics actifs
ros2 topic list

# Vérifier les poses publiées
ros2 topic echo /pose_enco
ros2 topic echo /pose_gyro
ros2 topic echo /pose_magn

# Fréquences de publication
ros2 topic hz /pose_enco
ros2 topic hz /pose_gyro
```

### Visualiser dans RViz2

```bash
rviz2
```

Dans RViz2 :
- Fixed Frame → `odom`
- Add → By topic → `/pose_enco` (PoseStamped) — trajectoire encodeurs
- Add → By topic → `/pose_gyro` (PoseStamped) — trajectoire gyroscope
- Add → By topic → `/pose_magn` (PoseStamped) — trajectoire magnétomètre

### Topics TP3

| Topic | Direction | Type |
|-------|-----------|------|
| `/sensor_state` | entrée | `turtlebot3_msgs/SensorState` |
| `/imu` | entrée | `sensor_msgs/Imu` |
| `/magnetic_field` | entrée | `sensor_msgs/MagneticField` |
| `/pose_enco` | sortie | `geometry_msgs/PoseStamped` |
| `/pose_gyro` | sortie | `geometry_msgs/PoseStamped` |
| `/pose_magn` | sortie | `geometry_msgs/PoseStamped` |

---

## TP4 — Pipeline LiDAR

### Contexte
Convertit le scan LiDAR polaire en nuage de points cartésien, filtre par intensité, puis regroupe en clusters et ajuste des formes géométriques.

```
/scan → transformer → /points → intensity_filter → /points_filtered → clusterer → /clusters → shaper_*
```

### Build

```bash
cd ~/sy31_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tp4 --symlink-install
source install/setup.bash
```

### Terminal 1 — transformer (LaserScan → PointCloud2 cartésien)

```bash
source /opt/ros/jazzy/setup.bash && source ~/sy31_ws/install/setup.bash
ros2 run tp4 transformer
```

### Terminal 2 — intensity_filter

```bash
source /opt/ros/jazzy/setup.bash && source ~/sy31_ws/install/setup.bash
ros2 run tp4 intensity_filter
```

Régler le seuil à la volée :
```bash
ros2 param set /intensity_filter intensity_threshold 8000.0
```

### Terminal 3 — clusterer

```bash
source /opt/ros/jazzy/setup.bash && source ~/sy31_ws/install/setup.bash
ros2 run tp4 clusterer
```

### Terminal 4 — un shaper au choix

```bash
ros2 run tp4 shaper_cylinder   # ajuste des cylindres sur les clusters
ros2 run tp4 shaper_bbox       # ajuste des boîtes englobantes
ros2 run tp4 shaper_polyline   # ajuste des polylignes
```

### Terminal 5 — jouer le bag labyrinthe

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play ~/sy31_ws/labyrinthe/ --loop
```

### Visualiser dans RViz2

```bash
rviz2
```

Dans RViz2 :
- Fixed Frame → `laser`
- Add → By topic → `/points` (PointCloud2) — tous les points bruts
- Add → By topic → `/points_filtered` (PointCloud2) — points réfléchissants
- Add → By topic → `/clusters` (PointCloud2) — clusters colorés
- Add → By topic → `/cylinders` ou `/bboxes` ou `/polylines` (MarkerArray) — formes ajustées

### Topics TP4

| Topic | Type | Producteur | Consommateur |
|-------|------|-----------|--------------|
| `/scan` | LaserScan | bag | `transformer` |
| `/points` | PointCloud2 | `transformer` | `intensity_filter`, `clusterer` |
| `/points_filtered` | PointCloud2 | `intensity_filter` | `clusterer` |
| `/clusters` | PointCloud2 | `clusterer` | `shaper_*` |
| `/cylinders` | MarkerArray | `shaper_cylinder` | rviz2 |
| `/bboxes` | MarkerArray | `shaper_bbox` | rviz2 |
| `/polylines` | MarkerArray | `shaper_polyline` | rviz2 |

---

## TP5 — Pipeline caméra

### Contexte
Détecte les pixels rouges dans l'image caméra par masque HSV et dessine les contours.

### Build

```bash
cd ~/sy31_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tp5 --symlink-install
source install/setup.bash
```

### Option A — nœud de détection seul

```bash
# Terminal 1 — nœud de détection
source /opt/ros/jazzy/setup.bash && source ~/sy31_ws/install/setup.bash
ros2 run tp5 detect_node

# Terminal 2 — bag
source /opt/ros/jazzy/setup.bash
ros2 bag play ~/sy31_ws/labyrinthe/ --loop

# Terminal 3 — visualiser
source /opt/ros/jazzy/setup.bash && source ~/sy31_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
# Sélectionner /detections dans le menu déroulant
```

### Option B — pipeline complet via launch (décompression + rectification + détection)

```bash
source /opt/ros/jazzy/setup.bash && source ~/sy31_ws/install/setup.bash
ros2 launch tp5 camera.launch.xml
```

Ce launch démarre :
- `image_transport/republish` : décompresse `/turtlecam/image_raw/compressed` → `/turtlecam/image_raw`
- `image_proc/rectify_node` : rectifie l'image (correction distorsion)
- `rqt_image_view` : visualisation

> **Remarque :** le bag ne contient pas `/turtlecam/camera_info`, donc `rectify_node` peut afficher des warnings. La détection par masque HSV dans `detect_node` fonctionne sans rectification.

### Topics TP5

| Topic | Type | Producteur | Consommateur |
|-------|------|-----------|--------------|
| `/turtlecam/image_raw/compressed` | CompressedImage | bag | `detect_node` |
| `/detections` | Image | `detect_node` | rqt_image_view |

---

## Projet — Pipeline complet labyrinthe

### Contexte
Fusionne les 3 capteurs (LiDAR + odométrie + caméra) pour construire une carte du labyrinthe dans un référentiel fixe.

```
bag ──→ /scan              → transformer → /points → intensity_filter → /points_filtered ──→ pipeline → /map_points
    ──→ /imu               → odompose → /pose_gyro                                         ↗
    ──→ /sensor_state      → odompose → /pose_enco                                        /
    ──→ /magnetic_field    → odompose → /pose_magn                                        /
    ──→ /turtlecam/...     → detector → /detections ─────────────────────────────────────/
```

### Build

```bash
cd ~/sy31_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select projet --symlink-install
source install/setup.bash
```

### Terminal 1 — lancer tous les nœuds

```bash
source /opt/ros/jazzy/setup.bash && source ~/sy31_ws/install/setup.bash
ros2 launch projet projet.launch.xml
```

Lance simultanément :
- `detector` — détection rouge + bleu par caméra
- `transformer` — LaserScan → PointCloud2 + trilatération `robot_position`
- `intensity_filter` — filtre réflectivité (seuil 10 000)
- `odompose` — pose encodeurs / gyro / magnéto
- `pipeline` — fusion LiDAR + odométrie → `/map_points`
- `rqt_image_view` — affiche `/detections`
- `rviz2` — visualisation 3D

### Terminal 2 — jouer le bag

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play ~/sy31_ws/labyrinthe/ --loop
```

`--loop` repart au début automatiquement (durée ~115 s).

### Vérifications

```bash
# 5 nœuds ROS2 doivent apparaître
ros2 node list

# Fréquences attendues
ros2 topic hz /detections        # ~12 Hz (caméra)
ros2 topic hz /points_filtered   # ~5 Hz (LiDAR filtré)
ros2 topic hz /map_points        # actif si turtlebot3_msgs installé

# Voir la position estimée du robot
ros2 topic echo /robot_position

# Voir les poses odométriques
ros2 topic echo /pose_gyro
```

### Régler le seuil d'intensité LiDAR à chaud

```bash
ros2 param set /intensity_filter intensity_threshold 8000.0
ros2 param get /intensity_filter intensity_threshold
```

### Calibrer les positions des amers (trilatération)

Mesurer physiquement les 3 marqueurs rétroréfléchissants dans le labyrinthe, puis :
```bash
ros2 param set /transformer landmark_1 "[x1, y1]"
ros2 param set /transformer landmark_2 "[x2, y2]"
ros2 param set /transformer landmark_3 "[x3, y3]"
```

### Configurer RViz2 pour voir la carte

Dans RViz2 :
- Fixed Frame → `odom`
- Add → By topic → `/map_points` (PointCloud2) — carte accumulée
- Add → By topic → `/points_filtered` (PointCloud2) — points LiDAR filtrés courants
- Add → By topic → `/pose_gyro` (PoseStamped) — position estimée par gyro
- Add → By topic → `/pose_enco` (PoseStamped) — position estimée par encodeurs

### Topics complets du projet

| Topic | Type | Producteur | Consommateur |
|-------|------|-----------|--------------|
| `/turtlecam/image_raw/compressed` | CompressedImage | bag | `detector` |
| `/detections` | Image | `detector` | rqt_image_view, `pipeline` |
| `/scan` | LaserScan | bag | `transformer` |
| `/points` | PointCloud2 | `transformer` | `intensity_filter` |
| `/points_filtered` | PointCloud2 | `intensity_filter` | `pipeline` |
| `/robot_position` | PointStamped | `transformer` | — |
| `/imu` | Imu | bag | `odompose`, `pipeline` |
| `/sensor_state` | SensorState | bag | `odompose`, `pipeline` |
| `/magnetic_field` | MagneticField | bag | `odompose` |
| `/pose_enco` | PoseStamped | `odompose` | — |
| `/pose_gyro` | PoseStamped | `odompose` | — |
| `/pose_magn` | PoseStamped | `odompose` | — |
| `/map_points` | PointCloud2 | `pipeline` | rviz2 |

---

## Aide au débogage

### Écran noir dans rqt_image_view / `/scan` vide / aucune carte LiDAR

Symptôme le plus courant sous WSL : `/imu` et `/odom` arrivent mais **`/scan` et
l'image restent à 0**. Cause = mémoire partagée Zenoh (voir la section
**« ⚠️ Prérequis critique WSL »** en haut du fichier).

```bash
# Diagnostic rapide : petits messages OK, gros messages à 0 ?
ros2 topic hz /imu       # doit tourner (~20 Hz)
ros2 topic hz /scan      # à 0 = mémoire partagée Zenoh active
```

Correctif : retirer `transport/shared_memory/enabled=true` de
`ZENOH_CONFIG_OVERRIDE` dans `~/.bashrc`, rouvrir les terminaux, relancer
`rmw_zenohd`.

### Un nœud ne reçoit pas de messages

```bash
# Vérifier que le bag publie bien sur le topic attendu
ros2 topic list
ros2 topic info /scan
ros2 topic info /turtlecam/image_raw/compressed

# Vérifier la compatibilité QoS
ros2 topic info /scan --verbose
```

### Erreur `package not found` après colcon build

```bash
# Vérifier que install/setup.bash est sourcé
source ~/sy31_ws/install/setup.bash
ros2 pkg list | grep projet
```

### `turtlebot3_msgs` non trouvé

```bash
sudo apt-get install -y ros-jazzy-turtlebot3-msgs
# Puis rebuilder
cd ~/sy31_ws && colcon build --packages-select projet --symlink-install
```

### Voir les logs d'un nœud en direct

```bash
ros2 run projet detector_node --ros-args --log-level debug
```
