# Lancer le projet sur le bag labyrinthe

## Prérequis

- WSL2 Ubuntu avec ROS2 Jazzy installé
- Le bag `labyrinthe/` présent dans `C:\Users\Maxime\OneDrive\Bureau\Travail\UTC\P26\SY31\Projet\`

---

## Étape 0 — Installer les dépendances (une seule fois)

```bash
sudo apt-get install -y ros-jazzy-turtlebot3-msgs
sudo apt-get install -y python3-transforms3d
```

- `ros-jazzy-turtlebot3-msgs` : messages encodeurs (`/sensor_state`) pour `odompose` et `pipeline`
- `python3-transforms3d` : conversion euler → quaternion dans `odompose`

---

## Étape 1 — Builder le package

**Important :** toujours utiliser `--symlink-install` pour que les scripts atterrissent dans `lib/projet/` (requis par `ros2 run` et `ros2 launch`).

**PowerShell :**

```powershell
wsl bash -c "source /opt/ros/jazzy/setup.bash && cd '/mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws' && colcon build --packages-select projet --symlink-install"
```

**Ubuntu WSL :**

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws
colcon build --packages-select projet --symlink-install
```

---

## Étape 2 — Lancer les nœuds du projet

**PowerShell :**

```powershell
wsl bash -c "source /opt/ros/jazzy/setup.bash && source '/mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws/install/setup.bash' && ros2 launch projet projet.launch.xml"
```

**Ubuntu WSL :**

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws/install/setup.bash
ros2 launch projet projet.launch.xml
```

Démarre les 5 nœuds simultanément :
- `detector` — détecte les formes rouges et bleues depuis la caméra
- `transformer` — convertit le LiDAR polaire en nuage de points cartésien
- `intensity_filter` — filtre les points selon leur réflectivité
- `odompose` — estime la pose robot (encodeurs / gyro / magnéto)
- `pipeline` — fusionne LiDAR + odométrie → carte globale `/map_points`

---

## Étape 3 — Jouer le bag (dans un second terminal)

**PowerShell :**

```powershell
wsl bash -c "source /opt/ros/jazzy/setup.bash && ros2 bag play '/mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/labyrinthe/' --loop"
```

**Ubuntu WSL :**

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play /mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/labyrinthe/ --loop
```

`--loop` repart automatiquement au début (durée ~115 s).

---

## Étape 4 — Visualiser (terminal Ubuntu WSL uniquement)

Les fenêtres graphiques nécessitent un terminal interactif, pas PowerShell.

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Dans le menu déroulant : sélectionner `/detections` (contours rouges en vert, bleus en jaune).

---

## Réglage du seuil d'intensité LiDAR

```bash
ros2 param set /intensity_filter intensity_threshold 8000.0
ros2 param get /intensity_filter intensity_threshold
```

---

## Vérifier que tout tourne

```bash
ros2 node list                       # 5 nœuds attendus
ros2 topic hz /detections            # ~12 Hz (caméra)
ros2 topic hz /points_filtered       # ~5 Hz (LiDAR)
ros2 topic hz /map_points            # actif si turtlebot3_msgs installé
```

---

## Résumé des topics

| Topic | Type | Producteur | Consommateur |
|-------|------|-----------|--------------|
| `/turtlecam/image_raw/compressed` | CompressedImage | bag | `detector` |
| `/detections` | Image | `detector` | rqt_image_view |
| `/scan` | LaserScan | bag | `transformer` |
| `/points` | PointCloud2 | `transformer` | `intensity_filter` |
| `/points_filtered` | PointCloud2 | `intensity_filter` | `pipeline` |
| `/imu` | Imu | bag | `odompose`, `pipeline` |
| `/sensor_state` | SensorState | bag | `odompose`, `pipeline` |
| `/pose_gyro` | PoseStamped | `odompose` | — |
| `/map_points` | PointCloud2 | `pipeline` | rviz2 |
