# Package `projet2` — Cartographie guidée (SY31 sujet 1.2)

Instructions spécifiques à ce package pour les sessions Claude Code.

## But

Implémente le **sujet 1.2 — Cartographie Guidée** : un TurtleBot3 piloté
manuellement parcourt un labyrinthe ; le système maintient une odométrie
(encodeurs + gyro), accumule les points LiDAR dans un repère fixe via une
matrice homogène 3×3, et détecte des flèches colorées (rouge=gauche,
bleu=droite) regroupées en clusters représentés chacun par un point.

C'est une **réécriture améliorée** du package `projet`, recentrée sur l'énoncé.
Le sujet officiel est dans `../projet/SY31-Projet-56517891.pdf`.

## Architecture (4 nœuds + utils)

| Nœud | Entrées | Sorties | Méthode |
|------|---------|---------|---------|
| `odometry` | `/sensor_state`, `/imu` | `/odom_est`, `/trajectory`, TF `odom→base_link` | v par encodeurs, θ par gyro (modèle TP3) |
| `mapper` | `/scan` (BEST_EFFORT), `/odom_est`, `/direction_cmd` | `/map_points`, `/filtered_scan`, `/arrow_map` | matrice homogène 3×3 + clustering angulaire (TP4) + voxels |
| `arrow_detector` | `/turtlecam/image_raw/compressed` | `/detections`, `/direction_cmd` | HSV rouge/bleu + clustering centroïdes |
| `direction_display` | `/direction_cmd` | `/direction_display` (Image) | flèche de consigne opérateur |

`utils.py` : `declare_param`, `make_pointcloud2`, `make_markers`.

## Conventions importantes

- **Rouge → gauche, Bleu → droite** (constantes `RED_DIRECTION`/`BLUE_DIRECTION`
  en tête de `arrow_detector.py`).
- Repère fixe de la carte : `odom` (origine = départ du robot).
- Le bag ne contient que le flux **compressé** → décodage direct par
  `cv2.imdecode` (pas de nœud décompresseur, pas d'`image_proc`).
- `/scan` est publié en **BEST_EFFORT** dans le bag → le subscriber doit avoir
  le même QoS, sinon aucun message reçu.
- `direction_display` publie une **Image** (pas de `cv2.imshow`) pour rester
  robuste sous WSL.
- La matrice de transformation de `mapper.py` doit rester **identique à
  l'équation du PDF** — c'est le cœur évalué du sujet, ne pas la « simplifier ».

## Choix d'implémentation à préserver

- L'odométrie **recalcule** la pose depuis encodeurs+gyro (objectif pédagogique
  du TP3) ; `/odom` du bag n'est utilisé que comme référence, pas comme source.
- Le clustering rejette les clusters < `min_cluster_size` points (anti-bruit).
- Bonus implémenté : `mapper` propage les flèches sur `/arrow_map` en posant un
  marker à la position du robot lors d'une nouvelle détection.
- `detect_wall`/ultrason **volontairement écarté** : hors sujet 1.2.

## Build & test (Ubuntu, `~/sy31_ws`)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select projet2 --symlink-install   # --symlink-install OBLIGATOIRE
source install/setup.bash
ros2 launch projet2 projet2.launch.xml          # terminal 1
ros2 bag play ~/sy31_ws/labyrinthe/ --loop      # terminal 2
```

Dépendances : `ros-jazzy-turtlebot3-msgs`, `python3-opencv`, `ros-jazzy-cv-bridge`.

Vérif syntaxe rapide sans ROS : `python -m py_compile projet2/*.py`.

## Visualisation

- rqt_image_view : `/detections`, `/direction_display`
- RViz2 (Fixed Frame `odom`) : `/map_points`, `/trajectory`, `/filtered_scan`,
  `/arrow_map`
