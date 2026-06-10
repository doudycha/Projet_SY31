# Suivi des modifications — package `projet`

Ce fichier documente chaque changement apporté au package `projet`, avec les choix d'implémentation et les raisons associées.

---

## Architecture initiale

**Date :** juin 2026  
**Contexte :** Création du package `projet` à partir de zéro pour le Projet 2 (labyrinthe).

### Structure créée

```
src/projet/
├── package.xml
├── setup.py
├── resource/projet
├── launch/
│   └── projet.launch.xml
└── projet/
    ├── __init__.py
    ├── utils.py
    ├── transformer.py
    ├── intensity_filter.py
    └── detector.py
```

### Dépendances déclarées (`package.xml`)
- `rclpy` — client Python ROS2
- `sensor_msgs` — types de messages capteurs (LaserScan, PointCloud2, CompressedImage, Image)
- `cv_bridge` — conversion OpenCV ↔ ROS Image

---

## `utils.py`

**Source :** copie directe de `tp4/tp4/utils.py`.  
**Changements :** aucun, à l'exception des imports (relatifs dans le package projet).  
**Contenu :**
- `declare_param()` — déclare un paramètre ROS2 et met à jour l'attribut correspondant via callback
- `make_pointcloud2()` — construit un message `PointCloud2` depuis des tableaux x, y, intensité, clusterId
- `make_markers()` — construit un `MarkerArray` RViz (cylindres, bboxes, polylines)
- Types `Cylinder`, `BBox`, `Polyline`

---

## `intensity_filter.py`

**Source :** basé sur `tp4/tp4/intensity_filter.py`, structure identique.  
**Rôle :** filtrer les points LiDAR selon leur réflectivité.

### Pipeline
```
/points (PointCloud2)  →  [seuil intensité]  →  /points_filtered (PointCloud2)
```

### Paramètres
| Paramètre | Défaut | Rôle |
|-----------|--------|------|
| `intensity_threshold` | `10000.0` | Seuil minimum de réflectivité LiDAR |

### Méthode utilisée
`read_points_numpy()` decode le PointCloud2 en tableau numpy `[x, y, intensité]`.  
Filtre vectoriel : `points[points[:, 2] > self.intensity_threshold]`.

### Choix d'implémentation
Réglable à chaud via `ros2 param set /intensity_filter intensity_threshold <valeur>` sans relancer le nœud.

---

## `transformer.py`

**Source :** basé sur `tp4/tp4/transformer.py`, étendu avec triangulation.  
**Rôle :** convertir le scan LiDAR polaire en nuage de points cartésien, et estimer la position du robot.

### Pipeline
```
/scan (LaserScan)  →  [conversion polaire→cartésien]  →  /points (PointCloud2)
                   →  [triangulation]                 →  /robot_position (PointStamped)
```

### Méthode de localisation — Triangulation (trilatération)

**Choix :** triangulation parmi les 3 méthodes envisagées (temps de vol, déphasage, triangulation).

**Pourquoi la triangulation :**
- Le temps de vol (`d = c × t / 2`) et le déphasage (`d = c × φ / (4π × f)`) sont les méthodes physiques internes au capteur pour calculer chaque `ranges[i]`. Elles donnent une distance, pas une position 2D.
- La triangulation est la seule des 3 qui permet de calculer une position `(rx, ry)` dans un référentiel absolu à partir de plusieurs mesures de distance.

**Principe mathématique :**  
3 amers à positions connues `(x1,y1)`, `(x2,y2)`, `(x3,y3)`. Le robot est à l'intersection de 3 cercles :
```
(rx-x1)² + (ry-y1)² = d1²
(rx-x2)² + (ry-y2)² = d2²
(rx-x3)² + (ry-y3)² = d3²
```
En soustrayant l'équation 1 aux deux autres, les termes quadratiques s'annulent → système linéaire 2×2 résolu avec `np.linalg.solve`.

**Identification des amers :** les 3 points LiDAR d'intensité maximale dans chaque scan (marqueurs rétroréfléchissants placés à des positions connues dans le labyrinthe).

### Paramètres
| Paramètre | Défaut | Rôle |
|-----------|--------|------|
| `landmark_1` | `[0.0, 0.0]` | Position (x, y) de l'amer 1 dans le référentiel fixe |
| `landmark_2` | `[2.0, 0.0]` | Position (x, y) de l'amer 2 |
| `landmark_3` | `[1.0, 1.5]` | Position (x, y) de l'amer 3 |

**À calibrer** en mesurant physiquement la position des marqueurs dans le labyrinthe.

---

## `detector.py`

**Source :** basé sur `tp5/tp5/detect.py`, étendu pour la détection bleue.  
**Rôle :** détecter les formes rouges et bleues depuis le flux caméra compressé.

### Pipeline
```
/turtlecam/image_raw/compressed (CompressedImage)  →  [HSV + contours]  →  /detections (Image)
```

### Méthode — Filtrage HSV + contours

Structure identique à `tp5/detect.py` :
- `callback()` : décode l'image JPEG compressée via `np.frombuffer` + `cv2.imdecode` (pas de `cv_bridge` pour le décodage)
- `detect()` : traitement HSV + contours
- Conversion retour via `cv_bridge.cv2_to_imgmsg`

### Plages HSV

| Couleur | Variable | H | S | V | Remarque |
|---------|----------|---|---|---|----------|
| Rouge bas | `lower1 / upper1` | 0–10 | 100–255 | 100–255 | Teinte autour de 0° |
| Rouge haut | `lower2 / upper2` | 170–180 | 100–255 | 100–255 | Teinte autour de 180° |
| Bleu | `lower_blue / upper_blue` | 100–130 | 100–255 | 50–255 | Teinte ~115° |

**Pourquoi deux masques pour le rouge :** dans l'espace HSV, la teinte rouge chevauche 0° et 180°. Les deux masques sont combinés avec `cv2.bitwise_or`.

### Détection des contours
Même méthode que tp5 :
```python
contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(img, contours, -1, color, 3)
```
Calcul systématique : moments `M`, aire, périmètre, approximation polygonale `approxPolyDP`.

### Couleurs des contours dessinés
- Formes **rouges** → contour dessiné en **vert** `(0, 255, 0)`
- Formes **bleues** → contour dessiné en **jaune** `(0, 255, 255)`

---

## Correctif `transformer.py` — QoS BEST_EFFORT sur `/scan` (2026-06-06)

**Changement :** la souscription à `/scan` utilisait le QoS par défaut (`10` =
RELIABLE). Le bag `labyrinthe` publie `/scan` en **BEST_EFFORT** → souscription
incompatible → **aucun message reçu**.

**Détecté par :** test fonctionnel (voir `src/test.md`). `/points`,
`/points_filtered` et `/map_points` étaient tous à 0 Hz.

**Correctif appliqué :**
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
self.sub = self.create_subscription(LaserScan, "scan", self.callback, qos)
```

Note : ce correctif était déjà documenté dans ce suivi mais était resté en
commentaire dans le code. Il est désormais réellement appliqué.

---

## Modification `transformer.py` — sélection optimale des amers

**Date :** juin 2026  
**Changement :** ajout de la méthode `select_landmarks()` pour remplacer la sélection naïve par les 3 intensités maximales globales.

### Problème précédent
Prendre les 3 points les plus réfléchissants sans contrainte spatiale peut regrouper tous les amers dans le même secteur angulaire. Cela rend la matrice A de la trilatération quasi-singulière (lignes presque colinéaires) et produit une position calculée très imprécise ou infinie.

### Solution — double critère : secteur + intensité

Le scan est divisé en **3 secteurs de 120°** (0°–120°, 120°–240°, 240°–360°). Dans chaque secteur on prend le point à **intensité maximale**.

```
Secteur 0 : [  0°, 120°)  → amer le plus réfléchissant de ce secteur
Secteur 1 : [120°, 240°)  → amer le plus réfléchissant de ce secteur
Secteur 2 : [240°, 360°)  → amer le plus réfléchissant de ce secteur
```

**Pourquoi 120° :** répartition uniforme maximale pour 3 points, équivalente à un triangle équilatéral autour du robot. Plus l'angle entre deux amers est proche de 120°, mieux le déterminant de A est conditionné.

Si un secteur est vide (pas de points valides dans cette direction), la méthode retourne `None` et la trilatération est ignorée pour ce scan.

### Nouveau pipeline dans `callback()`
- `angles_valid` ajouté à la liste des données collectées dans la boucle
- Appel de `select_landmarks(intensities, angles_valid)` → retourne 3 indices ou `None`
- La trilatération n'est déclenchée que si `select_landmarks` réussit

---

## `launch/projet.launch.xml`

Lance les 3 nœuds simultanément :
- `detector_node` (`projet/detector.py`)
- `transformer_node` (`projet/transformer.py`)
- `intensity_filter_node` (`projet/intensity_filter.py`) avec `intensity_threshold=10000.0`

---

## `odompose.py`

**Source :** copie de `tp3/tp3/odom2pose.py`, renommée et classe renommée `OdomPose`.  
**Rôle :** estimer la pose du robot (x, y, θ) à partir de 3 sources de mesure.

### Pipeline
```
/sensor_state  →  [encodeurs]     →  /pose_enco (PoseStamped)
/imu           →  [gyroscope]     →  /pose_gyro (PoseStamped)
/magnetic_field → [magnétomètre] →  /pose_magn (PoseStamped)
```

### Trois méthodes d'estimation
| Méthode | Source θ | Source v | Limitation |
|---------|----------|----------|-----------|
| Encodeurs | Intégration cinématique | Roues | Nécessite `turtlebot3_msgs` |
| Gyroscope | Intégration `ω_z` de `/imu` | `self.v` des encodeurs | θ dérive dans le temps |
| Magnétomètre | `arctan2(B_y, B_x) − offset` | `self.v` des encodeurs | Sensible aux perturbations magnétiques |

**Note :** dans le bag `labyrinthe`, `/sensor_state` est présent mais `turtlebot3_msgs` n'est pas installé par défaut → `callback_enco` ne se déclenche pas, `self.v = 0.0`. La pose gyroscopique donne uniquement l'orientation correcte. Pour une position complète, installer `turtlebot3_msgs` ou utiliser `/odom` directement dans `pipeline.py`.

---

## `pipeline.py`

**Date :** juin 2026  
**Rôle :** nœud de fusion des 3 capteurs — LiDAR, odométrie et caméra — pour construire une carte du labyrinthe dans le référentiel global.

### Pipeline complet

```
Capteur        Topic entrant         Traitement
──────────────────────────────────────────────────────────────────
LiDAR points   /points_filtered  ─→  transformation R(θ)·p + t
LiDAR pos      /robot_position   ─→  mémorisation (rx, ry) trilatération
Odométrie      /pose_gyro        ─→  mémorisation (x, y, θ) gyroscope
Caméra         /detections       ─→  reçu (extensible pour fusion carte)

                                 ↓
                         /map_points (PointCloud2)   carte accumulée dans "odom"
```

### Filtre de cohérence entre LiDAR et odométrie

Avant de transformer les points, on vérifie que les deux estimées de position du robot sont cohérentes :

```
dist = √((rx_lidar − x_odom)² + (ry_lidar − y_odom)²)
```

Si `dist > position_threshold` (défaut 0.5 m), le scan est rejeté avec un warning. Ce filtre détecte les scans où la trilatération a divergé (amer mal identifié) ou l'odométrie a dérivé.

### Transformation robot → référentiel global

Pour chaque point LiDAR `(x_r, y_r)` dans le référentiel robot :
```
x_g = x_odom + x_r · cos(θ) − y_r · sin(θ)
y_g = y_odom + x_r · sin(θ) + y_r · cos(θ)
```
θ est extrait du quaternion de `/pose_gyro` : `θ = 2 · arctan2(q_z, q_w)`.

### Paramètres
| Paramètre | Défaut | Rôle |
|-----------|--------|------|
| `position_threshold` | `0.5` m | Distance maximale tolérée entre position LiDAR et odométrie |

### Choix d'implémentation
- θ calculé **directement depuis les encodeurs** via `callback_enco` (même méthode exacte que `odompose.py`) — pas d'extraction depuis un quaternion. Le nœud subscribe à `/sensor_state` et intègre `w*dt` à chaque tick.
- Les points s'accumulent indéfiniment dans `self.map_x/y/i` — adapté pour une session de ~2 min.
- `frame_id = "odom"` sur `/map_points` pour indiquer que les points sont dans le référentiel fixe.

### Modification — sources séparées pour θ et (x, y) (juin 2026)

Remplacement du callback quaternion par deux callbacks distincts, identiques à ceux de `odompose.py` :

| Quantité | Source | Callback | Méthode |
|----------|--------|----------|---------|
| θ (orientation) | Gyroscope `/imu` | `callback_gyro` | `O_gyro += ω_z · dt` |
| (x, y) (position) | Encodeurs `/sensor_state` | `callback_enco` | intégration cinématique différentielle |

`self.v` reste partagé entre les deux callbacks (calculé dans `callback_enco`, utilisé implicitement via la même logique que `odompose.py`).

Drapeaux `enco_ready` / `gyro_ready` : la transformation n'est déclenchée que lorsque les deux capteurs ont envoyé au moins un message.

### Transformation homogène 3×3 (juin 2026)

Remplacement du calcul scalaire par une matrice de transformation homogène explicite :

```
T = [ cos(θ)  -sin(θ)  x ]
    [ sin(θ)   cos(θ)  y ]
    [   0        0     1 ]
```

Points en coordonnées homogènes `[x_r, y_r, 1]`, transformation vectorisée : `(T @ points_h.T).T`.

---

## Corrections suite aux tests sur le bag labyrinthe (juin 2026)

### `setup.cfg` ajouté
**Problème :** `colcon build` plaçait les scripts dans `install/projet/bin/` au lieu de `install/projet/lib/projet/`, rendant `ros2 run` et `ros2 launch` incapables de trouver les nœuds.  
**Cause :** le fichier `setup.cfg` était absent du package `projet` (présent dans tp4/tp5).  
**Correction :** ajout de `setup.cfg` avec :
```ini
[develop]
script_dir=$base/lib/projet
[install]
install_scripts=$base/lib/projet
```
**Rebuild requis :** `colcon build --packages-select projet --symlink-install`

---

### `transformer.py` — QoS BEST_EFFORT sur `/scan`
**Problème :** warning `Incompatible QoS` — le bag publie `/scan` en `BEST_EFFORT`, le subscriber était `RELIABLE` par défaut → aucun message reçu.  
**Correction :** `QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)` sur le subscriber `/scan`.

---

### `odompose.py` et `pipeline.py` — imports optionnels
**Problème :** `turtlebot3_msgs` et `transforms3d` non installés → crash immédiat des deux nœuds.  
**Dépendances à installer dans WSL :**
```bash
sudo apt-get install -y ros-jazzy-turtlebot3-msgs
sudo apt-get install -y python3-transforms3d
```
**Correction dans le code :** import conditionnel avec `try/except ImportError` + flag `TURTLEBOT3_MSGS_AVAILABLE`. Si absent, le subscriber `/sensor_state` n'est pas créé et un warning est loggé au démarrage.

### `pipeline.py` — architecture finale
| Quantité | Source | Callback | Méthode |
|----------|--------|----------|---------|
| θ (orientation) | Gyroscope `/imu` | `callback_gyro` | `O_gyro += ω_z · dt` (identique tp3) |
| (x, y) (position) | Encodeurs `/sensor_state` | `callback_enco` | intégration cinématique différentielle (identique tp3) |

Garde de déclenchement : `gyro_ready` uniquement (sans encodeurs, la position reste à (0,0) mais la rotation gyro s'applique).

Transformation par matrice homogène 3×3 :
```
T = [ cos(θ)  -sin(θ)  x ]
    [ sin(θ)   cos(θ)  y ]
    [   0        0     1 ]
```

---

## À faire

- [x] Architecture initiale (detector, transformer, intensity_filter, pipeline, odompose)
- [x] Correction setup.cfg (lib/projet/ manquant)
- [x] Correction QoS sur /scan
- [x] Imports turtlebot3_msgs et transforms3d optionnels
- [x] Ajout rqt_image_view et rviz2 dans projet.launch.xml
- [ ] Installer `ros-jazzy-turtlebot3-msgs` et `python3-transforms3d` en WSL
- [ ] Calibration des positions des amers (`landmark_1/2/3`) dans le labyrinthe réel
- [ ] Clustering des flèches colorées détectées par caméra (centroïde par couleur)
- [ ] Propagation des points flèches sur la carte accumulée (bonus)
- [ ] `README.md` avec instructions de lancement (exigé pour la note)

---

## Alignement fonctions projet ↔ TPs (2026-06-06)

**Vérification complète** que chaque fichier Python du projet suit à l'identique les fonctions du TP correspondant :

| Fichier projet | TP de référence | Résultat |
|---|---|---|
| `transformer.py` | `tp4/transformer.py` | Fonctions communes identiques |
| `intensity_filter.py` | `tp4/intensity_filter.py` | Identique |
| `detector.py` | `tp5/detect.py` | Fonctions rouge identiques |
| `odompose.py` | `tp3/odom2pose.py` | Fonctions identiques |

**Correction apportée** dans `transformer.py` : suppression des deux lignes de code commenté (QoS BEST_EFFORT commenté, lié à une ancienne tentative de fix) et de l'import `QoSProfile`/`ReliabilityPolicy` devenu inutile.

---

## Ajout visualisation automatique dans le launch

**Date :** 2026-06-04

### Changement
`projet.launch.xml` lance désormais automatiquement :
- `rqt_image_view` abonné à `/detections` — affiche les flèches colorées détectées par la caméra
- `rviz2` — visualisation 3D des points LiDAR filtrés, poses odométriques et carte accumulée

### Pourquoi
Auparavant il fallait lancer manuellement ces deux outils dans des terminaux séparés après avoir démarré le pipeline. Désormais un seul `ros2 launch projet projet.launch.xml` ouvre tout.

### Modèle
Même pattern que `tp5/launch/camera.launch.xml` qui lance `rqt_image_view` directement.
