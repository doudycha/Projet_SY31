# Lancer le projet sur le bag labyrinthe

## Prérequis

- WSL2 Ubuntu avec ROS2 Jazzy installé
- Le bag `labyrinthe/` présent dans `C:\Users\Maxime\OneDrive\Bureau\Travail\UTC\P26\SY31\Projet\`

---

## Étape 1 — Builder le package (une seule fois, ou après modif du code)

Depuis **PowerShell** :

```powershell
wsl bash -c "source /opt/ros/jazzy/setup.bash && cd '/mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws' && colcon build --packages-select projet"
```

Depuis un **terminal Ubuntu WSL** :

```bash
source /opt/ros/jazzy/setup.bash
cd /mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws
colcon build --packages-select projet
```

---

## Étape 2 — Lancer les nœuds du projet

Ouvre un terminal (PowerShell ou Ubuntu WSL) et lance le fichier de lancement :

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

Cela démarre simultanément :
- `detector` — détecte les formes rouges et bleues depuis la caméra
- `transformer` — convertit le LiDAR en nuage de points cartésien
- `intensity_filter` — filtre les points selon leur réflectivité

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

`--loop` repart automatiquement au début quand le bag se termine (durée ~115 s).

---

## Étape 4 — Visualiser les détections (terminal Ubuntu WSL uniquement)

Les fenêtres graphiques nécessitent un terminal Ubuntu interactif (pas PowerShell).

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Dans la fenêtre qui s'ouvre, sélectionne le topic **`/detections`** dans le menu déroulant.
Tu verras le flux vidéo avec :
- Les contours **verts** autour des formes rouges détectées
- Les contours **jaunes** autour des formes bleues détectées

---

## Réglage du seuil d'intensité LiDAR

Le filtre d'intensité supprime les points LiDAR peu réflectifs. La valeur par défaut est `10000.0`.
Tu peux la modifier à chaud sans relancer les nœuds :

```bash
ros2 param set /intensity_filter intensity_threshold 8000.0
```

Pour voir la valeur actuelle :

```bash
ros2 param get /intensity_filter intensity_threshold
```

---

## Vérifier que tout tourne

```bash
# Lister les nœuds actifs
ros2 node list

# Vérifier la fréquence de publication des détections (~12 Hz attendu)
ros2 topic hz /detections

# Vérifier les points LiDAR filtrés
ros2 topic hz /points_filtered
```

---

## Résumé des topics

| Topic | Type | Producteur | Consommateur |
|-------|------|-----------|--------------|
| `/turtlecam/image_raw/compressed` | CompressedImage | bag | `detector` |
| `/detections` | Image | `detector` | rqt_image_view |
| `/scan` | LaserScan | bag | `transformer` |
| `/points` | PointCloud2 | `transformer` | `intensity_filter` |
| `/points_filtered` | PointCloud2 | `intensity_filter` | — |
