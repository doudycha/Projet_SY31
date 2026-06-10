# Lancer le projet SY31 — Guide pas à pas

## Contexte

Le projet tourne sous **ROS2 Jazzy**, installé dans **WSL2 Ubuntu** (sous-système Linux de Windows).
ROS2 ne s'installe pas via `pip` : ses packages (`rclpy`, `sensor_msgs`, `cv_bridge`…) font partie d'une distribution Linux complète. C'est pourquoi tout s'exécute côté Ubuntu, pas dans le Python Windows.

---

## Depuis Windows (PowerShell ou CMD)

Chaque commande ci-dessous utilise `wsl bash -c "..."` pour exécuter une commande dans WSL sans ouvrir un terminal Ubuntu.

### 1. Vérifier que WSL/Ubuntu tourne

```powershell
wsl --list --verbose
```

Tu dois voir `Ubuntu` avec l'état `Running` ou `Stopped`. Si `Stopped`, la première commande `wsl bash -c` le démarrera automatiquement.

---

### 2. Builder le package ROS2

À faire une seule fois, ou après chaque modification du code Python.

```powershell
wsl bash -c "source /opt/ros/jazzy/setup.bash && cd '/mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws' && colcon build --packages-select tp5"
```

**Pourquoi `source /opt/ros/jazzy/setup.bash` ?**
ROS2 n'est pas dans le PATH par défaut. Ce script ajoute tous les outils ROS2 (`ros2`, `colcon`, etc.) à la session courante.

**Pourquoi `colcon build` ?**
ROS2 utilise `colcon` comme système de build. Il compile le package et génère un dossier `install/` contenant les exécutables et les métadonnées nécessaires à `ros2 run`.

---

### 3. Lancer le nœud de détection

```powershell
wsl bash -c "source /opt/ros/jazzy/setup.bash && source '/mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws/install/setup.bash' && ros2 run tp5 detect_node"
```

**Pourquoi deux `source` ?**
- Le premier (`/opt/ros/jazzy/setup.bash`) charge ROS2 lui-même.
- Le second (`install/setup.bash`) charge *ton* workspace, c'est-à-dire le nœud `detect_node` que tu viens de builder. Sans lui, `ros2 run tp5 detect_node` échouerait avec "package not found".

**Ce que fait le nœud :**
Il s'abonne au topic `/turtlecam/image_raw/compressed`, traite chaque image (détection de rouge par masque HSV + contours OpenCV), et publie le résultat sur `/detections`.

---

### 4. Jouer le bag ROS2 (dans un autre terminal PowerShell)

Ouvre un **deuxième PowerShell** et lance :

```powershell
wsl bash -c "source /opt/ros/jazzy/setup.bash && ros2 bag play '/mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/labyrinthe/' --loop"
```

**Pourquoi un deuxième terminal ?**
Le nœud de détection et le bag player doivent tourner *en même temps*. Chacun bloque le terminal tant qu'il tourne.

**Pourquoi `--loop` ?**
Sans cette option, le bag s'arrête après ~115 secondes (durée de l'enregistrement). Avec `--loop`, il repart automatiquement depuis le début.

**Note sur le topic image :**
Le bag contient `/turtlecam/image_raw/compressed` (images JPEG compressées) et non `/turtlecam/image_raw` (images brutes). C'est pourquoi `detect.py` a été modifié pour s'abonner au topic compressé et décoder avec `cv2.imdecode`.

---

### 5. Vérifier que tout tourne (optionnel)

```powershell
# Lister les nœuds actifs
wsl bash -c "source /opt/ros/jazzy/setup.bash && ros2 node list"

# Vérifier la fréquence de publication des détections (~12 Hz attendu)
wsl bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic hz /detections"
```

---

### 6. Visualiser avec rqt_image_view

`rqt_image_view` est une application graphique. Elle nécessite un affichage, ce qui ne fonctionne **pas** bien depuis `wsl bash -c` (session non-interactive). Il faut l'ouvrir depuis un terminal Ubuntu interactif (voir section suivante).

---

## Depuis Ubuntu WSL (terminal interactif)

Lance Ubuntu depuis le menu Démarrer ou depuis Windows Terminal (profil Ubuntu).
C'est la méthode recommandée pour tout ce qui nécessite un affichage graphique (rqt, rviz2).

### 1. Sourcer l'environnement (à faire dans chaque nouveau terminal)

```bash
source /opt/ros/jazzy/setup.bash
source /mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws/install/setup.bash
```

Pour ne pas avoir à le retaper à chaque fois, tu peux l'ajouter à ton `~/.bashrc` :

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source /mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws/install/setup.bash" >> ~/.bashrc
```

### 2. Builder le package

```bash
cd /mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/sy31_ws
colcon build --packages-select tp5
```

### 3. Terminal 1 — Lancer le nœud de détection

```bash
ros2 run tp5 detect_node
```

### 4. Terminal 2 — Jouer le bag

```bash
ros2 bag play /mnt/c/Users/Maxime/OneDrive/Bureau/Travail/UTC/P26/SY31/Projet/labyrinthe/ --loop
```

### 5. Terminal 3 — Visualiser les détections

```bash
ros2 run rqt_image_view rqt_image_view
```

Une fenêtre s'ouvre. Dans le menu déroulant en haut, sélectionne `/detections` pour voir le flux vidéo avec les contours détectés.

---

## Résumé des chemins importants

| Élément | Chemin |
|---|---|
| Distribution ROS2 | `/opt/ros/jazzy/` |
| Workspace | `/mnt/c/Users/Maxime/.../sy31_ws/` |
| Package tp5 | `sy31_ws/src/tp5/tp5/` |
| Bag labyrinthe | `/mnt/c/Users/Maxime/.../labyrinthe/` |
| Build output | `sy31_ws/install/` |

## Accès Windows ↔ Linux

Depuis Ubuntu WSL, le disque `C:` Windows est monté sous `/mnt/c/`.
Donc `C:\Users\Maxime\...` devient `/mnt/c/Users/Maxime/...`.

Depuis Windows, tu peux aussi accéder au filesystem Ubuntu via `\\wsl$\Ubuntu\` dans l'explorateur de fichiers.
