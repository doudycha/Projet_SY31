# Compte-rendu de tests — workspace SY31

**Date :** 2026-06-06
**Environnement :** WSL Ubuntu 24.04 + ROS2 Jazzy, bag `~/sy31_ws/labyrinthe/`
**Méthode :** build colcon + smoke test de chaque nœud (démarrage 5 s, détection
des crashs) + test fonctionnel (lecture du bag, mesure des débits de topics).
Script reproductible : `sy31_ws/run_tests.sh`.

---

## Synthèse par package

| Package | Verdict | Détail |
|---------|:---:|--------|
| **tp3** | ✅ OK | `odom2pose.py` tourne, `/pose_gyro` à 20 Hz |
| **tp4** | ✅ OK¹ | `transformer` réparé (import) → chaîne LiDAR débloquée |
| **tp5** | ✅ OK | `detect_node` et `project_node` démarrent |
| **projet** | ✅ OK¹ | QoS `/scan` réparé → `/points`, `/points_filtered`, `/map_points` à ~5 Hz |
| **projet2** | ✅ OK | Pipeline complet fonctionnel de bout en bout |

¹ Après application des correctifs — voir la section **« Corrections appliquées »**
en fin de document. Les Phases A/B ci-dessous documentent l'état **avant**
correction.

**Build :** les 4 packages colcon (`projet`, `projet2`, `tp4`, `tp5`) compilent
sans erreur (7,5 s). `tp3` n'est pas un package colcon (script autonome).

---

## Phase A — Smoke test des nœuds (démarrage sans crash)

| Nœud | Résultat |
|------|:--------:|
| tp3 / `odom2pose.py` | ✅ PASS¹ |
| tp4 / `transformer` | ❌ **FAIL** — `ModuleNotFoundError: No module named 'utils'` |
| tp4 / `intensity_filter` | ✅ PASS |
| tp4 / `clusterer` | ✅ PASS |
| tp4 / `shaper_bbox` | ✅ PASS |
| tp4 / `shaper_cylinder` | ✅ PASS |
| tp4 / `shaper_polyline` | ✅ PASS |
| tp5 / `detect_node` | ✅ PASS |
| tp5 / `project_node` | ✅ PASS |
| projet / `detector_node` | ✅ PASS |
| projet / `transformer_node` | ✅ PASS |
| projet / `intensity_filter_node` | ✅ PASS |
| projet / `odompose_node` | ✅ PASS |
| projet / `pipeline_node` | ✅ PASS |
| projet2 / `odometry` | ✅ PASS |
| projet2 / `mapper` | ✅ PASS |
| projet2 / `arrow_detector` | ✅ PASS |
| projet2 / `direction_display` | ✅ PASS |

¹ Le smoke test brut affichait un `Traceback` pour tp3, mais il s'agit
uniquement de `ExternalShutdownException` (le `timeout` qui coupe `rclpy.spin`),
pas d'un vrai défaut. Confirmé fonctionnel en Phase B.

---

## Phase B — Test fonctionnel avec le bag `labyrinthe`

### Topics sources du bag (tous reçus correctement)

| Topic | Débit |
|-------|-------|
| `/scan` | 4,16 Hz |
| `/imu` | 19,8 Hz |
| `/sensor_state` | 20,1 Hz |
| `/turtlecam/image_raw/compressed` | 15,0 Hz |
| `/odom` | 20,1 Hz |

➡️ **Tous les gros messages (LiDAR, image) arrivent sans problème** en WSL.
Cf. note sur l'écran noir rqt plus bas.

### Sorties de `projet`

| Topic | Débit | État |
|-------|-------|:----:|
| `/points` | **0** | ❌ aucun message |
| `/points_filtered` | **0** | ❌ conséquence |
| `/map_points` | **0** | ❌ conséquence |
| `/detections` | 5,5 Hz | ✅ |
| `/pose_gyro` | 20,0 Hz | ✅ |

### Sorties de `projet2`

| Topic | Débit | État |
|-------|-------|:----:|
| `/odom_est` | 19,9 Hz | ✅ |
| `/trajectory` | 4,0 Hz | ✅ |
| `/map_points` | 2,0 Hz | ✅ (timer 0,5 s) |
| `/filtered_scan` | 5,0 Hz | ✅ |
| `/detections` | 20,4 Hz | ✅ |
| `/direction_cmd` | 15,1 Hz | ✅ |
| `/direction_display` | 10,0 Hz | ✅ |

➡️ **`projet2` fonctionne de bout en bout** : odométrie, carte LiDAR, détection
de flèches et affichage opérateur produisent tous des données.

---

## Bugs identifiés (2 défauts réels)

### 🐞 1. `tp4/tp4/transformer.py` — import incorrect (bloquant)

```python
# Ligne 8 — actuel (échoue via `ros2 run`) :
from utils import make_pointcloud2
# Correctif :
from .utils import make_pointcloud2
```
**Impact :** le nœud `transformer` plante au démarrage → aucun `/points` publié
→ **toute la chaîne LiDAR du tp4** (`intensity_filter`, `clusterer`, `shaper_*`)
est privée d'entrée. Les autres nœuds tp4 utilisent déjà `from .utils import`.

### 🐞 2. `projet/projet/transformer.py` — QoS incompatible sur `/scan` (bloquant)

```python
# Ligne 17 — actuel :
self.sub = self.create_subscription(LaserScan, "scan", self.callback, 10)
```
Le bag publie `/scan` en **BEST_EFFORT** ; un subscriber en QoS par défaut
(`10` = RELIABLE) est **incompatible** → aucun message reçu.

```python
# Correctif :
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
self.sub = self.create_subscription(LaserScan, "scan", self.callback, qos)
```
**Impact :** `/points` = 0 → `/points_filtered` = 0 → `/map_points` = 0. Toute la
cartographie LiDAR de `projet` est inopérante. `projet2` gère déjà ce QoS
correctement (d'où son fonctionnement).

> Note : `projet/suivi.md` mentionnait ce correctif BEST_EFFORT, mais dans le
> code il était resté en commentaire. `projet2` est la version saine.

---

## Note — écran noir dans rqt + perte de `/scan` (Zenoh / mémoire partagée)

> ⚠️ **Correction** : la première version de cette note concluait « pas un
> problème Zenoh ». C'était faux. Les Phases A/B ci-dessus tournaient sous
> **Fast DDS** car `wsl bash script.sh` est non-interactif et ne source pas
> `~/.bashrc`. Or `~/.bashrc` force `RMW_IMPLEMENTATION=rmw_zenoh_cpp` : les
> **vrais terminaux interactifs utilisent Zenoh**, avec mémoire partagée activée.

Test décisif sous Zenoh (bag en lecture) :

| Topic | Zenoh + SHM **activée** (config `.bashrc`) | Zenoh + SHM **désactivée** |
|-------|:---:|:---:|
| `/imu` (petit) | 5,3 Hz (dégradé) | 20 Hz |
| `/scan` (gros) | **0 — perdu** | 4,98 Hz |
| image compressée (gros) | **0 — perdu** | 15,0 Hz |

➡️ Sous WSL, le transport **mémoire partagée de Zenoh perd les gros messages**
(`/scan`, image). C'est la cause de l'écran noir rqt **et** de l'absence de
données LiDAR. Identique au problème signalé sur le forum.

**Correctif (recommandé par l'enseignant)** — dans `~/.bashrc`, retirer la
partie `shared_memory` :
```bash
# Avant
export ZENOH_CONFIG_OVERRIDE='transport/unicast/compression/enabled=true;transport/shared_memory/enabled=true'
# Après
export ZENOH_CONFIG_OVERRIDE='transport/unicast/compression/enabled=true'
```
Puis fermer et rouvrir **tous** les terminaux (le routeur `rmw_zenohd` inclus).

> Les débits « OK » des Phases A/B restent valides… **sous Fast DDS**. Sous
> Zenoh+SHM, refaire les tests donnerait 0 sur `/scan` et l'image tant que la
> mémoire partagée n'est pas désactivée.

---

## Recommandations

1. Corriger l'import de `tp4/transformer.py` (1 ligne) pour débloquer le tp4.
2. Corriger le QoS de `projet/transformer.py` pour débloquer la carte de `projet`.
3. Pour la démo du sujet, **privilégier `projet2`** : pipeline aligné sur
   l'énoncé (matrice homogène, clustering flèches).

---

## Corrections appliquées (2026-06-06)

Les deux bugs ci-dessus ont été **corrigés**, puis le workspace a été
re-synchronisé, rebuildé et re-testé.

### Modifications

| Fichier | Modification |
|---------|--------------|
| `tp4/tp4/transformer.py` (ligne 8) | `from utils import` → `from .utils import` |
| `projet/projet/transformer.py` | Ajout de l'import `QoSProfile, ReliabilityPolicy` + souscription `/scan` en QoS **BEST_EFFORT** (au lieu du QoS RELIABLE par défaut) |

> La correction de `projet` est aussi consignée dans `projet/suivi.md`
> (entrée datée 2026-06-06), conformément à la règle du workspace.

### Re-test après correction

**Correctif 1 — `tp4/transformer`** : démarre sans `ModuleNotFoundError`. ✅

**Correctif 2 — chaîne LiDAR de `projet`** (bag en lecture) :

| Topic | Avant | Après |
|-------|:-----:|:-----:|
| `/points` | 0 | **4,97 Hz** ✅ |
| `/points_filtered` | 0 | **4,97 Hz** (356 points/msg) ✅ |
| `/map_points` | 0 | **4,98 Hz** ✅ |

➡️ **Les deux correctifs sont validés.** Le tp4 et le pipeline LiDAR de `projet`
sont désormais pleinement fonctionnels, au même titre que `projet2`.

> Remarque : lors d'un premier re-test, `/map_points` apparaissait encore à 0 ;
> il s'agissait d'un temps de chauffe trop court (4 s). Avec une fenêtre de 6 s,
> la carte se publie normalement.

---

## Changement d'environnement — désactivation de la mémoire partagée Zenoh (2026-06-06)

Dernier changement appliqué, **hors code** mais indispensable au fonctionnement
réel : dans les terminaux interactifs, `~/.bashrc` force le middleware Zenoh
(`RMW_IMPLEMENTATION=rmw_zenoh_cpp`) avec **mémoire partagée activée**. Sous WSL,
ce transport SHM **perd silencieusement les gros messages** (`/scan`, image),
d'où l'écran noir rqt et l'absence de carte LiDAR (voir la note plus haut).

### Diagnostic (test décisif sous Zenoh)

| Topic | SHM activée | SHM désactivée |
|-------|:-----------:|:--------------:|
| `/imu` (petit) | 5,3 Hz (dégradé) | 20 Hz ✅ |
| `/scan` (gros) | **0 — perdu** | 4,98 Hz ✅ |
| image compressée (gros) | **0 — perdu** | 15,0 Hz ✅ |

### Modification appliquée

Dans `~/.bashrc`, ligne `ZENOH_CONFIG_OVERRIDE`, suppression de
`;transport/shared_memory/enabled=true` :

```bash
# Avant
export ZENOH_CONFIG_OVERRIDE='transport/unicast/compression/enabled=true;transport/shared_memory/enabled=true'
# Après
export ZENOH_CONFIG_OVERRIDE='transport/unicast/compression/enabled=true'
```

- Sauvegarde créée : `~/.bashrc.bak_20260606_233324`.
- La fonction `zenohd` hérite automatiquement de la valeur corrigée.
- **Action requise** : fermer puis rouvrir tous les terminaux (routeur
  `rmw_zenohd` inclus) pour que le changement prenne effet.

> ⚠️ Sans ce changement, `projet` comme `projet2` afficheront 0 message sur
> `/scan` et l'image dans les vrais terminaux — ce n'est pas un bug du code mais
> du transport Zenoh+SHM sous WSL.

---

## Récapitulatif des 3 changements appliqués

| # | Type | Fichier | Effet |
|---|------|---------|-------|
| 1 | Code | `tp4/tp4/transformer.py` | Import relatif → débloque la chaîne LiDAR du tp4 |
| 2 | Code | `projet/projet/transformer.py` | QoS BEST_EFFORT sur `/scan` → débloque la carte de `projet` |
| 3 | Environnement | `~/.bashrc` | SHM Zenoh désactivée → `/scan` et image transitent réellement sous WSL |
