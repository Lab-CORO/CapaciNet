# Base Placement Plugin - Architecture Refactorée

## 📋 Table des Matières

1. [Vue d'ensemble](#vue-densemble)
2. [Architecture](#architecture)
3. [Packages](#packages)
4. [Interfaces ROS2](#interfaces-ros2)
5. [Classes Principales](#classes-principales)
6. [Flux de Données](#flux-de-données)
7. [Guide d'Utilisation](#guide-dutilisation)
8. [Migration depuis l'Ancienne Architecture](#migration)

---

## 🎯 Vue d'ensemble

Cette refactorisation sépare clairement les responsabilités entre :
- **Calcul** : Algorithmes de placement de base (sans dépendance Qt/RViz)
- **Interface ROS2** : Services et actions pour communication inter-processus
- **Visualisation** : Plugin RViz pour interaction utilisateur

### Avantages de la Nouvelle Architecture

✅ **Séparation des préoccupations** : Le code de calcul est indépendant de l'UI
✅ **Réutilisabilité** : `BasePlacementCore` peut être utilisé sans RViz
✅ **Testabilité** : Tests unitaires faciles sur la logique métier
✅ **Scalabilité** : Le serveur peut tourner sur une machine dédiée
✅ **Feedback en temps réel** : Action ROS2 avec progression itérative

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    RViz2 Environment                             │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  AddWayPoint (RViz Panel)                                  │ │
│  │  - UI pour définir waypoints                               │ │
│  │  - Marqueurs interactifs 3D                                │ │
│  └────────────────────────────────────────────────────────────┘ │
└──────────────────────────┬──────────────────────────────────────┘
                           │ Action Client
                           │ Service Clients
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│              ROS2 Interfaces (base_placement_interfaces)         │
│  ┌────────────┬──────────────────┬────────────────────────────┐ │
│  │  Messages  │    Services      │        Actions             │ │
│  ├────────────┼──────────────────┼────────────────────────────┤ │
│  │ WsSphere   │ UpdateReachMap   │ FindBase                   │ │
│  │ WorkSpace  │ GetUnionMap      │   - Goal: task poses       │ │
│  │ PoseNamed  │ UpdateParameters │   - Feedback: progress     │ │
│  │            │ AddNamedPose     │   - Result: base poses     │ │
│  │            │ RemoveNamedPose  │                            │ │
│  │            │ ClearMaps        │                            │ │
│  │            │ GetBasePoses     │                            │ │
│  └────────────┴──────────────────┴────────────────────────────┘ │
└──────────────────────────┬──────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│          BasePlacementServer (ROS2 Node)                         │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  - Action Server: find_base                                │ │
│  │  - 7 Service Servers                                       │ │
│  │  - Gestion du threading                                    │ │
│  │  - Publication de feedback                                 │ │
│  └──────────────────────┬─────────────────────────────────────┘ │
└─────────────────────────┼──────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│         BasePlacementCore (Pure C++ - No Qt/RViz)                │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  5 Algorithmes de Placement:                               │ │
│  │  1️⃣ PCA (Principal Component Analysis)                     │ │
│  │  2️⃣ GraspReachabilityScore                                 │ │
│  │  3️⃣ IKSolutionScore                                        │ │
│  │  4️⃣ VerticalRobotModel                                     │ │
│  │  5️⃣ UserIntuition                                          │ │
│  │                                                            │ │
│  │  Données:                                                  │ │
│  │  - Reachability Maps (IRM/RM)                              │ │
│  │  - Task Poses (named)                                      │ │
│  │  - Union Maps                                              │ │
│  │  - Computed Base Poses                                     │ │
│  └────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────┐
│              SphereDiscretization (Geometric Utils)              │
│  - Fibonacci sphere generation                                  │
│  - Archimedes spiral                                            │
│  - PCA optimization                                             │
│  - OctoMap integration                                          │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📦 Packages

### 1. `base_placement_interfaces`

**Description** : Package ROS2 contenant toutes les définitions d'interfaces

**Contenu** :
- `msg/` : Messages customisés
  - `WsSphere.msg`
  - `WorkSpace.msg`
  - `PoseNamed.msg`
- `srv/` : Définitions de services
  - `UpdateReachabilityMap.srv`
  - `GetUnionMap.srv`
  - `UpdateParameters.srv`
  - `AddNamedPose.srv`
  - `RemoveNamedPose.srv`
  - `ClearMaps.srv`
  - `GetBasePoses.srv`
- `action/` : Définitions d'actions
  - `FindBase.action`

**Dépendances** :
- `geometry_msgs`
- `std_msgs`
- `action_msgs`

### 2. `base_placement_plugin`

**Description** : Package principal contenant le code de calcul et l'interface ROS2

**Structure** :
```
base_placement_plugin/
├── include/base_placement_plugin/
│   ├── base_placement_core.h          # Classe de calcul (sans Qt)
│   ├── base_placement_server.h        # Serveur ROS2 action/services
│   ├── place_base.h                   # [Déprécié] Ancienne classe Qt
│   ├── add_way_point.h                # Plugin RViz (à adapter)
│   ├── create_marker.h                # Visualisation
│   ├── sphere_discretization.h        # Algorithmes géométriques
│   └── ...
├── src/
│   ├── base_placement_core.cpp        # Implémentation calcul
│   ├── base_placement_server.cpp      # Implémentation serveur + main()
│   ├── place_base.cpp                 # [Déprécié]
│   ├── add_way_point.cpp              # À adapter pour utiliser action
│   └── ...
└── docs/
    ├── REFACTORING_GUIDE.md           # Ce document
    └── functional_diagram.md
```

---

## 🔌 Interfaces ROS2

### Action : `FindBase`

**Topic** : `find_base`

**Goal** :
```yaml
PoseNamed[] task_poses              # Poses de tâches nommées
int32 method_index                  # 0-4 (PCA, GraspScore, IK, Vertical, User)
int32 num_base_locations            # Nombre de positions à calculer
int32 num_high_score_spheres        # Nombre de sphères high-score
```

**Feedback** :
```yaml
string current_phase                # Phase actuelle (ex: "Computing scores")
int32 iteration                     # Itération courante
int32 total_iterations              # Total d'itérations
float64 progress_percentage         # 0.0 - 100.0
string status_message               # Message détaillé
int32 candidates_evaluated          # Candidats évalués
float64 current_best_score          # Meilleur score actuel
```

**Result** :
```yaml
bool success
string message
Pose[] base_poses                   # Positions optimales calculées
float64[] scores                    # Score de chaque position
float64 best_score                  # Meilleur score
int32 best_index                    # Index de la meilleure position
float64 computation_time_seconds    # Temps de calcul
```

### Services

#### 1. `update_reachability_map`

Charger les fichiers IRM (Inverse Reachability Map) et RM.

**Request** :
```yaml
string irm_file_path
string rm_file_path
bool load_irm
bool load_rm
```

**Response** :
```yaml
bool success
string message
int32 num_spheres_loaded
float32 resolution
```

#### 2. `get_union_map`

Obtenir la carte d'union (visualisation de reachability combinée).

**Request** :
```yaml
bool compute_from_current_poses
```

**Response** :
```yaml
bool success
string message
WorkSpace union_map
```

#### 3. `update_parameters`

Mettre à jour les paramètres de calcul.

**Request** :
```yaml
int32 method_index                  # 0-4
int32 num_base_locations
int32 num_high_score_spheres
int32 visualization_type            # 0: Arrow, 1: RobotModel, 2: ArmModel
```

**Response** :
```yaml
bool success
string message
```

#### 4. `add_named_pose`

Ajouter une pose nommée.

**Request** :
```yaml
string name
Pose pose
```

**Response** :
```yaml
bool success
string message
int32 total_poses
```

#### 5. `remove_named_pose`

Supprimer une pose par nom.

**Request** :
```yaml
string name
```

**Response** :
```yaml
bool success
string message
int32 total_poses
```

#### 6. `clear_maps`

Effacer différentes données.

**Request** :
```yaml
bool clear_union_map
bool clear_reachability_data
bool clear_task_poses
bool clear_computed_bases
```

**Response** :
```yaml
bool success
string message
```

#### 7. `get_base_poses`

Récupérer les dernières positions calculées.

**Request** : (vide)

**Response** :
```yaml
bool success
string message
Pose[] base_poses
float64[] scores
float64 best_score
int32 best_index
```

---

## 🔧 Classes Principales

### `BasePlacementCore`

**Fichier** : [`base_placement_core.h`](../include/base_placement_plugin/base_placement_core.h)

**Responsabilité** : Logique de calcul pure (sans Qt, sans RViz)

**Méthodes principales** :
- `setReachabilityData()` : Charger les données de reachability
- `addNamedPose()` / `removeNamedPose()` : Gestion des poses
- `findBasePlacements()` : Calcul principal avec callback de feedback
- `getUnionMap()` : Calculer la carte d'union
- `clearAllData()` : Nettoyage

**Algorithmes** (méthodes protégées) :
- `findBaseByPCA()`
- `findBaseByGraspReachabilityScore()`
- `findBaseByIKSolutionScore()`
- `findBaseByVerticalRobotModel()`
- `findBaseByUserIntuition()`

### `BasePlacementServer`

**Fichier** : [`base_placement_server.h`](../include/base_placement_plugin/base_placement_server.h)

**Responsabilité** : Serveur ROS2 exposant action et services

**Contient** :
- `action_server_` : Action `FindBase`
- 7 service servers
- Instance de `BasePlacementCore`

**Threading** : L'action s'exécute dans un thread séparé pour ne pas bloquer l'executor ROS2.

### `AddWayPoint` (à adapter)

**Fichier** : [`add_way_point.h`](../include/base_placement_plugin/add_way_point.h)

**Responsabilité** : Plugin RViz pour interaction utilisateur

**À modifier** :
- ❌ Retirer la dépendance directe à `PlaceBase` (Qt)
- ✅ Utiliser un **Action Client** pour `FindBase`
- ✅ Utiliser des **Service Clients** pour les autres opérations
- ✅ Recevoir le feedback et afficher la progression dans l'UI

---

## 📊 Flux de Données

### Scénario 1 : Chargement de Reachability Map

```
User (RViz UI)
    │
    └──> Clic "Load Reachability Map"
         │
         └──> Service Client: update_reachability_map
              │
              └──> BasePlacementServer
                   │
                   └──> BasePlacementCore::loadReachabilityFromFile()
                        │
                        └──> Parse HDF5, stocke données
                             │
                             └──> Response: success + num_spheres
```

### Scénario 2 : Ajout de Waypoints

```
User (RViz)
    │
    └──> Place marqueur interactif
         │
         └──> AddWayPoint::processFeedback()
              │
              └──> Service Client: add_named_pose
                   │
                   └──> BasePlacementServer
                        │
                        └──> BasePlacementCore::addNamedPose()
                             │
                             └──> Response: total_poses
```

### Scénario 3 : Calcul de Base Placement (Action)

```
User (RViz UI)
    │
    └──> Clic "Find Base"
         │
         └──> Action Client: send_goal(find_base)
              │
              ├──> BasePlacementServer::handle_goal()
              │    │
              │    └──> Accept & Execute in thread
              │
              ├──> BasePlacementServer::execute_find_base()
              │    │
              │    ├──> BasePlacementCore::findBasePlacements()
              │    │    │
              │    │    ├──> Iteration 1
              │    │    │    └──> Callback → Publish Feedback
              │    │    │         └──> UI updates progress bar
              │    │    │
              │    │    ├──> Iteration 2
              │    │    │    └──> Callback → Publish Feedback
              │    │    │
              │    │    └──> ... (iterations)
              │    │
              │    └──> Return Result
              │         │
              │         └──> goal_handle->succeed(result)
              │
              └──> Action Client: result_callback()
                   │
                   └──> UI: Display base poses
                        └──> Service: get_base_poses (si nécessaire)
```

---

## 🚀 Guide d'Utilisation

### Compilation

```bash
cd /home/ros2_ws

# Compiler le package d'interfaces
colcon build --packages-select base_placement_interfaces

# Source
source install/setup.bash

# Compiler le package principal
colcon build --packages-select base_placement_plugin

# Source à nouveau
source install/setup.bash
```

### Lancer le Serveur

```bash
ros2 run base_placement_plugin base_placement_server
```

### Tester avec CLI

#### Charger une Reachability Map

```bash
ros2 service call /update_reachability_map \
  base_placement_interfaces/srv/UpdateReachabilityMap \
  "{irm_file_path: '/path/to/irm.h5', rm_file_path: '', load_irm: true, load_rm: false}"
```

#### Ajouter une Pose

```bash
ros2 service call /add_named_pose \
  base_placement_interfaces/srv/AddNamedPose \
  "{name: 'waypoint_1', pose: {position: {x: 1.0, y: 2.0, z: 0.5}, orientation: {w: 1.0}}}"
```

#### Mettre à jour les Paramètres

```bash
ros2 service call /update_parameters \
  base_placement_interfaces/srv/UpdateParameters \
  "{method_index: 1, num_base_locations: 5, num_high_score_spheres: 100, visualization_type: 0}"
```

#### Lancer le Calcul (Action)

```bash
ros2 action send_goal /find_base \
  base_placement_interfaces/action/FindBase \
  "{task_poses: [{name: 'waypoint_1', pose: {position: {x: 1.0, y: 2.0, z: 0.5}, orientation: {w: 1.0}}}], method_index: 1, num_base_locations: 5, num_high_score_spheres: 100}" \
  --feedback
```

#### Récupérer les Résultats

```bash
ros2 service call /get_base_poses \
  base_placement_interfaces/srv/GetBasePoses
```

#### Nettoyer les Données

```bash
ros2 service call /clear_maps \
  base_placement_interfaces/srv/ClearMaps \
  "{clear_union_map: true, clear_reachability_data: false, clear_task_poses: true, clear_computed_bases: true}"
```

---

## 🔄 Migration depuis l'Ancienne Architecture

### Étapes à Suivre

#### 1. ✅ Interfaces créées
- Package `base_placement_interfaces` avec messages, services, actions

#### 2. ✅ Core de calcul extrait
- Classe `BasePlacementCore` sans dépendance Qt
- Algorithmes à porter depuis `PlaceBase`

#### 3. ✅ Serveur ROS2 créé
- `BasePlacementServer` avec action et services
- Main function pour exécution standalone

#### 4. ⏳ Adapter le plugin RViz (TODO)

Modifier `AddWayPoint` pour utiliser les clients ROS2 :

**Ancien code (PlaceBase avec Qt signals)** :
```cpp
// ANCIEN
connect(this, &AddWayPoint::wayPoints_signal,
        place_base_, &PlaceBase::findbase);
```

**Nouveau code (Action Client)** :
```cpp
// NOUVEAU
#include <rclcpp_action/rclcpp_action.hpp>
#include <base_placement_interfaces/action/find_base.hpp>

using FindBase = base_placement_interfaces::action::FindBase;
using GoalHandleFindBase = rclcpp_action::ClientGoalHandle<FindBase>;

// Dans AddWayPoint class:
rclcpp_action::Client<FindBase>::SharedPtr action_client_;

// Initialisation:
action_client_ = rclcpp_action::create_client<FindBase>(node_, "find_base");

// Envoi du goal:
auto goal_msg = FindBase::Goal();
goal_msg.task_poses = /* ... */;
goal_msg.method_index = selected_method_;

auto send_goal_options = rclcpp_action::Client<FindBase>::SendGoalOptions();
send_goal_options.feedback_callback =
  std::bind(&AddWayPoint::feedback_callback, this, _1, _2);
send_goal_options.result_callback =
  std::bind(&AddWayPoint::result_callback, this, _1);

action_client_->async_send_goal(goal_msg, send_goal_options);
```

#### 5. ⏳ Porter les Algorithmes (TODO)

Les méthodes suivantes dans `BasePlacementCore` sont actuellement des stubs :
- `findBaseByPCA()` → Porter depuis `PlaceBase::findBaseByPCA()`
- `findBaseByGraspReachabilityScore()` → Porter depuis `PlaceBase`
- `findBaseByIKSolutionScore()` → Porter depuis `PlaceBase`
- `findBaseByVerticalRobotModel()` → Porter depuis `PlaceBase`
- `findBaseByUserIntuition()` → Porter depuis `PlaceBase`

**Important** : Ajouter des appels au `feedback_callback` dans les boucles d'itération pour publier la progression.

Exemple :
```cpp
BasePlacementCore::ComputationResult
BasePlacementCore::findBaseByGraspReachabilityScore(
  const std::vector<geometry_msgs::msg::Pose>& task_poses,
  FeedbackCallback feedback_callback)
{
  ComputationResult result;

  // ... initialisation ...

  int total_candidates = /* ... */;

  for (int i = 0; i < total_candidates; ++i) {
    // Calcul pour le candidat i
    // ...

    // Publier le feedback
    if (feedback_callback) {
      double progress = (double)(i + 1) / total_candidates * 100.0;
      feedback_callback(
        "Computing grasp reachability scores",  // phase
        i,                                       // iteration
        total_candidates,                        // total_iterations
        progress,                                // progress_percentage
        "Evaluating candidate " + std::to_string(i), // status_message
        i + 1,                                   // candidates_evaluated
        current_best_score                       // current_best_score
      );
    }
  }

  // ... finaliser résultat ...

  return result;
}
```

---

## 📈 Avantages de la Nouvelle Architecture

### Avant (Monolithique)

❌ PlaceBase hérite de QObject (dépendance Qt forte)
❌ Couplage fort entre calcul et UI
❌ Impossible de réutiliser sans RViz
❌ Tests difficiles (mock Qt signals/slots)
❌ Threading complexe avec QtConcurrent dans la logique métier

### Après (Modulaire)

✅ `BasePlacementCore` : C++ pur, aucune dépendance UI
✅ Interface ROS2 standard (action + services)
✅ Réutilisable dans n'importe quel nœud ROS2
✅ Tests unitaires simples sur `BasePlacementCore`
✅ Feedback en temps réel via action ROS2
✅ Possibilité de lancer le serveur sur une machine distante
✅ Support multi-client (plusieurs RViz peuvent se connecter)

---

## 🛠️ Tâches Restantes

- [ ] Porter les algorithmes depuis `PlaceBase` vers `BasePlacementCore`
- [ ] Implémenter `loadReachabilityFromFile()` avec HDF5
- [ ] Adapter `AddWayPoint` pour utiliser action/service clients
- [ ] Adapter `BasePlacementWidget` pour utiliser action/service clients
- [ ] Créer des tests unitaires pour `BasePlacementCore`
- [ ] Créer un launch file pour démarrer le serveur
- [ ] Documenter les paramètres ROS2
- [ ] Créer des exemples d'utilisation Python

---

## 📞 Contact

**Maintainer** : Guillaume Dupoiron
**Email** : guillaume.dupoiron@protonmail.com
**License** : Apache-2.0

---

**Date de création** : 2025-10-23
**Version** : 1.0.0 (Architecture refactorée)
