# 🎯 Base Placement Plugin - Architecture Refactorée

## 📌 Résumé Exécutif

Ce document résume la **refactorisation majeure** du plugin `base_placement_plugin` pour séparer les responsabilités entre **calcul**, **interface ROS2** et **visualisation RViz**.

---

## 🎨 Changements Architecturaux

### ❌ Ancienne Architecture (Monolithique)

```
┌─────────────────────────────────────┐
│  PlaceBase (QObject + Algorithmes)  │
│  ├─ Qt Signals/Slots                │
│  ├─ Algorithmes de calcul           │
│  ├─ Reachability data               │
│  └─ Visualisation markers           │
│            ↕                         │
│  AddWayPoint (RViz Panel)           │
│  └─ Connexion directe Qt            │
└─────────────────────────────────────┘
```

**Problèmes** :
- ❌ Couplage fort entre UI (Qt) et logique métier
- ❌ Impossible d'utiliser les algorithmes sans RViz
- ❌ Tests difficiles (mock Qt)
- ❌ Pas de feedback temps réel structuré

### ✅ Nouvelle Architecture (Modulaire)

```
┌───────────────────────────────────────────────────────┐
│                    RViz Plugin (UI)                    │
│  AddWayPoint + BasePlacementWidget                    │
│  └─ ROS2 Action Client + Service Clients             │
└────────────────────────┬──────────────────────────────┘
                         │ ROS2 Messages
                         ▼
┌───────────────────────────────────────────────────────┐
│         ROS2 Interfaces (base_placement_interfaces)    │
│  ├─ Action: FindBase (avec feedback)                  │
│  └─ Services: 7 services de configuration/données     │
└────────────────────────┬──────────────────────────────┘
                         │
                         ▼
┌───────────────────────────────────────────────────────┐
│          BasePlacementServer (ROS2 Node)               │
│  ├─ Action Server                                     │
│  ├─ 7 Service Servers                                 │
│  └─ Threading                                         │
└────────────────────────┬──────────────────────────────┘
                         │
                         ▼
┌───────────────────────────────────────────────────────┐
│      BasePlacementCore (Pure C++ - No Qt/RViz)         │
│  ├─ 5 Algorithmes de placement                        │
│  ├─ Reachability Maps                                 │
│  ├─ Task Poses (named)                                │
│  └─ Feedback callbacks                                │
└───────────────────────────────────────────────────────┘
```

**Avantages** :
- ✅ Séparation des responsabilités (UI ↔ ROS2 ↔ Calcul)
- ✅ `BasePlacementCore` réutilisable sans UI
- ✅ Feedback temps réel via action ROS2
- ✅ Tests unitaires faciles
- ✅ Serveur peut tourner sur machine distante
- ✅ Support multi-client

---

## 📦 Nouveaux Packages et Fichiers

### 1. Package `base_placement_interfaces`

**Nouveau package** pour les interfaces ROS2.

**Structure** :
```
base_placement_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   ├── WsSphere.msg
│   ├── WorkSpace.msg
│   └── PoseNamed.msg
├── srv/
│   ├── UpdateReachabilityMap.srv
│   ├── GetUnionMap.srv
│   ├── UpdateParameters.srv
│   ├── AddNamedPose.srv
│   ├── RemoveNamedPose.srv
│   ├── ClearMaps.srv
│   └── GetBasePoses.srv
└── action/
    └── FindBase.action
```

### 2. Nouveaux Fichiers dans `base_placement_plugin`

| Fichier | Description | Statut |
|---------|-------------|--------|
| `include/base_placement_plugin/base_placement_core.h` | Classe de calcul (sans Qt) | ✅ Créé |
| `src/base_placement_core.cpp` | Implémentation core | ✅ Créé (stubs) |
| `include/base_placement_plugin/base_placement_server.h` | Serveur ROS2 | ✅ Créé |
| `src/base_placement_server.cpp` | Implémentation serveur + main() | ✅ Créé |
| `docs/REFACTORING_GUIDE.md` | Documentation complète | ✅ Créé |
| `docs/refactored_architecture.mmd` | Diagramme Mermaid | ✅ Créé |

---

## 🔧 Interfaces ROS2

### Action : `find_base`

**Calcul itératif avec feedback en temps réel**

```yaml
# Goal
task_poses: PoseNamed[]
method_index: int32      # 0:PCA, 1:GraspScore, 2:IK, 3:Vertical, 4:User

# Feedback (publié à chaque itération)
current_phase: string
iteration: int32
progress_percentage: float64
current_best_score: float64

# Result
base_poses: Pose[]
scores: float64[]
best_score: float64
computation_time_seconds: float64
```

### 7 Services

1. **update_reachability_map** : Charger fichiers IRM/RM
2. **get_union_map** : Obtenir la carte d'union
3. **update_parameters** : Mettre à jour méthode, nombre de bases, etc.
4. **add_named_pose** : Ajouter une pose par nom
5. **remove_named_pose** : Supprimer une pose par nom
6. **clear_maps** : Effacer données (union map, reachability, poses, results)
7. **get_base_poses** : Récupérer les derniers résultats

---

## 🚀 Utilisation

### Compilation

```bash
cd /home/ros2_ws

# 1. Compiler les interfaces
colcon build --packages-select base_placement_interfaces
source install/setup.bash

# 2. Compiler le plugin
colcon build --packages-select base_placement_plugin
source install/setup.bash
```

### Lancement

**Terminal 1 : Serveur**
```bash
ros2 run base_placement_plugin base_placement_server
```

**Terminal 2 : RViz** (une fois adapté)
```bash
ros2 run rviz2 rviz2
# Charger le plugin base_placement_plugin
```

**Terminal 3 : Test CLI**
```bash
# Ajouter une pose
ros2 service call /add_named_pose base_placement_interfaces/srv/AddNamedPose \
  "{name: 'waypoint_1', pose: {position: {x: 1.0, y: 2.0, z: 0.5}, orientation: {w: 1.0}}}"

# Lancer le calcul
ros2 action send_goal /find_base base_placement_interfaces/action/FindBase \
  "{task_poses: [{name: 'waypoint_1', pose: {position: {x: 1.0, y: 2.0, z: 0.5}, orientation: {w: 1.0}}}], method_index: 1, num_base_locations: 5, num_high_score_spheres: 100}" \
  --feedback
```

---

## ✅ Ce qui est Fait

- [x] Package `base_placement_interfaces` créé
- [x] Messages définis (WsSphere, WorkSpace, PoseNamed)
- [x] 7 services définis
- [x] Action `FindBase` définie avec feedback
- [x] Classe `BasePlacementCore` créée (header + squelette)
- [x] Classe `BasePlacementServer` créée (header + implémentation complète)
- [x] Main function pour lancer le serveur standalone
- [x] Documentation complète ([REFACTORING_GUIDE.md](docs/REFACTORING_GUIDE.md))
- [x] Diagramme d'architecture ([refactored_architecture.mmd](docs/refactored_architecture.mmd))

---

## ⏳ Ce qui Reste à Faire

### Priorité 1 : Porter les Algorithmes

Les 5 algorithmes dans `BasePlacementCore` sont actuellement des **stubs** (placeholders).

**À faire** :
- [ ] Porter `findBaseByPCA()` depuis `PlaceBase::findBaseByPCA()`
- [ ] Porter `findBaseByGraspReachabilityScore()` depuis `PlaceBase`
- [ ] Porter `findBaseByIKSolutionScore()` depuis `PlaceBase`
- [ ] Porter `findBaseByVerticalRobotModel()` depuis `PlaceBase`
- [ ] Porter `findBaseByUserIntuition()` depuis `PlaceBase`

**Important** : Ajouter des appels à `feedback_callback()` dans les boucles pour publier la progression.

### Priorité 2 : Charger Reachability Maps

- [ ] Implémenter `BasePlacementCore::loadReachabilityFromFile()` avec HDF5
- [ ] Parser les fichiers `.h5` (IRM/RM)
- [ ] Stocker dans les structures de données appropriées

### Priorité 3 : Adapter RViz Plugin

**Modifier `AddWayPoint`** :
- [ ] Retirer dépendance Qt directe à `PlaceBase`
- [ ] Créer `rclcpp_action::Client<FindBase>` pour l'action
- [ ] Créer service clients pour les 7 services
- [ ] Implémenter callbacks de feedback pour mettre à jour l'UI
- [ ] Afficher progress bar basée sur feedback de l'action

**Modifier `BasePlacementWidget`** :
- [ ] Similaire à `AddWayPoint`, utiliser clients ROS2
- [ ] Mettre à jour l'UI en fonction du feedback

### Priorité 4 : Build System

- [ ] Mettre à jour `CMakeLists.txt` pour inclure les nouvelles classes
- [ ] Ajouter dépendance à `base_placement_interfaces`
- [ ] Créer un exécutable standalone pour `base_placement_server`

### Priorité 5 : Tests et Documentation

- [ ] Tests unitaires pour `BasePlacementCore`
- [ ] Tests d'intégration pour les services/action
- [ ] Créer un launch file pour le serveur
- [ ] Exemples Python pour utiliser l'action/services

---

## 📖 Documentation

- **Guide complet** : [docs/REFACTORING_GUIDE.md](docs/REFACTORING_GUIDE.md)
- **Diagramme architecture** : [docs/refactored_architecture.mmd](docs/refactored_architecture.mmd)
  - Visualiser sur [mermaid.live](https://mermaid.live)

---

## 🎓 Exemple de Migration

### Ancien Code (Qt Signals)

```cpp
// Dans AddWayPoint.cpp (ANCIEN)
connect(parseWayPointBtn, &QPushButton::clicked, this, [this]() {
  std::vector<geometry_msgs::msg::Pose> waypoints = parseWayPoints();
  emit wayPoints_signal(waypoints);
});

// PlaceBase reçoit le signal
void PlaceBase::findbase(std::vector<geometry_msgs::msg::Pose> poses) {
  // Calcul ici (bloque l'UI si long)
}
```

### Nouveau Code (Action ROS2)

```cpp
// Dans AddWayPoint.cpp (NOUVEAU)
#include <rclcpp_action/rclcpp_action.hpp>
#include <base_placement_interfaces/action/find_base.hpp>

// Initialisation (dans constructeur)
action_client_ = rclcpp_action::create_client<FindBase>(node_, "find_base");

// Callback du bouton
void AddWayPoint::onFindBaseClicked() {
  auto goal_msg = FindBase::Goal();

  // Remplir les poses
  for (const auto& pose : parseWayPoints()) {
    base_placement_interfaces::msg::PoseNamed named_pose;
    named_pose.name = "waypoint_" + std::to_string(goal_msg.task_poses.size());
    named_pose.pose = pose;
    goal_msg.task_poses.push_back(named_pose);
  }

  goal_msg.method_index = selected_method_;
  goal_msg.num_base_locations = 5;
  goal_msg.num_high_score_spheres = 100;

  // Options avec callbacks
  auto send_goal_options = rclcpp_action::Client<FindBase>::SendGoalOptions();

  // Feedback callback (appelé à chaque itération)
  send_goal_options.feedback_callback =
    [this](GoalHandleFindBase::SharedPtr,
           const std::shared_ptr<const FindBase::Feedback> feedback) {
      updateProgressBar(feedback->progress_percentage);
      updateStatusMessage(feedback->status_message);
    };

  // Result callback (appelé à la fin)
  send_goal_options.result_callback =
    [this](const GoalHandleFindBase::WrappedResult& result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        displayBasePoses(result.result->base_poses);
        showScore(result.result->best_score);
      }
    };

  // Envoyer le goal (non-bloquant)
  action_client_->async_send_goal(goal_msg, send_goal_options);
}
```

---

## 🔍 Voir Aussi

- **Diagrammes UML originaux** :
  - [architecture_diagram.mmd](architecture_diagram.mmd)
  - [sequence_diagram.mmd](sequence_diagram.mmd)
  - [component_diagram.mmd](component_diagram.mmd)

- **ROS2 Documentation** :
  - [Actions](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
  - [Services](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html)

---

## 📧 Contact

**Maintainer** : Guillaume Dupoiron
**Email** : guillaume.dupoiron@protonmail.com
**License** : Apache-2.0

---

**Date** : 2025-10-23
**Version** : 1.0.0 (Architecture refactorée)
