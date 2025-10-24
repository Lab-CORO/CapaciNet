# 🎯 Plan de Refactorisation - Base Placement Plugin

## 📋 Objectifs de la Refactorisation

Vous souhaitez séparer le projet `base_placement_plugin` en deux parties distinctes :

1. **🧮 Partie Calcul** : Classe pure C++ (sans Qt, sans RViz)
2. **🖼️ Partie RViz** : Interface utilisateur qui communique avec la partie calcul via ROS2

### Communication ROS2

- ✅ **1 Action** : `find_base` avec feedback itératif pour suivre le calcul
- ✅ **7 Services** pour la configuration et la gestion des données

---

## ✅ Ce qui a été Créé

### 1. Package d'Interfaces ROS2 (`base_placement_interfaces`)

**Emplacement** : `/home/ros2_ws/src/CapaciNet/base_placement_interfaces/`

#### Messages (3)
- `WsSphere.msg` : Sphère de workspace avec rayon et poses
- `WorkSpace.msg` : Collection de sphères workspace
- `PoseNamed.msg` : Pose avec nom unique

#### Services (7)

| Service | Description |
|---------|-------------|
| `UpdateReachabilityMap.srv` | Charger fichiers IRM/RM (HDF5) |
| `GetUnionMap.srv` | Obtenir la carte d'union |
| `UpdateParameters.srv` | Modifier méthode, nombre de bases, etc. |
| `AddNamedPose.srv` | Ajouter une pose par nom |
| `RemoveNamedPose.srv` | Supprimer une pose par nom |
| `ClearMaps.srv` | Effacer données (maps, poses, résultats) |
| `GetBasePoses.srv` | Récupérer les derniers résultats calculés |

#### Action (1)

**`FindBase.action`** : Action principale pour calculer les placements

**Goal** :
```yaml
PoseNamed[] task_poses           # Poses de tâches
int32 method_index               # 0:PCA, 1:GraspScore, 2:IK, 3:Vertical, 4:User
int32 num_base_locations
int32 num_high_score_spheres
```

**Feedback** (publié à chaque itération) :
```yaml
string current_phase             # Ex: "Computing scores"
int32 iteration
int32 total_iterations
float64 progress_percentage      # 0-100
string status_message
int32 candidates_evaluated
float64 current_best_score
```

**Result** :
```yaml
bool success
Pose[] base_poses
float64[] scores
float64 best_score
float64 computation_time_seconds
```

---

### 2. Classe de Calcul Pur (`BasePlacementCore`)

**Fichiers** :
- [`include/base_placement_plugin/base_placement_core.h`](include/base_placement_plugin/base_placement_core.h)
- [`src/base_placement_core.cpp`](src/base_placement_core.cpp)

**Caractéristiques** :
- ✅ Aucune dépendance Qt
- ✅ Aucune dépendance RViz
- ✅ Seulement C++ standard + ROS2 + bibliothèques mathématiques
- ✅ Peut être réutilisée dans n'importe quel nœud ROS2

**Méthodes principales** :

```cpp
// Configuration
void setParameters(Method method, int num_base_locations, int num_high_score_spheres);
void setMethod(Method method);

// Chargement données
bool setReachabilityData(...);
bool loadReachabilityFromFile(string irm_path, string rm_path);

// Gestion poses
bool addNamedPose(string name, Pose pose);
bool removeNamedPose(string name);
void clearTaskPoses();

// Calcul principal
ComputationResult findBasePlacements(
  vector<Pose> task_poses,
  FeedbackCallback callback  // Callback pour publier progression
);

// Résultats
vector<Pose> getComputedBasePoses();
vector<double> getComputedScores();
double getBestScore();
```

**5 Algorithmes** (méthodes protégées) :
1. `findBaseByPCA()`
2. `findBaseByGraspReachabilityScore()`
3. `findBaseByIKSolutionScore()`
4. `findBaseByVerticalRobotModel()`
5. `findBaseByUserIntuition()`

⚠️ **Note** : Les algorithmes sont actuellement des **stubs** (placeholders). Ils doivent être portés depuis `PlaceBase`.

---

### 3. Serveur ROS2 (`BasePlacementServer`)

**Fichiers** :
- [`include/base_placement_plugin/base_placement_server.h`](include/base_placement_plugin/base_placement_server.h)
- [`src/base_placement_server.cpp`](src/base_placement_server.cpp)

**Rôle** : Nœud ROS2 exposant action et services

**Composants** :
- ✅ **Action Server** : `find_base`
- ✅ **7 Service Servers** (un par service défini)
- ✅ **Threading** : Action exécutée dans thread séparé
- ✅ **Instance de BasePlacementCore**

**Main function** : Le fichier `base_placement_server.cpp` contient une fonction `main()` pour lancer le serveur standalone :

```bash
ros2 run base_placement_plugin base_placement_server
```

---

## 📊 Architecture Visuelle

Voir les diagrammes :
- **Architecture complète** : [docs/refactored_architecture.mmd](docs/refactored_architecture.mmd)
- **Guide détaillé** : [docs/REFACTORING_GUIDE.md](docs/REFACTORING_GUIDE.md)

---

## 🚧 Ce qui Reste à Faire

### Tâche 1 : Porter les Algorithmes ⏳

Les 5 algorithmes dans `BasePlacementCore` sont des stubs.

**Action requise** :
1. Ouvrir `src/place_base.cpp` (ancien code)
2. Copier la logique de chaque algorithme dans `src/base_placement_core.cpp`
3. **Important** : Ajouter des appels au `feedback_callback` pour publier la progression

**Exemple** :
```cpp
BasePlacementCore::ComputationResult
BasePlacementCore::findBaseByGraspReachabilityScore(
  const std::vector<geometry_msgs::msg::Pose>& task_poses,
  FeedbackCallback feedback_callback)
{
  ComputationResult result;

  // ... initialisation ...

  int total_candidates = 1000; // exemple

  for (int i = 0; i < total_candidates; ++i) {
    // 1. Calculer le score pour le candidat i
    // ... votre logique ici ...

    // 2. Publier le feedback
    if (feedback_callback) {
      double progress = (double)(i + 1) / total_candidates * 100.0;
      feedback_callback(
        "Computing grasp reachability scores",  // phase
        i + 1,                                   // iteration
        total_candidates,                        // total_iterations
        progress,                                // progress_percentage
        "Evaluating candidate " + std::to_string(i),
        i + 1,                                   // candidates_evaluated
        current_best_score                       // meilleur score actuel
      );
    }
  }

  // ... finalisation ...
  result.success = true;
  result.base_poses = computed_poses;
  result.scores = scores;
  // ...

  return result;
}
```

### Tâche 2 : Implémenter le Chargement HDF5 ⏳

**Fichier** : `src/base_placement_core.cpp`

**Méthode** : `loadReachabilityFromFile()`

**À faire** :
```cpp
bool BasePlacementCore::loadReachabilityFromFile(
  const std::string& irm_file_path,
  const std::string& rm_file_path)
{
  // Utiliser HighFive pour charger le fichier HDF5
  // Exemple:
  try {
    HighFive::File file(irm_file_path, HighFive::File::ReadOnly);

    // Lire les datasets
    // ...

    // Remplir pose_col_filter_, sphere_col_, resolution_
    // ...

    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load HDF5: %s", e.what());
    return false;
  }
}
```

### Tâche 3 : Adapter le Plugin RViz ⏳

**Fichiers à modifier** :
- `include/base_placement_plugin/add_way_point.h`
- `src/add_way_point.cpp`
- `include/base_placement_plugin/widgets/base_placement_widget.h`
- `src/widgets/base_placement_widget.cpp`

**Changements requis** :

#### Dans le Header (.h)

**Ancien** :
```cpp
class AddWayPoint : public rviz_common::Panel {
  Q_OBJECT
  // ...
  PlaceBase* place_base_;  // ❌ Dépendance directe
};
```

**Nouveau** :
```cpp
#include <rclcpp_action/rclcpp_action.hpp>
#include <base_placement_interfaces/action/find_base.hpp>
#include <base_placement_interfaces/srv/add_named_pose.hpp>
// ... autres services ...

class AddWayPoint : public rviz_common::Panel {
  Q_OBJECT
public:
  using FindBase = base_placement_interfaces::action::FindBase;
  using GoalHandleFindBase = rclcpp_action::ClientGoalHandle<FindBase>;

private:
  // Action client
  rclcpp_action::Client<FindBase>::SharedPtr action_client_;

  // Service clients
  rclcpp::Client<base_placement_interfaces::srv::AddNamedPose>::SharedPtr srv_add_pose_;
  rclcpp::Client<base_placement_interfaces::srv::RemoveNamedPose>::SharedPtr srv_remove_pose_;
  // ... autres services ...

  // Callbacks
  void feedback_callback(
    GoalHandleFindBase::SharedPtr,
    const std::shared_ptr<const FindBase::Feedback> feedback);

  void result_callback(const GoalHandleFindBase::WrappedResult& result);
};
```

#### Dans l'Implémentation (.cpp)

**Initialisation** :
```cpp
AddWayPoint::AddWayPoint(QWidget* parent)
  : rviz_common::Panel(parent)
{
  // Créer l'action client
  action_client_ = rclcpp_action::create_client<FindBase>(
    node_, "find_base"
  );

  // Créer les service clients
  srv_add_pose_ = node_->create_client<base_placement_interfaces::srv::AddNamedPose>(
    "add_named_pose"
  );
  // ... autres services ...
}
```

**Envoi du Goal** :
```cpp
void AddWayPoint::onFindBaseClicked() {
  // Préparer le goal
  auto goal_msg = FindBase::Goal();

  // Remplir les poses
  auto waypoints = parseWayPoints();
  for (size_t i = 0; i < waypoints.size(); ++i) {
    base_placement_interfaces::msg::PoseNamed named_pose;
    named_pose.name = "waypoint_" + std::to_string(i);
    named_pose.pose = waypoints[i];
    goal_msg.task_poses.push_back(named_pose);
  }

  goal_msg.method_index = selected_method_;
  goal_msg.num_base_locations = 5;
  goal_msg.num_high_score_spheres = 100;

  // Préparer les options avec callbacks
  auto send_goal_options = rclcpp_action::Client<FindBase>::SendGoalOptions();

  send_goal_options.feedback_callback =
    std::bind(&AddWayPoint::feedback_callback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&AddWayPoint::result_callback, this, _1);

  // Envoyer le goal (non-bloquant)
  action_client_->async_send_goal(goal_msg, send_goal_options);
}

void AddWayPoint::feedback_callback(
  GoalHandleFindBase::SharedPtr,
  const std::shared_ptr<const FindBase::Feedback> feedback)
{
  // Mettre à jour l'UI
  emit updateProgressBar(feedback->progress_percentage);
  emit updateStatusText(QString::fromStdString(feedback->status_message));
  emit updateBestScore(feedback->current_best_score);
}

void AddWayPoint::result_callback(const GoalHandleFindBase::WrappedResult& result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    // Succès
    emit basePlacementCompleted(result.result->best_score);
    visualizeBasePoses(result.result->base_poses);
  } else {
    // Échec ou annulation
    emit basePlacementFailed(QString::fromStdString(result.result->message));
  }
}
```

**Appel de Service** (exemple pour ajouter une pose) :
```cpp
void AddWayPoint::addWaypoint(const geometry_msgs::msg::Pose& pose) {
  auto request = std::make_shared<base_placement_interfaces::srv::AddNamedPose::Request>();
  request->name = "waypoint_" + std::to_string(waypoint_counter_++);
  request->pose = pose;

  auto result_future = srv_add_pose_->async_send_request(request);

  // Optionnel : attendre la réponse (ou utiliser callback)
  if (rclcpp::spin_until_future_complete(node_, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result_future.get();
    if (response->success) {
      RCLCPP_INFO(node_->get_logger(), "Pose added. Total: %d", response->total_poses);
    }
  }
}
```

### Tâche 4 : Mettre à Jour CMakeLists.txt ⏳

**Fichier** : `CMakeLists.txt`

**Ajouts nécessaires** :

```cmake
# Dépendances supplémentaires
find_package(base_placement_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

# Ajouter aux dependencies
set(dependencies
  rclcpp
  rclcpp_action                    # ← NOUVEAU
  base_placement_interfaces        # ← NOUVEAU
  # ... autres dépendances ...
)

# Nouvelle source pour BasePlacementCore
set(CORE_SOURCE_FILES
  src/base_placement_core.cpp
  src/sphere_discretization.cpp
  # ... autres utilitaires sans Qt ...
)

# Source pour le serveur ROS2
set(SERVER_SOURCE_FILES
  src/base_placement_server.cpp
  ${CORE_SOURCE_FILES}
)

# Créer l'exécutable du serveur
add_executable(base_placement_server
  ${SERVER_SOURCE_FILES}
)

target_include_directories(base_placement_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(base_placement_server
  ${dependencies}
)

target_link_libraries(base_placement_server
  ${HDF5_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  yaml-cpp
  ${HighFive_LIBRARIES}
)

# Installer l'exécutable
install(TARGETS base_placement_server
  DESTINATION lib/${PROJECT_NAME}
)

# Le plugin RViz continue d'utiliser la bibliothèque partagée
# Mais doit maintenant inclure base_placement_core.cpp
set(SOURCE_FILES
  src/place_base.cpp                    # ← Peut être déprécié à terme
  src/base_placement_core.cpp           # ← NOUVEAU
  src/add_way_point.cpp
  src/add_robot_base.cpp
  src/create_marker.cpp
  src/point_tree_item.cpp
  src/point_tree_model.cpp
  src/sphere_discretization.cpp
  src/widgets/base_placement_widget.cpp
  ${UIC_FILES}
  ${MOC_FILES}
)
```

### Tâche 5 : Créer un Launch File ⏳

**Fichier** : `launch/base_placement_system.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lancer le serveur de calcul
        Node(
            package='base_placement_plugin',
            executable='base_placement_server',
            name='base_placement_server',
            output='screen',
            parameters=[{
                # Paramètres par défaut
                'default_method': 1,  # GraspReachabilityScore
                'default_num_base_locations': 5,
                'default_num_high_score_spheres': 100,
            }]
        ),

        # Lancer RViz2 (optionnel)
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', '/path/to/config.rviz']
        # ),
    ])
```

**Utilisation** :
```bash
ros2 launch base_placement_plugin base_placement_system.launch.py
```

---

## 🧪 Tests

### Test 1 : Compiler les Interfaces

```bash
cd /home/ros2_ws
colcon build --packages-select base_placement_interfaces
source install/setup.bash

# Vérifier que les messages sont générés
ros2 interface show base_placement_interfaces/action/FindBase
ros2 interface show base_placement_interfaces/srv/UpdateReachabilityMap
```

### Test 2 : Compiler le Serveur

```bash
colcon build --packages-select base_placement_plugin
source install/setup.bash

# Lancer le serveur
ros2 run base_placement_plugin base_placement_server
```

### Test 3 : Tester l'Action en CLI

```bash
# Dans un autre terminal
ros2 action send_goal /find_base base_placement_interfaces/action/FindBase \
  "{task_poses: [{name: 'test', pose: {position: {x: 1.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}], method_index: 0, num_base_locations: 3, num_high_score_spheres: 50}" \
  --feedback
```

### Test 4 : Tester les Services

```bash
# Ajouter une pose
ros2 service call /add_named_pose base_placement_interfaces/srv/AddNamedPose \
  "{name: 'pose1', pose: {position: {x: 1.0, y: 2.0, z: 0.5}, orientation: {w: 1.0}}}"

# Mettre à jour les paramètres
ros2 service call /update_parameters base_placement_interfaces/srv/UpdateParameters \
  "{method_index: 1, num_base_locations: 5, num_high_score_spheres: 100, visualization_type: 0}"

# Récupérer les résultats
ros2 service call /get_base_poses base_placement_interfaces/srv/GetBasePoses
```

---

## 📚 Documentation

### Fichiers Créés

1. **[REFACTORING_README.md](REFACTORING_README.md)** : Vue d'ensemble en anglais
2. **[docs/REFACTORING_GUIDE.md](docs/REFACTORING_GUIDE.md)** : Guide détaillé complet
3. **[docs/refactored_architecture.mmd](docs/refactored_architecture.mmd)** : Diagramme Mermaid de l'architecture
4. **[PLAN_REFACTORISATION.md](PLAN_REFACTORISATION.md)** : Ce document (en français)

### Visualiser les Diagrammes

1. Aller sur [mermaid.live](https://mermaid.live)
2. Copier-coller le contenu de `docs/refactored_architecture.mmd`
3. Visualiser l'architecture interactive

---

## ✅ Checklist Finale

### Phase 1 : Fondations (✅ Terminé)
- [x] Créer package `base_placement_interfaces`
- [x] Définir messages (WsSphere, WorkSpace, PoseNamed)
- [x] Définir 7 services
- [x] Définir action `FindBase`
- [x] Créer classe `BasePlacementCore` (header + squelette)
- [x] Créer classe `BasePlacementServer` (header + implémentation)
- [x] Documentation complète

### Phase 2 : Implémentation (⏳ À Faire)
- [ ] Porter les 5 algorithmes dans `BasePlacementCore`
- [ ] Implémenter `loadReachabilityFromFile()` avec HDF5
- [ ] Ajouter les appels `feedback_callback` dans les algorithmes
- [ ] Mettre à jour CMakeLists.txt pour compiler le serveur
- [ ] Tester la compilation du serveur standalone

### Phase 3 : Adaptation RViz (⏳ À Faire)
- [ ] Modifier `AddWayPoint` pour utiliser action client
- [ ] Modifier `BasePlacementWidget` pour utiliser service clients
- [ ] Implémenter les callbacks de feedback
- [ ] Mettre à jour l'UI en fonction de la progression
- [ ] Tester l'intégration RViz ↔ Serveur

### Phase 4 : Finalisation (⏳ À Faire)
- [ ] Créer launch file
- [ ] Tests unitaires pour `BasePlacementCore`
- [ ] Tests d'intégration action/services
- [ ] Exemples Python
- [ ] Documentation utilisateur finale

---

## 🎉 Résumé

Vous disposez maintenant d'une **architecture modulaire et réutilisable** :

✅ **BasePlacementCore** : Calcul pur, sans UI
✅ **BasePlacementServer** : Serveur ROS2 standalone
✅ **Interfaces ROS2** : Communication standard (action + services)
✅ **Plugin RViz** : Interface utilisateur découplée

**Prochaines étapes** :
1. Porter les algorithmes
2. Implémenter le chargement HDF5
3. Adapter le plugin RViz pour utiliser l'action client

Bonne chance ! 🚀
