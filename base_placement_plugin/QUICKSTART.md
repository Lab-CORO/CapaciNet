# 🚀 Quick Start - Base Placement Server

## Compilation Rapide

```bash
cd /home/ros2_ws

# 1. Compiler les interfaces
colcon build --packages-select base_placement_interfaces
source install/setup.bash

# 2. Compiler le plugin
colcon build --packages-select base_placement_plugin
source install/setup.bash
```

## Lancer le Serveur

**Terminal 1 : Serveur**
```bash
cd /home/ros2_ws
source install/setup.bash
ros2 run base_placement_plugin base_placement_server
```

Ou directement :
```bash
/home/ros2_ws/install/base_placement_plugin/bin/base_placement_server
```

**Sortie attendue** :
```
[INFO] [base_placement_server]: Initializing BasePlacementServer
[INFO] [base_placement_server]: BasePlacementServer initialized successfully
[INFO] [base_placement_server]:   Action: find_base
[INFO] [base_placement_server]:   Services: update_reachability_map, get_union_map, update_parameters,
[INFO] [base_placement_server]:             add_named_pose, remove_named_pose, clear_maps, get_base_poses
[INFO] [base_placement_server]: BasePlacementCore initialized
[INFO] [base_placement_server]: BasePlacementServer spinning...
```

## Tests Rapides

**Terminal 2 : Tests**

### Test 1 : Ajouter une Pose

```bash
ros2 service call /add_named_pose base_placement_interfaces/srv/AddNamedPose \
  "{name: 'waypoint_1', pose: {position: {x: 1.0, y: 2.0, z: 0.5}, orientation: {w: 1.0}}}"
```

**Réponse attendue** :
```yaml
success: true
message: "Pose added successfully"
total_poses: 1
```

### Test 2 : Mettre à Jour les Paramètres

```bash
ros2 service call /update_parameters base_placement_interfaces/srv/UpdateParameters \
  "{method_index: 1, num_base_locations: 5, num_high_score_spheres: 100, visualization_type: 0}"
```

**Réponse attendue** :
```yaml
success: true
message: "Parameters updated successfully"
```

### Test 3 : Lancer l'Action (Calcul)

```bash
ros2 action send_goal /find_base base_placement_interfaces/action/FindBase \
  "{task_poses: [{name: 'wp1', pose: {position: {x: 1.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}], method_index: 0, num_base_locations: 3, num_high_score_spheres: 50}" \
  --feedback
```

**Note** : Les algorithmes sont actuellement des stubs, donc le résultat sera vide. Cela permet de vérifier que la communication fonctionne.

### Test 4 : Récupérer les Résultats

```bash
ros2 service call /get_base_poses base_placement_interfaces/srv/GetBasePoses
```

### Test 5 : Nettoyer

```bash
ros2 service call /clear_maps base_placement_interfaces/srv/ClearMaps \
  "{clear_union_map: true, clear_reachability_data: false, clear_task_poses: true, clear_computed_bases: true}"
```

## Vérifier les Interfaces Disponibles

```bash
# Lister tous les services
ros2 service list | grep -E "(add_named|update_param|clear_maps)"

# Lister les actions
ros2 action list | grep find_base

# Voir la définition d'un service
ros2 interface show base_placement_interfaces/srv/AddNamedPose

# Voir la définition de l'action
ros2 interface show base_placement_interfaces/action/FindBase
```

## Interfaces ROS2 Créées

### Messages (3)
- `base_placement_interfaces/msg/WsSphere`
- `base_placement_interfaces/msg/WorkSpace`
- `base_placement_interfaces/msg/PoseNamed`

### Services (7)
1. `/update_reachability_map` - Charger IRM/RM
2. `/get_union_map` - Obtenir carte union
3. `/update_parameters` - Configurer méthode/paramètres
4. `/add_named_pose` - Ajouter pose
5. `/remove_named_pose` - Supprimer pose
6. `/clear_maps` - Nettoyer données
7. `/get_base_poses` - Récupérer résultats

### Action (1)
- `/find_base` - Calculer placement avec feedback

## Troubleshooting

### Erreur : "No executable found"

Assurez-vous de sourcer l'environnement :
```bash
source /home/ros2_ws/install/setup.bash
```

### Erreur : "Package 'base_placement_interfaces' not found"

Recompilez les interfaces :
```bash
cd /home/ros2_ws
rm -rf build/base_placement_interfaces install/base_placement_interfaces
colcon build --packages-select base_placement_interfaces
source install/setup.bash
```

### Le serveur ne démarre pas

Vérifiez les logs et recompilez :
```bash
cd /home/ros2_ws
colcon build --packages-select base_placement_plugin
source install/setup.bash
```

## Prochaines Étapes

1. **Porter les algorithmes** depuis `PlaceBase` vers `BasePlacementCore`
2. **Implémenter le chargement HDF5** dans `loadReachabilityFromFile()`
3. **Adapter le plugin RViz** pour utiliser l'action client

Voir [PLAN_REFACTORISATION.md](PLAN_REFACTORISATION.md) pour les détails.

---

**Documentation complète** : [docs/REFACTORING_GUIDE.md](docs/REFACTORING_GUIDE.md)
