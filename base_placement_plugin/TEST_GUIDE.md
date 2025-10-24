# Guide de Test - Architecture Refactorisée

## Préparation

### 1. Compilation
```bash
cd /home/ros2_ws
colcon build --packages-select base_placement_interfaces base_placement_plugin
source install/setup.bash
```

### 2. Vérification
```bash
# Vérifier que les packages sont installés
ros2 pkg list | grep base_placement
# Devrait afficher:
# base_placement_interfaces
# base_placement_plugin
```

## Test 1: Lancement du Serveur Standalone

### Objectif
Vérifier que le serveur `base_placement_server` démarre correctement.

### Commandes
```bash
# Terminal 1: Lancer le serveur
cd /home/ros2_ws
source install/setup.bash
ros2 run base_placement_plugin base_placement_server
```

### Résultat Attendu
```
[INFO] [base_placement_server]: Initializing BasePlacementServer
[INFO] [base_placement_server]: BasePlacementServer initialized successfully
[INFO] [base_placement_server]:   Action: find_base
[INFO] [base_placement_server]:   Services: update_reachability_map, get_union_map, update_parameters,
[INFO] [base_placement_server]:             add_named_pose, remove_named_pose, clear_maps, get_base_poses
[INFO] [base_placement_server]: BasePlacementCore initialized
[INFO] [base_placement_server]: BasePlacementServer spinning...
```

✅ **Succès**: Le serveur est en attente de requêtes.

## Test 2: Vérification des Interfaces

### Objectif
Vérifier que l'action et les services sont disponibles.

### Commandes
```bash
# Terminal 2 (le serveur doit tourner dans Terminal 1)
source /home/ros2_ws/install/setup.bash

# Lister les actions disponibles
ros2 action list
# Devrait afficher: /find_base

# Lister les services disponibles
ros2 service list | grep base
# Devrait afficher:
# /add_named_pose
# /clear_maps
# /get_base_poses
# /get_union_map
# /remove_named_pose
# /update_parameters
# /update_reachability_map

# Afficher le type de l'action
ros2 action info /find_base
# Devrait afficher le type et le nombre de clients/serveurs
```

✅ **Succès**: Tous les services et l'action sont visibles.

## Test 3: Test d'un Service Simple

### Objectif
Tester le service `update_parameters`.

### Commande
```bash
# Terminal 2
ros2 service call /update_parameters base_placement_interfaces/srv/UpdateParameters \
  "{method_index: 0, num_base_locations: 5, num_high_score_spheres: 100}"
```

### Résultat Attendu
```
response:
base_placement_interfaces.srv.UpdateParameters_Response(success=True, message='Parameters updated: method=PCA, num_base_locations=5, num_high_score_spheres=100')
```

✅ **Succès**: Le service répond correctement.

## Test 4: Test du Service Clear Maps

### Commande
```bash
ros2 service call /clear_maps base_placement_interfaces/srv/ClearMaps \
  "{clear_union_map: true, clear_task_poses: true}"
```

### Résultat Attendu
```
response:
base_placement_interfaces.srv.ClearMaps_Response(success=True, message='Cleared union map and task poses')
```

✅ **Succès**: Le nettoyage fonctionne.

## Test 5: Test d'Ajout de Pose Nommée

### Commande
```bash
ros2 service call /add_named_pose base_placement_interfaces/srv/AddNamedPose \
  "{name: 'test_pose', pose: {position: {x: 1.0, y: 2.0, z: 0.5}, orientation: {w: 1.0}}}"
```

### Résultat Attendu
```
response:
base_placement_interfaces.srv.AddNamedPose_Response(success=True, message='...')
```

✅ **Succès**: La pose est ajoutée.

## Test 6: Envoi d'un Goal d'Action (Minimal)

### Objectif
Tester l'action `find_base` avec un goal minimal.

### Commande
```bash
# Envoyer un goal simple
ros2 action send_goal /find_base base_placement_interfaces/action/FindBase \
  "{task_poses: [{name: 'pose1', pose: {position: {x: 1.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}], \
    method_index: 4, num_base_locations: 1, num_high_score_spheres: 10}" \
  --feedback
```

### Résultat Attendu
Comme la méthode 4 (UserIntuition) nécessite des poses de base prédéfinies, ce test échouera proprement:

```
Goal accepted with ID: ...
Feedback:
  current_phase: UserIntuition
  ...
Result:
  success: False
  message: 'No user-defined base poses available. Use setUserBasePoses() first.'
  ...
```

✅ **Succès**: L'action répond et gère l'erreur correctement.

## Test 7: Lancement avec RViz

### Objectif
Tester l'intégration complète avec le plugin RViz.

### Commandes
```bash
# Terminal 1: Lancer le serveur
cd /home/ros2_ws
source install/setup.bash
ros2 run base_placement_plugin base_placement_server

# Terminal 2: Lancer RViz avec le plugin
source install/setup.bash
rviz2
```

### Dans RViz:
1. Charger la configuration qui inclut le plugin `BasePlacementPlugin`
2. Le plugin devrait se connecter automatiquement au serveur
3. Vérifier les logs du serveur (Terminal 1) pour voir la connexion

✅ **Succès**: Le plugin RViz se connecte au serveur.

## Test 8: Test Complet de Workflow

### Prérequis
- Serveur lancé (Terminal 1)
- RViz avec plugin lancé (Terminal 2)
- Fichier de réachabilité disponible

### Workflow dans RViz:
1. **Charger carte de réachabilité**:
   - Via l'interface du plugin
   - Charger un fichier `.h5` de réachabilité

2. **Définir poses de tâche**:
   - Utiliser l'outil interactif
   - Placer 2-3 poses de tâche

3. **Configurer paramètres**:
   - Number of base locations: 3
   - Number of high score spheres: 50
   - Méthode: PCA (0)

4. **Lancer le calcul**:
   - Cliquer "Find Base"
   - Observer le feedback dans les logs

### Logs Attendus (Terminal 1):
```
[INFO] [base_placement_server]: Received goal request
[INFO] [base_placement_server]: Goal accepted, starting computation
[INFO] [base_placement_server]: Executing PCA method...
[INFO] [base_placement_server]: Feedback: PCA, iteration 1/3, progress 33%
[INFO] [base_placement_server]: Feedback: PCA, iteration 2/3, progress 66%
[INFO] [base_placement_server]: Feedback: PCA, iteration 3/3, progress 100%
[INFO] [base_placement_server]: PCA completed: 3 poses, score=XX.XX, time=X.XXs
[INFO] [base_placement_server]: Goal succeeded
```

### Logs Attendus (RViz/Terminal 2):
```
[INFO] [place_base]: Sending goal to base_placement_server with 3 task poses
[INFO] [place_base]: Goal accepted by server, waiting for result
[INFO] [place_base]: Feedback: phase='PCA', iteration=1/3, progress=33.0%, best_score=0.00
[INFO] [place_base]: Received 3 base poses with best score: XX.XX, computation time: X.XXXs
[INFO] [place_base]: FindBase Task Finished
```

### Visualisation:
- Les poses de base calculées s'affichent dans RViz
- Score affiché dans l'interface
- Visualisation selon le mode choisi (flèches, robot, etc.)

✅ **Succès**: Workflow complet fonctionne end-to-end.

## Tableau de Tests

| Test | Description | Statut |
|------|-------------|--------|
| 1 | Lancement serveur | ⏳ À tester |
| 2 | Vérification interfaces | ⏳ À tester |
| 3 | Service update_parameters | ⏳ À tester |
| 4 | Service clear_maps | ⏳ À tester |
| 5 | Service add_named_pose | ⏳ À tester |
| 6 | Action find_base (minimal) | ⏳ À tester |
| 7 | RViz + Serveur | ⏳ À tester |
| 8 | Workflow complet | ⏳ À tester |

## Dépannage

### Problème: Le serveur ne démarre pas
**Solution**:
```bash
# Vérifier la compilation
cd /home/ros2_ws
colcon build --packages-select base_placement_plugin --cmake-clean-cache
source install/setup.bash
```

### Problème: "Action server not available"
**Causes possibles**:
1. Le serveur n'est pas lancé → Lancer `base_placement_server`
2. Problème de namespace → Vérifier avec `ros2 action list`
3. Timeout trop court → Augmenter le timeout dans le code

### Problème: "Service call failed"
**Solution**:
```bash
# Vérifier que le service existe
ros2 service list | grep <nom_service>

# Tester manuellement
ros2 service call /<nom_service> <type> "{...}"
```

### Problème: Le plugin RViz ne se connecte pas
**Solution**:
1. Vérifier les logs du serveur
2. Vérifier les logs de RViz
3. S'assurer que `source install/setup.bash` a été exécuté dans les deux terminaux

### Problème: Calculs retournent des résultats vides
**Causes possibles**:
1. Pas de carte de réachabilité chargée
2. Poses de tâche non définies
3. Paramètres invalides (num_base_locations trop élevé)

**Solution**:
- Charger une carte de réachabilité valide
- Vérifier que les poses de tâche sont définies
- Ajuster les paramètres

## Performance

### Temps d'Exécution Typiques

| Méthode | Poses de Tâche | Base Locations | Temps Estimé |
|---------|----------------|----------------|--------------|
| UserIntuition | N/A | N/A | < 1s |
| VerticalRobotModel | 3 | 5 | 2-5s |
| PCA | 3 | 5 | 5-15s |
| GraspReachabilityScore | 3 | 5 | 10-30s |
| IKSolutionScore | 3 | 5 | 10-30s |

*Note: Ces temps dépendent du matériel, de la taille de la carte de réachabilité, et du nombre de poses.*

## Logs de Diagnostic

### Activer les logs détaillés
```bash
# Lancer avec niveau de log DEBUG
ros2 run base_placement_plugin base_placement_server --ros-args --log-level debug
```

### Surveiller les topics de feedback
```bash
# Surveiller le feedback de l'action
ros2 topic echo /_action/feedback
```

## Conclusion

Une fois tous les tests réussis (✅), l'architecture refactorisée est **pleinement fonctionnelle**.

Pour un usage quotidien, il suffit de:
1. Lancer le serveur: `ros2 run base_placement_plugin base_placement_server`
2. Lancer RViz avec le plugin
3. Utiliser l'interface normalement

Le serveur peut rester actif en arrière-plan et être utilisé par plusieurs clients simultanément.
