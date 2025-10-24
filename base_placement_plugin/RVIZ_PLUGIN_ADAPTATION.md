# Adaptation du Plugin RViz - Résumé des Modifications

## Date
2025-10-23

## Objectif
Adapter le plugin RViz `PlaceBase` pour qu'il utilise le serveur `base_placement_server` via des clients d'action et de services, au lieu de faire les calculs directement.

## Modifications Apportées

### 1. Header (`place_base.h`)

#### Nouveaux includes ajoutés:
```cpp
#include <rclcpp_action/rclcpp_action.hpp>
#include <base_placement_interfaces/action/find_base.hpp>
#include <base_placement_interfaces/srv/update_reachability_map.hpp>
#include <base_placement_interfaces/srv/get_union_map.hpp>
#include <base_placement_interfaces/srv/update_parameters.hpp>
#include <base_placement_interfaces/srv/add_named_pose.hpp>
#include <base_placement_interfaces/srv/remove_named_pose.hpp>
#include <base_placement_interfaces/srv/clear_maps.hpp>
#include <base_placement_interfaces/srv/get_base_poses.hpp>
```

#### Nouveaux membres privés:
- **Action client**: `find_base_action_client_` pour l'action `FindBase`
- **Service clients** (7 services):
  - `update_reachability_client_`
  - `get_union_map_client_`
  - `update_parameters_client_`
  - `add_named_pose_client_`
  - `remove_named_pose_client_`
  - `clear_maps_client_`
  - `get_base_poses_client_`
- **Callback group**: `service_cb_group_` pour les appels de services
- **Goal handle**: `goal_handle_` pour suivre l'exécution de l'action

#### Nouvelles méthodes de callback:
```cpp
void goalResponseCallback(const GoalHandleFindBase::SharedPtr& goal_handle);
void feedbackCallback(GoalHandleFindBase::SharedPtr,
                     const std::shared_ptr<const FindBaseAction::Feedback> feedback);
void resultCallback(const GoalHandleFindBase::WrappedResult& result);
```

### 2. Implémentation (`place_base.cpp`)

#### Constructeur modifié
- Initialisation de tous les clients d'action et de services
- Création du callback group `service_cb_group_`

#### Méthode `findbase()` - COMPLÈTEMENT RÉÉCRITE
**Avant**: Effectuait tous les calculs localement (union map, algorithmes, etc.)

**Après**:
- Vérifie la disponibilité du serveur d'action
- Convertit les poses en `PoseNamed` (format attendu par l'action)
- Envoie un goal à l'action `find_base` avec:
  - `task_poses`: poses de tâches
  - `method_index`: méthode sélectionnée (0-4)
  - `num_base_locations`: nombre de positions de base
  - `num_high_score_spheres`: nombre de sphères à haute score
- Configure les callbacks (goal_response, feedback, result)
- Exécution asynchrone - ne bloque pas l'interface

#### Callbacks d'action implémentés

**`goalResponseCallback()`**:
- Vérifie si le goal a été accepté ou rejeté
- Stocke le goal_handle pour référence future

**`feedbackCallback()`**:
- Reçoit les mises à jour de progression pendant le calcul
- Affiche: phase, itération, progression %, meilleur score
- Peut émettre des signaux Qt pour mettre à jour l'interface

**`resultCallback()`**:
- Traite le résultat final (succès, avorté, annulé)
- Stocke les poses de base calculées dans `final_base_poses`
- Stocke le meilleur score dans `score_`
- Émet les signaux Qt:
  - `basePlacementProcessCompleted(score_)`
  - `basePlacementProcessFinished()`
- Appelle `OuputputVizHandler()` pour visualiser les résultats

#### Méthode `setBasePlaceParams()` - MODIFIÉE
**Avant**: Mise à jour locale uniquement

**Après**:
- Mise à jour locale des paramètres
- Appel au service `update_parameters` pour synchroniser avec le serveur
- Appel asynchrone (non-bloquant)

#### Méthode `clearUnionMap()` - MODIFIÉE
**Avant**: TODO stub

**Après**:
- Appel au service `clear_maps` pour nettoyer le serveur
- Nettoyage local des structures de données:
  - `baseTrnsCol`
  - `sphereColor`
  - `highScoreSp`
  - `robot_PoseColfilter`
  - `GRASP_POSES_`
  - `final_base_poses`

## Flux d'Exécution

### Avant (architecture monolithique):
```
Interface RViz → PlaceBase::findbase()
  ↓
  ├─ Calcul union map localement
  ├─ Création des sphères localement
  ├─ Exécution algorithme localement
  └─ Visualisation résultats
```

### Après (architecture client-serveur):
```
Interface RViz → PlaceBase::findbase()
  ↓
  └─ Envoi goal action → BasePlacementServer
                           ↓
                           BasePlacementCore (calculs)
                           ↓
                           Feedback périodique
                           ↓
                           Résultat final
  ↓
  Callbacks PlaceBase
  ↓
  Visualisation résultats
```

## Avantages de cette Architecture

### 1. Séparation des Responsabilités
- **Plugin RViz**: Uniquement interface et visualisation
- **Serveur**: Calculs lourds isolés
- **Core**: Logique métier pure (pas de Qt, pas de RViz)

### 2. Performance
- Calculs dans un processus séparé
- Interface RViz reste réactive
- Feedback en temps réel via action

### 3. Réutilisabilité
- Le serveur peut être utilisé sans RViz
- API standardisée (actions/services ROS2)
- Plusieurs clients peuvent utiliser le même serveur

### 4. Maintenance
- Code découplé plus facile à maintenir
- Tests unitaires possibles sur Core
- Pas de dépendances Qt/RViz dans la logique métier

### 5. Scalabilité
- Serveur peut tourner sur une machine différente
- Plusieurs instances possibles
- Calculs parallèles possibles

## État de la Compilation

✅ **Succès**: Le package compile sans erreurs
- Avertissement mineur: paramètre `numOfSolns` non utilisé (non critique)

## Fichiers Modifiés

1. **`include/base_placement_plugin/place_base.h`**
   - Ajout des includes pour action/services
   - Ajout des membres clients
   - Ajout des méthodes de callback

2. **`src/place_base.cpp`**
   - Constructeur: initialisation des clients
   - `findbase()`: utilise l'action au lieu de calculs locaux
   - `setBasePlaceParams()`: appelle le service update_parameters
   - `clearUnionMap()`: appelle le service clear_maps
   - Implémentation des 3 callbacks d'action

## Tests Recommandés

### Test 1: Connexion au serveur
1. Lancer `base_placement_server`
2. Ouvrir RViz avec le plugin
3. Vérifier les logs: "Action server available"

### Test 2: Envoi de goal
1. Charger une carte de réachabilité
2. Définir des poses de tâche
3. Lancer "Find Base"
4. Vérifier:
   - Goal accepté
   - Feedback reçu
   - Résultat final reçu

### Test 3: Visualisation
1. Après calcul réussi
2. Vérifier l'affichage des poses de base
3. Vérifier l'affichage du score

### Test 4: Services
1. Modifier les paramètres (nb locations, nb spheres)
2. Vérifier que le serveur est mis à jour
3. Clear maps et vérifier le nettoyage

## Méthodes Non Encore Adaptées

Les méthodes suivantes appellent encore directement les algorithmes locaux et pourraient être supprimées ou adaptées:
- `BasePlaceMethodHandler()` (lignes 484-510) - appelle les algorithmes locaux
- `findBaseByPCA()` (ligne 546+)
- `findBaseByGraspReachabilityScore()` (ligne 643+)
- `findBaseByIKSolutionScore()` (ligne 689+)
- `findBaseByVerticalRobotModel()` (ligne 509)
- `findBaseByUserIntuition()` (ligne 501)

Ces méthodes ne sont plus appelées par `findbase()` mais restent dans le code. Elles peuvent être:
1. **Supprimées** si le serveur est toujours utilisé
2. **Conservées** comme fallback si le serveur n'est pas disponible

## Notes Importantes

1. **Conversion des Types**:
   - Les poses sont converties de `geometry_msgs::msg::Pose` à `base_placement_interfaces::msg::PoseNamed`
   - Chaque pose reçoit un nom auto-généré: `task_pose_0`, `task_pose_1`, etc.

2. **Synchronisation**:
   - La méthode `findbase()` retourne immédiatement après l'envoi du goal
   - Les résultats arrivent de manière asynchrone via `resultCallback()`
   - Les signaux Qt maintiennent l'interface à jour

3. **Gestion d'Erreurs**:
   - Timeout de 5 secondes pour la connexion au serveur
   - Vérification de disponibilité des services avant appel
   - Gestion des goals rejetés/abortés/annulés

## Prochaines Étapes Possibles

1. **Ajouter un signal Qt** pour le feedback de progression
2. **Implémenter les services** pour les poses nommées (add/remove)
3. **Adapter ShowUnionMap()** pour utiliser le service get_union_map
4. **Nettoyer le code** en supprimant les anciennes méthodes d'algorithmes
5. **Ajouter tests** unitaires et d'intégration
6. **Documentation utilisateur** pour le nouveau workflow

## Conclusion

✅ Le plugin RViz a été **adapté avec succès** pour utiliser l'architecture client-serveur.
✅ La **compilation réussit** sans erreurs.
✅ L'architecture est maintenant **découplée et maintenable**.

Le plugin est maintenant **prêt à être testé** avec le serveur `base_placement_server`.
