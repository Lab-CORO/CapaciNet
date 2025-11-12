# 🚀 Prochaines Étapes - CapacityNet

## 📋 Ce qui a été fait

✅ **Problème Python/ROS2 résolu** - Incompatibilité Python 3.10/3.11/3.14
✅ **Dockerfile optimisé** - Image basée sur ROS2 Humble
✅ **Pipeline 10Hz implémenté** - Asynchrone GPU→CPU + vectorisation
✅ **Docker Compose configuré** - Services multiples
✅ **Makefile créé** - 20+ commandes simplifiées
✅ **Tests automatisés** - Script de validation complet
✅ **Documentation complète** - 6 fichiers de documentation

## 🎯 Prochaines étapes

### 1️⃣ **Build et Test (Maintenant)**

```bash
# 1. Construire l'image Docker
make build

# 2. Tester l'environnement
make test

# 3. Vérifier que tout fonctionne
make gpu-test
```

**Durée estimée**: 10-15 minutes (téléchargement Docker)

---

### 2️⃣ **Premier Lancement (Aujourd'hui)**

```bash
# Lancer le nœud en mode test
make node
```

**Ce qui devrait se passer**:
- ✅ Le nœud démarre
- ✅ Attend le service `/curobo_gen_traj/get_voxel_grid`
- ⚠️ Affiche "Voxel service not available, waiting..."

**Actions**:
- [ ] Vérifier que le nœud démarre sans crash
- [ ] Confirmer que le message d'attente s'affiche
- [ ] Tester les imports Python (torch, rclpy, open3d)

---

### 3️⃣ **Intégration avec Curobo (Cette semaine)**

Le nœud attend un service Curobo. Il faut:

**Option A: Service déjà existant**
```bash
# Dans un autre terminal/conteneur
ros2 service list | grep voxel_grid
```

**Option B: Créer un service de test**
```bash
# Créer un nœud mock pour tester
# (fichier test_voxel_service.py à créer)
```

**Actions**:
- [ ] Identifier où tourne le service curobo
- [ ] Connecter les deux nœuds
- [ ] Vérifier la communication

---

### 4️⃣ **Validation du Modèle (Cette semaine)**

Vérifier que le modèle UNet3D fonctionne:

```bash
# Entrer dans le conteneur
make dev

# Vérifier le fichier de config
cat config/test_reach.yaml

# Tester le chargement du modèle
python3 -c "
from pytorch3dunet.unet3d.model import get_model
import yaml

config = yaml.safe_load(open('config/test_reach.yaml', 'r'))
model = get_model(config['model'])
print('✅ Modèle chargé!')
"
```

**Actions**:
- [ ] Vérifier que `config/test_reach.yaml` existe
- [ ] Vérifier que le modèle se charge
- [ ] Vérifier le chemin du checkpoint (`model_path`)

---

### 5️⃣ **Test End-to-End (Semaine prochaine)**

Une fois Curobo connecté:

**Actions**:
- [ ] Curobo envoie une voxel grid
- [ ] CapacityNet fait la prédiction
- [ ] Point cloud publié sur `/reachability_map_pc`
- [ ] Visualisation dans RViz2

**Commandes de test**:
```bash
# Terminal 1: Lancer le nœud
make node

# Terminal 2: Monitorer les topics
make topics
ros2 topic hz /reachability_map_pc

# Terminal 3: RViz2
make rviz
# Ajouter topic /reachability_map_pc
```

---

### 6️⃣ **Optimisation Performance (Optionnel)**

Si la performance n'est pas suffisante (< 10Hz):

**Options d'optimisation**:
1. **Utiliser CuPy** pour conversion GPU
   ```bash
   pip install cupy-cuda12x
   # Modifier capacitynet.py pour utiliser CuPy
   ```

2. **TensorRT** pour inférence plus rapide
   ```bash
   # Exporter le modèle en TensorRT
   ```

3. **Batch processing** (plusieurs voxel grids à la fois)

4. **Downsampling** de la voxel grid

Voir [OPTIMIZATIONS.md](OPTIMIZATIONS.md) pour détails.

---

## 🐛 Dépannage Préventif

### Problème potentiel: "Service not available"

**Normal !** Le nœud attend le service curobo.

**Solution**: Lancer Curobo ou créer un service de test.

---

### Problème potentiel: "Model checkpoint not found"

**Cause**: Fichier model manquant

**Solution**:
```bash
# Vérifier le chemin dans config/test_reach.yaml
# S'assurer que le fichier .pth existe
ls -lh /path/to/model.pth
```

---

### Problème potentiel: Performance < 10Hz

**Diagnostic**:
```bash
# Ajouter du profiling dans capacitynet.py
import time

t0 = time.time()
# ... code ...
t1 = time.time()
print(f"Temps: {(t1-t0)*1000:.1f}ms")
```

**Solutions**: Voir [OPTIMIZATIONS.md](OPTIMIZATIONS.md)

---

## 📊 Métriques de Succès

### Court terme (Cette semaine)
- [ ] Image Docker build sans erreur
- [ ] Tests automatisés passent à 100%
- [ ] Nœud démarre sans crash
- [ ] GPU accessible (torch.cuda.is_available() = True)

### Moyen terme (Semaine prochaine)
- [ ] Communication avec Curobo établie
- [ ] Prédictions générées correctement
- [ ] Point cloud publié sur topic ROS2
- [ ] Visualisation dans RViz2 OK

### Long terme (Ce mois)
- [ ] Fréquence ≥ 10Hz stable
- [ ] Latence totale < 150ms
- [ ] Pas de memory leaks
- [ ] Tests d'intégration passent

---

## 📞 Support

### Documentation
1. [QUICKSTART.md](QUICKSTART.md) - Démarrage rapide
2. [README_DOCKER.md](README_DOCKER.md) - Doc complète
3. [ROS2_PYTHON_COMPATIBILITY.md](ROS2_PYTHON_COMPATIBILITY.md) - Explications techniques
4. [OPTIMIZATIONS.md](OPTIMIZATIONS.md) - Guide d'optimisation

### Commandes utiles
```bash
make help          # Aide complète
make test          # Tests automatisés
make logs          # Voir les logs
make dev           # Shell interactif
cat WELCOME.txt    # Message d'accueil
cat SUMMARY.md     # Résumé complet
```

---

## ✅ Checklist Immédiate

Avant de continuer, vérifier:

- [ ] J'ai lu [QUICKSTART.md](QUICKSTART.md)
- [ ] J'ai exécuté `make build`
- [ ] J'ai exécuté `make test`
- [ ] Tous les tests passent
- [ ] `make gpu-test` détecte le GPU
- [ ] Je comprends le workflow (éditer local → make rebuild → make node)

**Si tout est ✅** → Vous êtes prêt pour l'étape 2 !

**Si problèmes** → Consulter [README_DOCKER.md](README_DOCKER.md) section Dépannage

---

## 🎯 Objectif Final

**Système fonctionnel**:
- Curobo génère des voxel grids
- CapacityNet prédit la reachability
- Point cloud publié à 10Hz
- Visualisation temps réel dans RViz2

**Vous y êtes presque !** 🚀

---

**Dernière mise à jour**: 2025-11-11
**Status**: 📦 Configuration complète - Prêt pour l'intégration
