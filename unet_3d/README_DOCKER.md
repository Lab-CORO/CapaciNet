# 🐳 CapacityNet Docker Setup

Guide complet pour utiliser CapacityNet avec Docker et ROS2.

## 📋 Prérequis

- Docker (version 20.10+)
- Docker Compose (version 1.29+)
- NVIDIA GPU + drivers
- NVIDIA Container Toolkit

### Installation du NVIDIA Container Toolkit

```bash
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

## 🏗️ Construction de l'Image

### Option 1: Docker seul

```bash
docker build -f Dockerfile.ros2-base -t capacitynet:latest .
```

### Option 2: Docker Compose (RECOMMANDÉ)

```bash
docker-compose build
```

## 🚀 Utilisation

### Mode Interactif (pour développement)

```bash
# Avec Docker
docker run --rm -it --gpus all \
  -v $(pwd):/workspace/capacitynet \
  --network host \
  capacitynet:latest

# Avec Docker Compose
docker-compose run --rm capacitynet
```

Une fois dans le conteneur :
```bash
# Afficher l'aide
ros2 --help

# Lancer le nœud
launch  # alias pour /workspace/launch_node.sh

# OU directement avec ros2
ros2 run capacitynet reachability_node

# Recompiler après modifications
rebuild  # alias pour /workspace/rebuild.sh

# Tester les imports
python3 -c "import torch, rclpy, open3d; print('✅ OK')"
```

### Lancer le Nœud Directement

```bash
# Avec Docker
docker run --rm -it --gpus all \
  -v $(pwd):/workspace/capacitynet \
  --network host \
  capacitynet:latest /workspace/launch_node.sh

# Avec Docker Compose
docker-compose up capacitynet-node
```

### Mode Développement (avec accès git, etc.)

```bash
docker-compose run --rm capacitynet-dev
```

## 🔧 Workflows Communs

### 1. Développer et tester le code

```bash
# Lancer le conteneur en mode développement
docker-compose run --rm capacitynet

# Dans le conteneur:
cd /workspace/capacitynet/ros2_ws/src/capacitynet/capacitynet
vim capacitynet.py  # Modifier le code

# Recompiler (si nécessaire)
rebuild

# Tester
launch
```

### 2. Déboguer le code

```bash
# Lancer avec Python directement
docker-compose run --rm capacitynet bash
python3 /workspace/capacitynet/ros2_ws/src/capacitynet/capacitynet/capacitynet.py
```

### 3. Voir les logs ROS2

```bash
# Dans un terminal
docker-compose run --rm capacitynet

# Dans le conteneur
ros2 topic list
ros2 topic echo /reachability_map_pc
ros2 node list
ros2 node info /reachability_node
```

### 4. Utiliser RViz2 pour visualiser

```bash
# Permettre X11 forwarding
xhost +local:docker

# Lancer le conteneur avec affichage graphique
docker run --rm -it --gpus all \
  -v $(pwd):/workspace/capacitynet \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --network host \
  capacitynet:latest

# Dans le conteneur
rviz2
```

## 🐛 Dépannage

### Problème: "No module named 'torch'"

**Cause**: Python système utilisé au lieu de Python du conteneur

**Solution**: Utiliser le script `launch_node.sh` au lieu de `ros2 run`
```bash
/workspace/launch_node.sh
```

### Problème: "CUDA not available"

**Cause**: GPU non accessible dans le conteneur

**Solution**: Vérifier NVIDIA Container Toolkit
```bash
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
```

### Problème: "Service /curobo_gen_traj/get_voxel_grid not available"

**Cause**: Le service curobo n'est pas lancé

**Solution**: Lancer le service curobo dans un autre terminal/conteneur
```bash
# Dépend de votre configuration curobo
ros2 service list  # Vérifier les services disponibles
```

### Problème: Workspace pas compilé

**Solution**: Recompiler
```bash
docker-compose run --rm capacitynet /workspace/rebuild.sh
```

### Problème: "Cannot connect to X server"

**Solution**: Autoriser X11
```bash
xhost +local:docker
```

## 📦 Structure du Projet dans le Conteneur

```
/workspace/capacitynet/
├── config/                          # Fichiers de configuration
│   └── test_reach.yaml
├── ros2_ws/                        # Workspace ROS2
│   ├── src/
│   │   ├── capacitynet/           # Package principal
│   │   └── curobo_msgs/           # Messages custom
│   ├── build/                     # Fichiers compilés
│   └── install/                   # Packages installés
├── Dockerfile.ros2-base           # Dockerfile principal
├── docker-compose.yml             # Configuration Docker Compose
├── launch_node.sh                 # Script de lancement
└── rebuild.sh                     # Script de recompilation
```

## 🔬 Tester l'Environnement

```bash
# Test complet
docker-compose run --rm capacitynet bash -c '
echo "🧪 Test de l'\''environnement..."
echo ""
echo "1️⃣  Python version:"
python3 --version
echo ""
echo "2️⃣  PyTorch:"
python3 -c "import torch; print(f'\''  Version: {torch.__version__}'\''); print(f'\''  CUDA: {torch.cuda.is_available()}'\'')"
echo ""
echo "3️⃣  ROS2:"
python3 -c "import rclpy; print('\''  ✅ rclpy importé'\'')"
echo ""
echo "4️⃣  Open3D:"
python3 -c "import open3d; print(f'\''  Version: {open3d.__version__}'\'')"
echo ""
echo "5️⃣  pytorch-3dunet:"
python3 -c "from pytorch3dunet.unet3d.model import get_model; print('\''  ✅ pytorch-3dunet importé'\'')"
echo ""
echo "✅ Tous les tests passés!"
'
```

## 🎯 Performance et Optimisations

### Utiliser des volumes pour la vitesse

```yaml
# Dans docker-compose.yml, ajouter:
volumes:
  - ./:/workspace/capacitynet:cached  # Mode cached pour MacOS
```

### Limiter l'usage mémoire GPU

```bash
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512
# (déjà configuré dans le conteneur)
```

### Désactiver le profiling PyTorch

```bash
export CUDA_LAUNCH_BLOCKING=0
# (déjà configuré dans le conteneur)
```

## 📚 Ressources

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [PyTorch Docker Hub](https://hub.docker.com/r/pytorch/pytorch)
- [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker)
- [Docker Compose Reference](https://docs.docker.com/compose/)

## 🤝 Contribution

Pour contribuer :
1. Modifier le code dans votre éditeur local (synchronisé avec le conteneur via volumes)
2. Tester dans le conteneur
3. Commiter vos changements

## 📝 Notes

- **Python 3.10** : Requis pour compatibilité ROS2 Humble
- **PyTorch 2.6.0** : Version stable avec CUDA 12.4
- **CUDA 12.4** : Compatible avec GPUs récents (rétrocompatible 12.9)
- **Réseau host** : Requis pour communication ROS2 optimale

---

**Dernière mise à jour**: 2025-11-11
**Maintenu par**: Lab-CORO
