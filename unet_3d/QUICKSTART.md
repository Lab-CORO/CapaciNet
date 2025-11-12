# 🚀 Guide de Démarrage Rapide - CapacityNet

## ⚡ Installation en 3 minutes

### 1️⃣ Prérequis (une seule fois)

```bash
# Installer NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
    sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Vérifier
docker run --rm --gpus all nvidia/cuda:12.4.0-base-ubuntu22.04 nvidia-smi
```

### 2️⃣ Build l'image

```bash
cd /workspace/capacitynet

# Méthode 1: Make (recommandé)
make build

# Méthode 2: Docker Compose
docker-compose build

# Méthode 3: Docker
docker build -f Dockerfile.ros2-base -t capacitynet:latest .
```

### 3️⃣ Lancer le nœud

```bash
# Méthode 1: Make (recommandé)
make node

# Méthode 2: Docker Compose
docker-compose up capacitynet-node

# Méthode 3: Docker
docker run --rm -it --gpus all \
  -v $(pwd):/workspace/capacitynet \
  --network host \
  capacitynet:latest /workspace/launch_node.sh
```

## 🎯 Commandes Essentielles

```bash
make help          # Liste toutes les commandes
make build         # Construire l'image
make test          # Tester l'environnement
make node          # Lancer le nœud
make dev           # Mode développement
make logs          # Voir les logs
make stop          # Arrêter le nœud
make clean         # Nettoyer
```

## 💻 Développement

### Modifier le code et tester

```bash
# 1. Éditer le code localement
vim ros2_ws/src/capacitynet/capacitynet/capacitynet.py

# 2. Recompiler
make rebuild

# 3. Tester
make node
```

### Déboguer

```bash
# Lancer un shell interactif
make dev

# Dans le conteneur:
python3 /workspace/capacitynet/ros2_ws/src/capacitynet/capacitynet/capacitynet.py
```

## 🐛 Problèmes Courants

### "No module named 'torch'"
```bash
# Utiliser le script de lancement, pas ros2 run directement
make node
```

### "CUDA not available"
```bash
# Vérifier le GPU
make gpu-test
```

### Recompiler après modifications
```bash
make rebuild
```

## 📊 Visualiser avec RViz2

```bash
# Autoriser X11
xhost +local:docker

# Lancer RViz2
make rviz

# Dans RViz2, ajouter:
# - Topic: /reachability_map_pc
# - Type: PointCloud2
```

## 🔍 Monitoring

```bash
# Voir les topics
make topics

# Voir les nœuds
make nodes

# Logs en temps réel
make logs
```

## 📖 Plus d'infos

- [README_DOCKER.md](README_DOCKER.md) - Documentation complète
- [ROS2_PYTHON_COMPATIBILITY.md](ROS2_PYTHON_COMPATIBILITY.md) - Explication des problèmes de compatibilité

---

**Besoin d'aide?** Consultez le README complet: `cat README_DOCKER.md`
