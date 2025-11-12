# Pourquoi ROS2 Python ne fonctionne pas avec votre environnement PyTorch

## 🔴 Le Problème Fondamental

### Extensions C++ et Compatibilité Binaire

ROS2 utilise des **extensions C++** (fichiers `.so`) qui sont compilées pour une version **spécifique** de Python. Ces extensions ne sont **PAS portables** entre versions de Python différentes.

```
Python 3.10 → _rclpy_pybind11.cpython-310-x86_64-linux-gnu.so ✅
Python 3.11 → _rclpy_pybind11.cpython-311-x86_64-linux-gnu.so ❌ N'existe pas
Python 3.14 → _rclpy_pybind11.cpython-314-x86_64-linux-gnu.so ❌ N'existe pas
```

### Votre Configuration Actuelle

1. **ROS2 Humble** : Compilé pour Python 3.10
2. **Conda base** : Python 3.11.13 + PyTorch 2.8.0
3. **Conda 3dunet** : Python 3.14.0 + PyTorch

→ **Aucun n'est compatible avec ROS2 !**

## 📚 Pourquoi Python 3.10 Exactement ?

Les extensions C++ utilisent l'**ABI Python** (Application Binary Interface) qui change entre versions :

- **Python 3.10** : ABI stable, utilisée par ROS2 Humble (Ubuntu 22.04 par défaut)
- **Python 3.11** : Nouvel ABI, incompatible binaire avec 3.10
- **Python 3.14** : ABI encore différente

### Exemple de l'erreur :

```python
# Avec Python 3.11
import rclpy
# → ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
# → Le fichier _rclpy_pybind11.cpython-311-*.so n'existe pas
```

ROS2 cherche :
```
/opt/ros/humble/lib/python3.10/site-packages/_rclpy_pybind11.cpython-311-*.so
```

Mais seul celui-ci existe :
```
/opt/ros/humble/lib/python3.10/site-packages/_rclpy_pybind11.cpython-310-*.so
```

## ✅ Solutions

### Solution 1 : Nouvel environnement conda avec Python 3.10 (RECOMMANDÉ)

Créez un environnement conda **spécifiquement** pour ROS2 avec Python 3.10 :

```bash
# Exécuter le script fourni
bash /workspace/capacitynet/create_ros2_env.sh

# OU manuellement :
conda create -n ros2_torch python=3.10 -y
conda activate ros2_torch
conda install -c pytorch pytorch torchvision pytorch-cuda=12.9 -y
conda install -c conda-forge pytorch-3dunet open3d -y
pip install empy==3.3.4 pyyaml lark catkin_pkg
```

### Solution 2 : Recompiler ROS2 pour Python 3.11/3.14 (NON RECOMMANDÉ)

Techniquement possible mais **TRÈS compliqué** :
- Nécessite de recompiler tous les packages ROS2 depuis les sources
- Temps : plusieurs heures
- Risque d'erreurs et problèmes de dépendances
- Pas maintenu officiellement

### Solution 3 : Downgrade l'environnement 3dunet à Python 3.10

```bash
# Recréer l'environnement 3dunet avec Python 3.10
conda create -n 3dunet_py310 python=3.10 -y
conda activate 3dunet_py310
# Réinstaller toutes vos dépendances
```

## 🎯 Recommandation

**Utilisez la Solution 1** : créez `ros2_torch` avec Python 3.10

### Avantages :
- ✅ Compatible avec ROS2 Humble
- ✅ PyTorch fonctionne parfaitement
- ✅ Garde vos autres environnements intacts
- ✅ Simple et rapide (10 minutes)

### Utilisation :

```bash
# 1. Créer l'environnement (une seule fois)
bash /workspace/capacitynet/create_ros2_env.sh

# 2. Lancer le nœud
bash /workspace/capacitynet/launch_reachability.sh
```

## 📖 Références

- [ROS2 Installation Troubleshooting](https://docs.ros.org/en/humble/Guides/Installation-Troubleshooting.html)
- [Python C Extension ABI Compatibility](https://docs.python.org/3/c-api/stable.html)
- [PEP 384 – Defining a Stable ABI](https://peps.python.org/pep-0384/)
