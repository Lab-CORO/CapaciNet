# CapaciNet - Guide d'Utilisation Complet

## Table des matières

1. [Introduction](#introduction)
2. [Architecture du projet](#architecture-du-projet)
3. [Installation et configuration](#installation-et-configuration)
4. [Génération du dataset](#génération-du-dataset)
5. [Entraînement du modèle UNet3D](#entraînement-du-modèle-unet3d)
6. [Utilisation du modèle dans ROS2](#utilisation-du-modèle-dans-ros2)
7. [Paramètres de configuration](#paramètres-de-configuration)
8. [Dépannage](#dépannage)

---

## Introduction

**CapaciNet** est un pipeline complet permettant de générer des **cartes d'accessibilité** (Reachability Maps - RM) pour robots manipulateurs, de les formater sous forme de grilles voxelisées, et d'entraîner un réseau de neurones 3D U-Net pour prédire l'accessibilité dans des environnements inconnus.

### Fonctionnalités principales

- Génération automatique de cartes d'accessibilité en présence d'obstacles
- Entraînement d'un modèle de deep learning pour prédire l'accessibilité
- Intégration ROS2 pour l'inférence en temps réel
- Support Docker pour une reproductibilité maximale

### Prérequis système

- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble
- **Python**: 3.10
- **GPU**: CUDA 11.8+ (fortement recommandé pour l'entraînement)
- **RAM**: Minimum 16 GB
- **Espace disque**: ~50 GB pour les datasets

---

## Architecture du projet

```
CapaciNet/
├── data_generation/          # Génération de données et cartes d'accessibilité
│   ├── src/                  # Code C++ pour la génération de RM
│   ├── include/              # Fichiers d'en-tête
│   ├── launch/               # Fichiers de lancement ROS2
│   │   ├── create_reachability_map.launch.py
│   │   └── generate_data.launch.py
│   ├── script/               # Utilitaires Python
│   ├── data/                 # Répertoire de sortie des données
│   └── docker/               # Configuration Docker
│
└── unet_3d/                  # Entraînement et inférence 3D U-Net
    ├── ros2_ws/              # Workspace ROS2
    │   └── src/capacitynet/  # Package ROS2 pour l'inférence
    ├── script/               # Scripts de formatage de données
    ├── config/               # Configurations d'entraînement
    │   ├── train_reach.yaml
    │   └── test_reach.yaml
    ├── data/                 # Données d'entraînement
    │   ├── train/
    │   └── val/
    └── Dockerfile            # Configuration Docker
```

### Composants principaux

#### 1. Génération de données (C++)

- **create_reachability_map**: Crée la carte d'accessibilité de base
- **generate_data**: Génère des paires (grille de voxels, carte d'accessibilité)
- **obstacle_adder**: Ajoute des obstacles aléatoires dans la scène
- **scene_manager**: Orchestre la génération du dataset

#### 2. Entraînement (Python)

- **format_data.py**: Formate les données HDF5 pour pytorch-3dunet
- **train3dunet**: Entraîne le modèle UNet3D
- **Configurations YAML**: Hyperparamètres et architecture du réseau

#### 3. Intégration ROS2 (Python)

- **capacitynet.py**: Nœud ROS2 pour l'inférence en temps réel
- **Services**: Communication avec curobo pour les grilles de voxels
- **Publisher**: Publication des prédictions en PointCloud2

---

## Installation et configuration

### Option 1: Installation native

#### Étape 1: Installation de ROS2 Humble

```bash
# Ajouter le dépôt ROS2
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Installer ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
```

#### Étape 2: Installation des dépendances C++

```bash
# Dépendances pour data_generation
sudo apt install -y \
  liboctomap-dev \
  libeigen3-dev \
  libhdf5-dev \
  libopen3d-dev \
  ros-humble-pcl-ros \
  ros-humble-tf2-geometry-msgs
```

#### Étape 3: Configuration de l'environnement Python

```bash
# Créer un environnement conda
conda create -n capacinet python=3.10
conda activate capacinet

# Installer PyTorch avec CUDA
conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia

# Installer les dépendances Python
pip install pytorch-3dunet h5py scikit-learn open3d tensorboard
```

#### Étape 4: Compilation du workspace ROS2

```bash
cd CapaciNet
source /opt/ros/humble/setup.bash

# Compiler data_generation
cd data_generation
colcon build
source install/setup.bash

# Compiler le nœud ROS2 d'inférence
cd ../unet_3d/ros2_ws
colcon build --packages-select capacitynet
source install/setup.bash
```

### Option 2: Utilisation de Docker (recommandé)

#### Pour la génération de données

```bash
cd CapaciNet/data_generation/docker
docker build -t capacinet-datagen .

docker run --name datagen -it --gpus all --network=host \
  -v $PWD/../data:/workspace/data \
  capacinet-datagen
```

#### Pour l'entraînement et l'inférence

```bash
cd CapaciNet/unet_3d
docker build -f Dockerfile -t capacitynet .

# Lancer le conteneur
./start.bash  # ou:
docker run --name capacitynet2 -it --gpus all --network=host \
  --env DISPLAY=$DISPLAY \
  -v $PWD:/workspace/capacitynet \
  capacitynet
```

---

## Génération du dataset

Le processus de génération de dataset se déroule en deux phases principales :

### Phase 1: Création de la carte d'accessibilité de base

Cette étape génère une carte d'accessibilité complète pour votre robot dans un environnement vide.

#### Lancement

```bash
# Source l'environnement ROS2
source /opt/ros/humble/setup.bash
cd CapaciNet/data_generation
source install/setup.bash

# Lancer la création de la carte d'accessibilité
ros2 launch data_generation create_reachability_map.launch.py
```

#### Paramètres configurables

Vous pouvez modifier les paramètres dans le fichier de lancement ou via la ligne de commande :

```bash
ros2 launch data_generation create_reachability_map.launch.py \
  voxel_size:=0.08 \
  batch_size:=1000
```

**Paramètres disponibles:**

- `voxel_size` (défaut: 0.08): Résolution de la grille en mètres
- `batch_size` (défaut: 1000): Nombre de requêtes IK par batch
- `reach_max` (défaut: 1.3): Rayon maximal de l'espace de travail en mètres

#### Processus détaillé

1. **Discrétisation de l'espace de travail**
   - Crée une grille 3D autour de la base du robot
   - Résolution typique: 0.08m (environ 38³ voxels pour un espace de 3m)

2. **Génération de poses**
   - Pour chaque voxel, génère 50 poses sur une sphère
   - Les poses sont uniformément distribuées (échantillonnage de Fibonacci)

3. **Calcul de l'accessibilité**
   - Appelle le service IK de curobo pour chaque pose
   - Calcule le score d'accessibilité (0-1) basé sur le nombre de solutions IK valides

4. **Sauvegarde**
   - Format: HDF5 + NPZ
   - Emplacement: `~/.ros/master_ik_data0.08.h5`
   - Contient: positions des voxels, scores d'accessibilité, métadonnées

#### Durée estimée

- Petit robot (6 DDL): ~30 minutes - 1 heure
- Grand robot (7 DDL): ~1-2 heures
- Dépend fortement de la résolution choisie

### Phase 2: Génération du dataset d'entraînement

Cette phase génère de multiples scènes avec obstacles et calcule la carte d'accessibilité correspondante.

#### Lancement

```bash
# Dans le même terminal que précédemment
ros2 launch data_generation generate_data.launch.py \
  dataset_size:=100 \
  voxel_size:=0.02 \
  obj_max:=20
```

#### Paramètres configurables

**Paramètres de dataset:**

- `dataset_size` (défaut: 5): Nombre de scènes à générer
- `voxel_size` (défaut: 0.02): Résolution pour l'entraînement (plus fine)
- `obj_max` (défaut: 20): Nombre maximum d'obstacles par scène

**Paramètres d'obstacles:**

- Taille minimale: 0.05m
- Taille maximale: 0.3m
- Forme: cubes
- Position: aléatoire dans l'espace de travail
- Validation de collision avec le robot

#### Processus détaillé

1. **Initialisation**
   - Charge la carte d'accessibilité de base
   - Initialise les services ROS2 (curobo_ik, get_voxel_grid)

2. **Pour chaque scène (itération):**

   a. **Génération de la scène** (obstacle_adder)
      - Génère 3 à `obj_max` cubes aléatoires
      - Valide qu'aucun obstacle n'est en collision avec le robot
      - Ajoute les obstacles à la simulation

   b. **Acquisition de la grille de voxels** (generate_data)
      - Appelle le service `/curobo_gen_traj/get_voxel_grid`
      - Récupère une grille 3D d'occupation (0=occupé, 1=libre)

   c. **Calcul de l'accessibilité**
      - Utilise la carte de base
      - Filtre les voxels en collision avec les obstacles
      - Calcule les scores d'accessibilité ajustés

   d. **Sauvegarde**
      - Format: HDF5 avec structure en groupes
      - Emplacement: `data_generation/data/[timestamp].h5`

3. **Nettoyage**
   - Supprime les obstacles de la scène
   - Prépare l'itération suivante

#### Structure des données générées

```python
# Fichier HDF5: data_generation/data/scene_001.h5
/group/0/
├── voxel_grid        # Array 3D (N×N×N): grille d'occupation
│                     # 0 = voxel occupé par un obstacle
│                     # 1 = voxel libre
└── reachability_map  # Array 3D (N×N×N): scores d'accessibilité
                      # 0.0 = inaccessible
                      # 1.0 = totalement accessible
```

#### Durée estimée

- Par scène: ~30 secondes - 2 minutes
- Dataset de 100 scènes: ~1-3 heures
- Dépend du nombre d'obstacles et de la résolution

#### Conseils pour un bon dataset

1. **Diversité des obstacles**
   - Utilisez un `obj_max` élevé (15-20)
   - Cela créera des scènes avec différents niveaux de complexité

2. **Taille du dataset**
   - Minimum recommandé: 100 scènes
   - Optimal: 500-1000 scènes
   - Plus de données = meilleure généralisation

3. **Résolution**
   - Entraînement: 0.02m (fine)
   - Permet une prédiction précise
   - Compromis entre précision et vitesse de calcul

4. **Validation**
   - Vérifiez visuellement quelques scènes avec `reachability_map_viz`
   - Assurez-vous que les données sont cohérentes

### Vérification des données générées

```bash
# Lister les fichiers générés
ls -lh data_generation/data/*.h5

# Visualiser une carte d'accessibilité (nécessite un affichage graphique)
ros2 run data_generation reachability_map_viz \
  --input data_generation/data/scene_001.h5
```

---

## Entraînement du modèle UNet3D

Une fois le dataset généré, vous pouvez procéder à l'entraînement du modèle de deep learning.

### Étape 1: Formatage des données

Les données doivent être converties au format attendu par `pytorch-3dunet`.

#### Script de formatage

```bash
cd CapaciNet
python unet_3d/script/format_data.py \
  --input data_generation/data \
  --output unet_3d/data/formatted
```

#### Options du script

```bash
python unet_3d/script/format_data.py \
  --input <répertoire_source> \
  --output <répertoire_destination> \
  --upsample 4  # Facteur de suréchantillonnage (optionnel)
```

**Que fait ce script ?**

1. **Lecture des fichiers HDF5 sources**
   - Charge `voxel_grid` et `reachability_map` de chaque groupe

2. **Suréchantillonnage (si nécessaire)**
   - Si les données de base ont une résolution de 0.08m
   - Suréchantillonne à 0.02m pour l'entraînement
   - Méthode: interpolation trilinéaire

3. **Conversion de format**
   - Sauvegarde en format pytorch-3dunet:
     - `/raw`: grille de voxels d'entrée (obstacles)
     - `/label`: carte d'accessibilité cible (labels)
   - Compression: gzip (niveau 4)

4. **Métadonnées**
   - Résolution, origine de la grille, dimensions

#### Vérification du formatage

```python
import h5py
import numpy as np

# Ouvrir un fichier formaté
with h5py.File('unet_3d/data/formatted/sample_001.h5', 'r') as f:
    raw = f['raw'][:]      # Grille d'entrée
    label = f['label'][:]  # Carte cible

    print(f"Shape: {raw.shape}")
    print(f"Input range: [{raw.min()}, {raw.max()}]")
    print(f"Label range: [{label.min()}, {label.max()}]")
```

### Étape 2: Division train/validation

Séparez votre dataset en ensembles d'entraînement et de validation.

#### Script Python automatique

```bash
cd CapaciNet
python - <<'EOF'
from pathlib import Path
import shutil
from sklearn.model_selection import train_test_split

# Lister tous les fichiers
data_dir = Path('unet_3d/data/formatted')
files = sorted(data_dir.glob('*.h5'))

print(f"Total de fichiers: {len(files)}")

# Division 80/20
train_files, val_files = train_test_split(
    files,
    test_size=0.2,
    random_state=42
)

# Créer les répertoires
train_dir = Path('unet_3d/data/train')
val_dir = Path('unet_3d/data/val')
train_dir.mkdir(parents=True, exist_ok=True)
val_dir.mkdir(parents=True, exist_ok=True)

# Copier les fichiers
for f in train_files:
    shutil.copy2(f, train_dir / f.name)

for f in val_files:
    shutil.copy2(f, val_dir / f.name)

print(f"Train: {len(train_files)} fichiers")
print(f"Validation: {len(val_files)} fichiers")
EOF
```

#### Division manuelle

```bash
# Créer les répertoires
mkdir -p unet_3d/data/train unet_3d/data/val

# Déplacer manuellement les fichiers
# 80% vers train/, 20% vers val/
```

### Étape 3: Configuration de l'entraînement

Le fichier de configuration contrôle tous les aspects de l'entraînement.

#### Fichier: `unet_3d/config/train_reach.yaml`

```yaml
# Model configuration
model:
  name: UNet3D
  in_channels: 1           # Grille de voxels (1 canal = niveaux de gris)
  out_channels: 1          # Carte d'accessibilité (1 canal)
  f_maps: [16, 32, 64, 128, 256]  # Nombre de filtres par niveau
  final_sigmoid: true      # Activation finale pour scores [0,1]
  num_groups: 8            # GroupNorm groups
  layer_order: gcr         # Group norm + Conv + ReLU
  num_levels: 5            # Profondeur du U-Net

# Loss function
loss:
  name: BCEDiceLoss       # Combinaison BCE + Dice
  skip_last_target: false

# Optimizer
optimizer:
  name: Adam
  learning_rate: 0.0001   # 1e-4
  weight_decay: 0.00001   # 1e-5 (régularisation L2)

# Learning rate scheduler
lr_scheduler:
  name: ReduceLROnPlateau
  mode: min
  factor: 0.5             # Divise le LR par 2
  patience: 10            # Après 10 epochs sans amélioration
  min_lr: 0.0000001       # LR minimal (1e-7)

# Training parameters
trainer:
  checkpoint_dir: unet_3d/checkpoints
  epochs: 500
  validate_after_iters: 10  # Validation tous les 10 itérations
  log_after_iters: 5        # Logging tous les 5 itérations
  eval_score_higher_is_better: false  # Loss (plus bas = meilleur)

# Data loader
loaders:
  train:
    file_paths:
      - unet_3d/data/train
    slice_builder:
      name: SliceBuilder
      patch_shape: [152, 152, 152]  # Taille des patchs
      stride_shape: [76, 76, 76]    # Chevauchement 50%
    transformer:
      raw:
        - name: Standardize         # Normalisation z-score
        - name: RandomFlip          # Augmentation: flip aléatoire
        - name: RandomRotate90      # Augmentation: rotation 90°
        - name: ToTensor
          expand_dims: true
      label:
        - name: RandomFlip
        - name: RandomRotate90
        - name: ToTensor
          expand_dims: true
    batch_size: 1
    shuffle: true
    num_workers: 4

  val:
    file_paths:
      - unet_3d/data/val
    slice_builder:
      name: SliceBuilder
      patch_shape: [152, 152, 152]
      stride_shape: [152, 152, 152]  # Pas de chevauchement pour validation
    transformer:
      raw:
        - name: Standardize
        - name: ToTensor
          expand_dims: true
      label:
        - name: ToTensor
          expand_dims: true
    batch_size: 1
    shuffle: false
    num_workers: 4
```

#### Paramètres clés à ajuster

**Architecture du modèle:**
- `f_maps`: Augmenter pour plus de capacité (attention à la RAM)
- `num_levels`: Profondeur du réseau (5 niveaux = bon compromis)

**Entraînement:**
- `learning_rate`: Commencer à 1e-4, ajuster si nécessaire
- `batch_size`: Limité par la RAM GPU (généralement 1 pour des volumes 3D)
- `patch_shape`: Doit être divisible par 2^(num_levels-1)

**Augmentation de données:**
- `RandomFlip`: Flip aléatoire sur chaque axe
- `RandomRotate90`: Rotation par multiples de 90°
- Ajustez selon vos besoins de généralisation

### Étape 4: Lancement de l'entraînement

#### Avec GPU (recommandé)

```bash
# Activer l'environnement conda
conda activate capacinet

# Lancer l'entraînement
cd CapaciNet
train3dunet --config unet_3d/config/train_reach.yaml
```

#### Monitoring avec TensorBoard

Dans un terminal séparé:

```bash
conda activate capacinet
cd CapaciNet
tensorboard --logdir unet_3d/checkpoints --port 6006
```

Ouvrez votre navigateur: `http://localhost:6006`

**Métriques à surveiller:**

- **Training loss**: Doit diminuer régulièrement
- **Validation loss**: Doit suivre la loss d'entraînement
- **Learning rate**: Diminue automatiquement avec le scheduler
- **Temps par epoch**: Dépend du GPU et de la taille du dataset

#### Utilisation avec Docker

```bash
# Lancer le conteneur
cd CapaciNet/unet_3d
docker run --name training -it --gpus all \
  -v $PWD:/workspace/capacitynet \
  -p 6006:6006 \
  capacitynet bash

# Dans le conteneur
cd /workspace/capacitynet
train3dunet --config config/train_reach.yaml
```

### Étape 5: Gestion des checkpoints

Le modèle sauvegarde automatiquement des checkpoints pendant l'entraînement.

#### Types de checkpoints

```
unet_3d/checkpoints/
├── last_checkpoint.pytorch     # Dernier état (reprendre l'entraînement)
├── best_checkpoint.pytorch     # Meilleur modèle (validation loss minimale)
└── checkpoint_epoch_XXX.pytorch  # Checkpoints intermédiaires
```

#### Reprendre un entraînement interrompu

Modifiez `train_reach.yaml`:

```yaml
trainer:
  resume: unet_3d/checkpoints/last_checkpoint.pytorch
  checkpoint_dir: unet_3d/checkpoints
  # ... reste de la configuration
```

Puis relancez:

```bash
train3dunet --config unet_3d/config/train_reach.yaml
```

### Étape 6: Évaluation du modèle

#### Configuration de test

Fichier: `unet_3d/config/test_reach.yaml`

```yaml
model:
  name: UNet3D
  in_channels: 1
  out_channels: 1
  f_maps: [16, 32, 64, 128, 256]
  final_sigmoid: true
  num_groups: 8
  layer_order: gcr
  num_levels: 5

# Charger le meilleur checkpoint
model_path: unet_3d/checkpoints/best_checkpoint.pytorch

# Données de test
loaders:
  test:
    file_paths:
      - unet_3d/data/val  # ou un ensemble de test séparé
    slice_builder:
      name: SliceBuilder
      patch_shape: [152, 152, 152]
      stride_shape: [76, 76, 76]
    transformer:
      raw:
        - name: Standardize
        - name: ToTensor
          expand_dims: true
      label:
        - name: ToTensor
          expand_dims: true
```

#### Lancer l'évaluation

```bash
predict3dunet --config unet_3d/config/test_reach.yaml
```

#### Visualisation des résultats

Utilisez le script de visualisation:

```bash
python unet_3d/script/format_vizualyze.py \
  --predictions unet_3d/predictions \
  --ground_truth unet_3d/data/val \
  --output unet_3d/visualizations
```

### Conseils d'entraînement

#### Performance attendue

- **Loss finale**: ~0.01 - 0.05 (BCE+Dice)
- **Convergence**: 100-300 epochs typiquement
- **Temps d'entraînement**:
  - 100 scènes: ~2-4 heures (GPU RTX 3080)
  - 500 scènes: ~10-20 heures

#### Signes de problèmes

1. **Loss ne diminue pas**
   - Learning rate trop élevé → réduire à 1e-5
   - Learning rate trop bas → augmenter à 5e-4
   - Données mal formatées → vérifier le prétraitement

2. **Overfitting** (train loss << val loss)
   - Augmenter le dropout (ajouter dans la config)
   - Augmenter l'augmentation de données
   - Réduire la complexité du modèle

3. **Underfitting** (train loss élevée)
   - Augmenter `f_maps` (ex: [32, 64, 128, 256, 512])
   - Augmenter `num_levels`
   - Entraîner plus longtemps

4. **Problèmes de mémoire GPU**
   - Réduire `patch_shape` (ex: [128, 128, 128])
   - Utiliser `batch_size: 1`
   - Activer gradient checkpointing

---

## Utilisation du modèle dans ROS2

Une fois le modèle entraîné, vous pouvez l'utiliser pour l'inférence en temps réel dans ROS2.

### Architecture du nœud ROS2

Le nœud `capacitynet` effectue les tâches suivantes:

1. **Client de service**: Récupère la grille de voxels de curobo
2. **Inférence**: Exécute le modèle UNet3D
3. **Publication**: Publie la prédiction en PointCloud2

### Étape 1: Préparation du checkpoint

Copiez votre meilleur checkpoint à l'emplacement attendu:

```bash
mkdir -p unet_3d/ros2_ws/src/capacitynet/checkpoints
cp unet_3d/checkpoints/best_checkpoint.pytorch \
   unet_3d/ros2_ws/src/capacitynet/checkpoints/last_checkpoint.pytorch
```

### Étape 2: Configuration du nœud

Le nœud est configuré via des paramètres ROS2.

#### Fichier: `unet_3d/ros2_ws/src/capacitynet/capacitynet/capacitynet.py`

**Paramètres configurables:**

```python
# Chemins
MODEL_PATH = '/workspace/capacitynet/checkpoints/last_checkpoint.pytorch'
CONFIG_PATH = '/workspace/capacitynet/config/test_reach.yaml'

# Seuils
REACHABILITY_THRESHOLD = 0.5  # Seuil de confiance (0-1)

# Fréquences
INFERENCE_RATE = 0.5  # Hz (toutes les 2 secondes)
PUBLISH_RATE = 1.0    # Hz (1 fois par seconde)
```

#### Modifier les paramètres

Éditez le fichier `capacitynet.py`:

```python
class ReachabilityNode(Node):
    def __init__(self):
        super().__init__('reachability_predictor')

        # Déclarer les paramètres
        self.declare_parameter('model_path', MODEL_PATH)
        self.declare_parameter('threshold', REACHABILITY_THRESHOLD)
        self.declare_parameter('inference_rate', INFERENCE_RATE)

        # Récupérer les valeurs
        self.model_path = self.get_parameter('model_path').value
        self.threshold = self.get_parameter('threshold').value
        # ...
```

### Étape 3: Compilation du package ROS2

```bash
cd CapaciNet/unet_3d/ros2_ws
source /opt/ros/humble/setup.bash

# Compiler
colcon build --packages-select capacitynet

# Source le workspace
source install/setup.bash
```

### Étape 4: Lancement du nœud

#### Option A: Lancement direct

```bash
# Terminal 1: S'assurer que curobo est lancé
ros2 run curobo_ros curobo_node

# Terminal 2: Lancer le nœud de prédiction
source /opt/ros/humble/setup.bash
cd CapaciNet/unet_3d/ros2_ws
source install/setup.bash

ros2 run capacitynet talker
```

#### Option B: Avec paramètres personnalisés

```bash
ros2 run capacitynet talker \
  --ros-args \
  -p model_path:=/path/to/checkpoint.pytorch \
  -p threshold:=0.6 \
  -p inference_rate:=1.0
```

#### Option C: Avec Docker (recommandé pour déploiement)

```bash
# Lancer le conteneur
cd CapaciNet/unet_3d
./start.bash

# Dans le conteneur
source /opt/ros/humble/setup.bash
cd /workspace/capacitynet/ros2_ws
source install/setup.bash
ros2 run capacitynet talker
```

### Étape 5: Interaction avec le nœud

#### Topics publiés

```bash
# Visualiser les topics
ros2 topic list

# Topic principal
/reachability_map_pc  # PointCloud2 (carte d'accessibilité prédite)
```

#### Visualisation dans RViz2

```bash
# Lancer RViz2
rviz2

# Configuration:
# 1. Fixed Frame: "world" ou "base_link"
# 2. Add → PointCloud2
# 3. Topic: /reachability_map_pc
# 4. Color Transformer: "Intensity" ou "RGB8"
```

**Interprétation du PointCloud:**
- **Vert/clair**: Zones fortement accessibles (score > 0.7)
- **Jaune**: Zones moyennement accessibles (0.4 - 0.7)
- **Rouge/sombre**: Zones faiblement accessibles (< 0.4)
- **Absent**: Inaccessible (score < seuil)

#### Services utilisés

Le nœud appelle le service de curobo:

```bash
# Vérifier que le service est disponible
ros2 service list | grep voxel

# Tester manuellement le service
ros2 service call /curobo_gen_traj/get_voxel_grid \
  curobo_msgs/srv/GetVoxelGrid "{}"
```

#### Monitoring des performances

```bash
# Visualiser les logs
ros2 run rqt_console rqt_console

# Monitorer la fréquence de publication
ros2 topic hz /reachability_map_pc

# Monitorer la bande passante
ros2 topic bw /reachability_map_pc
```

### Étape 6: Intégration dans votre application

#### Subscriber Python

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class ReachabilityConsumer(Node):
    def __init__(self):
        super().__init__('reachability_consumer')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/reachability_map_pc',
            self.reachability_callback,
            10
        )

    def reachability_callback(self, msg):
        # Extraire les points du PointCloud2
        points = pc2.read_points(msg, field_names=['x', 'y', 'z', 'intensity'])

        for point in points:
            x, y, z, score = point
            print(f"Voxel ({x:.2f}, {y:.2f}, {z:.2f}): score = {score:.2f}")

        # Utiliser la carte d'accessibilité pour planification
        # ...

def main():
    rclpy.init()
    node = ReachabilityConsumer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber C++

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ReachabilityConsumer : public rclcpp::Node {
public:
    ReachabilityConsumer() : Node("reachability_consumer") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/reachability_map_pc", 10,
            std::bind(&ReachabilityConsumer::callback, this, std::placeholders::_1)
        );
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convertir en PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        // Traiter les points
        for (const auto& point : cloud->points) {
            RCLCPP_INFO(this->get_logger(),
                "Voxel (%.2f, %.2f, %.2f): score = %.2f",
                point.x, point.y, point.z, point.intensity
            );
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReachabilityConsumer>());
    rclcpp::shutdown();
    return 0;
}
```

### Étape 7: Optimisation des performances

#### Accélération de l'inférence

1. **TensorRT** (optimisation NVIDIA)

```python
import torch
from torch2trt import torch2trt

# Convertir le modèle PyTorch en TensorRT
model_trt = torch2trt(
    model,
    [torch.randn(1, 1, 152, 152, 152).cuda()],
    fp16_mode=True  # Précision réduite
)

# Utiliser dans le nœud ROS2
torch.save(model_trt.state_dict(), 'model_trt.pth')
```

2. **Quantification**

```python
# Quantification post-entraînement
quantized_model = torch.quantization.quantize_dynamic(
    model,
    {torch.nn.Linear, torch.nn.Conv3d},
    dtype=torch.qint8
)
```

3. **Inférence par batch**

Modifiez le nœud pour traiter plusieurs grilles simultanément si votre application le permet.

#### Réduction de la latence

```python
# Dans capacitynet.py
class ReachabilityNode(Node):
    def __init__(self):
        # ...

        # Précharger le modèle
        self.model.eval()
        with torch.no_grad():
            # Warm-up
            dummy_input = torch.randn(1, 1, 152, 152, 152).to(self.device)
            _ = self.model(dummy_input)

        # Utiliser CUDA streams pour parallélisme
        self.stream = torch.cuda.Stream()
```

### Étape 8: Fichier de lancement ROS2 (optionnel)

Créez un fichier de lancement pour démarrer tous les nœuds nécessaires:

#### Fichier: `unet_3d/ros2_ws/src/capacitynet/launch/inference.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Paramètres
        DeclareLaunchArgument(
            'model_path',
            default_value='/workspace/capacitynet/checkpoints/last_checkpoint.pytorch',
            description='Chemin vers le checkpoint du modèle'
        ),
        DeclareLaunchArgument(
            'threshold',
            default_value='0.5',
            description='Seuil de reachability (0-1)'
        ),

        # Nœud de prédiction
        Node(
            package='capacitynet',
            executable='talker',
            name='reachability_predictor',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'threshold': LaunchConfiguration('threshold'),
            }],
            output='screen'
        ),

        # RViz2 pour visualisation (optionnel)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/workspace/capacitynet/config/reachability.rviz'],
            output='screen'
        ),
    ])
```

#### Utilisation

```bash
ros2 launch capacitynet inference.launch.py \
  model_path:=/path/to/model.pytorch \
  threshold:=0.6
```

---

## Paramètres de configuration

### Résumé des paramètres clés

#### Génération de données

| Paramètre | Défaut | Description | Recommandation |
|-----------|--------|-------------|----------------|
| `voxel_size` | 0.02m | Résolution de la grille | 0.02m pour précision |
| `dataset_size` | 5 | Nombre de scènes | 100-500 pour bon modèle |
| `batch_size` | 1000 | Taille batch IK | 1000-2000 selon GPU |
| `reach_max` | 1.3m | Rayon workspace | Selon votre robot |
| `obj_max` | 20 | Max obstacles/scène | 15-20 pour diversité |

#### Entraînement

| Paramètre | Défaut | Description | Recommandation |
|-----------|--------|-------------|----------------|
| `learning_rate` | 1e-4 | Taux d'apprentissage | 1e-4 à 1e-5 |
| `batch_size` | 1 | Images par batch | 1 (limité par RAM) |
| `patch_shape` | [152³] | Taille des patchs | Divisible par 16 |
| `epochs` | 500 | Nombre d'epochs | 200-500 |
| `f_maps` | [16,32,64,128,256] | Canaux U-Net | Augmenter si besoin |

#### Inférence ROS2

| Paramètre | Défaut | Description | Recommandation |
|-----------|--------|-------------|----------------|
| `threshold` | 0.5 | Seuil de confiance | 0.4-0.6 selon usage |
| `inference_rate` | 0.5 Hz | Fréquence inférence | 0.5-2 Hz |
| `publish_rate` | 1.0 Hz | Fréquence publication | 1-5 Hz |

---

## Dépannage

### Problèmes courants et solutions

#### 1. Génération de données

**Problème:** "Service /curobo_ik/ik_batch not available"
```bash
# Vérifier que curobo est lancé
ros2 service list | grep curobo

# Solution: Lancer curobo
ros2 run curobo_ros curobo_node
```

**Problème:** "No IK solution found for any pose"
```bash
# Vérifier la configuration du robot
# Augmenter reach_max ou ajuster les limites articulaires
ros2 launch data_generation create_reachability_map.launch.py reach_max:=1.5
```

**Problème:** Dataset vide ou incomplet
```bash
# Vérifier l'espace disque
df -h

# Vérifier les permissions
ls -la data_generation/data/

# Vérifier les logs
ros2 launch data_generation generate_data.launch.py 2>&1 | tee generation.log
```

#### 2. Entraînement

**Problème:** "CUDA out of memory"
```yaml
# Réduire patch_shape dans train_reach.yaml
patch_shape: [128, 128, 128]  # Au lieu de [152, 152, 152]

# Ou réduire f_maps
f_maps: [16, 32, 64, 128]  # Au lieu de [16, 32, 64, 128, 256]
```

**Problème:** Loss = NaN
```yaml
# Réduire le learning rate
optimizer:
  learning_rate: 0.00001  # 1e-5 au lieu de 1e-4

# Vérifier les données
python -c "
import h5py
import numpy as np
f = h5py.File('unet_3d/data/train/sample.h5', 'r')
print('Raw:', np.isnan(f['raw'][:]).any())
print('Label:', np.isnan(f['label'][:]).any())
"
```

**Problème:** Training très lent
```bash
# Vérifier l'utilisation GPU
nvidia-smi

# Réduire num_workers si CPU saturé
num_workers: 2  # Au lieu de 4

# Utiliser un GPU plus puissant ou réduire la résolution
```

#### 3. Inférence ROS2

**Problème:** "Model checkpoint not found"
```bash
# Vérifier le chemin
ls -la unet_3d/ros2_ws/src/capacitynet/checkpoints/

# Copier le checkpoint
cp unet_3d/checkpoints/best_checkpoint.pytorch \
   unet_3d/ros2_ws/src/capacitynet/checkpoints/last_checkpoint.pytorch
```

**Problème:** "No PointCloud published"
```bash
# Vérifier que le service voxel est disponible
ros2 service call /curobo_gen_traj/get_voxel_grid curobo_msgs/srv/GetVoxelGrid

# Vérifier les logs du nœud
ros2 run capacitynet talker --ros-args --log-level debug
```

**Problème:** Inférence très lente
```python
# Activer le mode évaluation
model.eval()

# Désactiver les gradients
with torch.no_grad():
    prediction = model(input_tensor)

# Vérifier que CUDA est utilisé
print(f"Device: {next(model.parameters()).device}")
```

#### 4. Visualisation

**Problème:** RViz ne affiche pas le PointCloud
```bash
# Vérifier le topic
ros2 topic echo /reachability_map_pc --no-arr

# Vérifier le Fixed Frame dans RViz
# Doit correspondre au frame_id du PointCloud

# Vérifier le format
ros2 topic info /reachability_map_pc
```

**Problème:** PointCloud est vide
```python
# Réduire le seuil
threshold: 0.3  # Au lieu de 0.5

# Vérifier que le modèle prédit des valeurs correctes
# Ajouter des logs dans capacitynet.py
```

### Logs et debugging

#### Activer les logs détaillés

```bash
# ROS2 logs
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1

ros2 run capacitynet talker --ros-args --log-level debug
```

#### Vérifier l'état du système

```bash
# GPU
nvidia-smi

# Topics ROS2
ros2 topic list
ros2 topic hz /reachability_map_pc

# Nœuds actifs
ros2 node list

# Services disponibles
ros2 service list
```

#### Profiling des performances

```python
# Ajouter dans capacitynet.py
import time

def inference_callback(self):
    start = time.time()

    # Appel service
    t1 = time.time()
    voxel_grid = self.get_voxel_grid()
    self.get_logger().info(f"Voxel fetch: {(time.time()-t1)*1000:.1f}ms")

    # Inférence
    t2 = time.time()
    prediction = self.predict(voxel_grid)
    self.get_logger().info(f"Inference: {(time.time()-t2)*1000:.1f}ms")

    # Total
    self.get_logger().info(f"Total: {(time.time()-start)*1000:.1f}ms")
```

---

## Annexes

### A. Structure des fichiers HDF5

#### Fichier de données générées

```
data_generation/data/scene_XXX.h5
├── /group/0/
│   ├── voxel_grid        # (N, N, N) float32
│   └── reachability_map  # (N, N, N) float32
├── /group/1/
│   ├── voxel_grid
│   └── reachability_map
└── ...
```

#### Fichier formaté pour entraînement

```
unet_3d/data/train/sample_XXX.h5
├── /raw           # (1, N, N, N) float32 - Grille de voxels
├── /label         # (1, N, N, N) float32 - Carte d'accessibilité
└── attributes:
    ├── resolution  # float (ex: 0.02)
    ├── origin      # [x, y, z]
    └── shape       # [N, N, N]
```

### B. Architecture du modèle UNet3D

```
UNet3D(
  Encoder:
    Level 1: Conv3D(1→16) + GroupNorm + ReLU
    Level 2: Conv3D(16→32) + GroupNorm + ReLU + MaxPool3D
    Level 3: Conv3D(32→64) + GroupNorm + ReLU + MaxPool3D
    Level 4: Conv3D(64→128) + GroupNorm + ReLU + MaxPool3D
    Level 5: Conv3D(128→256) + GroupNorm + ReLU + MaxPool3D

  Decoder:
    Level 4: Upsample + Concat + Conv3D(256+128→128)
    Level 3: Upsample + Concat + Conv3D(128+64→64)
    Level 2: Upsample + Concat + Conv3D(64+32→32)
    Level 1: Upsample + Concat + Conv3D(32+16→16)

  Output: Conv3D(16→1) + Sigmoid
)

Total parameters: ~7.8M
```

### C. Services ROS2 de curobo

```bash
# IK batch
ros2 service call /curobo_ik/ik_batch curobo_msgs/srv/IkBatch \
  "{poses: [{position: {x: 0.5, y: 0.2, z: 0.3}, orientation: {w: 1.0}}]}"

# Get voxel grid
ros2 service call /curobo_gen_traj/get_voxel_grid curobo_msgs/srv/GetVoxelGrid

# Add object
ros2 service call /curobo_gen_traj/add_object curobo_msgs/srv/AddObject \
  "{name: 'cube1', type: 'cube', dimensions: [0.1, 0.1, 0.1], ...}"

# Remove object
ros2 service call /curobo_gen_traj/remove_object curobo_msgs/srv/RemoveObject \
  "{name: 'cube1'}"
```

### D. Exemple de workflow complet

```bash
#!/bin/bash
# Script complet: génération → entraînement → déploiement

set -e  # Arrêt en cas d'erreur

echo "=== PHASE 1: Génération de la carte de base ==="
cd ~/CapaciNet/data_generation
source install/setup.bash
ros2 launch data_generation create_reachability_map.launch.py

echo "=== PHASE 2: Génération du dataset (100 scènes) ==="
ros2 launch data_generation generate_data.launch.py dataset_size:=100

echo "=== PHASE 3: Formatage des données ==="
cd ~/CapaciNet
conda activate capacinet
python unet_3d/script/format_data.py \
  --input data_generation/data \
  --output unet_3d/data/formatted

echo "=== PHASE 4: Division train/val ==="
python - <<'EOF'
from pathlib import Path
import shutil
from sklearn.model_selection import train_test_split

files = sorted(Path('unet_3d/data/formatted').glob('*.h5'))
train, val = train_test_split(files, test_size=0.2, random_state=42)

Path('unet_3d/data/train').mkdir(exist_ok=True)
Path('unet_3d/data/val').mkdir(exist_ok=True)

for f in train: shutil.copy2(f, Path('unet_3d/data/train')/f.name)
for f in val: shutil.copy2(f, Path('unet_3d/data/val')/f.name)

print(f"Train: {len(train)}, Val: {len(val)}")
EOF

echo "=== PHASE 5: Entraînement ==="
train3dunet --config unet_3d/config/train_reach.yaml

echo "=== PHASE 6: Préparation du déploiement ==="
mkdir -p unet_3d/ros2_ws/src/capacitynet/checkpoints
cp unet_3d/checkpoints/best_checkpoint.pytorch \
   unet_3d/ros2_ws/src/capacitynet/checkpoints/last_checkpoint.pytorch

echo "=== PHASE 7: Compilation ROS2 ==="
cd unet_3d/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select capacitynet

echo "=== TERMINÉ! ==="
echo "Pour lancer l'inférence:"
echo "  source unet_3d/ros2_ws/install/setup.bash"
echo "  ros2 run capacitynet talker"
```

### E. Ressources et références

#### Documentation officielle

- **ROS2 Humble**: https://docs.ros.org/en/humble/
- **PyTorch**: https://pytorch.org/docs/stable/
- **pytorch-3dunet**: https://github.com/wolny/pytorch-3d-unet
- **curobo**: https://curobo.org/

#### Papers de référence

- **3D U-Net**: Çiçek et al., "3D U-Net: Learning Dense Volumetric Segmentation from Sparse Annotation" (2016)
- **Reachability Maps**: Zacharias et al., "The Capability Map: A Tool for Reachability Analysis" (2013)

#### Support

- **Issues GitHub**: https://github.com/Lab-CORO/CapaciNet/issues
- **Email**: coro-dev@etsmtl.ca

---

**Dernière mise à jour**: 2025-01-13
**Version**: 1.0.0
**Licence**: MIT
