#!/bin/bash
# =============================================================================
#  CapaciNet – Pipeline d'entraînement
# =============================================================================
#
#  Usage :
#    bash run_training.sh              # reprend depuis le dernier checkpoint
#    bash run_training.sh --scratch    # repart de zéro (ignore le checkpoint)
#    bash run_training.sh --format     # relance aussi la préparation des données
#    bash run_training.sh --scratch --format   # les deux
#
# =============================================================================


# ─── 1. CHEMINS ──────────────────────────────────────────────────────────────
# Dossier contenant les fichiers HDF5 bruts (sortie de la génération de données)
RAW_DATA_DIR="/lustre06/project/6089348/willore/data_reel_valide"

# Dossier de sortie de format_data.py (les sous-dossiers train/ et val/ y seront créés)
FORMATTED_DATA_DIR="/lustre06/project/6089348/willore/CapaciNet/unet_3d/data_real"

# Dossier où les checkpoints sont sauvegardés
CHECKPOINT_DIR="/lustre06/project/6089348/willore/CapaciNet/unet_3d/checkpoint"

# Fichier YAML généré automatiquement (ne pas éditer à la main)
GENERATED_CONFIG="/tmp/capacinet_train_config.yaml"


# ─── 2. PRÉPARATION DES DONNÉES (format_data.py) ─────────────────────────────
# Nombre de copies augmentées par groupe (0 = pas d'augmentation)
N_COPIES=5

# Angle de rotation maximum en degrés pour l'augmentation
ANGLE_SPECTRUM=180

# Fraction des données réservée à la validation (0.20 = 20 %)
VAL_RATIO=0.20

# Graine aléatoire pour la séparation train/val
SPLIT_SEED=42

# Graine aléatoire pour l'augmentation
AUG_SEED=20


# ─── 3. HYPERPARAMÈTRES DU MODÈLE ────────────────────────────────────────────
# Taille du patch d'entrée — doit correspondre à la taille de ta grille voxel
PATCH_SIZE="[152, 152, 152]"

# Nombre de feature maps à chaque niveau de l'encodeur
# [niveau1, niveau2, niveau3, niveau4, niveau5]
# Doubler à chaque niveau est la convention U-Net standard
F_MAPS="[32, 64, 128, 256, 512]"

# Nombre de canaux d'entrée (1 = grille voxel binaire)
IN_CHANNELS=1

# Nombre de canaux de sortie (1 = carte de reachability scalaire)
OUT_CHANNELS=1

# Nombre de groupes pour la Group Normalization (doit diviser le nb de feature maps)
NUM_GROUPS=8


# ─── 4. HYPERPARAMÈTRES D'ENTRAÎNEMENT ───────────────────────────────────────
# Taux d'apprentissage initial
LEARNING_RATE=0.00005

# Régularisation L2 (weight decay)
WEIGHT_DECAY=0.00001

# ReduceLROnPlateau : divise le LR par LR_FACTOR si pas d'amélioration
# après LR_PATIENCE validations consécutives
LR_FACTOR=0.5
LR_PATIENCE=50
LR_COOLDOWN=10        # attente minimale entre deux réductions
LR_MIN=0.00000001     # LR plancher (jamais réduit en dessous)

# Nombre maximum d'epochs et d'itérations
# L'entraînement s'arrête dès que l'un des deux est atteint
MAX_EPOCHS=500
MAX_ITERATIONS=2000000

# Fréquence de validation et de log (en nombre d'itérations)
VALIDATE_EVERY=100
LOG_EVERY=100


# =============================================================================
#  NE PAS MODIFIER EN DESSOUS DE CETTE LIGNE
# =============================================================================

FROM_SCRATCH=false
RUN_FORMAT=false

for arg in "$@"; do
    case $arg in
        --scratch) FROM_SCRATCH=true ;;
        --format)  RUN_FORMAT=true   ;;
    esac
done

set -e  # stoppe le script si une commande échoue

# ─── Préparation des données ──────────────────────────────────────────────────
if [ "$RUN_FORMAT" = true ]; then
    echo "==> Préparation des données..."
    python script/format_data.py \
        --input_dir      "$RAW_DATA_DIR" \
        --output_dir     "$FORMATTED_DATA_DIR" \
        --n_copies       "$N_COPIES" \
        --angle_spectrum "$ANGLE_SPECTRUM" \
        --aug_seed       "$AUG_SEED" \
        --split \
        --val_ratio      "$VAL_RATIO" \
        --seed           "$SPLIT_SEED"
    echo "==> Données prêtes."
fi

# ─── Checkpoint ──────────────────────────────────────────────────────────────
mkdir -p "$CHECKPOINT_DIR"
LAST_CHECKPOINT="$CHECKPOINT_DIR/last_checkpoint.pytorch"

if [ "$FROM_SCRATCH" = true ]; then
    echo "==> Départ de zéro : suppression des checkpoints existants..."
    rm -f "$CHECKPOINT_DIR"/*.pytorch
    RESUME_PATH="null"
else
    if [ -f "$LAST_CHECKPOINT" ]; then
        echo "==> Reprise depuis : $LAST_CHECKPOINT"
        RESUME_PATH="\"$LAST_CHECKPOINT\""
    else
        echo "==> Aucun checkpoint trouvé, départ de zéro."
        RESUME_PATH="null"
    fi
fi

# ─── Génération du fichier de configuration YAML ─────────────────────────────
cat > "$GENERATED_CONFIG" <<YAML
manual_seed: 42

model:
  name: UNet3D
  in_channels: $IN_CHANNELS
  out_channels: $OUT_CHANNELS
  final_sigmoid: true
  f_maps: $F_MAPS
  layer_order: gcr
  num_groups: $NUM_GROUPS

loss:
  name: BCEDiceLoss

eval_metric:
  name: DiceCoefficient

optimizer:
  learning_rate: $LEARNING_RATE
  weight_decay: $WEIGHT_DECAY

lr_scheduler:
  name: ReduceLROnPlateau
  mode: max
  factor: $LR_FACTOR
  patience: $LR_PATIENCE
  cooldown: $LR_COOLDOWN
  min_lr: $LR_MIN

trainer:
  eval_score_higher_is_better: true
  checkpoint_dir: "$CHECKPOINT_DIR"
  resume: $RESUME_PATH
  pre_trained: null
  validate_after_iters: $VALIDATE_EVERY
  log_after_iters: $LOG_EVERY
  max_num_epochs: $MAX_EPOCHS
  max_num_iterations: $MAX_ITERATIONS

loaders:
  num_workers: 1
  raw_internal_path: raw
  label_internal_path: label

  train:
    file_paths: ["$FORMATTED_DATA_DIR/train"]
    slice_builder:
      name: SliceBuilder
      patch_shape: $PATCH_SIZE
      stride_shape: $PATCH_SIZE
    transformer:
      raw:
        - name: Normalize
        - name: ToTensor
          expand_dims: true
      label:
        - name: Identity
        - name: ToTensor
          expand_dims: true

  val:
    file_paths: ["$FORMATTED_DATA_DIR/val"]
    slice_builder:
      name: SliceBuilder
      patch_shape: $PATCH_SIZE
      stride_shape: $PATCH_SIZE
    transformer:
      raw:
        - name: Normalize
        - name: ToTensor
          expand_dims: true
      label:
        - name: Identity
        - name: ToTensor
          expand_dims: true
YAML

echo "==> Configuration générée : $GENERATED_CONFIG"

# ─── Lancement de l'entraînement ─────────────────────────────────────────────
echo "==> Lancement de l'entraînement..."
train3dunet --config "$GENERATED_CONFIG"
