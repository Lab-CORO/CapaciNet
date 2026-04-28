# Optimisations de performance — capacitynet

## Contexte

Le node `reachability_node` effectue une inférence UNet3D (5 niveaux, f-maps `[32,64,128,256,512]`)
sur un Jetson Orin AGX 64 Go pour prédire des cartes d'accessibilité à partir de grilles voxel 3D.

Baseline de départ : **~1387 ms** par cycle (GPU : Orin, PyTorch 2.3.0, cuDNN 8.9.4).

---

## 1. Optimisations PyTorch

### Fichiers modifiés
- `capacitynet/capacitynet.py`

### Modifications

#### `cudnn.benchmark = True`
PyTorch utilisait un plan cuDNN non supporté sur l'Orin → fallback lent
(`CUDNN_STATUS_NOT_SUPPORTED`). Activer le benchmark force cuDNN à tester tous
les algorithmes disponibles et retenir le plus rapide.

```python
torch.backends.cudnn.benchmark = True
```

**Gain : ~1277 ms → ~897 ms (-30%)**

#### Suppression de `torch.cuda.empty_cache()`
L'appel à `empty_cache()` à chaque cycle détruisait le memory pool CUDA et
forçait une réallocation au cycle suivant (~130 ms de pénalité).

**Gain : Cleanup 130 ms → 0 ms**

#### Pipeline GPU-first
La conversion CPU→GPU se faisait en plusieurs étapes (3 copies intermédiaires).
Refactorisé pour transférer les données brutes en un seul `memcpy` puis effectuer
l'inversion d'occupancy et le cast de type directement sur GPU.

```python
# Avant
voxel_map = 1 - np.frombuffer(...).astype(np.float32)
voxel_map_ts = torch.from_numpy(voxel_map).to('cuda')

# Après
raw_gpu = torch.from_numpy(np.frombuffer(..., dtype=np.int32).copy()).to('cuda')
voxel_map_ts = (1 - raw_gpu).to(dtype).unsqueeze(0).unsqueeze(0)
```

#### Correction du timing
Les `torch.cuda.synchronize()` étaient placés **après** les captures de temps,
ce qui attribuait le temps d'attente GPU à la mauvaise phase (`CPU Xfer` au lieu
de `Prediction`). Déplacés avant chaque `time.time()`.

---

## 2. Intégration TensorRT

### Prérequis

TensorRT 8.6.2 est installé sur la machine (`trtexec` disponible).

```bash
pip install onnx          # pour le script d'export uniquement
```

### Nouveaux fichiers

| Fichier | Rôle |
|---|---|
| `scripts/export_to_trt.py` | Conversion one-shot : PyTorch → ONNX → engine TRT |
| `capacitynet/trt_model.py` | Wrapper TRT Python API pour inférence GPU |

### Fichiers modifiés

| Fichier | Modification |
|---|---|
| `capacitynet/capacitynet.py` | Charge `TRTModel` si engine présent, fallback PyTorch sinon |
| `config/test_reach.yaml` | Champ `trt_engine_path` (commenté par défaut) |

---

## 3. Générer l'engine TensorRT (one-shot)

> **Durée : ~2h30 sur Jetson Orin AGX** (kernel search exhaustif pour les Conv3D 152³)

```bash
cd /home/ros2_ws/src/capacitynet
python3 scripts/export_to_trt.py
```

Le script :
1. Charge le modèle en FP32 depuis `config/best_checkpoint.pytorch`
2. Exporte en ONNX (`config/unet3d.onnx`, opset 17, batch dynamique)
3. Lance `trtexec` avec FP16 et profils de batch :
   - min : `1×1×152×152×152`
   - opt/max : `6×1×152×152×152` ← limité à 6 par la contrainte int32 TRT
     (le tensor `decoders.3/Concat` à pleine résolution dépasse 2³¹-1 éléments pour batch > 6)
4. Sauvegarde l'engine dans `config/unet3d.trt`

Le log complet est dans `config/trtexec_build.log`.

### Contraintes importantes

- **Dims spatiales baked à 152³** : les skip connections de `F.interpolate` utilisent
  des tailles concrètes pendant le tracing ONNX. Si la taille de la grille voxel change,
  il faut re-exporter.
- **Batch max = 6** : au-delà, le volume du tensor dépasse la limite int32 de TRT 8.x.
  En mode gradient (batch=9), `TRTModel.infer()` découpe automatiquement en `6+3`.

---

## 4. Activer l'engine TRT

Dans `config/test_reach.yaml`, décommenter :

```yaml
trt_engine_path: "/home/ros2_ws/src/capacitynet/config/unet3d.trt"
```

Si le fichier `.trt` est absent ou corrompu, le node bascule automatiquement
sur PyTorch sans planter (log : `"TRT load failed (...), falling back to PyTorch"`).

---

## 5. Build et lancement

```bash
# Build
cd /home/ros2_ws
colcon build --packages-select capacitynet
source install/setup.bash

# Lancement (mode single, gradient désactivé)
ros2 launch capacitynet gradient_reachability.launch.py enable_gradient_control:=False

# Lancement (mode gradient activé)
ros2 launch capacitynet gradient_reachability.launch.py
```

Au démarrage, le log confirme le mode d'inférence :
- `"TRT engine loaded and warmed up: ..."` → engine TRT actif
- `"No TRT engine configured, using PyTorch inference"` → fallback PyTorch

---

## 6. Benchmark mesuré

Test direct (script Python, Jetson Orin AGX, sans service ROS) :

| Mode | Backend | Temps |
|---|---|---|
| batch=1 (single) | PyTorch FP16 + cudnn.benchmark | ~871 ms |
| batch=1 (single) | **TensorRT FP16** | **~608 ms** |
| batch=9 (gradient, split 6+3) | PyTorch FP16 + cudnn.benchmark | ~5910 ms |
| batch=9 (gradient, split 6+3) | **TensorRT FP16** | **~5008 ms** |

Timing complet du cycle node (mode single, TRT actif) :

```
Conversion: ~0.4ms | Resize: ~1ms | GPU Xfer: ~4ms | Prediction: ~608ms | CPU Xfer: ~4ms | Cleanup: 0ms
```

### Récapitulatif des gains vs baseline

| Optimisation | Gain |
|---|---|
| `cudnn.benchmark=True` | −380 ms sur Prediction |
| Suppression `empty_cache()` | −130 ms sur Cleanup |
| TRT engine (batch=1) | −260 ms supplémentaires |
| **Total** | **1387 ms → ~620 ms (−55%)** |

---

## 7. Arborescence des fichiers générés

```
config/
├── best_checkpoint.pytorch   # poids du modèle (FP32, 189 MB) — non modifié
├── unet3d.onnx               # export ONNX intermédiaire (63 MB)
├── unet3d.trt                # engine TensorRT FP16 (33 MB)
├── trtexec_build.log         # log de compilation TRT
└── test_reach.yaml           # config modèle + trt_engine_path
```

---

## 8. Références

### `CUDNN_STATUS_NOT_SUPPORTED` sur Jetson Orin

Le backend cuDNN sélectionne parfois des algorithmes de convolution non supportés sur Orin (architecture Ampere, cuDNN v8). L'activation du benchmark force une sélection exhaustive des algorithmes disponibles.

- PyTorch docs — `torch.backends.cudnn.benchmark` :
  https://pytorch.org/docs/stable/backends.html#torch.backends.cudnn.benchmark
- Forum PyTorch — *cuDNN NOT_SUPPORTED on Jetson* :
  https://discuss.pytorch.org/t/cudnn-status-not-supported-on-jetson-orin/168834
- Forum NVIDIA Jetson — même problème signalé sur Orin AGX :
  https://forums.developer.nvidia.com/t/cudnn-status-not-supported-pytorch-on-jetson-orin/233022

### `torch.cuda.empty_cache()` — pénalité en boucle temps réel

`empty_cache()` libère le memory pool CUDA vers le système, forçant des réallocations coûteuses au cycle suivant. Les docs officielles précisent que ce n'est utile que pour libérer de la mémoire à d'autres processus GPU, pas pour accélérer l'exécution.

- PyTorch docs — `torch.cuda.empty_cache()` :
  https://pytorch.org/docs/stable/generated/torch.cuda.empty_cache.html
- PyTorch forum — impact de `empty_cache()` sur les performances :
  https://discuss.pytorch.org/t/about-torch-cuda-empty-cache/34232

### Mesure de temps correcte sur GPU (CUDA asynchrone)

CUDA exécute les kernels de façon asynchrone. Sans `torch.cuda.synchronize()` avant la capture du temps, le timestamp est enregistré avant la fin du kernel, faussant l'attribution des phases.

- PyTorch CUDA Semantics — asynchronous execution :
  https://pytorch.org/docs/stable/notes/cuda.html#asynchronous-execution
- PyTorch docs — `torch.cuda.synchronize()` :
  https://pytorch.org/docs/stable/generated/torch.cuda.synchronize.html

### Mémoire unifiée Jetson — pourquoi `cudaMemcpy` persiste

Sur Jetson, CPU et GPU partagent physiquement la DRAM. Cependant, les tenseurs PyTorch alloués via `torch.zeros(device='cuda')` utilisent des mappages de mémoire virtuels distincts. PyTorch émet des `cudaMemcpy` même sur Jetson car il ne gère pas les mappages CUDA unified memory directement.

- CUDA blog — Unified Memory in CUDA 6 :
  https://developer.nvidia.com/blog/unified-memory-in-cuda-6/
- Issue PyTorch — zero-copy tensors sur Jetson :
  https://github.com/pytorch/pytorch/issues/72581
- Forum NVIDIA — confirmation du comportement sur Orin :
  https://forums.developer.nvidia.com/t/zero-copy-memory-with-pytorch-on-jetson-orin/239468

### `torch.compile` — Triton indisponible sur ARM/aarch64

Triton, le compilateur de kernels utilisé par `torch.compile`, ne supporte pas les architectures ARM. Tentative rejetée sur Jetson.

- Issue PyTorch — torch.compile fails on ARM :
  https://github.com/pytorch/pytorch/issues/130558
- Issue Triton — support aarch64 :
  https://github.com/triton-lang/triton/issues/5498

### TensorRT — limite int32 du volume des tenseurs

TRT 8.x utilise l'indexation int32 pour accéder aux éléments des tenseurs. Un volume dépassant 2³¹-1 éléments provoque une erreur à la compilation (`kOPT values violate shape constraints`). Pour UNet3D 152³ au niveau de décodeur le plus profond (96 canaux), le batch maximum est 6.

- NVIDIA TRT docs — capacités et limitations :
  https://docs.nvidia.com/deeplearning/tensorrt/latest/architecture/capabilities.html
- Issue TRT — tensor volume limit int32 :
  https://github.com/NVIDIA/TensorRT/issues/1948

### TensorRT — `InstanceNormalization_TRT` plugin

Le runtime Python de TRT ne charge pas automatiquement les plugins built-in (contrairement à `trtexec`). Sans `trt.init_libnvinfer_plugins()`, la désérialisation d'un engine utilisant `InstanceNormalization_TRT` échoue.

- NVIDIA TRT Python API docs — `init_libnvinfer_plugins` :
  https://docs.nvidia.com/deeplearning/tensorrt/latest/reference/python-api.html#tensorrt.init_libnvinfer_plugins

### TensorRT — temps de compilation et options de timing

Par défaut, `trtexec` effectue 4 runs de timing par kernel (`--minTiming=4`) et 8 runs moyennés (`--avgTiming=8`). Sur un réseau 3D comme UNet3D avec ses nombreuses Conv3D 152³, cela peut représenter plusieurs heures. Réduire à `--minTiming=1 --avgTiming=1` accélère significativement la compilation au prix d'une légère variance.

- NVIDIA TRT `IBuilderConfig` — timing iterations :
  https://docs.nvidia.com/deeplearning/tensorrt/latest/reference/python-api.html#tensorrt.IBuilderConfig.min_timing_iterations
- `trtexec` reference — timing flags :
  https://docs.nvidia.com/deeplearning/tensorrt/latest/developer-tools/trtexec.html

### TensorRT — précision mixte FP16 (fp32 aux frontières I/O)

Même avec `--fp16`, TRT peut conserver des entrées/sorties en float32 si le graphe le requiert (e.g., opération non supportée en fp16 en bordure). L'engine indique le dtype réel via `engine.get_tensor_dtype()`.

- NVIDIA TRT docs — working with mixed precision :
  https://docs.nvidia.com/deeplearning/tensorrt/latest/performance/optimizing-tensorrt-performance.html#mixed-precision

### Export ONNX — GroupNorm(num_groups=1) → InstanceNorm

Quand `num_groups == num_channels`, GroupNorm est équivalent à InstanceNorm. L'export ONNX de PyTorch exploite cette équivalence et émet un nœud `InstanceNormalization`, qui est supporté par TRT via le plugin `InstanceNormalization_TRT`.

- Issue PyTorch — GroupNorm exporté en InstanceNorm dans ONNX :
  https://github.com/pytorch/pytorch/issues/32341

### Export ONNX — `dynamic_axes` pour batch dynamique

`torch.onnx.export` avec `dynamic_axes` permet de déclarer des dimensions symboliques dans le graphe ONNX. Sans cela, toutes les dimensions sont figées à la valeur du dummy input, rendant impossible le batch dynamique dans TRT.

- PyTorch docs — `torch.onnx.export` et `dynamic_axes` :
  https://pytorch.org/docs/stable/onnx.html#torch.onnx.export
