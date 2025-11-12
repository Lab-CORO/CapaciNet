# Optimisations supplémentaires possibles

## Si vous avez encore des problèmes de performance :

### 1. Utiliser Numba pour la conversion (2-10x plus rapide)
```python
from numba import jit

@jit(nopython=True, parallel=True)
def voxel_to_pc_numba(pred, threshold):
    # Conversion ultra-rapide compilée
    pass
```

### 2. Conversion directement sur GPU avec CuPy
```python
import cupy as cp

def voxel_to_pc_gpu(prediction_gpu, vg, threshold=0.5):
    # Garde tout sur GPU jusqu'au dernier moment
    occupied = cp.argwhere(prediction_gpu > threshold)
    points = cp.zeros((len(occupied), 3), dtype=cp.float32)
    # ... conversion sur GPU
    return points.get()  # Un seul transfert GPU->CPU
```

### 3. Réduire la résolution de la voxel grid
- Downsampling avant prédiction
- Upsampling après (si nécessaire)

### 4. TensorRT pour l'inférence
- Peut accélérer l'inférence de 2-5x

### 5. Batch processing
- Accumuler plusieurs voxel grids et les traiter en batch

### 6. Pinned memory pour transferts GPU-CPU
```python
# Dans __init__
self.pinned_buffer = torch.zeros(shape, pin_memory=True)

# Dans conversion
prediction_cpu = prediction_gpu.to(self.pinned_buffer, non_blocking=True)
```

## Monitoring des performances

Ajoutez du profiling :
```python
import time

class ReachabilityNode(Node):
    def __init__(self):
        # ...
        self.timings = {'inference': [], 'transfer': [], 'conversion': []}

    def handle_voxel_response(self, future):
        t0 = time.perf_counter()
        # ... inference
        t1 = time.perf_counter()
        self.timings['inference'].append(t1 - t0)

        if len(self.timings['inference']) % 100 == 0:
            avg = np.mean(self.timings['inference'][-100:])
            self.get_logger().info(f"Avg inference time: {avg*1000:.1f}ms")
```
