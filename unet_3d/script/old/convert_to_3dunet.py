#!/usr/bin/env python
"""
Convertit /mnt/data/data.h5 (organisation par groupes array_N)
vers une série de fichiers HDF5 prêts pour pytorch-3dunet.
"""

import h5py, os, numpy as np, tqdm
from pathlib import Path

SRC = Path("./data.h5")        # fichier fourni
DST_DIR = Path("../data/unet3d")        # dossier de sortie
DST_DIR.mkdir(parents=True, exist_ok=True)

with h5py.File(SRC, "r") as src:
    keys = list(src.keys())            # ['array_0', 'array_1', ...]
    for idx, k in enumerate(tqdm.tqdm(keys, desc="Convert")):
        g = src[k]
        # chargement
        vox = g["voxel_map"][()]       # (38,38,38)  float64
        reach = g["reachability_map"][()]
        # conversion dtype + ajout channel dim (C=1)
        vox   = vox.astype("float32")[None, ...]    # (1,38,38,38)
        reach = reach.astype("float32")[None, ...]  # idem
        # écriture
        dst = DST_DIR / f"sample_{idx:03d}.h5"
        with h5py.File(dst, "w") as f:
            f.create_dataset("raw",   data=vox,   compression="gzip")
            f.create_dataset("label", data=reach, compression="gzip")
print(f"✔  {len(keys)} fichiers créés dans {DST_DIR.resolve()}")


#### Separe les data de train et de validation ######
# mkdir -p data/unet3d/{train,val}
# python - <<'PY'
# import random, shutil, pathlib
# root = pathlib.Path("data/unet3d")
# files = sorted(root.glob("sample_*.h5"))
# random.seed(0)
# val = set(random.sample(files, int(len(files)*0.1)))
# for f in files:
#     dest = root/"val"/f.name if f in val else root/"train"/f.name
#     shutil.move(f, dest)
# PY