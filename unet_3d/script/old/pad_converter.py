#!/usr/bin/env python3
"""
Centre les volumes raw/label dans une boîte TARGET_SHAPE
et complète les bords :
  • raw   → valeur de pad = 1
  • label → valeur de pad = 0
Les fichiers .h5 padés sont écrits dans OUTPUT_ROOT / même arborescence.
"""

import h5py, numpy as np, pathlib, shutil, argparse

# ---------------------------------------------------------------------
TARGET_SHAPE = (130, 130, 130)                     # (Z, Y, X)
INPUT_ROOT   = pathlib.Path(
    "/workspace/capacitynet/data/unet3d") # train & val dessous
OUTPUT_ROOT  = INPUT_ROOT.with_name(INPUT_ROOT.name + "_padded")
# ---------------------------------------------------------------------


def centered_pad(arr: np.ndarray, target, pad_value):
    """
    Centre arr (C,Z,Y,X) ou (Z,Y,X) dans une boîte target en le complétant
    par 'pad_value'. Si arr est plus grand que target, lève ValueError.
    """
    is_4d = arr.ndim == 4
    spatial = arr.shape[-3:]               # Z,Y,X existants
    pads = []
    for dim, tgt in zip(spatial, target):
        if dim > tgt:
            raise ValueError("le volume dépasse TARGET_SHAPE")
        missing = tgt - dim
        before  = missing // 2
        after   = missing - before
        pads.append((before, after))
    if is_4d:
        pads = [(0, 0)] + pads             # pas de pad sur C
    return np.pad(arr, pads, mode="constant", constant_values=pad_value)


def process_file(src: pathlib.Path, dst: pathlib.Path):
    with h5py.File(src, "r") as f_in, h5py.File(dst, "w") as f_out:
        # RAW -----------------------------------------------------------------
        raw  = f_in["raw"][()]
        raw  = centered_pad(raw, TARGET_SHAPE, pad_value=1)
        f_out.create_dataset("raw", data=raw, compression="gzip")

        # LABEL ---------------------------------------------------------------
        lbl  = f_in["label"][()]
        lbl  = centered_pad(lbl, TARGET_SHAPE, pad_value=0)
        f_out.create_dataset("label", data=lbl, compression="gzip")

        # copier (éventuellement) les attributs
        for k, v in f_in.attrs.items():
            f_out.attrs[k] = v


def main():
    if OUTPUT_ROOT.exists():
        print(f"Le dossier {OUTPUT_ROOT} existe déjà → supprimé puis recréé.")
        shutil.rmtree(OUTPUT_ROOT)
    for src in INPUT_ROOT.rglob("*.h5"):
        rel = src.relative_to(INPUT_ROOT)
        dst = OUTPUT_ROOT / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        print(f"{rel}  →  {dst.relative_to(OUTPUT_ROOT.parent)}")
        process_file(src, dst)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Padding centré (raw=1, label=0) vers unet3d_padded")
    _ = parser.parse_args()
    main()
