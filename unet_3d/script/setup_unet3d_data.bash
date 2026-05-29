#!/bin/bash


# # source python env
# source ~/envs/h5env/bin/activate
# # run convert dataset
# python format_data.py

#### Separe les data de train et de validation ######
mkdir -p ../data/{train,val}
python - <<'PY'
import random, shutil, pathlib
root = pathlib.Path("../data/")
files = sorted(root.glob("*.h5"))
random.seed(0)
val = set(random.sample(files, int(len(files)*0.1)))
for f in files:
    dest = root/"val"/f.name if f in val else root/"train"/f.name
    shutil.move(f, dest)
PY

