"""
CapaciNet – Interactive training launcher for Narval (Slurm / Apptainer).

Usage:
    python launch_training.py           # interactive mode
    python launch_training.py --list    # list all experiments and exit
"""
import argparse
import atexit
import datetime
import os
import shutil
import subprocess
import sys

# =============================================================================
#  TENSORBOARD SESSION STATE
# =============================================================================

_tb_symlinks_added: list = []


def _cleanup_tb_symlinks():
    for link in _tb_symlinks_added:
        try:
            if os.path.islink(link):
                os.unlink(link)
        except OSError:
            pass


atexit.register(_cleanup_tb_symlinks)


# =============================================================================
#  CONSTANTS
# =============================================================================

def _find_project_root():
    """Return the directory that directly contains the CapaciNet/ folder.

    Finds the first 'CapaciNet' component in this script's own absolute path
    and returns its parent.  This is immune to spurious CapaciNet/ subdirs
    that may have been created by earlier bugs.
    """
    path  = os.path.abspath(__file__)
    parts = path.split(os.sep)
    for i, part in enumerate(parts):
        if part == "CapaciNet":
            return os.sep.join(parts[:i]) or os.sep
    raise RuntimeError(
        f"Could not find 'CapaciNet' in the script path: {path}\n"
        "Make sure launch_training.py lives inside the CapaciNet/ directory tree."
    )

SCRIPT_DIR       = _find_project_root()
CAPACINET_DIR    = os.path.join(SCRIPT_DIR, "CapaciNet")
UNET3D_DIR       = os.path.join(CAPACINET_DIR, "unet_3d")
EXPERIMENTS_DIR  = os.path.join(UNET3D_DIR, "experiments")
FORMAT_SCRIPT    = os.path.join(UNET3D_DIR, "script", "format_data.py")
FORMAT_ENV       = os.path.expanduser("~/envs/format_data_env")
CONTAINER_SIF        = os.path.join(SCRIPT_DIR, "capa_unet.sif")
BIND_MOUNT           = "./CapaciNet/:/workspace/"
PYTORCH3DUNET_LOSSES = "/opt/conda/lib/python3.10/site-packages/pytorch3dunet/unet3d/losses.py"
CUSTOM_LOSSES_HOST   = os.path.join(CAPACINET_DIR, "unet_3d", "custom", "losses.py")

# Slurm defaults
DEFAULT_ACCOUNT   = "def-jerobg"
DEFAULT_TIME      = "30:00:00"
DEFAULT_MEM_CPU   = "30G"
DEFAULT_GPUS      = 1
DEFAULT_CPUS      = 32

# Hyperparameter defaults (mirrored from run_training.sh)
DEFAULT_LR             = 5e-5
DEFAULT_WEIGHT_DECAY   = 1e-5
DEFAULT_MAX_EPOCHS     = 500
DEFAULT_MAX_ITERS      = 2_000_000
DEFAULT_VALIDATE_EVERY = 100
DEFAULT_LOG_EVERY      = 100
DEFAULT_LR_FACTOR      = 0.5
DEFAULT_LR_PATIENCE    = 50
DEFAULT_LR_COOLDOWN    = 10
DEFAULT_LR_MIN         = 1e-8
DEFAULT_PATCH_SIZE     = "[152, 152, 152]"
DEFAULT_F_MAPS         = "[32, 64, 128, 256, 512]"

# format_data.py defaults
DEFAULT_N_COPIES       = 5
DEFAULT_ANGLE_SPECTRUM = 180
DEFAULT_VAL_RATIO      = 0.20
DEFAULT_SPLIT_SEED     = 42
DEFAULT_AUG_SEED       = 20
DEFAULT_RAW_DATA_DIR   = "/lustre06/project/6089348/willore/data_reel_valide"


# =============================================================================
#  INTERACTIVE PROMPT HELPERS
# =============================================================================

def prompt(message, default=None, cast=str):
    """Display a prompt with the default shown in brackets.

    Returns the cast value, or the default if the user presses Enter.
    Raises SystemExit on Ctrl-C / EOF.
    """
    suffix = f" [{default}]" if default is not None else ""
    while True:
        try:
            raw = input(f"  {message}{suffix}: ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nAborted.")
            sys.exit(0)
        if raw == "":
            if default is not None:
                return default
            print("    This field is required.")
            continue
        try:
            return cast(raw)
        except (ValueError, TypeError) as exc:
            print(f"    Invalid value ({exc}). Try again.")


def prompt_yes_no(message, default="n"):
    """Return True for yes, False for no."""
    answer = prompt(message + " (y/n)", default=default)
    return answer.lower() in ("y", "yes")


def prompt_choice(message, choices, default_index=0):
    """Present a numbered list and return the chosen item."""
    print(f"\n  {message}")
    for i, c in enumerate(choices):
        marker = "  (default)" if i == default_index else ""
        print(f"    {i + 1}. {c}{marker}")
    while True:
        raw = input(f"  Choice [{default_index + 1}]: ").strip()
        if raw == "":
            return choices[default_index]
        try:
            idx = int(raw) - 1
            if 0 <= idx < len(choices):
                return choices[idx]
        except ValueError:
            pass
        print("  Enter a number from the list.")


# =============================================================================
#  EXPERIMENT DISCOVERY
# =============================================================================

def find_experiments(experiments_dir):
    """Scan experiments_dir and return a list of dicts describing each experiment.

    Also scans known legacy checkpoint dirs under unet_3d/ for backward compatibility.

    Returns:
        list[dict]: Keys: name, path, checkpoint_dir, has_last_ckpt, has_best_ckpt,
                    mtime, log_count, description, is_legacy.
    """
    results = []

    def _read_description(exp_path):
        desc_file = os.path.join(exp_path, "description.txt")
        if os.path.isfile(desc_file):
            with open(desc_file) as f:
                return f.read().strip()
        return ""

    def _build_entry(name, exp_path, ckpt_dir, is_legacy=False):
        last_ckpt = os.path.join(ckpt_dir, "last_checkpoint.pytorch")
        best_ckpt = os.path.join(ckpt_dir, "best_checkpoint.pytorch")
        logs_dir  = os.path.join(exp_path, "logs")
        log_count = len(os.listdir(logs_dir)) if os.path.isdir(logs_dir) else 0
        mtime = datetime.datetime.fromtimestamp(os.path.getmtime(exp_path))
        return {
            "name":          name,
            "path":          exp_path,
            "checkpoint_dir": ckpt_dir,
            "has_last_ckpt": os.path.isfile(last_ckpt),
            "has_best_ckpt": os.path.isfile(best_ckpt),
            "mtime":         mtime,
            "log_count":     log_count,
            "description":   _read_description(exp_path),
            "is_legacy":     is_legacy,
        }

    # New-style experiments
    if os.path.isdir(experiments_dir):
        for name in sorted(os.listdir(experiments_dir)):
            exp_path = os.path.join(experiments_dir, name)
            if not os.path.isdir(exp_path):
                continue
            ckpt_dir = os.path.join(exp_path, "checkpoint")
            results.append(_build_entry(name, exp_path, ckpt_dir))

    # Legacy checkpoints (under CapaciNet/unet_3d/)
    legacy_dirs = [
        ("legacy: data_008",         os.path.join(UNET3D_DIR, "data_008")),
        ("legacy: data_finetune_002m", os.path.join(UNET3D_DIR, "data_finetune_002m")),
    ]
    for label, base_path in legacy_dirs:
        ckpt_dir = os.path.join(base_path, "checkpoint")
        if os.path.isdir(ckpt_dir):
            results.append(_build_entry(label, base_path, ckpt_dir, is_legacy=True))

    return results


def list_experiments(experiments_dir):
    """--list mode: print a formatted table and exit."""
    experiments = find_experiments(experiments_dir)
    if not experiments:
        print(f"\nNo experiments found in: {experiments_dir}")
        sys.exit(0)

    w = max(len(e["name"]) for e in experiments)
    header = f"  {'Experiment':<{w}}  {'Last Modified':<19}  {'Status':<8}  Logs"
    sep    = "  " + "-" * (len(header) - 2)
    print(f"\n{header}")
    print(sep)
    for e in experiments:
        if e["has_best_ckpt"]:
            status = "TRAINED"
        elif e["has_last_ckpt"]:
            status = "RUNNING"
        else:
            status = "NEW"
        mtime_str = e["mtime"].strftime("%Y-%m-%d %H:%M")
        prefix = "* " if e["is_legacy"] else "  "
        print(f"{prefix}{e['name']:<{w}}  {mtime_str:<19}  {status:<8}  {e['log_count']} file(s)")
        if e["description"]:
            print(f"    {e['description']}")
    print(f"\n  (* = legacy checkpoint, not created by this launcher)")
    sys.exit(0)


# =============================================================================
#  CONTAINER PATH CONVERSION
# =============================================================================

def to_container_path(host_path):
    """Convert an absolute host path under CapaciNet/ to its /workspace/ equivalent.

    The Apptainer bind is: ./CapaciNet/:/workspace/
    So CapaciNet/unet_3d/experiments/foo → /workspace/unet_3d/experiments/foo

    Raises:
        ValueError: If host_path is not under CAPACINET_DIR.
    """
    rel = os.path.relpath(os.path.realpath(host_path), CAPACINET_DIR)
    if rel.startswith(".."):
        raise ValueError(
            f"Path '{host_path}' is outside the Apptainer bind mount "
            f"(must be under {CAPACINET_DIR})."
        )
    return "/workspace/" + rel


# =============================================================================
#  QUESTION FUNCTIONS
# =============================================================================

def ask_loss_function():
    """Ask the user which loss function to use. Return (loss_name: str, loss_params: dict)."""
    print("\n-- Loss function --")
    choice = prompt_choice(
        "Loss function:",
        choices=[
            "BCEDiceLoss (standard)",
            "BCEDiceGradientLoss (BCEDiceLoss + 3D gradient smoothness penalty)",
        ],
        default_index=0,
    )
    if "Gradient" in choice:
        w = prompt("  Gradient weight", default=0.1, cast=float)
        return "BCEDiceGradientLoss", {"gradient_weight": w}
    return "BCEDiceLoss", {}


def ask_experiment_name():
    """Return (sanitized_name, description, full_name_with_timestamp)."""
    print("\n-- Experiment identity --")
    name = prompt("Experiment name (short, no spaces)", default="reach_v1")
    name = name.replace(" ", "_").replace("/", "_")
    desc = prompt("Description (optional, press Enter to skip)", default="")
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M")
    full_name = f"{name}_{timestamp}"
    return name, desc, full_name


def ask_raw_data_dir():
    """Return the raw HDF5 input directory."""
    print("\n-- Input data --")
    path = prompt("Raw HDF5 input directory", default=DEFAULT_RAW_DATA_DIR)
    if not os.path.isdir(path):
        print(f"  Warning: directory does not exist: {path}")
    return path


def ask_format_data(raw_data_dir, formatted_data_dir):
    """Ask whether to run format_data.py and collect its parameters.

    Returns:
        dict | None: Parameters for format_data.py, or None if skipping.
    """
    print("\n-- Data preparation --")
    if not prompt_yes_no("Run format_data.py to prepare/augment data?", default="n"):
        return None

    n_copies       = prompt("  Augmented copies per group (0 = no augmentation)",
                            default=DEFAULT_N_COPIES, cast=int)
    angle_spectrum = prompt("  Max rotation angle in degrees",
                            default=DEFAULT_ANGLE_SPECTRUM, cast=float)
    val_ratio      = prompt("  Validation split ratio (0.0 – 1.0)",
                            default=DEFAULT_VAL_RATIO, cast=float)
    split_seed     = prompt("  Train/val split seed",
                            default=DEFAULT_SPLIT_SEED, cast=int)
    aug_seed       = prompt("  Augmentation random seed",
                            default=DEFAULT_AUG_SEED, cast=int)
    do_crop = prompt_yes_no("  Apply crop to remove margins?", default="n")
    crop_str = None
    if do_crop:
        crop_str = prompt(
            "  Crop indices (x0 x1 y0 y1 z0 z1)",
            default="12 140 12 140 20 148"
        )
    return {
        "input_dir":      raw_data_dir,
        "output_dir":     formatted_data_dir,
        "n_copies":       n_copies,
        "angle_spectrum": angle_spectrum,
        "val_ratio":      val_ratio,
        "split_seed":     split_seed,
        "aug_seed":       aug_seed,
        "crop":           crop_str,
    }


def _ask_checkpoint_from_experiment(label):
    """Ask the user to pick an experiment with a checkpoint.

    Args:
        label (str): verb shown in prompts, e.g. "resume" or "fine-tune from".

    Returns:
        tuple: (ckpt_host_path, train_data_dir | None, val_data_dir | None)
               Data dirs are None when the experiment has no data/ folder.
    """
    experiments = find_experiments(EXPERIMENTS_DIR)
    resumable   = [e for e in experiments if e["has_last_ckpt"]]

    if not resumable:
        print(f"  No experiments with checkpoints found.")
        return None, None, None

    choices     = [e["name"] for e in resumable]
    chosen_name = prompt_choice(f"Select experiment to {label}:", choices=choices)
    chosen      = next(e for e in resumable if e["name"] == chosen_name)

    ckpt_path = os.path.join(chosen["checkpoint_dir"], "last_checkpoint.pytorch")
    print(f"  Checkpoint: {ckpt_path}")

    exp_train = os.path.join(chosen["path"], "data", "train")
    exp_val   = os.path.join(chosen["path"], "data", "val")
    if os.path.isdir(exp_train) and os.path.isdir(exp_val):
        print(f"  Reusing data from: {os.path.join(chosen['path'], 'data')}")
        return ckpt_path, exp_train, exp_val

    print("  Could not find data/ in the selected experiment (legacy checkpoint).")
    return ckpt_path, None, None


def ask_resume_mode(formatted_data_dir):
    """Ask whether to start from scratch, resume, or fine-tune from a checkpoint.

    Returns:
        tuple: (resume_host_path | None, pretrained_host_path | None,
                train_data_dir, val_data_dir)
    """
    print("\n-- Training mode --")
    mode = prompt_choice(
        "Training mode:",
        choices=[
            "Start from scratch",
            "Resume from existing experiment",
            "Resume from checkpoint path",
            "Fine-tune from existing experiment (pre_trained)",
            "Fine-tune from checkpoint path (pre_trained)",
        ],
        default_index=0,
    )

    default_train = os.path.join(formatted_data_dir, "train")
    default_val   = os.path.join(formatted_data_dir, "val")

    if mode == "Start from scratch":
        return None, None, default_train, default_val

    # ── helpers shared by resume and fine-tune ────────────────────────────────
    def ask_ckpt_path(verb):
        while True:
            path = prompt(f"Checkpoint path (.pytorch file)", default="")
            if not path:
                print("  Path is required.")
                continue
            path = os.path.realpath(os.path.expanduser(path))
            if not os.path.isfile(path):
                print(f"  Warning: file not found: {path}")
            return path

    def ask_data_dir():
        formatted = prompt("Formatted data directory (must contain train/ and val/)",
                           default=formatted_data_dir)
        return os.path.join(formatted, "train"), os.path.join(formatted, "val")

    # ── Resume from checkpoint path ───────────────────────────────────────────
    if mode == "Resume from checkpoint path":
        ckpt = ask_ckpt_path("resume")
        print(f"  Resuming from: {ckpt}")
        train, val = ask_data_dir()
        return ckpt, None, train, val

    # ── Fine-tune from checkpoint path ────────────────────────────────────────
    if mode == "Fine-tune from checkpoint path (pre_trained)":
        ckpt = ask_ckpt_path("fine-tune from")
        print(f"  Pre-trained weights from: {ckpt}")
        train, val = ask_data_dir()
        return None, ckpt, train, val

    # ── Resume from existing experiment ───────────────────────────────────────
    if mode == "Resume from existing experiment":
        ckpt, exp_train, exp_val = _ask_checkpoint_from_experiment("resume")
        if ckpt is None:
            print("  Starting from scratch.")
            return None, None, default_train, default_val
        if exp_train is None:
            exp_train, exp_val = ask_data_dir()
        return ckpt, None, exp_train, exp_val

    # ── Fine-tune from existing experiment ────────────────────────────────────
    # mode == "Fine-tune from existing experiment (pre_trained)"
    # Only the checkpoint is taken from the old experiment; data comes from the new one.
    ckpt, _, _ = _ask_checkpoint_from_experiment("fine-tune from")
    if ckpt is None:
        print("  Starting from scratch.")
        return None, None, default_train, default_val
    return None, ckpt, default_train, default_val


def ask_hyperparams():
    """Collect training hyperparameters. Enter accepts the default."""
    print("\n-- Hyperparameters (press Enter to accept default) --")
    return {
        "learning_rate":        prompt("Learning rate",             default=DEFAULT_LR,             cast=float),
        "weight_decay":         prompt("Weight decay",              default=DEFAULT_WEIGHT_DECAY,   cast=float),
        "max_num_epochs":       prompt("Max epochs",                default=DEFAULT_MAX_EPOCHS,     cast=int),
        "max_num_iterations":   prompt("Max iterations",            default=DEFAULT_MAX_ITERS,      cast=int),
        "validate_after_iters": prompt("Validate every N iters",    default=DEFAULT_VALIDATE_EVERY, cast=int),
        "log_after_iters":      prompt("Log every N iters",         default=DEFAULT_LOG_EVERY,      cast=int),
        "patch_size":           prompt("Patch shape (YAML list)",   default=DEFAULT_PATCH_SIZE),
        "f_maps":               prompt("Feature maps (YAML list)",  default=DEFAULT_F_MAPS),
        "lr_factor":            prompt("LR reduction factor",       default=DEFAULT_LR_FACTOR,      cast=float),
        "lr_patience":          prompt("LR patience (validations)", default=DEFAULT_LR_PATIENCE,    cast=int),
        "lr_cooldown":          prompt("LR cooldown",               default=DEFAULT_LR_COOLDOWN,    cast=int),
        "lr_min":               prompt("LR minimum",                default=DEFAULT_LR_MIN,         cast=float),
    }


def ask_slurm_params():
    """Collect Slurm resource parameters."""
    print("\n-- Slurm resources --")
    return {
        "account":    DEFAULT_ACCOUNT,
        "time":       prompt("Time limit (HH:MM:SS)", default=DEFAULT_TIME),
        "mem_per_cpu": prompt("Memory per CPU",       default=DEFAULT_MEM_CPU),
        "gpus":       prompt("GPUs per node",         default=DEFAULT_GPUS,  cast=int),
        "cpus":       prompt("CPUs per task",         default=DEFAULT_CPUS,  cast=int),
    }


# =============================================================================
#  HELPERS
# =============================================================================

def _yaml_float(x):
    """Format a float so PyYAML always parses it back as a float.

    PyYAML only recognises scientific notation as a float when the mantissa
    contains a decimal point (e.g. '5.0e-05').  Python's default str(5e-5)
    produces '5e-05' (no dot), which PyYAML reads as a string.
    """
    s = repr(float(x))
    if "e" in s or "E" in s:
        e_pos = s.lower().index("e")
        mantissa, exp = s[:e_pos], s[e_pos:]
        if "." not in mantissa:
            mantissa += ".0"
        return mantissa + exp
    if "." not in s:
        return s + ".0"
    return s


# =============================================================================
#  FILE GENERATORS (pure functions — return strings, never write to disk)
# =============================================================================

def generate_config_yaml(exp_dir, train_data_dir, val_data_dir, resume_host_path,
                         pretrained_host_path, hyperparams,
                         loss_name="BCEDiceLoss", loss_params=None):
    """Build the pytorch-3dunet YAML config as a string.

    All paths embedded in the YAML are translated to container paths via
    to_container_path(), because train3dunet runs inside Apptainer.
    """
    if loss_params is None:
        loss_params = {}

    ckpt_dir_container   = to_container_path(os.path.join(exp_dir, "checkpoint"))
    train_dir_container  = to_container_path(train_data_dir)
    val_dir_container    = to_container_path(val_data_dir)

    if resume_host_path is not None:
        resume_line      = f'  resume: "{to_container_path(resume_host_path)}"'
        pretrained_line  = "  pre_trained: null"
    elif pretrained_host_path is not None:
        resume_line      = "  resume: null"
        pretrained_line  = f'  pre_trained: "{to_container_path(pretrained_host_path)}"'
    else:
        resume_line      = "  resume: null"
        pretrained_line  = "  pre_trained: null"

    loss_extra = "".join(
        f"\n  {k}: {_yaml_float(v) if isinstance(v, float) else v}"
        for k, v in loss_params.items()
    )

    hp = hyperparams
    return f"""\
manual_seed: 42

model:
  name: UNet3D
  in_channels: 1
  out_channels: 1
  final_sigmoid: true
  f_maps: {hp['f_maps']}
  layer_order: gcr
  num_groups: 8

loss:
  name: {loss_name}{loss_extra}

eval_metric:
  name: DiceCoefficient

optimizer:
  learning_rate: {_yaml_float(hp['learning_rate'])}
  weight_decay: {_yaml_float(hp['weight_decay'])}

lr_scheduler:
  name: ReduceLROnPlateau
  mode: max
  factor: {_yaml_float(hp['lr_factor'])}
  patience: {hp['lr_patience']}
  cooldown: {hp['lr_cooldown']}
  min_lr: {_yaml_float(hp['lr_min'])}

trainer:
  eval_score_higher_is_better: true
  checkpoint_dir: "{ckpt_dir_container}"
{resume_line}
{pretrained_line}
  validate_after_iters: {hp['validate_after_iters']}
  log_after_iters: {hp['log_after_iters']}
  max_num_epochs: {hp['max_num_epochs']}
  max_num_iterations: {hp['max_num_iterations']}

loaders:
  num_workers: 1
  raw_internal_path: raw
  label_internal_path: label

  train:
    file_paths: ["{train_dir_container}"]
    slice_builder:
      name: SliceBuilder
      patch_shape: {hp['patch_size']}
      stride_shape: {hp['patch_size']}
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
    file_paths: ["{val_dir_container}"]
    slice_builder:
      name: SliceBuilder
      patch_shape: {hp['patch_size']}
      stride_shape: {hp['patch_size']}
    transformer:
      raw:
        - name: Normalize
        - name: ToTensor
          expand_dims: true
      label:
        - name: Identity
        - name: ToTensor
          expand_dims: true
"""


def generate_job_sh(exp_name, exp_dir, slurm_params, format_data_params, config_container_path,
                    loss_name="BCEDiceLoss"):
    """Build the Slurm job script as a string."""
    sp       = slurm_params
    logs_dir = os.path.join(exp_dir, "logs")

    format_block = ""
    if format_data_params:
        fd = format_data_params
        aug_flag  = f"--n_copies {fd['n_copies']} --angle_spectrum {fd['angle_spectrum']} --aug_seed {fd['aug_seed']}" \
                    if fd['n_copies'] > 0 else ""
        crop_flag = f"--crop {fd['crop']}" if fd.get('crop') else ""
        format_block = f"""\
# ── Step 1: Format data ──────────────────────────────────────────────────────
module load python/3.11.5
source {FORMAT_ENV}/bin/activate

python {FORMAT_SCRIPT} \\
    --input_dir  "{fd['input_dir']}" \\
    --output_dir "{fd['output_dir']}" \\
    {aug_flag} \\
    {crop_flag} \\
    --split \\
    --val_ratio  {fd['val_ratio']} \\
    --seed       {fd['split_seed']}

deactivate
echo "==> Data formatting complete."
echo ""

"""

    losses_bind = (
        f"    --bind {CUSTOM_LOSSES_HOST}:{PYTORCH3DUNET_LOSSES} \\\n"
        if loss_name == "BCEDiceGradientLoss" else ""
    )

    return f"""\
#!/bin/bash
#SBATCH --account={sp['account']}
#SBATCH --job-name={exp_name}
#SBATCH --output={logs_dir}/slurm_%j.out
#SBATCH --error={logs_dir}/slurm_%j.err
#SBATCH --time={sp['time']}
#SBATCH --mem-per-cpu={sp['mem_per_cpu']}
#SBATCH --gpus-per-node={sp['gpus']}
#SBATCH --cpus-per-task={sp['cpus']}
#SBATCH --export=ALL,DISABLE_DCGM=1

set -e

# The Apptainer bind uses a relative path, so we must cd to SCRIPT_DIR
cd {SCRIPT_DIR}

{format_block}\
# ── Step 2: Train ─────────────────────────────────────────────────────────────
module load apptainer

apptainer exec -C --nv \\
    --bind {BIND_MOUNT} \\
{losses_bind}    {CONTAINER_SIF} \\
    train3dunet --config {config_container_path}

echo "==> Training complete: {exp_name}"
"""


# =============================================================================
#  WRITE TO DISK
# =============================================================================

def write_experiment(exp_dir, config_yaml, job_sh, description):
    """Create the experiment directory tree and write all generated files.

    Files are written only after the user has confirmed (called by main()
    after show_summary() returns True).

    Returns:
        tuple[str, str]: (config_path, job_sh_path)
    """
    for subdir in ("checkpoint", "logs", "data"):
        os.makedirs(os.path.join(exp_dir, subdir), exist_ok=True)

    config_path = os.path.join(exp_dir, "config.yaml")
    job_sh_path = os.path.join(exp_dir, "job.sh")
    desc_path   = os.path.join(exp_dir, "description.txt")

    with open(config_path, "w") as f:
        f.write(config_yaml)
    with open(job_sh_path, "w") as f:
        f.write(job_sh)
    os.chmod(job_sh_path, 0o755)
    if description:
        with open(desc_path, "w") as f:
            f.write(description + "\n")

    return config_path, job_sh_path


# =============================================================================
#  SUMMARY & SUBMISSION
# =============================================================================

def show_summary(exp_name, description, exp_dir, slurm_params, hyperparams,
                 format_data_params, resume_host_path, pretrained_host_path,
                 train_data_dir, val_data_dir):
    """Print a human-readable summary and ask for confirmation.

    Returns:
        bool: True if the user confirms submission.
    """
    sep = "=" * 64
    print(f"\n{sep}")
    print("  EXPERIMENT SUMMARY")
    print(sep)
    print(f"  Name        : {exp_name}")
    if description:
        print(f"  Description : {description}")
    print(f"  Directory   : {exp_dir}")
    print(f"  Resume from : {resume_host_path or '(none)'}")
    print(f"  Pre-trained : {pretrained_host_path or '(none)'}")
    print(f"  Train data  : {train_data_dir}")
    print(f"  Val data    : {val_data_dir}")

    print("\n  -- Slurm --")
    for k, v in slurm_params.items():
        print(f"  {k:<14}: {v}")

    print("\n  -- Key hyperparameters --")
    short_keys = ["learning_rate", "max_num_epochs", "patch_size", "f_maps"]
    for k in short_keys:
        print(f"  {k:<22}: {hyperparams[k]}")

    if format_data_params:
        print("\n  -- Data formatting --")
        for k, v in format_data_params.items():
            print(f"  {k:<22}: {v}")

    print(sep)
    return prompt_yes_no("\nSubmit job?", default="y")


def submit_job(job_sh_path):
    """Call sbatch from SCRIPT_DIR and print the Slurm job ID."""
    result = subprocess.run(
        ["sbatch", job_sh_path],
        cwd=SCRIPT_DIR,
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        print(f"\n  {result.stdout.strip()}")
    else:
        print(f"\n  sbatch failed:\n{result.stderr}")
        sys.exit(1)


# =============================================================================
#  EXPERIMENT EXPLORER
# =============================================================================

def _dir_size_str(path):
    """Return human-readable size and file count for a directory."""
    if not os.path.isdir(path):
        return "—"
    total = 0
    count = 0
    for dirpath, _, filenames in os.walk(path):
        for f in filenames:
            try:
                total += os.path.getsize(os.path.join(dirpath, f))
            except OSError:
                pass
            count += 1
    if total >= 1_073_741_824:
        size_str = f"{total / 1_073_741_824:.1f} GB"
    elif total >= 1_048_576:
        size_str = f"{total / 1_048_576:.1f} MB"
    else:
        size_str = f"{total / 1024:.1f} KB"
    return f"{size_str}, {count} file(s)"


def show_experiment_details(exp):
    """Print all information about a single experiment."""
    sep = "=" * 64
    print(f"\n{sep}")
    print(f"  {exp['name']}")
    print(sep)

    if exp["description"]:
        print(f"  Description  : {exp['description']}")

    status = "TRAINED" if exp["has_best_ckpt"] else ("RUNNING" if exp["has_last_ckpt"] else "NEW")
    print(f"  Status       : {status}")
    print(f"  Modified     : {exp['mtime'].strftime('%Y-%m-%d %H:%M')}")
    print(f"  Directory    : {exp['path']}")

    # Checkpoints
    last_ckpt = os.path.join(exp["checkpoint_dir"], "last_checkpoint.pytorch")
    best_ckpt = os.path.join(exp["checkpoint_dir"], "best_checkpoint.pytorch")
    last_mark = "✓" if exp["has_last_ckpt"] else "✗"
    best_mark = "✓" if exp["has_best_ckpt"] else "✗"
    ckpt_size = _dir_size_str(exp["checkpoint_dir"])
    print(f"  Checkpoints  : last {last_mark}  best {best_mark}  ({ckpt_size})")

    # Data
    train_dir = os.path.join(exp["path"], "data", "train")
    val_dir   = os.path.join(exp["path"], "data", "val")
    print(f"  Train data   : {_dir_size_str(train_dir)}")
    print(f"  Val data     : {_dir_size_str(val_dir)}")

    # Logs
    logs_dir = os.path.join(exp["path"], "logs")
    print(f"  Logs         : {_dir_size_str(logs_dir)}")
    print(sep)


def _print_file(filepath, max_lines=40):
    """Print up to max_lines lines of a text file."""
    if not os.path.isfile(filepath):
        print(f"  (file not found: {filepath})")
        return
    with open(filepath) as f:
        lines = f.readlines()
    if len(lines) > max_lines:
        print(f"  (showing last {max_lines} of {len(lines)} lines)\n")
        lines = lines[-max_lines:]
    print("".join(lines))


def delete_experiment(exp):
    """Ask for confirmation then permanently delete the experiment directory."""
    print(f"\n  Warning: this will permanently delete:")
    print(f"    {exp['path']}")
    if exp["has_last_ckpt"] or exp["has_best_ckpt"]:
        print("  This experiment has checkpoints — they will be lost.")
    print()
    confirm = prompt(f"Type the experiment name to confirm deletion", default="")
    if confirm != exp["name"]:
        print("  Name does not match. Deletion cancelled.")
        return False
    shutil.rmtree(exp["path"])
    print(f"  Deleted: {exp['path']}")
    return True


def resubmit_experiment(exp):
    """Update time / mem in job.sh, set resume checkpoint in config.yaml, and resubmit.

    Behaviour:
    - If last_checkpoint.pytorch exists  → config.yaml updated to resume from it.
    - If no checkpoint                   → warns user; training would restart from scratch.
    - If formatted data already exists   → offers to skip the format_data step in job.sh.
    """
    job_sh_path    = os.path.join(exp["path"], "job.sh")
    config_path    = os.path.join(exp["path"], "config.yaml")
    last_ckpt_host = os.path.join(exp["checkpoint_dir"], "last_checkpoint.pytorch")

    if not os.path.isfile(job_sh_path):
        print(f"  No job.sh found in {exp['path']}")
        return

    # ── Read job.sh and extract current SBATCH values ─────────────────────────
    with open(job_sh_path) as f:
        job_lines = f.readlines()

    current_time    = DEFAULT_TIME
    current_mem_cpu = DEFAULT_MEM_CPU
    for line in job_lines:
        if line.startswith("#SBATCH --time="):
            current_time = line.split("=", 1)[1].strip()
        elif line.startswith("#SBATCH --mem-per-cpu="):
            current_mem_cpu = line.split("=", 1)[1].strip()

    # ── Ask for new resource values ───────────────────────────────────────────
    print(f"\n-- Resubmit: {exp['name']} --")
    new_time    = prompt("New time limit (HH:MM:SS)", default=current_time)
    new_mem_cpu = prompt("Memory per CPU",            default=current_mem_cpu)

    # ── Patch SBATCH directives ───────────────────────────────────────────────
    new_job_lines = []
    for line in job_lines:
        if line.startswith("#SBATCH --time="):
            new_job_lines.append(f"#SBATCH --time={new_time}\n")
        elif line.startswith("#SBATCH --mem-per-cpu="):
            new_job_lines.append(f"#SBATCH --mem-per-cpu={new_mem_cpu}\n")
        else:
            new_job_lines.append(line)

    # ── Optionally skip format_data if data already exists ───────────────────
    has_format_block = any("Step 1: Format data" in l for l in new_job_lines)
    train_dir = os.path.join(exp["path"], "data", "train")
    val_dir   = os.path.join(exp["path"], "data", "val")
    data_ready = os.path.isdir(train_dir) and bool(os.listdir(train_dir)) \
                 and os.path.isdir(val_dir) and bool(os.listdir(val_dir))

    if has_format_block and data_ready:
        n_train = len(os.listdir(train_dir))
        n_val   = len(os.listdir(val_dir))
        print(f"\n  Formatted data already exists: {n_train} train / {n_val} val files.")
        skip_format = prompt_yes_no("Skip format_data step?", default="y")
        if skip_format:
            filtered = []
            inside_format_block = False
            for line in new_job_lines:
                if "Step 1: Format data" in line:
                    inside_format_block = True
                if "Step 2: Train" in line:
                    inside_format_block = False
                if not inside_format_block:
                    filtered.append(line)
            new_job_lines = filtered
            print("  format_data step removed from job.sh.")
    elif has_format_block and not data_ready:
        print("  Formatted data not found — format_data step will run again.")

    with open(job_sh_path, "w") as f:
        f.writelines(new_job_lines)
    print(f"  job.sh updated  (time={new_time}, mem-per-cpu={new_mem_cpu})")

    # ── Patch config.yaml resume line ─────────────────────────────────────────
    if not os.path.isfile(config_path):
        print("  config.yaml not found — cannot update resume path.")
    elif os.path.isfile(last_ckpt_host):
        try:
            resume_container = to_container_path(last_ckpt_host)
        except ValueError as exc:
            print(f"  Warning: could not convert checkpoint path: {exc}")
            resume_container = None
        if resume_container:
            with open(config_path) as f:
                cfg_lines = f.readlines()
            new_cfg_lines = []
            for line in cfg_lines:
                stripped = line.lstrip()
                if stripped.startswith("resume:"):
                    indent = line[: len(line) - len(stripped)]
                    new_cfg_lines.append(f'{indent}resume: "{resume_container}"\n')
                else:
                    new_cfg_lines.append(line)
            with open(config_path, "w") as f:
                f.writelines(new_cfg_lines)
            print("  config.yaml updated  (resume → last_checkpoint.pytorch)")
    else:
        print("\n  Warning: no last_checkpoint.pytorch found.")
        print("  Training will restart from scratch (resume: null in config.yaml).")
        if not prompt_yes_no("Continue anyway?", default="n"):
            print("  Resubmit cancelled.")
            return

    # ── Submit ────────────────────────────────────────────────────────────────
    if prompt_yes_no("Submit job?", default="y"):
        submit_job(job_sh_path)


def setup_tensorboard_jupyterhub(exp):
    """Add exp logs to ~/tensorboard_logs/ and print JupyterHub access instructions.

    Creates a per-experiment symlink inside ~/tensorboard_logs/ so multiple
    experiments can be viewed simultaneously. All symlinks are removed via
    atexit when this script exits.
    """
    tb_dir      = os.path.expanduser("~/tensorboard_logs")
    tb_logs_src = os.path.join(exp["checkpoint_dir"], "logs")
    link_path   = os.path.join(tb_dir, exp["name"])

    if not os.path.isdir(tb_logs_src):
        print(f"\n  No TensorBoard logs found at: {tb_logs_src}")
        print("  (Training may not have started yet.)")
        return

    os.makedirs(tb_dir, exist_ok=True)

    if os.path.islink(link_path):
        print(f"\n  {exp['name']} is already linked in ~/tensorboard_logs/")
    elif os.path.exists(link_path):
        print(f"\n  Warning: ~/tensorboard_logs/{exp['name']} exists as a real entry — skipping.")
        return
    else:
        os.symlink(tb_logs_src, link_path)
        _tb_symlinks_added.append(link_path)
        print(f"\n  Added: ~/tensorboard_logs/{exp['name']} → {tb_logs_src}")

    active = [os.path.basename(p) for p in _tb_symlinks_added if os.path.islink(p)]
    sep = "=" * 64
    print(f"\n{sep}")
    print(f"  TensorBoard via JupyterHub")
    print(sep)
    print(f"  Active : {', '.join(active) if active else '(none)'}")
    print(f"")
    print(f"  1. Open  : https://jupyterhub.narval.alliancecan.ca")
    print(f"  2. Click : TensorBoard icon in the Launcher tab")
    print(f"")
    print(f"  Symlinks are removed when this script exits.")
    print(sep)


def generate_evaluation_report(exp):
    """Build and submit a SLURM evaluation job for the given experiment."""
    best_ckpt_host = os.path.join(exp["checkpoint_dir"], "best_checkpoint.pytorch")
    config_host    = os.path.join(exp["path"], "config.yaml")
    output_host    = os.path.join(exp["path"], "evaluation")

    evaluate_host = os.path.join(UNET3D_DIR, "script", "evaluate.py")

    time_limit = prompt("  Time limit (HH:MM:SS)", default="2:00:00")

    # Resolve val directory: prefer <exp>/data/val if it exists locally,
    # otherwise ask the user (config may have container /workspace/ paths)
    default_val = os.path.join(exp["path"], "data", "val")
    if not os.path.isdir(default_val):
        # Try to parse val path from config, translating container → host
        try:
            import yaml as _yaml
            with open(config_host) as _f:
                _cfg = _yaml.safe_load(_f)
            _cfg_val = _cfg["loaders"]["val"]["file_paths"][0]
            _cfg_val = _cfg_val.replace("/workspace/", os.path.join(CAPACINET_DIR, ""))
            if os.path.isdir(_cfg_val):
                default_val = _cfg_val
        except Exception:
            pass
    val_dir_host = prompt("  Val data directory", default=default_val)

    os.makedirs(output_host, exist_ok=True)

    logs_dir = os.path.join(exp["path"], "logs")
    os.makedirs(logs_dir, exist_ok=True)

    eval_env = os.path.expanduser("~/envs/eval_env")
    exp_name = exp["name"]
    job_content = f"""\
#!/bin/bash
#SBATCH --account={DEFAULT_ACCOUNT}
#SBATCH --job-name=eval_{exp_name}
#SBATCH --output={logs_dir}/slurm_eval_%j.out
#SBATCH --error={logs_dir}/slurm_eval_%j.err
#SBATCH --time={time_limit}
#SBATCH --mem-per-cpu=10G
#SBATCH --gpus-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --export=ALL,DISABLE_DCGM=1

set -e

module load python/3.11.5
source {eval_env}/bin/activate

python {evaluate_host} \\
    --config     {config_host} \\
    --checkpoint {best_ckpt_host} \\
    --output_dir {output_host} \\
    --val_dir    {val_dir_host}

deactivate
echo "==> Evaluation complete: {exp_name}"
echo "    Results: {output_host}/"
"""

    job_sh_path = os.path.join(exp["path"], "job_eval.sh")
    with open(job_sh_path, "w") as f:
        f.write(job_content)
    os.chmod(job_sh_path, 0o755)

    if prompt_yes_no("Submit evaluation job?", default="y"):
        submit_job(job_sh_path)
        print(f"\n  Results will appear in: {output_host}/")
        print(f"    confusion_matrix.png  error_histogram.png  report.txt")


def explore_experiments():
    """Interactive experiment explorer: list → select → inspect → act."""
    while True:
        experiments = find_experiments(EXPERIMENTS_DIR)
        if not experiments:
            print(f"\n  No experiments found in: {EXPERIMENTS_DIR}")
            return

        # Build choice labels
        w = max(len(e["name"]) for e in experiments)
        labels = []
        for e in experiments:
            status = "TRAINED" if e["has_best_ckpt"] else ("RUNNING" if e["has_last_ckpt"] else "NEW   ")
            desc   = f"  {e['description']}" if e["description"] else ""
            labels.append(f"{e['name']:<{w}}  [{status}]{desc}")
        labels.append("(quit)")

        chosen_label = prompt_choice("Select an experiment to explore:", labels, default_index=len(labels) - 1)
        if chosen_label == "(quit)":
            return

        exp = experiments[labels.index(chosen_label)]
        show_experiment_details(exp)

        # Action menu
        while True:
            config_path   = os.path.join(exp["path"], "config.yaml")
            job_sh_path   = os.path.join(exp["path"], "job.sh")
            logs_dir      = os.path.join(exp["path"], "logs")
            log_files     = sorted(os.listdir(logs_dir)) if os.path.isdir(logs_dir) else []
            tb_logs_dir   = os.path.join(exp["checkpoint_dir"], "logs")
            action_labels = ["View config.yaml"]
            if log_files:
                action_labels.append(f"View latest log  ({log_files[-1]})")
            if os.path.isdir(tb_logs_dir):
                action_labels.append("View in TensorBoard (JupyterHub)")
            if os.path.isfile(job_sh_path):
                action_labels.append("Resubmit with new time")
            if exp["has_best_ckpt"]:
                action_labels.append("Generate evaluation report")
            action_labels += ["Delete experiment", "Back to list"]

            action = prompt_choice("Action:", action_labels, default_index=len(action_labels) - 1)

            if action == "View config.yaml":
                print()
                _print_file(config_path)

            elif action.startswith("View latest log"):
                print()
                _print_file(os.path.join(logs_dir, log_files[-1]))

            elif action == "View in TensorBoard (JupyterHub)":
                setup_tensorboard_jupyterhub(exp)

            elif action == "Resubmit with new time":
                resubmit_experiment(exp)

            elif action == "Generate evaluation report":
                generate_evaluation_report(exp)

            elif action == "Delete experiment":
                deleted = delete_experiment(exp)
                if deleted:
                    break   # experiment gone, go back to list

            elif action == "Back to list":
                break


# =============================================================================
#  MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Interactive training launcher for CapaciNet on Narval (Slurm)"
    )
    parser.add_argument(
        "--list", action="store_true",
        help="List all experiments with their status and exit"
    )
    args = parser.parse_args()

    if args.list:
        list_experiments(EXPERIMENTS_DIR)

    # ── Main menu ─────────────────────────────────────────────────────────────
    print("\n" + "=" * 64)
    print("  UNet3D Training Launcher – CapaciNet")
    print("=" * 64)

    action = prompt_choice(
        "What do you want to do?",
        choices=["Create new training", "Explore experiments"],
        default_index=0,
    )

    if action == "Explore experiments":
        explore_experiments()
        sys.exit(0)

    # 1. Identity
    _, description, exp_name = ask_experiment_name()
    exp_dir           = os.path.join(EXPERIMENTS_DIR, exp_name)
    formatted_data_dir = os.path.join(exp_dir, "data")

    # 2. Loss function
    loss_name, loss_params = ask_loss_function()

    # 3. Raw data input
    raw_data_dir = ask_raw_data_dir()

    # 4. Data formatting (optional)
    format_data_params = ask_format_data(raw_data_dir, formatted_data_dir)

    # 5. Resume or scratch — if the user formatted data just above, train/val
    #    dirs live inside this experiment. ask_resume_mode may override them
    #    if the user picks an existing experiment.
    if format_data_params is not None:
        # Warn if user also wants to resume (data conflict)
        experiments_with_ckpt = [e for e in find_experiments(EXPERIMENTS_DIR) if e["has_last_ckpt"]]
        if experiments_with_ckpt:
            print("\n  Note: you selected data formatting AND resume mode.")
            print("  If you resume, the formatted data will be placed in the NEW experiment's data/ folder.")
            print("  The checkpoint will come from the old experiment.")

    resume_host_path, pretrained_host_path, train_data_dir, val_data_dir = ask_resume_mode(formatted_data_dir)

    # If the user asked to format data but is reusing data from a resumed experiment,
    # update the output dir so format_data writes to the correct place.
    if format_data_params is not None and train_data_dir != os.path.join(formatted_data_dir, "train"):
        print(f"\n  Formatting output will go to: {formatted_data_dir}")
        format_data_params["output_dir"] = formatted_data_dir
        train_data_dir = os.path.join(formatted_data_dir, "train")
        val_data_dir   = os.path.join(formatted_data_dir, "val")

    # 4b. Warn if both resume and pre_trained are set (shouldn't happen, but guard)
    if resume_host_path and pretrained_host_path:
        print("\n  Warning: both resume and pre_trained are set. resume takes priority.")
        pretrained_host_path = None

    # 5. Hyperparameters
    hyperparams = ask_hyperparams()

    # 6. Slurm resources
    slurm_params = ask_slurm_params()

    # 7. Generate file contents
    config_container_path = to_container_path(os.path.join(exp_dir, "config.yaml"))

    try:
        config_yaml = generate_config_yaml(
            exp_dir, train_data_dir, val_data_dir, resume_host_path,
            pretrained_host_path, hyperparams,
            loss_name=loss_name, loss_params=loss_params,
        )
    except ValueError as exc:
        print(f"\n  Error generating config: {exc}")
        sys.exit(1)

    job_sh = generate_job_sh(
        exp_name, exp_dir, slurm_params, format_data_params, config_container_path,
        loss_name=loss_name,
    )

    # 8. Confirm
    confirmed = show_summary(
        exp_name, description, exp_dir, slurm_params, hyperparams,
        format_data_params, resume_host_path, pretrained_host_path,
        train_data_dir, val_data_dir
    )
    if not confirmed:
        print("  Cancelled. No files written.")
        sys.exit(0)

    # 9. Write to disk (only after confirmation)
    config_path, job_sh_path = write_experiment(exp_dir, config_yaml, job_sh, description)
    print(f"\n  Config  : {config_path}")
    print(f"  Job     : {job_sh_path}")

    # 10. Submit
    submit_job(job_sh_path)
    print(f"\n  Logs will appear in: {os.path.join(exp_dir, 'logs')}/")
    print(f"  Track experiments : python launch_training.py --list\n")


if __name__ == "__main__":
    main()
