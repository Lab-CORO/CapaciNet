# Architecture — capacitynet

This document describes how the reachability-prediction and base-positioning code
in `capacitynet/` fits together: the per-cycle data flow, which nodes instantiate
which classes, and the class relationships underneath `reachability_pipeline.py`.

There are two integration styles in this package, and knowing which one a file
uses is the fastest way to understand it:

- **The componentized stack** (`WorkspaceRegionSource` + `ReachabilityPipeline` +
  `BaseCommander`, all `NodeComponent`s) — used by `gradient_base_controller.py`
  (production) and `workspace_probe.py` (bench, no `BaseCommander` so it cannot
  move the base). New nodes that need live gradient control should use this.
- **Direct use of the ROS-free core classes** (`ReachabilityEngine`,
  `WorkspaceEvaluation`, `GradientBasedController`, ...) — used ad hoc by
  `reachability_node`, `brain`, `workspace_reachability_node` and the legacy
  `enable_gradient_control` branch of `brain_orchestrator`, each of which builds
  its own instances rather than sharing the componentized stack.

## Per-cycle data flow

What happens every `cycle_period` seconds (default 2.0s), on the latest cached
voxel grid, in the componentized stack (`ReachabilityPipeline._on_timer`). The
voxel grid subscription (`_on_voxel_grid`) only caches the newest message; a
timer drives the actual cycle, decoupling the inference rate from whatever
rate the voxel grid happens to publish at:

```mermaid
flowchart LR
    VG["SparseVoxelGrid\n(curobo topic)"] --> PRE["ReachabilityEngine\n.preprocess()\nsparse → dense"]
    PRE --> XFORM["ObstacleMapTransformer\n.generate_grid_transforms()\ngrid_size×grid_size shifted obstacle maps\n(default 3×3)"]
    XFORM --> PRED["ReachabilityEngine\n.predict_batch()\nTRT if configured, else PyTorch"]
    PRED --> MASK["WorkspaceEvaluation → VoxelMask\nregion_mask() over the union of spheres"]
    MASK --> Q["grid_size² × Q_i\n(VoxelMask.compute_mean)"]
    Q --> GRAD["GradientBasedController\n.compute_gradient()\n∇Q by least-squares plane / central diff"]
    GRAD --> VEL["gradient_to_velocity()\ngain + saturation"]
    VEL --> SHAPE["BaseCommander.submit()\nadaptive cap, taper, deadband, interlocks"]
    SHAPE --> CMD["/cmd_vel"]

    REGION["WorkspaceRegionSource\n.resolve()\nMPC goal+path tail, or marker"] -.->|region centers| MASK
```

The region source and the inference chain run in different callback groups
(`control_group` vs `cuda_group`) so a ~440 ms inference cycle cannot stall
`/cmd_vel` publishing — see `gradient_base_controller.py`.

## Nodes → components

Which console-script node instantiates which class. Solid arrows are the
componentized stack; dashed arrows are a node building its own instance of a
ROS-free core class directly.

```mermaid
flowchart TB
    subgraph Nodes
        gbc["gradient_base_controller\n(production, owns /cmd_vel)"]
        wp["workspace_probe\n(bench: region + inference,\nno BaseCommander)"]
        rn["reachability_node\n(capacitynet.py)"]
        br["brain\n(single-position debug)"]
        wrn["workspace_reachability_node\n(offline QA/metrics)"]
        bo["brain_orchestrator\n(enable_gradient_control branch)"]
        gcm["gradient_controller_mock\n(synthetic test harness)"]
    end

    subgraph "NodeComponent stack"
        WRS[WorkspaceRegionSource]
        RP[ReachabilityPipeline]
        BC[BaseCommander]
    end

    subgraph "ROS-free core"
        RE[ReachabilityEngine]
        OMT[ObstacleMapTransformer]
        GBCtrl[GradientBasedController]
        WE[WorkspaceEvaluation]
        VM[VoxelMask]
        TRT[TRTModel]
    end

    gbc --> WRS
    gbc --> RP
    gbc --> BC
    wp --> WRS
    wp --> RP

    RP --> RE
    RP --> OMT
    RP --> GBCtrl
    GBCtrl --> WE
    WE --> VM
    RE --> TRT

    rn -. direct .-> RE
    br -. direct .-> RE
    br -. direct .-> WE
    wrn -. direct .-> RE
    wrn -. direct .-> WE
    bo -. direct, own instances .-> RE
    bo -. direct, own instances .-> GBCtrl
    gcm -. direct, synthetic input .-> OMT
    gcm -. direct, synthetic input .-> GBCtrl
```

`grasp.py`, `metrics_recorder.py`, `plot_metrics.py` and `replay_maps.py` touch
none of these classes and are omitted.

## Classes around ReachabilityPipeline

```mermaid
classDiagram
    class NodeComponent {
        <<base>>
        +node
        +log
        +callback_group
        #_declare(name, default)
        #_private(name) str
    }

    class WorkspaceRegionSource {
        +workspace_radius
        +resolve(target_frame) centers
        +fits(centers, vg_info) bool
    }

    class ReachabilityPipeline {
        +grid_spacing
        +grid_size
        +cycle_period
        +gradient_method
        +gate callback
        +on_result callback
        -_on_voxel_grid(msg)
        -_on_timer()
    }

    class BaseCommander {
        +base_speed
        +control_gain
        +allows_motion() bool
        +submit(gradient, cycle_s) vx_vy
        +stop()
    }

    class GradientBaseController {
        <<Node — wiring only>>
        +region
        +pipeline
        +commander
    }

    class GradientBasedController {
        <<ROS-free>>
        +update_workspace_region(centers)
        +compute_quality_scores(rms, vg_info) scores
        +compute_gradient(scores) gx_gy
        +evaluate(rms, vg_info) dict
    }

    class WorkspaceEvaluation {
        <<ROS-free>>
        +region_mask(...) VoxelMask
        +compute_quality_score(...) float
    }

    class VoxelMask {
        <<ROS-free>>
        +compute_mean(voxelmap) float
        +translated(di, dj) VoxelMask
        +union_of_spheres(centers, radius, ...)$ VoxelMask
    }

    class ObstacleMapTransformer {
        <<ROS-free>>
        +transform(voxel_map, dx, dy) Tensor
        +generate_grid_transforms(voxel_map, delta) Tensor
    }

    class ReachabilityEngine {
        <<ROS-free>>
        +trt_model
        +preprocess(indices, sx, sy, sz) Tensor
        +predict_batch(maps) Tensor
    }

    class TRTModel {
        <<ROS-free>>
        +infer(x) Tensor
        +warmup(spatial)
    }

    class GradientResult {
        <<dataclass>>
        +gradient
        +scores
        +centers
        +cycle_s
    }

    NodeComponent <|-- WorkspaceRegionSource
    NodeComponent <|-- ReachabilityPipeline
    NodeComponent <|-- BaseCommander

    GradientBaseController *-- WorkspaceRegionSource : region
    GradientBaseController *-- ReachabilityPipeline : pipeline
    GradientBaseController *-- BaseCommander : commander

    ReachabilityPipeline o-- WorkspaceRegionSource : region — injected, not owned
    ReachabilityPipeline *-- ReachabilityEngine : engine
    ReachabilityPipeline *-- ObstacleMapTransformer : obstacle_transformer
    ReachabilityPipeline *-- GradientBasedController : gradient_ctrl
    ReachabilityPipeline ..> GradientResult : builds per cycle

    ReachabilityEngine *-- TRTModel : trt_model, optional TRT path

    GradientBasedController *-- WorkspaceEvaluation : _region_eval
    WorkspaceEvaluation *-- VoxelMask : _cached_mask
    ObstacleMapTransformer *-- VoxelMask : _static_mask, cached

    GradientBaseController ..> BaseCommander : pipeline.gate = commander.allows_motion
    GradientBaseController ..> ReachabilityPipeline : pipeline.on_result = _on_result, calls commander.submit
    GradientBaseController ..> BaseCommander : region.on_lost = commander.stop
```

`ReachabilityPipeline` and `BaseCommander` never import each other — the
coupling above (`gate`, `on_result`, `on_lost`) is assigned by
`GradientBaseController` after construction, so a node that omits
`BaseCommander` (`workspace_probe`) gets a pipeline that runs identically but
has nothing wired to `gate` and no way to reach `/cmd_vel`.

## Related docs

- `TESTING.md` — running the gradient controller against synthetic or bag data.
- `OPTIMIZATIONS.md` — TensorRT export, CUDA Graph capture, batching constraints.
