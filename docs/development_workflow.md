# Development Workflow & Hardware Integration

## 1. Branching Strategy

`main` is the original Humble-era documentation baseline. `development_v1` is the Jazzy behaviour this refactor started from. `refactor_v1` is the layout and Candle correction.

```bash
# 1. Ensure you are on the branch you mean to extend
git switch refactor_v1
git pull

# 2. Create a new branch for your experiment
git switch -c experiment/my_law

# Write the change, then:
ur5 test --unit

# 3. Add, commit, and push your branch
git add -A
git commit -m "feat(controller): describe the change"
git push -u origin HEAD
```

One concern per commit. Do not put plans or logs in the tree. Those go in `.agent/`, which is gitignored. Source comments do not carry defect ids or changelog notes.

## 2. Synchronizing Workstations (Mac VM & Windows)

The supported hosts are a Windows WSL2 Ubuntu guest, a dedicated Ubuntu VM (including an Apple Silicon ARM64 VM), and native Ubuntu 24.04. See [environment setup](environment_setup.md).

Keep the repository at `<workspace>/src/ur5_digital_twin`. Do not commit `$UR5_DATA_DIR`. Copy HDF5 between machines from `ur5_controller/src/data/` (or from `$UR5_DATA_DIR` if you overrode it). On the Apple Silicon VM set `GZ_SIM_RENDER_ENGINE=ogre` before `ur5 simgui`.

`ur5 doctor` writes a text report under `$UR5_DATA_DIR/doctor`. Attach that report when a session behaves differently on another host.

## 3. Hardware Integration Roadmap (Platinum Maestro)

The sim is the stand-in for an Elmo Platinum Maestro drive on EtherCAT. The ROS graph stays. The physics plugin does not.

### Step 1: Isolate the Hardware Branch

Branch from `refactor_v1`. Do not retune the nominal gains on that branch until the same `experiment_nominal_check.yaml` has been run against the hardware topics.

### Step 2: URDF Hardware Injection

Replace `gz_ros2_control/GazeboSimSystem` with a `ros2_control` hardware interface for the Platinum Maestro. One URDF argument selects sim or hardware. Joint names stay the seven names in the SRDF. Do not widen a joint limit to make a trial pass.

### Step 3: Hardware Launch File

`platform.yaml` is the seam. Point `topics.joint_states` at the drive's joint-state topic and point `brain_a.controller` and `brain_b.controller` at the hardware controller names. Both brains read those keys. No Python edit is required for the topic rename.

### Step 4: Revise the Deployment Guide

Update [operation](operation_and_deployment.md) with the hardware launch command beside the Gazebo one. Candle, the HDF5 layout, and `ur5 validate` stay the same.

## 4. Runtime rules

* The node spins on the main thread. Blocking work (a trajectory, an experiment, a Candle move) runs on a `threading.Thread`.
* `use_sim_time` is set by the launch file.
* Joint vectors are resolved by name. `/joint_states` order is not controller order.
* `ur_manipulator_controller` and `forward_effort_controller` are never active together.
* Gains and model terms are matched at the nominal condition and frozen before a perturbation. Do not retune a law to absorb a perturbation.

## 5. Adding a law

Add a class in `ur5_controller/src/control_laws.py` with `compute(self, t, q, qd, q_ref, qd_ref, qdd_ref)` and register it in `LAWS`. A model-based law uses `NominalModel` terms `rigid_body`, `viscous`, and `coulomb`. Viscous and Coulomb take reference velocity. Computed torque's `C(q, q̇)` takes measured velocity. Coulomb is `tanh(q̇_ref / ε)`.

If the law is scaled into acceleration space, follow `joint_gain_match_at_candle`: divide the shared N·m gains by `M_jj(q_candle)` once at construction.

## 6. Experiment YAML

Shipped files:

| File | `name` | What it runs |
|---|---|---|
| `experiment_nominal_check.yaml` | `nominal_check` | Both bench trajectories, three model-based laws, no perturbation |
| `experiment_slice.yaml` | `slice` | `bench_reach`, PD+gravity and computed torque, gain loss 1.0 and 0.8, two seeds |
| `experiment_benchmark.yaml` | `benchmark` | Four laws, every perturbation, five seeds |

Fields: `trajectories`, `controllers` (`plugin`, `gains`), `perturbations` (`type`, `levels`, optional `joints`), `seeds`, `model_terms`, `coulomb_epsilon`, `hold_sec`, `post_hold_sec`.

| Perturbation | Side | Notes |
|---|---|---|
| `none` | plant | Baseline |
| `viscous_friction` | plant | Four proximal joints, measured `q̇`, levels 0, 0.25, 0.5 of URDF `b`. Requires `b_p T / m_eff <= 0.5` and `(Kd + b_p) T / m_eff <= 1.0` |
| `gain_loss` | plant | Scales the torque that reaches the joint |
| `velocity_noise` | controller | Seeded noise on the velocity the law sees |
| `tip_payload` | plant | Added tip mass |
| `tip_force` | plant | Wrench at the tip |

`learned_id` reads `$UR5_DATA_DIR/models/learned_id.npz`. The file is not shipped. Remove that controller from the YAML to run the other laws.

## 7. Validity

A torque trial is invalid when any joint's Nyquist ratio exceeds 8.5. The ratio is periodogram power near `fs/2` divided by power in the reference band. The runner reruns an invalid trial up to three times and moves the rejected files to `rejected/`. Exit codes are 0 ok, 1 run failed, 2 bad input, 3 platform, 4 safety abort, 130 interrupted.

On a 4-vCPU host the physics step can stall and drop a 10 ms sample. That fails validation. The rerun exists for that case. Do not raise the gap tolerance.
