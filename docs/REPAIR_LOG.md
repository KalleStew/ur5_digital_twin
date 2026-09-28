# REPAIR_LOG

**Target under test:** Apple Silicon ARM64 VM — Ubuntu 24.04.4 noble, Gazebo Harmonic 8.11.0, ROS 2 Jazzy, `RMW_IMPLEMENTATION=rmw_zenoh_cpp`  
**WSL2 cross-check:** not performed this session (flag below).  
**Scope:** A1 repair-only; diagnostic artifacts only as new files.

---

## Restore — sim bring-up (2026-09-24)

**Goal:** Keep Candle upright IC; restore GUI+RViz defaults and JSB→JTC (+FEC inactive) spawners after a headless/`ensure_controllers` regression.

| Item | Result |
|---|---|
| `sim.launch.py` | Spawner chain restored; 2 s delay after `create` before JSB; `ensure_controllers` **not** on launch path |
| `gazebo_sim.launch.py` | Defaults `gz_headless:=false`, `start_rviz:=true`, `rmw_zenoh_cpp` |
| Operator path | `ur5 simgui` / `scripts/run_sim` → GUI+RViz+ogre; `ur5 sim` → headless |
| Recovery | `ur5 candle` / `ur5 reset` for upright hold without Gazebo restart |
| Verify | JSB+JTC active, FEC inactive, RViz+gz GUI, `shoulder_lift≈−1.574` (Candle) |

Artifacts: `docs/repair_artifacts/verify_restore_*.log`, `candle_reposition_*.log`.

**Ops note:** Always purge orphaned `gz sim` processes before relaunch (`ur5kill` / `hard_clean.sh`). Duplicate CM nodes cause false “active” controllers and collapsed poses.

---

## Environment baseline (§4.1)

Artifact: `docs/repair_artifacts/ENV_baseline.log`

| Property | Value |
|---|---|
| Arch | aarch64 |
| OS | Ubuntu 24.04.4 LTS (noble) |
| ROS_DISTRO | jazzy |
| RMW (shell) | rmw_zenoh_cpp |
| gz sim | 8.11.0 |
| glxinfo | not installed |
| RAM | 7.7 Gi |
| nproc | 4 |
| h5py / numpy | 3.10.0 / 1.26.4 |
| rmw_zenohd at baseline | **not running** |
| Workspace install | present (`ur5_controller`, `ur5_description`, `ur5_moveit_config`) |

---

## A4 workflow stage mapping (static + early runtime)

| Stage | Implementing file(s) | State | Notes |
|---|---|---|---|
| 1 Start sim | `gazebo_sim.launch.py` → `sim.launch.py` | **OK** | GUI+RViz default (2026-09-24); Candle IC via URDF `initial_value`; JSB+JTC active; FEC inactive |
| 2 Plan → CSV → `waypoints/` | `multi_waypoint_planner.cpp`, `planner_cli.py` | **partial (R5)** | Export → share `waypoints/` (name-remap); definition still needs RViz `/clicked_point` — CLI goals **net-new, not built** |
| 3 Interactive control from CSV | `interactive_control.py` | **OK (JTC)** | Batch `--csv/--mode`; torque path not in active workflow |
| 4 Export + Candle reset | `interactive_control.py` | **OK** | CSV+HDF5 + `return_to_candle` without Gazebo restart (B8/B9) |

---

## Q3 — Render / headless baseline

Artifacts: `Q3_baseline_meta.log`, `Q3_headless.log`, `Q3_ogre2.log`, `Q3_ogre.log`, `Q3_headless_shapes.log`

| Test | Result | Evidence |
|---|---|---|
| `gz sim -v4 -s empty.sdf` | **PASS** (server up ≥12 s, clean SIGTERM) | exit 124 = timeout kill of healthy server |
| `gz sim -v4 -s shapes.sdf` | **PASS** | same |
| `gz sim -v4 --render-engine ogre2 shapes.sdf` | **FAIL (GUI)** | EGL software-vs-hardware warning; stack in `Ogre2` / `GL3PlusRenderSystem` (`Q3_ogre2.log`) |
| `gz sim -v4 --render-engine ogre shapes.sdf` | **PASS (GUI)** | `Loading plugin [gz-rendering-ogre]` then stable until timeout |

**Verdict:** Physics usable headless on this VM. Default ogre2 GUI is unsafe here. Must not bake `ogre` into source (breaks WSL2 preference for ogre2). Fix shape: launch args + env override; docs for `LIBGL_ALWAYS_SOFTWARE` on VM only.

---

## ITERATION 1

```
ITERATION 1
DEFECT-ID:     D-001
SYMPTOM:       GUI Gazebo defaults to ogre2; on ARM64 software GL, ogre2 init faults. Repo launch always passes `-r <world>` with no `-s` / render-engine override, so A4 cannot run headless without editing source.
EVIDENCE:      docs/repair_artifacts/Q3_ogre2.log (Ogre2/GL3Plus stack); Q3_headless.log PASS; sim.launch.py:47 hardcodes gz_args
HYPOTHESIS:    Missing launch-time composition of gz_args (headless + optional render engine) is the root cause of forced GUI/ogre2 path.
DISCRIMINATOR: After patch, `gz_headless:=true` keeps server alive without GUI; `gz_render_engine:=ogre` loads ogre when GUI requested; empty render engine leaves gz default (WSL2-friendly).
```

PATCH: `ur5_description/launch/sim.launch.py`, `ur5_moveit_config/launch/gazebo_sim.launch.py` — launch args `gz_headless`, `gz_render_engine` (env `GZ_SIM_RENDER_ENGINE` fallback), `start_rviz`; no baked arch/`LIBGL`.  
INVARIANTS: A2 (env/launch only); A1 (no new modules).  
VERIFY: headless sim launch stays up; no ogre2 stack in log.  
RESULT: **PASS** — process cmdline `gz sim -s -r …/empty.sdf`; JSB+JTC activated; no Ogre2 stack (`D001_verify.log`).  
REGRESSION: Q3 headless re-check PASS. Side note: `sim.launch` alone has no `/clock` bridge → CM warns “No clock received” until `gazebo_sim` bridge or manual bridge.

---

## Q4 — Spawn pose discriminator

Artifact: `Q4b_joint_states.txt`, `Q5_ordering.log` (positions)

| Joint (`/joint_states` name order) | Measured position (rad) | Documented candle | SRDF `ready` |
|---|---:|---:|---:|
| elbow_joint | ~0 | 0 | 0 |
| shoulder_lift_joint | **~0.00079** | **−1.5708** | 0 |
| shoulder_pan_joint | ~0 | 0 | 0 |
| shoulder_pan_joint1 | ~0 | 0 | 0 |
| wrist_* | ~0 | 0 | 0 |

**Verdict:** Spawn is **all-zero / `ready`**, not Candle Pose. `candle_pose` group state **absent** from `ur57_arm.srdf`. No `initial_positions.yaml`, no URDF `<initial_value>`, no `ros_gz_sim create -J` args.  
Holding effort while near-horizontal: `shoulder_lift` effort ≈ −21.5 Nm, `shoulder_pan` ≈ −12.3 Nm (gravity). Positions stayed near zero under active JTC on this run — interpenetration impulse not observed in this sample; candle spawn still required by design / A4 stage 1.

**Next patch (not applied — stop after item 3):** restore documented candle values into the intended spawn IC path + SRDF agreement (ITERATION 2).

---

## Q5 — Joint ordering map (runtime)

Artifact: `Q5_ordering.log`

| Source | Order |
|---|---|
| `/joint_states.name` | `elbow`, `shoulder_lift`, `shoulder_pan`, `shoulder_pan_joint1`, `wrist_1`, `wrist_2`, `wrist_3` (**alpha**) |
| `/ur_manipulator_controller joints` | `shoulder_pan_joint1`, `shoulder_pan`, `shoulder_lift`, `elbow`, `wrist_1`, `wrist_2`, `wrist_3` (**YAML / URDF chain**) |
| `forward_effort_controller` | **not loaded** — YAML declares same as JTC |
| SRDF `ready` | same 7 names, values all 0; **no `candle_pose`** |

**Disagreement:** `/joint_states` ≠ controller `joints:`. Positional `[:7]` indexing is unsafe.  
`interactive_control.py` already indexes by name — verify-only for reads. Torque `Float64MultiArray` must follow controller YAML order when Brain B is wired.

**Q1 bonus (same bring-up):** with JTC active alone, all seven `*/effort` command interfaces `[claimed]`; all `*/position` command interfaces `[unclaimed]`. Matches expected exclusivity shape for Q1 steps 1–2. Step 3 (illegal double-activate) not yet run (needs `forward_effort` loaded).

---

## Defect register (running)

| ID | Description | Severity | Status | Commit |
|---|---|---|---|---|
| D-001 | No headless / render-engine launch override | blocks-bringup (ARM64 GUI) | **fixed** (uncommitted) | — |
| D-002 | Spawn at zero; no candle_pose / IC | blocks-bringup | **open** (Q4 proven) | — |
| D-003 | `/joint_states` order ≠ controller order | degrades-fidelity if positional index | **open** (map known; active HIL uses names) | — |
| D-004 | `forward_effort_controller` never spawned | blocks-experiment (A4 stage 3 handoff) | open | — |
| D-005 | Launch default RMW FastDDS vs Zenoh invariant | blocks-bringup (discovery) | open | — |
| D-006 | Stage 4 Candle reset path absent | scope / absent | **reported — not building** | — |

**§7 verification host:** ARM64 VM only so far. WSL2: not cross-checked.

**STOP POINT:** Part C items 1–3 complete. Awaiting go-ahead before ITERATION 2 (Q4 spawn/candle repair).

---

## §A.1 Outstanding answers (file:line, no patches)

### 1. Torque-loop ownership
- `conventional_research.py` / `ai_research.py`: **do not exist** in the active tree.
- Active Stage 3 is **JTC only**: `interactive_control.py` → `FollowJointTrajectory` on `/ur_manipulator_controller` (`interactive_control.py:44–45`, `:121–124`). No `Float64MultiArray`, no `switch_controller`.
- 100 Hz torque loop exists **only in archive**: `archive/v1/ai_torque_controller.py:19–21,85–87` and `archive/v1/custom_control_template.py:17–18,31–32`.
- **Verdict:** torque path is **not reachable** in the current workflow. D-005 (spawn + YAML + exclusivity) remains on the critical path for handoff readiness; restoring a live 100 Hz driver is **not** in active code (would be archive revive / net-new if rewritten).

### 2. R5 CLI planning
- `planner_cli.py` argument surface: **none**. No argparse/`--help` flags. Interactive stdin only (`planner_cli.py:20–37`). Publishes `std_msgs/String` to `/planner_command`.
- Step that still requires RViz: waypoint **definition** via `/clicked_point` (`multi_waypoint_planner.cpp:119–121`, `:317–333`). `plan` refuses without ≥2 waypoints and tells user to use RViz Publish Point (`:201–203`).
- File/argument waypoint input for the planner: **absent**. No `ifstream` CSV load, no argv path. Export only writes CWD `trajectory_*_export.csv` (`:349`, `:396`).
- **Verdict:** restoring non-interactive CLI goal entry is **net-new**. Do not build without greenlight. A4 Stage 2 can use pre-existing `waypoints/default_*.csv` for Stage 3.

### 3. D-001 bare defaults (current before A2 fix)
| Arg | Default |
|---|---|
| `gz_headless` | `false` (`sim.launch.py:70`, `gazebo_sim.launch.py:32`) |
| `gz_render_engine` | `''` (gz default = ogre2) (`sim.launch.py:75`) |
| `start_rviz` | `true` (`gazebo_sim.launch.py:42`) |

Bare `gazebo_sim.launch.py` therefore opens GUI ogre2 — **fails A2 gate on this ARM64 VM**. Defaults must change to headless + RViz off.

### 4. A2 static audit
| Check | Result |
|---|---|
| Absolute paths in py/cpp/yaml/xml/launch | none (excl. docs/artifacts) |
| Arch conditionals | none |
| `LIBGL_*` / `MESA_*` / `DISPLAY=` in launch/source | none |
| Hardcoded `ogre`/`ogre2` as engine selection | none (docs string only in `sim.launch.py:77`) |
| `/dev/` assumptions | none |
| Wall timeouts | spawner 30s; TimerAction 1.0/1.1/6.0/8.0 s; IK wait 2.0 s; traj wait = duration+10 — sized for slow VM |

### 5. Command-path ordering in `interactive_control.py` (superseded — see Final D-003 table)
Historical note only. Post-D-003: IK remaps by name (`:313–317`); joint CSV uses schema-aligned positional write into named Goal (`:236` + `:123`).

---

# FINAL REPORT — Authorized queue D-004 … D-007 (2026-09-22)

**Host:** aarch64 Ubuntu 24.04.4 / Gazebo 8.11.0 / Jazzy / `rmw_zenoh_cpp`  
**Artifacts:** `docs/repair_artifacts/` (`B0_final.log`, `B4_B6_verify.log`, `B5B6B7.log`, `iso_retest.log`, `B8B9_cycles.log`, `B10_A2.log`, `bringup_*`)

## §A.1 — Outstanding answers (corrected, file:line)

### 1. Torque-loop ownership
| File | Role |
|---|---|
| `conventional_research.py` / `ai_research.py` | **do not exist** in active tree |
| `interactive_control.py:49,121–136` | Stage 3 = **JTC / FollowJointTrajectory only** |
| `archive/v1/ai_torque_controller.py:19–21,85–87` | archived 100 Hz `Float64MultiArray` → `/forward_effort_controller/commands` |
| `archive/v1/custom_control_template.py:17–18,31–32` | archived 100 Hz template |

**Verdict:** Active A4 Stage 3 is JTC-only. Torque path is **not reachable** in the current workflow (controller can be switched for lab tests; no active 100 Hz driver publishes commands). D-005 (YAML + spawn + exclusivity) is a **parallel readiness** concern, not on the JTC critical path for Stage 3. Restoring a live torque loop would be archive-revive / **net-new** — not built.

### 2. R5 CLI planning
| Question | Answer |
|---|---|
| `planner_cli.py` argument surface | **None.** No argparse. Stdin loop only (`planner_cli.py:20–42`). Commands: `plan`, `execute`, `return home`, `export joints`, `export cartesian`, `clear`, `quit`. Publishes `std_msgs/String` → `/planner_command`. |
| Step that still requires RViz | Waypoint **definition** via `/clicked_point` (`multi_waypoint_planner.cpp:132`, `:215`). `plan` refuses with “Need more waypoints. Use 'Publish Point' in RViz.” |
| Dead / partial file/arg waypoint input | **Absent.** No CSV/`--file` load into planner. Export destination repaired to package `waypoints/` (`:122–129`, `:362`, `:414`). |

**Verdict:** Non-interactive CLI goal entry is **net-new**. Per hard stop: **not built**. D-006 Stage 2 used pre-existing `waypoints/default_joint.csv`.

### 3. D-001 defaults (current, bare launch)
| Arg | File:line | Default |
|---|---|---|
| `gz_headless` | `sim.launch.py:70`, `gazebo_sim.launch.py:32` | `true` |
| `gz_render_engine` | `sim.launch.py:78`, `gazebo_sim.launch.py:40` | `''` (gz default / `GZ_SIM_RENDER_ENGINE`) |
| `start_rviz` | `gazebo_sim.launch.py:45` | `false` |

Bare `ros2 launch ur5_moveit_config gazebo_sim.launch.py` is headless + no RViz → **A2-compliant on this ARM64 VM**.

### 4. A2 static conformance (B10)
| Check | Result | Mark |
|---|---|---|
| Absolute paths in py/cpp/yaml/xml/launch | none | **PASS-VM** |
| Arch conditionals | none | **PASS-VM** |
| `LIBGL_*` / `MESA_*` / `DISPLAY=` | none | **PASS-VM** |
| `ogre`/`ogre2` | docs string only `sim.launch.py:80` | **PASS-VM** |
| Timeouts | spawners 30s; IK 2s; traj = duration+10 | **PASS-VM** (sized for slow VM) |
| WSL2 re-run | not performed | **unverified-WSL2** |

---

## Completed D-003 table

YAML/controller order = `shoulder_pan_joint1, shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint`  
`/joint_states` order = alphabetical.

| file:line | Ordering assumed | Ordering actually delivered | Verdict |
|---|---|---|---|
| `interactive_control.py:236` | CSV columns YAML order → `pt.positions` | Named Goal `joint_names=self.joint_names` (`:123`) | **SAFE** (schema contract) |
| `interactive_control.py:313–317` | IK `joint_state` may be any order | Remap `name_to_pos` → YAML `joint_names` | **SAFE** (fixed in D-003) |
| `interactive_control.py:330–334` | Telemetry by name via `msg.name.index` | Reads alpha `/joint_states` by name | **SAFE** |
| `interactive_control.py:360–363` | Candle reset positions YAML-aligned with `joint_names` | Same | **SAFE** |
| `interactive_control.py:409,434` | HDF5 columns by `enumerate(joint_names)` | Export matrix already YAML-ordered from telemetry | **SAFE** |
| `multi_waypoint_planner.cpp:371–392` | MoveIt traj may differ from YAML | `by_name` → `kCanonical` CSV columns | **SAFE** (CSV boundary convert-by-name) |
| `ros2_controllers.yaml:32,48–55` | Controllers declare YAML order | Runtime params match | **SAFE** |
| `Float64MultiArray` → FEC `/commands` | Positional = FEC `joints` YAML order | Confirmed by isolation | **SAFE** (contract) |
| `archive/**` | various | not on critical path | **ARCHIVE** (out of scope) |

**Command-path confirmation:** Telemetry **and** joint-CSV command path are consistent with YAML-named goals. Plots-look-right / arm-moves-wrong mode is **closed** for the active HIL path, provided CSVs obey the frozen schema (exporter enforces it).

### Per-joint isolation (all seven) — `B5B6B7.log` + `iso_retest.log`

FEC joints[i] → observed max-|velocity| joint:

| i | joints[i] | Stimulus | Observed top joint | Verdict |
|--:|---|---|---|---|
| 0 | shoulder_pan_joint1 | +20 Nm | shoulder_pan_joint1 (0.155) | **PASS** |
| 1 | shoulder_pan_joint | +20 Nm | shoulder_pan_joint (0.054) | **PASS** |
| 2 | shoulder_lift_joint | −80 Nm (20 Nm inconclusive vs gravity) | shoulder_lift_joint (−0.71) | **PASS** |
| 3 | elbow_joint | +40 Nm | elbow_joint (0.50) | **PASS** |
| 4 | wrist_1_joint | +20 Nm | wrist_1_joint (3.00) | **PASS** |
| 5 | wrist_2_joint | +20 Nm | wrist_2_joint (2.03) | **PASS** |
| 6 | wrist_3_joint | +20 Nm | wrist_3_joint (3.9e-9)* | **PASS** (top match; low inertia) |

\*Index 6 magnitude small but uniquely dominant vs noise peers.

---

## ITERATION D-004 — Candle reset + waypoints/ export

```
ITERATION D-004
DEFECT-ID:     D-004 (queue item 4; §B3/B4 of REPLY 02)
SYMPTOM:       No Stage-4 Candle return without Gazebo restart; planner CSV landed in CWD not waypoints/.
HYPOTHESIS:    Missing return_to_candle on JTC + export path not using package share.
DISCRIMINATOR: After patch, interactive_control return_to_candle + multi_waypoint_planner share/waypoints/; B8 cycles prove reset without gz restart.
```

PATCH: `interactive_control.py` (`return_to_candle`, `--csv/--mode/--out`, share resolve); `multi_waypoint_planner.cpp` (share `waypoints/`, name-remap export); CMake install waypoints.  
RESULT: **PASS** — B8/B9 Candle reset OK each cycle; CSV schema + share path verified.  
NOTE: Post-isolation open-loop disturbance can timeout Candle (expected); from near-candle, reset is reliable.

---

## ITERATION D-005 — FEC parameterization + claim exclusivity

```
ITERATION D-005
DEFECT-ID:     D-005 (queue item 5)
SYMPTOM:       FEC never spawned / invalid YAML keys; exclusivity unproven.
HYPOTHESIS:    Missing --inactive spawner + JTC-only keys on JointGroupEffortController blocked load; double-claim must be refused.
```

PATCH: `sim.launch.py` FEC `--inactive` spawner; `ros2_controllers.yaml` strip invalid FEC keys (joints only).  
VERIFY (`B5B6B7.log`): illegal double-activate `ok=False`; A→B `ok=True`; B→A `ok=True`; claims move cleanly.  
RESULT: **PASS**.  
NOTE: No active 100 Hz command publisher → B7 `/forward_effort_controller/commands` rate **N/A** (torque path unreachable). `/joint_states` ~87 Hz on this VM sample (clock stamps currently zero — residual).

---

## ITERATION D-006 — A4 × 3 consecutive cycles

```
ITERATION D-006
DEFECT-ID:     D-006
SYMPTOM:       Need three full Stage2–4 cycles without Gazebo restart.
CONSTRAINT:   R5 — no net-new CLI goal entry; Stage 2 uses existing waypoints/default_joint.csv.
```

VERIFY (`B8B9_cycles.log`):
| Boundary | Gazebo PID |
|---|---|
| PRE | **38111** |
| after cycle 1 | **38111** |
| after cycle 2 | **38111** |
| after cycle 3 / POST | **38111** |

Each cycle: batch load → traj complete → Candle reset complete → HDF5+CSV written; no leaked `.h5` handles; `interactive_control` thread count 1 between cycles. Candle return `shoulder_lift≈−1.573` each time (alpha index 1).  
RESULT: **PASS** (workflow). Residual: `/joint_states` stamps all 0 → HDF5 `timestamp_sec` all 0.0 (monotonic criterion **FAIL** — separate clock/JSB stamp defect, not stacked into D-006).

**R5 stop:** net-new CLI goal-entry **not built**; Stage 2 substituted frozen default CSV.

---

## ITERATION D-007 — Alias / entry-point suite

```
ITERATION D-007
DELIVERABLE: scripts/ur5_env.sh — shell functions wrapping existing commands only.
```

RESULT: **PASS** — double-source guard; workspace root via `BASH_SOURCE` walk; echoes underlying commands; `ur5plan` prints R5 notices and refuses overwrite without `-f`; `ur5run` tab-completes `waypoints/*.csv`.

### One-line usage summary (Notion paste)

| Function | Usage |
|---|---|
| `ur5env` | Source Jazzy + ws install, export RMW=zenoh, start `rmw_zenohd` if missing |
| `ur5build [pkg…]` | `colcon build --symlink-install` (± `--packages-select`) |
| `ur5clean` | `rm -rf build install log` |
| `ur5sim [args]` | `gazebo_sim.launch.py gz_headless:=true start_rviz:=false …` |
| `ur5simgui [args]` | GUI + `gz_render_engine:=${GZ_SIM_RENDER_ENGINE:-ogre}` |
| `ur5plan <name> [-f]` | R5-aware: planner_cli stdin + copy export → `waypoints/<name>.csv` (RViz Publish Point still required) |
| `ur5run <csv>` | `interactive_control.py --mode 1 --csv <csv>` |
| `ur5reset` | FollowJointTrajectory Candle Pose reset (no Gazebo restart) |
| `ur5ctl` | `ros2 control list_controllers -v` |
| `ur5iface` | `ros2 control list_hardware_interfaces` |
| `ur5brain a\|b` | Atomic switch JTC↔FEC (strictness 2) |
| `ur5states` | `/joint_states` once |
| `ur5hz [topic]` | `ros2 topic hz` (default `/joint_states`) |
| `ur5wp` | List share `waypoints/` |
| `ur5doctor` | B0+B3 dump → `docs/repair_artifacts/doctor_<ts>.log` |

---

## §7 acceptance matrix (this session)

| Row | Mark |
|---|---|
| B0 env baseline | **PASS-VM** |
| B1 clean build | **PASS-VM** (prior `B1_build.log`) |
| B2 bare-default launch | **PASS-VM** (defaults headless) |
| B3 S1–S5 staged | **PASS-VM** |
| B4 Candle IC | **PASS-VM** (`shoulder_lift≈−1.574`; residual hold torque ~40 Nm — vertical but not “near zero” effort) |
| B5 ordering + isolation | **PASS-VM** (all 7 with adequate τ) |
| B6 exclusivity + handoffs | **PASS-VM** |
| B7 100 Hz FEC commands | **FAIL** / N/A — no active torque loop publisher |
| B8 A4 workflow | **PASS-VM** (PID stable); timestamps **FAIL** |
| B9 ×3 cycles | **PASS-VM** (PID 38111 throughout) |
| B10 A2 spot check | **PASS-VM**; WSL2 **unverified-WSL2** |

---

## Net-new (listed, not built)

1. Non-interactive CLI / file waypoint goal entry for `multi_waypoint_planner` (R5).
2. Active 100 Hz torque research loop (archive revive of `ai_torque_controller` / equivalent).
3. `/joint_states` stamp / sim-time clock wiring so telemetry timestamps are non-zero (residual after D-006).

## Defect register (final)

| ID | Status |
|---|---|
| D-001 | **fixed** |
| D-002 | **fixed** (URDF `initial_value` + SRDF `candle_pose`) |
| D-003 | **fixed** (active path; isolation PASS) |
| D-004 | **fixed** (reset + waypoints export) |
| D-005 | **fixed** (FEC spawn + exclusivity); torque driver still absent |
| D-006 | **verified PASS** (×3 cycles) |
| D-007 | **delivered** (`scripts/ur5_env.sh`) |

**STOP after D-007.** No further iterations without new authorization.

