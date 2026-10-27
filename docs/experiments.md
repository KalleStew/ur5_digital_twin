# Torque experiments

```bash
ur5 sim
ur5 torque <experiment.yaml> [--trial <id>] [--seed <n>]
ur5 validate "$UR5_DATA_DIR"
```

A bare file name is resolved from `ur5_controller/config/experiments/`. Shipped files:


| File                  | What it runs                                                                                     |
| --------------------- | ------------------------------------------------------------------------------------------------ |
| `nominal_check.yaml`  | Both trajectories, PD + gravity, PD + inverse dynamics, computed torque, no perturbation, seed 0 |
| `slice_s10.yaml`      | `bench_reach.csv`, PD + gravity and computed torque, gain loss 1.0 and 0.8, seeds 0 and 1        |
| `benchmark_full.yaml` | Both trajectories, four laws, every perturbation, five seeds                                     |


`learned_id` in `benchmark_full.yaml` reads `$UR5_DATA_DIR/models/learned_id.npz`. The file is not shipped. Training is not part of this repository. Remove that controller from the YAML to run the other three laws. The `.npz` holds `W0, b0, ...` with a tanh hidden stack and a linear output, plus `x_mean`, `x_std` (21) and `y_mean`, `y_std` (7). Input is `[q_ref, qd_ref, qdd_ref]`.

Each trial writes one HDF5 file under `$UR5_DATA_DIR/telemetry/<experiment>_<stamp>/` and a `summary.csv`. The trial id is `<trajectory>-<plugin>-<perturbation><level index>-s<seed>`.

A failed trial is recorded and the experiment continues. The run stops if the Candle pre-flight before the next trial fails, or on interrupt. A trial whose file fails `ur5 validate` is repeated from a fresh controller and the same seed, at most three times. Rejected files move to `$UR5_DATA_DIR/rejected/<experiment>_<stamp>/`. The accepted file records `attempt` and `rejected_attempts`. The process still exits non-zero if any trial failed or exhausted its three attempts.

## Laws

Gains below are N·m/rad and N·m·s/rad, joint order `shoulder_pan_joint1`, `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint`.


| Plug-in           | Law                                                                                                                  |
| ----------------- | -------------------------------------------------------------------------------------------------------------------- |
| `pd_gravity`      | `Kp e + Kd ė + g(q)`. Gravity only. This is the baseline.                                                            |
| `pd_id_ff`        | The same `Kp` and `Kd`, plus `M(q_ref) q̈_ref + C(q_ref, q̇_ref) q̇_ref + g(q_ref)` and the joint terms on `q̇_ref`. |
| `computed_torque` | `M(q) (q̈_ref + kp e + kd ė) + C(q, q̇) q̇ + g(q)` plus the joint terms on `q̇_ref`. `C` uses measured `q̇`.         |
| `learned_id`      | `Kp e + Kd ė` plus the network on `(q_ref, q̇_ref, q̈_ref)`. `model_terms` are not evaluated.                        |


`model_terms` selects what the nominal model adds for `pd_id_ff` and `computed_torque`: `rigid_body`, `viscous`, `coulomb`. The default is all three. Viscous is `b q̇_ref` and Coulomb is `Fc tanh(q̇_ref / ε)` with `coulomb_epsilon: 0.05`. `b` and `Fc` are read from the URDF `<dynamics>` at startup. `pd_gravity` does not use them. Both velocity sources are recorded as `desired`.

`computed_torque` gains in the YAML are the PD + gravity gains. With `protocol: joint_gain_match_at_candle` they are divided, per joint, by `M_jj` at `candle_pose`: `kp_j = Kp_j / M_jj`, `kd_j = Kd_j / M_jj`. The protocol name, `M_jj`, and the resulting gains are stored in the HDF5 metadata. They are fixed before any perturbation. PD + inverse dynamics and the learned law use the PD + gravity gains unchanged.

## Perturbations

Levels are frozen in `benchmark_full.yaml`. Nothing is retuned per level or per trajectory.


| Type               | Side       | Levels                              | What is applied                                                              |
| ------------------ | ---------- | ----------------------------------- | ---------------------------------------------------------------------------- |
| `none`             | plant      | 0                                   | Nothing                                                                      |
| `viscous_friction` | plant      | 0, 0.25, 0.5 of URDF `b`            | Extra `b_p q̇` on the four proximal joints, `q̇` measured                    |
| `gain_loss`        | plant      | 0.9, 0.8, 0.7                       | That fraction of the commanded torque reaches the joints                     |
| `velocity_noise`   | controller | 0.005, 0.02, 0.05 rad/s             | Gaussian noise on the `q̇` the law sees. The logged `q̇` stays clean         |
| `tip_payload`      | plant      | 0.5, 1.0, 2.0 kg                    | A constant world force `(0, 0, -9.81 m)` at `wrist_3_link`. No added inertia |
| `tip_force`        | plant      | `[5,0,0]`, `[10,0,0]`, `[20,0,0]` N | That constant world force at `wrist_3_link`                                  |


Payload and tip force go through the Gazebo persistent wrench topic and are cleared after the trial.

Coulomb mismatch is not perturbed in v1: a Coulomb term fed back from measured `q̇` at 100 Hz produces a stick-slip cycle against the plant stiction at every slope that is still sharp at speed. `viscous_friction` also skips the three wrists. At 100 Hz, extra viscous feedback on those inertias is unstable. A level is refused at startup unless, on every perturbed joint and for that controller, `b_p T / m_eff ≤ 0.5` and `(Kd + b_p) T / m_eff ≤ 1`, with `T = 10 ms` and `m_eff` the smallest `M_jj` over `candle_pose` and every trajectory in the file.

## When a trial is invalid

`ur5 validate` rejects a torque trial if any joint has `nyquist_ratio` above 8.5. That ratio is the mean Hann periodogram power of `q̇` from 48 Hz to 50 Hz, divided by the mean power from 35 Hz to 45 Hz, at the 100 Hz sample rate. The file also stores `artefact_amplitude_rad` and `artefact_fraction_of_rms`; those two do not decide the check. The same command checks shape, monotonic sim time, rate within 2 percent, gaps over 15 ms, finite samples, and the required metadata.