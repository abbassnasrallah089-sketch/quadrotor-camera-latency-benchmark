# Camera latency compensation for vision-based quadrotor target tracking

MATLAB/Simulink code of the closed-loop benchmark presented in the article
**"Camera latency compensation for vision-based UAV target tracking: A structured review of models and strategies"** (submitted to *Annual Reviews in Control*).

A quadrotor with a body-fixed, forward-looking camera tracks a moving spherical target with a partitioned image-based visual servoing (IBVS) controller. The image measurement reaches the controller **160 ms late** and is refreshed only once per camera frame (about 30 Hz). Four compensators are compared. Each one replaces the delayed measurement before the controller; the plant, the camera, the controller and the initial state are the same in every condition.

| Compensator | Family | Idea |
|---|---|---|
| Delayed extended Kalman filter (DKF) | Model-based | Rewind to the exposure instant, correct there, replay to the present; adaptive (Kalman) gain |
| Delayed Luenberger observer (DLO) | Model-based | Same window and replay as the DKF, but a fixed α–β gain from pole placement |
| LSTM residual predictor | Learned | A recurrent network predicts the correction to add to the delayed measurement; first-order output filter (τ = 20 ms) |
| Neural Luenberger observer (NLO) | Hybrid | A network predicts the image-plane velocity, an integrator turns it into an estimate, and a delayed innovation (gain L = 0.10) removes drift |

---

## Requirements

- **MATLAB R2024a with Simulink.** All models were saved in R2024a.
- No other toolbox is needed. The two networks are written out by hand inside MATLAB Function blocks, so the Deep Learning Toolbox is **not** required.

## Repository structure

```
.
├── QuadVisionControl.slx               no delay (reference / ceiling)
├── QuadVisionControl_delay.slx         160 ms delay, uncompensated (floor)
├── QuadVisionControl_dkf.slx           delay + DKF
├── QuadVisionControl_luenberger.slx    delay + DLO
├── QuadVisionControl_lstm.slx          delay + LSTM residual predictor
├── QuadVisionControl_obs.slx           delay + Neural Luenberger observer
├── QuadVisionControl_noise_delay.slx   noisy camera, uncompensated
├── QuadVisionControl_noise_dkf.slx     noisy camera + DKF
├── QuadVisionControl_noise_luen.slx    noisy camera + DLO
│
├── observers/
│   ├── dkf/                 dkf_block.m  dkf_core.m  dkf_init.m  dkf_params.m
│   ├── dkf+noise/           dkf_block_noise.m  dkf_core.m  dkf_init.m  dkf_params_noise.m
│   ├── luenberger/          luen_block.m  luen_core.m  luen_init.m  luen_params.m
│   ├── luenberger+noise/    luen_block_noise.m  luen_core.m  luen_init.m  luen_params_noise.m
│   ├── lstm/                lstm_block.m  lstm_forward_core.m  smooth_block.m  lstm_config.m
│   │                        lstm_weights.mat  (+ backup copy lstm_weights_r4f.mat)
│   └── neural luenberger/   velocity_net_block.m  obs_integrator_block.m  lstm_forward_core.m
│                            lstm_weights.mat  (+ backup copy lstm_weights_obs2.mat)
│
└── plots/                   run_all_sims.m, plotting and visualisation scripts, helpers
```

Each model contains the quadrotor dynamics, the target trajectory generator, the pinhole camera, the delay and sample-and-hold block, and the three outer control loops as MATLAB Function blocks. The only external files are the compensators in `observers/` and the scripts in `plots/`.

---

## Quick start

Open MATLAB, go to the folder where you cloned or unzipped the repository, and run:

```matlab
ROOT = pwd;                                   % the repository folder
addpath(fullfile(ROOT, 'plots'))
addpath(fullfile(ROOT, 'observers', 'dkf'))

idx = 8;                                      % scenario (see below)
out1 = sim('QuadVisionControl');              % no delay
out3 = sim('QuadVisionControl_dkf');          % delay + DKF

RUN_ID_FORCE = 1; plot_distance_error
RUN_ID_FORCE = 3; plot_distance_error
clear RUN_ID_FORCE
```

You should get two figures: the delay-free reference and the DKF on the hardest of the three reported scenarios. `plot_distance_error` also prints the settling time, the final error, the minimum separation and the RMS error.

## What every run needs

**The scenario, `idx`.** Every model reads the workspace variable `idx`, which selects the target motion. The model will not start if it is undefined.

| `idx` | Target motion | Paper |
|---|---|---|
| 1 | Stationary | |
| 2 | Vertical climb, Vz = 20 m/s | **Scenario 1** |
| 3 | Move −X, 3 m/s | |
| 4 | Move +Y, 3 m/s | |
| 5 | Lateral and climbing, Vy = 5, Vz = 20 m/s | **Scenario 2** |
| 6 | X + Y, 1 m/s each | |
| 7 | X + Z, Vx = 1, Vz = 20 m/s | |
| 8 | Fully three-dimensional, Vx = Vy = 1, Vz = 20 m/s | **Scenario 3** |
| 9–15 | Manoeuvring target: same vertical path, speed Vz = 15 + A·sin(2πt/Tp) | Manoeuvre sweep |

The manoeuvring scenarios 9–15 are implemented in `QuadVisionControl`, `_lstm`, `_obs`, `_noise_dkf` and `_noise_luen`. In the other models these indices fall back to a stationary target. idx 9, 10 and 11 use Tp = 10 s with A = 1, 2 and 4 m/s; idx 14, 15, 12 and 13 use Tp = 6, 5, 4 and 2.5 s with A = 4 m/s.

**The initial condition.** Every run starts from the same state: 9.27 m from the target, at rest, level, yaw 2 rad. The LSTM and NLO models read it from four workspace variables, so define them once per session:

```matlab
IC_pos   = [15; 5; 7];   IC_vel   = [0; 0; 0];
IC_euler = [0; 0; 2];    IC_omega = [0; 0; 0];
```

Solver (`ode4`, fixed step 5 ms) and stop time (30 s) are saved inside every model.

## Running each condition

| Condition | Folder to add to the path | Clear before the run | Command | Result |
|---|---|---|---|---|
| No delay | – | – | `out1 = sim('QuadVisionControl');` | `out1` |
| Delay, uncompensated | – | – | `out2 = sim('QuadVisionControl_delay');` | `out2` |
| Delay + DKF | `observers/dkf` | `clear dkf_block dkf_core dkf_init` | `out3 = sim('QuadVisionControl_dkf');` | `out3` |
| Delay + DLO | `observers/luenberger` | `clear luen_block luen_core luen_init` | `out4 = sim('QuadVisionControl_luenberger');` | `out4` |
| Delay + LSTM | `observers/lstm` | `clear lstm_block lstm_forward_core` | `out5 = sim('QuadVisionControl_lstm');` | `out5` |
| Delay + NLO | `observers/neural luenberger` | `clear velocity_net_block obs_integrator_block lstm_forward_core` | `out10 = sim('QuadVisionControl_obs');` | `out10` |
| Noise, uncompensated | – | – | `out8 = sim('QuadVisionControl_noise_delay');` | `out8` |
| Noise + DKF | `observers/dkf+noise` | `clear dkf_block_noise dkf_core dkf_init` | `out6 = sim('QuadVisionControl_noise_dkf');` | `out6` |
| Noise + DLO | `observers/luenberger+noise` | `clear luen_block_noise luen_core luen_init` | `out7 = sim('QuadVisionControl_noise_luen');` | `out7` |

Two rules matter:

1. **One compensator folder on the path at a time.** `observers/lstm` and `observers/neural luenberger` both contain a file called `lstm_weights.mat` holding *different* networks. If both folders are on the path, the first one wins and the other model silently runs the wrong network. The same applies to the noise-free and noisy DKF and DLO folders, whose parameter files differ. Use `rmpath` on the folder you are not using.
2. **Clear the persistent memory before every run.** The compensators keep their state (filter window, network memory, integrator) in persistent variables, which survive between simulations. Run the `clear` line of the table before each `sim`.

The noise campaign adds Gaussian detector noise of 1.5 px on the centroid and 0.25 px on the apparent radius. It is drawn once per frame from a fixed seed, so every method meets exactly the same noise sequence.

**Everything at once.** `run_all_sims` runs the eight models other than the NLO and fills `out1` … `out8`:

```matlab
addpath(fullfile(ROOT,'observers','dkf'), fullfile(ROOT,'observers','luenberger'), ...
        fullfile(ROOT,'observers','lstm'))
scen = 8;            % same meaning as idx
run_all_sims
```

## Plotting the results

Every plotting script picks a run by its number (1–8, or 10 for the NLO) and fetches the matching `outN` from the workspace.

| Script | What it draws |
|---|---|
| `plot_distance_error.m` | Separation from the target and error about the 2 m stand-off; prints settling time, final error, minimum separation and RMSE |
| `compare_distance.m` | Separation of several conditions on one axis, with a comparison table (set `IDS`, e.g. `[1 2 3 4 5]` or `[8 6 7]`) |
| `compare_trajectory.m` | 3-D flight paths of several conditions |
| `compare_z_saturation.m`, `plot_z_saturation.m` | The altitude channel against the vertical-speed limit |
| `visualize_3d_trajectory_spotlight.m` | Animated 3-D pursuit with playback controls |
| `visualize_camera.m`, `visualize_combined.m` | What the camera sees, alone or next to the 3-D view |

`pickrun.m`, `getsig.m`, `loadruns.m` and `runmany.m` are helpers called by the scripts above.

## Main parameters

| Quantity | Value | Where it is set |
|---|---|---|
| Control / simulation step Δt | 5 ms (200 Hz) | model solver; `DT` in each compensator |
| Camera delay τ | 160 ms (D = 32 steps) | `DELAY_S` in the delay block; `DELAY_S`/`D_STEPS` in each compensator |
| Camera period | 7 steps (35 ms, nominal 30 Hz) | `DT_CAM`/`CAM_STEP` in the delay block and the learned compensators |
| Camera | 3280 × 2464 px, 160° horizontal field of view, f = 289.19 px | camera model block |
| Target | sphere of radius 0.3 m | camera model block |
| Stand-off distance | 2 m | constant in the models |
| DKF (noise-free / noisy) | σa = [5 5 10] / [2 2 4] m/s², σuv = 0.5 / 1.5 px, σs = 0.25 px | `dkf_params.m`, `dkf_params_noise.m` |
| DLO (noise-free / noisy) | ωn = 8 / 4 rad/s, ζ = 1 | `luen_params.m`, `luen_params_noise.m` |
| LSTM | 64 hidden units, 20 input features, output low-pass τ = 20 ms | `lstm_block.m`, `smooth_block.m` |
| NLO | innovation gain L = 0.10, spread as L/7 per step | `obs_integrator_block.m` |

### Changing the delay

The compensators are formulated for any delay of D steps, but the value is written as a named constant in several files: `DELAY_S` in the delay block of each model, `dkf_params.m` and `luen_params.m` (plus the window size `NB = D + 1` in `dkf_init.m` and `luen_init.m`), and `D_STEPS` in `lstm_block.m`, `velocity_net_block.m` and `obs_integrator_block.m`. The DKF and DLO only need these values updated. The two networks were trained for D = 32 and must be **retrained** for another delay, and the NLO gain must respect L/7 < 2 sin(π / (2(2D + 1))).

## Notes

- **Trained weights.** The weights provided are the networks reported in the article: the round-4 filter-aware LSTM, and the round-2 velocity network of the NLO. Each must be used with the element it was trained with: the 20 ms output filter for the LSTM, and L = 0.10 for the NLO. In the reported tests, the same weights diverged when the filter was removed or when the NLO was flown at L = 0.219 instead of 0.10. `lstm_weights_r4f.mat` and `lstm_weights_obs2.mat` are identical backup copies.
- **Training pipeline.** The offline data generation and network training are not part of this repository; it provides everything needed to run and reproduce the closed-loop results.
- **Stale build cache.** If you get code-generation errors after moving the folder, close the models, delete any `slprj/` folder and `.slxc` files beside them, and reopen.

## License

Released under the MIT License (see `LICENSE`).
