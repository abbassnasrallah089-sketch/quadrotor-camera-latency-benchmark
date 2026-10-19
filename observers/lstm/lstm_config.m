function cfg = lstm_config()
%% LSTM_CONFIG — every constant for the LSTM delay-compensation pipeline
%
%  ROLE (plain words)
%  One file that holds every number and every signal name the pipeline uses.
%  Nothing else is allowed to hard-code a constant. If a number disagrees
%  between MATLAB and Python, it is a bug, and this file is the referee.
%
%  This version is NOISE-FREE. The camera reports exact truth, so the raw
%  camera signals ARE the training target and there is no second camera.

%% ── Where things live ──────────────────────────────────────
cfg.model_name = 'QuadVisionControl_obs';   % observer plant. was: QuadVisionControl_lstm
%  Paths are resolved relative to this file, so the repository can sit
%  anywhere. The offline training pipeline (data generation, Python
%  training, dataset aggregation) is NOT part of this repository; data_dir
%  and weight_dir are kept only so the field names stay documented.
here           = fileparts(mfilename('fullpath'));      % ...\observers\lstm
cfg.model_dir  = fullfile(here, '..', '..');            % repository root (.slx files)
cfg.code_dir   = here;
cfg.lstm_dir   = here;
cfg.data_dir   = fullfile(cfg.lstm_dir, 'data');
cfg.weight_dir = fullfile(cfg.lstm_dir, 'model');

%% ── Camera geometry (must match camera_model.m EXACTLY) ────
cfg.cx    = 1640.0;
cfg.cy    = 1232.0;
cfg.focal = 289.19;

%% ── Timing ─────────────────────────────────────────────────
cfg.dt          = 0.005;   % 200 Hz control loop
cfg.cam_step    = 7;       % camera runs at 30 Hz  -> new frame every 7 steps
cfg.delay_steps = 32;      % 160 ms / 5 ms         -> the delay we must undo
cfg.t_end       = 30.0;    % seconds per run
cfg.warmup_steps = 100;    % first 0.5 s dropped from training (transients)

%% ── The FEATURE CONTRACT ───────────────────────────────────
%  This number appears in exactly three places: here, in lstm_block.m, and
%  in py/common.py. All three are checked against each other before any
%  training happens. The 18-vs-20 failure of the previous attempt was
%  precisely this contract being broken silently.
cfg.n_features = 20;
cfg.n_outputs  = 3;        % residuals for u, v, radius

cfg.feature_names = { ...
    'u_tilde','v_tilde','r_tilde', ...        %  1- 3  normalised delayed meas.
    'd_u','d_v','d_r', ...                    %  4- 6  change over one cam period
    'valid_flag', ...                         %  7     new delayed frame landed
    'phi','theta','sin_psi','cos_psi', ...    %  8-11  attitude
    'deul_phi','deul_theta','deul_psi', ...   % 12-14  attitude change over delay
    'p','q','r', ...                          % 15-17  body rates
    'a_d_x','a_d_y','a_d_z'};                 % 18-20  accel after unit delay

%  dt_since (position in the 30 Hz camera cycle) was tried as feature 21 and
%  REJECTED. It removed the ~1 px sawtooth as intended and gave the best
%  validation loss of the project (0.00777), but the closed loop got worse:
%  21/40 -> 12/40 converged, median final distance 4 m -> 22 m. See the note
%  in py/common.py for the full record.

%% ── Signal names as logged by the Simulink model ───────────
%  These are a MAP, not a rename. The model already calls them these things
%  and the DKF scripts read the same names — do not change them.
cfg.sig.u_raw      = 'cam_u';        % camera, exact (no noise)  <- TARGET
cfg.sig.v_raw      = 'cam_v';
cfg.sig.area_raw   = 'cam_area';
cfg.sig.in_view    = 'inview_log';
cfg.sig.u_del      = 'u_del';        % after the 160 ms delay    <- INPUT
cfg.sig.v_del      = 'v_del';
cfg.sig.area_del   = 'area_del';
cfg.sig.valid_flag = 'valid_log';
cfg.sig.euler      = 'euler_log';
cfg.sig.omega      = 'log_omega';
cfg.sig.drone_pos  = 'position';
cfg.sig.target_pos = 'target_pos';

%  ACCELERATION — read this before changing anything.
%  The network is fed accel(k-1), not accel(k), because accel(k) depends on
%  the command at step k, which depends on the network output at step k — an
%  algebraic loop Simulink cannot solve.
%
%  That one step comes from a real block in the model: A_D_UnitDelay, fed
%  from Quad_Dynamics port 3. Its OUTPUT is logged as a_d_log, and the same
%  output feeds LSTM_Block port 7 at deployment.
%
%  One block, one signal path, identical during training and deployment.
%  Python therefore applies NO shift of its own — doing so would double it.
%
%  accel_log (the raw, undelayed tap) is still logged, but is not a feature.
cfg.sig.a_d       = 'a_d_log';
cfg.sig.a_d_raw   = 'accel_log';
cfg.a_d_lag_steps = 0;   % already applied in the model

%% ── Model variables the .slx reads from the workspace ──────
cfg.var_scenario = 'idx';   % Constant2 -> Target_Trajectory picks the motion
cfg.desired_distance = 2.0; % Constant  -> Distance, the stand-off setpoint (m)

%% ── The model's own nominal start (used as the sanity check) ─
cfg.ic.nominal.pos   = [15; 5; 7];
cfg.ic.nominal.vel   = [0; 0; 0];
cfg.ic.nominal.euler = [0; 0; 2];
cfg.ic.nominal.omega = [0; 0; 0];

%% ── Scenarios ──────────────────────────────────────────────
%  MEASURED, not assumed: 1, 4 and 8 fly. 3 fails (settles 41 m out) and
%  6 fails badly (target in view only 26% of the run). Do not use 3 or 6.
%  MEASURED on this model, nominal start, 30 s:
%     idx=2  delayed: settles 2.04 m but takes 24.1 s, dips to 0.87 m
%     idx=4  delayed: settles 2.03 m in 9.8 s, barely disturbed
%     idx=8  delayed: ENDS 34.2 m AWAY — target lost
%     idx=8  no delay: 9.2 m -> 2.0 m in ~8 s, no overshoot, rock steady
%
%  So scenario 8 is the headline: the 160 ms delay does not merely degrade
%  tracking there, it loses the target outright. That is the gap the LSTM
%  has to close, and it is enormous.
cfg.scenario_train = 8;
cfg.scenario_ood   = 2;    % held back to test generalisation

%% ── Initial conditions: PURSUIT geometry ───────────────────
%  The single most expensive lesson of the last attempt. Sampling yaw
%  independently of where the target is means runs open with the target out
%  of frame; the drone spins blind and acquisition becomes a coin flip.
%  Here yaw is aimed AT the target and then perturbed by a bounded error,
%  so every run starts with the target visible.
cfg.ic.mode         = 'pursuit';
cfg.ic.pursuit_dist = [3.0, 6.0];       % metres, uniform
cfg.ic.pursuit_elev = deg2rad(15);      % drone sits above the target
cfg.ic.yaw_err      = deg2rad(15);      % how far off perfect aim
cfg.ic.rp_jitter    = deg2rad(0);       % roll/pitch start level
cfg.ic.min_z        = 0.5;
cfg.ic.vel_mode     = 'match';          % start moving with the target
cfg.ic.vel_jitter   = 0.5;              % m/s around that
cfg.ic.seed         = 42;

%% ── Dataset sizes ──────────────────────────────────────────
cfg.n_train = 160;
cfg.n_val   = 40;
cfg.n_testA = 40;
cfg.n_testB = 40;
cfg.n_testC = 30;   % out-of-distribution (scenario 1)

end
