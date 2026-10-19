function cfg = dkf_params()
%% DKF_PARAMS — All DKF constants and tuning knobs in ONE place.
%
%  Everything the Delayed Kalman Filter needs lives here. The offline
%  tuner (dkf_tune.m) and the Simulink block (dkf_block.m) both read
%  this same struct, so a gain you tune offline is EXACTLY the gain
%  that runs in Simulink. Never hard-code a gain anywhere else.
%
%  Reference: Yang, Bai, She, Quan, "High-Speed Interception Multicopter
%  Control by Image-based Visual Servoing", arXiv:2404.08296v1, Sec. IV.
%
%  ── WHICH NUMBERS TO TUNE ─────────────────────────────────────
%    cfg.sigma_a   → process noise. THE main knob. Raise if the filter
%                    lags a manoeuvring target; lower if it is jittery.
%    cfg.sigma_uv  → pixel measurement noise (px).
%    cfg.sigma_s   → apparent-radius measurement noise (px). Controls
%                    how much the filter trusts the area→range channel.
%  ──────────────────────────────────────────────────────────────

%% ══ CODEGEN RULE, DO NOT BREAK IT ════════════════════════════
%  Every derived value is computed into a LOCAL variable first, and cfg is
%  only ever written to, never read from, inside this function.
%
%  Simulink's code generator forbids adding a field to a struct after that
%  struct has been read. The natural way to write this file --
%      cfg.DT = 0.005;
%      cfg.D  = round(cfg.DELAY_S / cfg.DT);   % <- READS cfg
%      cfg.img_width = 3280;                   % <- then adds a field
%  runs fine in MATLAB and fails at codegen with
%      "Structure field 'D' does not exist. Code generation does not
%       support the addition of new fields after a structure has been
%       read or used."
%  which is reported against dkf_params but actually blocks the whole
%  MATLAB Function block from compiling, taking the port dimensions with
%  it. Keep the two phases below separate and this cannot come back.

% ── phase 1: locals, all arithmetic happens here ─────────────
DT       = 0.005;                    % control / filter step, s (200 Hz)
DELAY_S  = 0.160;                    % camera imaging + processing delay, s
D        = round(DELAY_S / DT);      % = 32 steps
NBUF     = D + 1;                    % slot 1 = k-D, slot NBUF = k

img_w    = 3280;
img_h    = 2464;
fov_h    = 160;                      % deg
fx       = (img_w/2) / tand(fov_h/2);   % = 289.1936 px

% ── phase 2: assemble cfg, writes only ───────────────────────
%% ── Timing (MUST match your Simulink model) ──────────────────
cfg.DT      = DT;
cfg.DELAY_S = DELAY_S;
cfg.D       = D;
cfg.NBUF    = NBUF;

%% ── Camera intrinsics (MUST match camera_model.m EXACTLY) ────
cfg.img_width  = img_w;
cfg.img_height = img_h;
cfg.fov_h      = fov_h;
cfg.fx         = fx;
cfg.fy         = fx;
cfg.cx         = img_w / 2;                             % = 1640
cfg.cy         = img_h / 2;                             % = 1232
cfg.r_t        = 0.3;                                   % target radius, m
cfg.min_area   = 16;                                    % px^2 detection floor

%% ── Motion model ────────────────────────────────────────────
%  model_id = 1 -> 'CV' : x = [p_r; v_r]        (6 states)  <-- default
%         Target acceleration is treated as an unknown disturbance and
%         absorbed by Q. This is exactly what the paper does (Sec. II-A3:
%         "e_a_t is considered as a disturbance"). It corrects ANY target
%         motion as long as sigma_a covers the target's acceleration.
%  model_id = 2 -> 'CA' : x = [p_r; v_r; a_t]   (9 states)
%         Explicitly estimates target acceleration. Better for sustained
%         manoeuvres, slightly noisier. Try it if CV shows a steady lag.
%
%  NOTE: kept numeric (not a string) so the struct stays codegen-safe
%        when it is passed into the Simulink MATLAB Function block.
cfg.model_id = 1;

%% ── Process noise ───────────────────────────────────────────
%  sigma_a = std-dev of the UNMODELLED target acceleration, m/s^2.
%  Per-axis [x y z] so you can allow a fast-climbing target more slack.
%    ~0.5  : target moves at constant velocity (your scenarios 1-8)
%    ~3-5  : target manoeuvres gently
%    ~10+  : aggressive / evasive target
%  TUNED on the real Simulink logs (scenarios 3, 6, 8, 2), 80 gain sets x
%  4 logs, ranked by rel-pos RMSE + px/50 + 0.02*|lag| -- see dkf_tune.m.
%  Result: residual lag 0, 0, -1, 0 steps against a raw delay of 34, 31,
%  54, 36. The 160 ms is fully recovered.
%
%  READ THIS BEFORE QUOTING THESE NUMBERS AS GENERAL:
%  The winning set sits on three edges of the search grid - sigma_a at the
%  top, sigma_uv and sigma_s at the bottom. That is not a failed search,
%  it is the correct answer to the problem as posed, because THIS CAMERA
%  MODEL ADDS NO MEASUREMENT NOISE. Second differences of cam_u are ~3e-6
%  of its range: the image coordinates are exact geometry, merely delayed
%  and held at 30 Hz. Against a perfect measurement the optimal filter is
%  "believe the measurement, use the model only to bridge the delay",
%  which drives sigma_uv, sigma_s down and sigma_a up until the grid stops
%  it. Do not extend the grid chasing lower sigma_uv; the limit is a
%  degenerate gain, not a better filter.
%
%  Consequence: these gains are optimal FOR THIS SIMULATION. On real
%  vision, where blob centroid and area are genuinely noisy, sigma_uv and
%  sigma_s must be raised to the measured detector noise or the filter
%  will chase every bad detection. A defensible starting point for
%  hardware is sigma_a = [2 2 4], sigma_uv = 1.5, sigma_s = 0.25, which
%  cost only slightly more here (lag 1, 0, -23, -3) and tolerates noise.
cfg.sigma_a = [5.0, 5.0, 10.0];     % (CV mode) m/s^2
cfg.sigma_j = [2.0, 2.0, 4.0];      % (CA mode) target JERK std, m/s^3

%% ── Measurement noise ───────────────────────────────────────
%  Tuned; see the caveat above - the simulated camera is noise-free, so
%  these are lower than any real detector would justify.
cfg.sigma_uv = 0.5;      % horizontal/vertical pixel noise, px
cfg.sigma_s  = 0.25;     % apparent-radius noise, px  (radius = sqrt(area/pi))

%  Use the area→range channel? Your camera gives target area, which the
%  paper's camera did NOT. true = 3-D measurement [u; v; radius] (range is
%  directly observable, fast convergence). false = bearing-only [u; v],
%  exactly the paper's Eq. (32)-(33) formulation.
cfg.use_area = true;

%% ── Initial covariance ──────────────────────────────────────
cfg.P0_pos = 5.0^2;      % m^2      initial relative-position variance
cfg.P0_vel = 5.0^2;      % (m/s)^2  initial relative-velocity variance
cfg.P0_acc = 2.0^2;      % (m/s^2)^2 initial target-accel variance (CA only)

%% ── Robustness ──────────────────────────────────────────────
cfg.gate_chi2     = 5000.0; % innovation gate (chi^2, 3 dof). 0 = disabled.
                            % MEASURED, not chosen from theory. Swept over
                            % 60/200/500/1000/5000/off on scenarios 3,6,8,2
                            % (dkf_gatesweep.m); total gate lockouts across the
                            % four runs went 14 / 8 / 8 / 5 / 2 / 1. 5000 is the
                            % tightest setting that still avoids lockouts, so it
                            % keeps some protection against absurd blobs for
                            % real vision while not rejecting good frames.
                            %
                            % Do NOT "tighten to ~16 once trusted" as the
                            % original note suggested. chi^2(3) at 99.9% IS 16.3,
                            % yet 60 -- four times that -- still rejected good
                            % frames until the filter locked itself out and
                            % re-acquired 14 times. The innovations are simply
                            % not chi^2 distributed here: the CV model understates
                            % its own uncertainty when the target crosses frame,
                            % so P is too small and d2 = y'S^-1 y comes out huge
                            % for a perfectly good measurement. Tightening this
                            % re-creates the lockouts; the honest fix is a motion
                            % model whose covariance means what it claims.
cfg.max_coast     = 400;    % steps (=2.0 s) the filter may dead-reckon with no
                            % valid image before it declares itself unhealthy.
cfg.min_depth     = 0.15;   % m. Guard against divide-by-zero in projection.
cfg.reinit_on_lost = true;  % re-initialise from scratch after max_coast

end
