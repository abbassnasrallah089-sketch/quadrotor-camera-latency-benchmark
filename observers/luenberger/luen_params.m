function cfg = luen_params()
%% LUEN_PARAMS — All Luenberger-observer constants in ONE place.
%
%  Third delay-compensation method, next to DKF and LSTM.
%  Same I/O as dkf_block: delayed u,v,area in → current-time u,v,area out.
%
%  WHAT IS DIFFERENT FROM THE DKF
%    DKF:   adaptive Kalman gain K from covariance P, Q, R
%    Luen.: FIXED gain L from pole placement (alpha-beta / 2nd-order)
%
%  Same delay handling as the DKF (and the paper): rewind to time k-D,
%  correct there, replay stored drone accelerations back to now.
%
%  TUNE THESE TWO:
%    cfg.omega_n  observer bandwidth (rad/s). Raise if it lags the target.
%    cfg.zeta     damping. 0.7–1.0. Lower → snappier, more overshoot.

%% ══ CODEGEN RULE (same as dkf_params.m) ══════════════════════
%  All arithmetic in locals first. cfg is write-only in this function.

% ── phase 1: locals ──────────────────────────────────────────
DT       = 0.005;                    % 200 Hz control / observer step
DELAY_S  = 0.160;                    % must match the delay block
D        = round(DELAY_S / DT);      % 32 steps
NBUF     = D + 1;                    % 33 slots
CAM_HZ   = 30;
TS_MEAS  = 1 / CAM_HZ;               % ~0.033 s  (correction only at 30 Hz)

img_w    = 3280;
img_h    = 2464;
fov_h    = 160;
fx       = (img_w/2) / tand(fov_h/2);

% Pole-placement (per axis, constant-velocity double integrator).
% Discrete alpha-beta gains applied ONCE per new camera frame:
%   r     = exp(-zeta * omega_n * Ts_meas)
%   alpha = 1 - r^2          → position blend
%   beta  = (1 - r)^2        → velocity blend
%   Lp = alpha * I
%   Lv = (beta / Ts_meas) * I
omega_n  = 8.0;                      % rad/s  (~ 0.4 s rise). Raise if lag remains.
zeta     = 1.00;                     % 1 = critically damped (less u/v overshoot)
r        = exp(-zeta * omega_n * TS_MEAS);
alpha    = 1 - r*r;
beta     = (1 - r)^2;
Lp_s     = alpha;
Lv_s     = beta / TS_MEAS;
% 2026-08-16: was 0.35. That value existed only to mask the R_EB(3,2) typo
% in luen_core.m (spsi*sphi instead of cpsi*sphi), fixed the same day. With
% the rotation correct, u RMSE is 0.0-1.2 px on scenarios 2/5/8, so the
% anisotropy has nothing left to suppress. 1.0 = isotropic Lv.
% Revisit under camera noise, where a lower Lv may genuinely help.
Lv_xy    = 1.00;                     % scale Lv on Earth X,Y only

% ── phase 2: assemble cfg, writes only ───────────────────────
cfg.DT      = DT;
cfg.DELAY_S = DELAY_S;
cfg.D       = D;
cfg.NBUF    = NBUF;
cfg.TS_MEAS = TS_MEAS;

cfg.img_width  = img_w;
cfg.img_height = img_h;
cfg.fov_h      = fov_h;
cfg.fx         = fx;
cfg.fy         = fx;
cfg.cx         = img_w / 2;
cfg.cy         = img_h / 2;
cfg.r_t        = 0.3;
cfg.min_area   = 16;
cfg.min_depth  = 0.15;

cfg.omega_n = omega_n;
cfg.zeta    = zeta;
cfg.Lp      = Lp_s;                  % position blend, all axes
cfg.Lv      = Lv_s;                  % velocity gain on Z (range)
cfg.Lv_xy   = Lv_xy;                 % multiply Lv on X,Y (bearing / u)

cfg.max_coast      = 400;            % 2.0 s of no valid frame
cfg.reinit_on_lost = true;

%  NOISE-FREE BASELINE. Gains omega_n = 8, zeta = 1 belong to this version
%  and reproduce pos/u/range RMSE = 0.000 on scenarios 2/5/8.
%  The detector-noise campaign is a SEPARATE copy in ..\luen_noise\
%  (luen_params_noise.m, omega_n = 4). Do not add noise fields to this file.

end
