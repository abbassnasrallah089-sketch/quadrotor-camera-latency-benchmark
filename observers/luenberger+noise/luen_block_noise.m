function [u_est, v_est, area_est, dist_est, health] = luen_block_noise( ...
            a_d, euler, u_del, v_del, area_del, valid, in_view)  %#codegen
%% LUEN_BLOCK_NOISE — paste into the MATLAB Function block of
%  QuadVisionControl_noise_luen ONLY.
%
%  IDENTICAL to luen_block except for ONE line: it calls luen_params_noise()
%  instead of luen_params(). The observer code (luen_core) is shared and
%  unchanged — a fixed-gain Luenberger has no R, so nothing inside it depends
%  on the noise level. Only the tuned gain differs:
%
%      luen_params()        omega_n = 8   <- noise-free baseline (out4)
%      luen_params_noise()  omega_n = 4   <- tuned for 1.5 px / 0.25 px (out7)
%
%  WHY THIS FILE HAS TO EXIST: if you pasted plain luen_block into the noise
%  model it would load omega_n = 8 with no error and no warning, and the
%  Luenberger would be running gains tuned for a camera it no longer has.
%  That would understate its performance and quietly hand the comparison to
%  the DKF for the wrong reason.
%
%  ── BLOCK PORTS (unchanged from luen_block) ───────────────────────
%   IN  1  a_d       [3x1]  accel_earth <- Quad_Dynamics out 3, VIA UNIT DELAY
%   IN  2  euler     [3x1]  euler       <- Euler_Integrator (NOT delayed)
%   IN  3  u_del     [1x1]  noise delay block out 1
%   IN  4  v_del     [1x1]  noise delay block out 2
%   IN  5  area_del  [1x1]  noise delay block out 3
%   IN  6  valid     [1x1]  noise delay block out 4 (valid_flag)
%   IN  7  in_view   [1x1]  camera_model out 4 (NOT delayed)
%
%   OUT 1  u_est     -> P_yaw  AND Distance u input
%   OUT 2  v_est     -> P_alt
%   OUT 3  area_est  -> Distance area input
%   OUT 4  dist_est  logging
%   OUT 5  health    1 = tracking, 0 = fell back to raw delayed
%
%  Sample time: 0.005. Edit Data: a_d and euler size [3 1].
%  Path: BOTH fyp\luen (for luen_core, luen_init) and fyp\luen_noise.
%
%  Model InitFcn must contain:
%      clear luen_block_noise
%      clear camera_delay_block_noise_160ms

persistent S cfg ready

if isempty(ready)
    cfg   = coder.const(luen_params_noise());   % <-- the only difference
    S     = luen_init(cfg);
    ready = 1;
end

[S, u_est, v_est, area_est, dist_est, health] = luen_core( ...
        S, a_d, euler, u_del, v_del, area_del, valid, in_view, cfg);

end
