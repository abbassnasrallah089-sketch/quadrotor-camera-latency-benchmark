function [u_est, v_est, area_est, dist_est, health] = luen_block( ...
            a_d, euler, u_del, v_del, area_del, valid, in_view)  %#codegen
%% LUEN_BLOCK — Paste this into a Simulink MATLAB Function block.
%
%  Drop-in swap for dkf_block: SAME 7 inputs, SAME 5 outputs.
%
%  ── BLOCK PORTS ───────────────────────────────────────────────────
%   IN  1  a_d       [3x1]  accel_earth  <- Quad_Dynamics output 3
%   IN  2  euler     [3x1]  euler        <- Euler_Integrator
%   IN  3  u_del     [1x1]  delay block out 1
%   IN  4  v_del     [1x1]  delay block out 2
%   IN  5  area_del  [1x1]  delay block out 3
%   IN  6  valid     [1x1]  delay block out 4 (valid_flag)
%   IN  7  in_view   [1x1]  camera_model out 4  (NOT delayed)
%
%   OUT 1  u_est     -> P_yaw  (and Distance u input)
%   OUT 2  v_est     -> P_alt
%   OUT 3  area_est  -> Distance area input
%   OUT 4  dist_est  logging
%   OUT 5  health    1 = tracking, 0 = fall back to raw delayed
%
%  Sample time: 0.005. Edit Data: a_d and euler size [3 1].
%  Path: addpath the luen folder (luen_params, luen_init, luen_core).
%
%  Between runs:  clear luen_block

persistent S cfg ready

if isempty(ready)
    cfg   = coder.const(luen_params());
    S     = luen_init(cfg);
    ready = 1;
end

[S, u_est, v_est, area_est, dist_est, health] = luen_core( ...
        S, a_d, euler, u_del, v_del, area_del, valid, in_view, cfg);

end
