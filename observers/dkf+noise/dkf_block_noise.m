function [u_est, v_est, area_est, dist_est, health] = dkf_block_noise( ...
            a_d, euler, u_del, v_del, area_del, valid, in_view)  %#codegen
%% DKF_BLOCK_NOISE — paste into the MATLAB Function block of
%  QuadVisionControl_noise_dkf ONLY.
%
%  IDENTICAL to dkf_block except for ONE line: it calls dkf_params_noise()
%  instead of dkf_params(). dkf_core / dkf_init are shared and unchanged.
%
%      dkf_params()        sigma_uv 0.5, sigma_a [5 5 10]  <- noise-free (out3)
%      dkf_params_noise()  sigma_uv 1.5, sigma_a [2 2 4]   <- noisy (out6)
%
%  WHY THIS FILE HAS TO EXIST: pasting plain dkf_block into the noise model
%  would load R tuned for a NOISE-FREE camera - sigma_uv = 0.5 against a
%  real 1.5 px detector makes R nine times too small in variance. The filter
%  would then believe every bad detection and chase it. It would lose the
%  comparison for a reason that has nothing to do with the DKF, with no
%  error and no warning. Same trap as luen_block_noise.m.
%
%  ── BLOCK PORTS (unchanged from dkf_block) ────────────────────────
%   IN  1  a_d       [3x1]  accel_earth <- Quad_Dynamics out 3, VIA UNIT DELAY
%   IN  2  euler     [3x1]  euler       <- Euler_Integrator (NOT delayed)
%   IN  3  u_del     [1x1]  <- camera_delay_block_noise_160ms out 1
%   IN  4  v_del     [1x1]  <- camera_delay_block_noise_160ms out 2
%   IN  5  area_del  [1x1]  <- camera_delay_block_noise_160ms out 3
%   IN  6  valid     [1x1]  <- camera_delay_block_noise_160ms out 4
%   IN  7  in_view   [1x1]  <- camera_model out 4 (NOT delayed)
%
%   OUT 1  u_est     -> P_yaw  AND Distance u input
%   OUT 2  v_est     -> P_alt
%   OUT 3  area_est  -> Distance area input
%   OUT 4  dist_est  logging
%   OUT 5  health    1 = converged, 0 = coasting on raw measurement
%
%  Sample time: 0.005. Edit Data: a_d and euler size [3 1].
%  Path: BOTH fyp\dkf (for dkf_core, dkf_init) and fyp\dkf_noise.
%
%  Model InitFcn must contain:
%      clear dkf_block_noise
%      clear camera_delay_block_noise_160ms

persistent S cfg ready

if isempty(ready)
    % coder.const is NOT optional - see the long note in dkf_block.m. Without
    % it the persistent cfg makes model_id/NBUF runtime values, dkf_init
    % allocates a variable-sized state buffer, and dkf_predict fails with
    % "Size mismatch (size [6 x 6] ~= size [9 x 9])" reported in dkf_core.
    cfg   = coder.const(dkf_params_noise());   % <-- the only difference
    S     = dkf_init(cfg);
    ready = 1;
end

[S, u_est, v_est, area_est, dist_est, health] = dkf_core( ...
        S, a_d, euler, u_del, v_del, area_del, valid, in_view, cfg);

end
