function [u_est, v_est, area_est, dist_est, health] = dkf_block( ...
            a_d, euler, u_del, v_del, area_del, valid, in_view)  %#codegen
%% DKF_BLOCK — Paste this into a Simulink MATLAB Function block.
%
%  This is the ONLY file that goes into Simulink. It holds the filter
%  state in persistents and delegates every line of maths to dkf_core.m,
%  the same function the offline tuner runs. Tune offline, drop in here,
%  behaviour is identical.
%
%  ── BLOCK PORTS ───────────────────────────────────────────────────
%   IN  1  a_d       [3x1]  accel_earth  <- Quad_Dynamics output 3
%   IN  2  euler     [3x1]  euler        <- Euler_Integrator output
%   IN  3  u_del     [1x1]  <- camera_delay_block_with_area_160ms out 1
%   IN  4  v_del     [1x1]  <- camera_delay_block_with_area_160ms out 2
%   IN  5  area_del  [1x1]  <- camera_delay_block_with_area_160ms out 3
%   IN  6  valid     [1x1]  <- camera_delay_block_with_area_160ms out 4
%   IN  7  in_view   [1x1]  <- camera_model out 4  (NOT delayed)
%
%   OUT 1  u_est     -> P_yaw controller   (replaces u_del)
%   OUT 2  v_est     -> P_alt              (replaces v_del)
%   OUT 3  area_est  -> Distance           (replaces area_del)
%   OUT 4  dist_est  estimated range, m    (logging / optional direct use)
%   OUT 5  health    1 = filter converged, 0 = coasting on raw measurement
%
%  ── BLOCK SETTINGS ────────────────────────────────────────────────
%   Sample time MUST be 0.005 (200 Hz), matching cfg.DT and the rest of
%   the control loop. Leave it at -1 only if the whole model is fixed-step
%   at 0.005. In the block: Edit Data -> set a_d and euler to size [3 1].
%
%  IMPORTANT: dkf_params.m, dkf_init.m and dkf_core.m must be on the
%  MATLAB path (same folder as the model, or addpath the dkf folder).

persistent S cfg ready

if isempty(ready)
    % coder.const is what actually makes this work, and it is not optional.
    %
    % dkf_params() takes no arguments and returns literals, so its result
    % CAN be evaluated at compile time -- but the coder will not do so on
    % its own once the struct is stored in a persistent. It then treats
    % cfg.model_id and cfg.NBUF as runtime values, so dkf_init's
    %     NS = 6; if cfg.model_id == 2, NS = 9; end
    %     S.xb = zeros(NS, NB);
    % produces a VARIABLE-SIZED state buffer. Every size() taken from it
    % downstream is then non-constant too, so dkf_predict must compile both
    % the 6-state and 9-state branches and fails with
    %     "Size mismatch (size [6 x 6] ~= size [9 x 9])" at F.
    % The error is reported in dkf_core, but the cause is here.
    %
    % Forcing the fold makes model_id and NBUF literals, so dkf_init
    % allocates a fixed 6x33 buffer and the dead branch is deleted.
    % coder.const is an identity function outside code generation, so the
    % offline tuner and dkf_selftest are completely unaffected.
    cfg   = coder.const(dkf_params());
    S     = dkf_init(cfg);
    ready = 1;
end

[S, u_est, v_est, area_est, dist_est, health] = dkf_core( ...
        S, a_d, euler, u_del, v_del, area_del, valid, in_view, cfg);

end
