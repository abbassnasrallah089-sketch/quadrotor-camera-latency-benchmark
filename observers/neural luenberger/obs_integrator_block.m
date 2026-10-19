function [u_hat, v_hat, area_hat] = obs_integrator_block(du, dv, dr, ...
                                        u_del, v_del, area_del, valid_flag)
%#codegen
%OBS_INTEGRATOR_BLOCK  Block 2: integrate the velocity, anchor it to the
%  measurement (Luenberger), output the estimate of the target NOW.
%
%      innov(k)  = y_del(k) - xhat(k-32)     latched at frame time
%      xhat(k+1) = xhat(k) + dt*[du;dv;dr] + (L/7)*innov
%
%  THE THREE DECISIONS, all measured in test_rollout.py:
%   1. innovation vs xhat 160 ms AGO, never vs now (wrong way: -52 px bias)
%   2. innovation EVALUATED only when a new frame lands, held between
%      (evaluating on held steps: -4.5 px bias at 306 px/s, grows w/ speed)
%   3. correction SPREAD as L/7 every step (no 30 Hz impulse for P_alt's
%      derivative to amplify)
%
%  ZERO-VELOCITY BEHAVIOUR: the correction alone pulls xhat onto the
%  delayed measurement -> flight == raw delayed baseline. Plumbing test.
%
%  STABILITY, derived + confirmed: keep L below 0.30. Start at 0.10.
%  L is part of the ARCHITECTURE: train and fly with the same value.
%
%  BLOCK SETTINGS: sample time 0.005 (NOT -1). 7 in, 3 out.
%  Reset between runs: clear obs_integrator_block

BYPASS   = 0;        % 1 = pass the delayed measurement straight through (wiring proof). 0 = observer on.
L_UV = 0.1000;
L_R  = 0.1000;
SPREAD   = 1;        % 1 = deal correction across the camera period
MAX_LEAD = 600.0;    % px; drift leash when target is out of frame
R_MIN_PX = 1.0;      % px; distance ~ 1/radius, radius 0 = fly to infinity

CX = 1640.0;  CY = 1232.0;  FOCAL = 289.19;
DT = 0.005;   CAM_STEP = 7;  D_STEPS = 32;
NB = D_STEPS + 1;

persistent xhat x_buf xi innov_l primed
if isempty(primed)
    xhat = zeros(3,1); x_buf = zeros(NB,3); xi = 0;
    innov_l = zeros(3,1); primed = false;
end

if BYPASS > 0.5
    u_hat = u_del; v_hat = v_del; area_hat = area_del;
    return;
end

ut = (u_del - CX) / FOCAL;
vt = (v_del - CY) / FOCAL;
if area_del > 0, rt = sqrt(area_del/pi) / FOCAL; else, rt = 0; end

if ~primed
    xhat(1) = ut; xhat(2) = vt; xhat(3) = rt;
    for kk = 1:NB
        x_buf(kk,1) = ut; x_buf(kk,2) = vt; x_buf(kk,3) = rt;
    end
    primed = true;
end

%% xhat one delay window ago, then store the current one
xi = xi + 1; if xi > NB, xi = 1; end
old_x = xi + 1; if old_x > NB, old_x = 1; end
xt1 = x_buf(old_x,1); xt2 = x_buf(old_x,2); xt3 = x_buf(old_x,3);
x_buf(xi,1) = xhat(1); x_buf(xi,2) = xhat(2); x_buf(xi,3) = xhat(3);

%% OUTPUT the estimate on ENTRY to this step — order matches the Python
%  rollout exactly; emitting post-update would shift everything one sample
r_px = xhat(3) * FOCAL;
if r_px < R_MIN_PX, r_px = R_MIN_PX; end
u_hat    = xhat(1) * FOCAL + CX;
v_hat    = xhat(2) * FOCAL + CY;
area_hat = pi * r_px * r_px;

%% innovation, latched at frame time
if valid_flag > 0.5
    innov_l(1) = ut - xt1;
    innov_l(2) = vt - xt2;
    innov_l(3) = rt - xt3;
end
if SPREAD > 0.5
    c1 = (L_UV/CAM_STEP)*innov_l(1);
    c2 = (L_UV/CAM_STEP)*innov_l(2);
    c3 = (L_R /CAM_STEP)*innov_l(3);
elseif valid_flag > 0.5
    c1 = L_UV*innov_l(1); c2 = L_UV*innov_l(2); c3 = L_R*innov_l(3);
else
    c1 = 0; c2 = 0; c3 = 0;
end

%% integrate
xhat(1) = xhat(1) + DT*du + c1;
xhat(2) = xhat(2) + DT*dv + c2;
xhat(3) = xhat(3) + DT*dr + c3;

%% guards
lead = MAX_LEAD / FOCAL;
if xhat(1)-ut >  lead, xhat(1) = ut+lead; end
if xhat(1)-ut < -lead, xhat(1) = ut-lead; end
if xhat(2)-vt >  lead, xhat(2) = vt+lead; end
if xhat(2)-vt < -lead, xhat(2) = vt-lead; end
if xhat(3)-rt >  lead, xhat(3) = rt+lead; end
if xhat(3)-rt < -lead, xhat(3) = rt-lead; end
rmin = R_MIN_PX / FOCAL;
if xhat(3) < rmin, xhat(3) = rmin; end
end
