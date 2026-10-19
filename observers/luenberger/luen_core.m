function [S, u_est, v_est, area_est, dist_est, health] = luen_core( ...
            S, a_d, euler, u_del, v_del, area_del, valid, in_view, cfg)  %#codegen
%% LUEN_CORE — Delayed Luenberger observer, one 200 Hz step.
%
%  Same delay architecture as dkf_core (Yang et al. Alg. 2):
%    1. PREDICT  every 5 ms using the drone's own Earth acceleration
%    2. CORRECT  when a delayed frame lands, at the slot for time k-D
%    3. REPLAY   the corrected state forward through stored a_d samples
%
%  Difference from DKF: the correction is a FIXED gain L, not Kalman K.
%  Measurement is inverted to Earth-frame relative position (area → range,
%  pixels → bearing), then a linear Luenberger update on the CV plant:
%
%      p_r^+ = p_r + Lp * (p_meas - p_r)
%      v_r^+ = v_r + Lv * (p_meas - p_r)
%
%  That is an alpha-beta / 2nd-order observer. Lp, Lv come from pole
%  placement in luen_params.m (omega_n, zeta).
%
%  INPUTS / OUTPUTS: identical to dkf_core, so the Simulink block is a
%  drop-in swap for DKF.

NB = cfg.NBUF;

%% ═══ 1. SHIFT THE WINDOW ══════════════════════════════════════════
S.xb(:, 1:NB-1)    = S.xb(:, 2:NB);
S.ab(:, 1:NB-1)    = S.ab(:, 2:NB);
S.Rb(:, :, 1:NB-1) = S.Rb(:, :, 2:NB);
S.vb(1, 1:NB-1)    = S.vb(1, 2:NB);

%% ═══ 2. PREDICT k-1 -> k ══════════════════════════════════════════
xn = luen_predict(S.xb(:,NB-1), S.ab(:,NB-1), cfg);
S.xb(:, NB) = xn;

Reb_now        = luen_Reb(euler);
S.ab(:, NB)    = a_d(:);
S.Rb(:, :, NB) = Reb_now;
S.vb(1, NB)    = in_view;
S.n            = min(S.n + 1, 2*NB);

%% ═══ 3. CORRECT AT SLOT 1 (time k-D) ══════════════════════════════
meas_ok = (valid > 0.5) && (S.n >= NB) && (S.vb(1,1) > 0.5) && ...
          (area_del >= cfg.min_area) && isfinite(u_del) && isfinite(v_del);

if meas_ok
    s_meas = sqrt(area_del / pi);
    z      = [u_del; v_del; s_meas];
    p_meas = luen_invert(z, S.Rb(:,:,1), cfg);

    if S.init < 0.5
        S.xb(1:3, 1) = p_meas;
        S.xb(4:6, 1) = 0;
        S.init  = 1;
        S.coast = 0;
        do_replay = true;
    else
        y = p_meas - S.xb(1:3, 1);           % position residual (Earth)
        S.xb(1:3, 1) = S.xb(1:3, 1) + cfg.Lp * y;
        % Weaker velocity correction sideways: a large Lv on X/Y caused the
        % u-pixel spike in self-test scen 2 and 8. Keep full Lv on Z (range).
        S.xb(4, 1) = S.xb(4, 1) + (cfg.Lv * cfg.Lv_xy) * y(1);
        S.xb(5, 1) = S.xb(5, 1) + (cfg.Lv * cfg.Lv_xy) * y(2);
        S.xb(6, 1) = S.xb(6, 1) +  cfg.Lv              * y(3);
        S.coast = 0;
        do_replay = true;
    end

    %% ═══ 4. REPLAY k-D -> k ═══════════════════════════════════════
    if do_replay
        for i = 1:NB-1
            S.xb(:, i+1) = luen_predict(S.xb(:,i), S.ab(:,i), cfg);
        end
    end
else
    S.coast = S.coast + 1;
end

%% ═══ 5. HEALTH ════════════════════════════════════════════════════
health = 1;
if S.init < 0.5
    health = 0;
elseif S.coast > cfg.max_coast
    health = 0;
    if cfg.reinit_on_lost > 0.5
        S.init  = 0;
        S.coast = 0;
    end
end

%% ═══ 6. PROJECT TO NOW ════════════════════════════════════════════
[u_est, v_est, area_est, dist_est] = luen_project(S.xb(:,NB), Reb_now, cfg);

if health < 0.5
    u_est    = u_del;
    v_est    = v_del;
    area_est = area_del;
    if area_del > 1e-9
        dist_est = cfg.fx * cfg.r_t / sqrt(area_del / pi);
    else
        dist_est = 100;
    end
end

end


%% ─────────────────────────────────────────────────────────────────
function x = luen_predict(x, a_d, cfg)
%  Constant-velocity relative kinematics. Drone accel is a KNOWN input
%  (same F, B as dkf_predict CV branch). Target accel is a disturbance.
dt = cfg.DT;
I3 = eye(3);
F  = [I3, dt*I3; zeros(3), I3];
B  = [-0.5*dt*dt*I3; -dt*I3];
x  = F*x + B*a_d(:);
end


%% ─────────────────────────────────────────────────────────────────
function p_e = luen_invert(z, Reb, cfg)
%  Invert the pinhole + apparent-size model → Earth-frame relative pos.
s  = max(z(3), 1e-6);
xb = cfg.fx * cfg.r_t / s;
yb = -(z(1) - cfg.cx) * xb / cfg.fx;
zb = -(z(2) - cfg.cy) * xb / cfg.fy;
xb = max(xb, cfg.min_depth);
p_e = Reb.' * [xb; yb; zb];
end


%% ─────────────────────────────────────────────────────────────────
function [u, v, area, dist] = luen_project(x, Reb, cfg)
p_b  = Reb * x(1:3);
dist = max(p_b(1), cfg.min_depth);
u = cfg.fx * (-p_b(2)/dist) + cfg.cx;
v = cfg.fy * (-p_b(3)/dist) + cfg.cy;
u = max(0, min(cfg.img_width,  u));
v = max(0, min(cfg.img_height, v));
r_px = cfg.fx * cfg.r_t / dist;
area = pi * r_px * r_px;
end


%% ─────────────────────────────────────────────────────────────────
function Reb = luen_Reb(euler)
phi = euler(1);  theta = euler(2);  psi = euler(3);
cphi = cos(phi); sphi = sin(phi);
cth  = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);
Reb = [ cpsi*cth,                    spsi*cth,                   -sth    ; ...
        cpsi*sth*sphi - spsi*cphi,   spsi*sth*sphi + cpsi*cphi,   cth*sphi; ...
        cpsi*sth*cphi + spsi*sphi,   spsi*sth*cphi - cpsi*sphi,   cth*cphi];
end
