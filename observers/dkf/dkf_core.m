function [S, u_est, v_est, area_est, dist_est, health] = dkf_core( ...
            S, a_d, euler, u_del, v_del, area_del, valid, in_view, cfg)  %#codegen
%% DKF_CORE — Delayed Kalman Filter, one 200 Hz step.
%
%  THE SINGLE SOURCE OF TRUTH. Both the offline tuner (dkf_offline_run.m)
%  and the Simulink block (dkf_block.m) call THIS function, so a filter
%  you validate offline is bit-for-bit the filter that flies. Never fork it.
%
%  ── WHAT IT DOES ──────────────────────────────────────────────────
%  The camera hands the controllers an image that is 160 ms (D = 32 steps)
%  old. At 20 m/s that is 3.2 m of pure position lag, which no amount of
%  controller tuning can recover. This filter fixes it by:
%
%    1. PREDICT   every step at 200 Hz using the drone's OWN measured
%                 acceleration (known input), so an estimate of the
%                 target exists at the CURRENT instant.       [Eq. 30-31]
%    2. CORRECT   when a delayed frame arrives, rewind to the slot that
%                 the frame actually describes (time k-D) and fuse it
%                 there, where it is consistent.               [Eq. 34-36]
%    3. REPLAY    re-propagate that corrected state forward through the
%                 D stored acceleration samples back up to NOW.
%                                                       [Fig. 7 / Alg. 2]
%
%  The estimate at slot NBUF is then re-projected through the CURRENT
%  attitude to produce u/v/area exactly as camera_model.m would have
%  produced them with no delay -> drop-in replacement for u_del/v_del/
%  area_del, so P_yaw / P_alt / distance_controller_v2 need NO changes.
%
%  ── STATE (Earth frame, z-up; matches quad_dynamics.m) ────────────
%    p_r = target_pos - drone_pos          relative position   [m]
%    v_r = target_vel - drone_vel          relative velocity   [m/s]
%    a_t = target acceleration (CA model only)                 [m/s^2]
%
%  Attitude is NOT a state: your AHRS/integrator supplies euler exactly,
%  so the paper's quaternion + gyro/accel-bias states (its x in R^18)
%  collapse to 6. Nothing is lost - those states existed to estimate an
%  attitude you already know.
%
%  ── INPUTS (all at the CURRENT step k) ────────────────────────────
%    S         filter state struct from dkf_init()
%    a_d       [3x1] drone acceleration, EARTH frame  (= accel_earth)
%    euler     [3x1] drone attitude [phi; theta; psi] (rad)
%    u_del     [1x1] delayed horizontal pixel   from the delay block
%    v_del     [1x1] delayed vertical pixel     from the delay block
%    area_del  [1x1] delayed target area (px^2) from the delay block
%    valid     [1x1] 1 ONLY on the step a NEW delayed frame lands
%                    (= valid_flag output of camera_delay_block_*.m).
%                    Between frames the delay block holds its output, and
%                    re-fusing a held sample would double-count it.
%    in_view   [1x1] current camera in_view flag (NOT delayed - the filter
%                    buffers it internally and uses its k-D value)
%    cfg       struct from dkf_params()
%
%  ── OUTPUTS ───────────────────────────────────────────────────────
%    u_est, v_est, area_est   delay-compensated camera measurements
%    dist_est                 estimated range along the optical axis [m]
%    health                   1 = converged & fusing, 0 = coasting/unhealthy
%
%  Ref: Yang, Bai, She, Quan, "High-Speed Interception Multicopter Control
%       by Image-based Visual Servoing", arXiv:2404.08296v1, Section IV.

NS   = size(S.xb, 1);      % 6 (CV) or 9 (CA)
NB   = cfg.NBUF;           % D+1 slots; slot 1 = time k-D, slot NB = time k

%% ═══ 1. SHIFT THE WINDOW ══════════════════════════════════════════
%  Drop the oldest slot and open a fresh slot at the end for time k.
%  After this shift slot 1 holds time k-D -- exactly the instant that a
%  frame arriving NOW was actually exposed.
S.xb(:, 1:NB-1)    = S.xb(:, 2:NB);
S.Pb(:, :, 1:NB-1) = S.Pb(:, :, 2:NB);
S.ab(:, 1:NB-1)    = S.ab(:, 2:NB);
S.Rb(:, :, 1:NB-1) = S.Rb(:, :, 2:NB);
S.vb(1, 1:NB-1)    = S.vb(1, 2:NB);

%% ═══ 2. PREDICT k-1 -> k  (Eq. 30-31) ═════════════════════════════
[xn, Pn]        = dkf_predict(S.xb(:,NB-1), S.Pb(:,:,NB-1), S.ab(:,NB-1), cfg, NS);
S.xb(:, NB)     = xn;
S.Pb(:, :, NB)  = Pn;

% Log this step's exogenous inputs into the new slot.
Reb_now         = dkf_Reb(euler);       % Earth -> Body (== camera_model.m)
S.ab(:, NB)     = a_d(:);
S.Rb(:, :, NB)  = Reb_now;
S.vb(1, NB)     = in_view;
S.n             = min(S.n + 1, 2*NB);   % history counter (saturating)

%% ═══ 3. FUSE THE DELAYED FRAME AT SLOT 1  (Eq. 34-36) ═════════════
%  Gate hard. A measurement is usable ONLY if a new frame landed, the
%  window is full, the target was genuinely visible when the shutter
%  fired, and the blob was big enough to trust.
meas_ok = (valid > 0.5) && (S.n >= NB) && (S.vb(1,1) > 0.5) && ...
          (area_del >= cfg.min_area) && isfinite(u_del) && isfinite(v_del);

if meas_ok
    % Apparent radius in px. Far better conditioned than raw area:
    % radius ~ 1/d (noise roughly constant in px) whereas area ~ 1/d^2,
    % whose noise blows up as d^4 and wrecks the covariance at range.
    s_meas = sqrt(area_del / pi);
    z      = [u_del; v_del; s_meas];

    if S.init < 0.5
        % ── COLD START: invert the camera model to place the target ──
        % The first argument is a size template, not data: it lets the
        % callee get NS from size() of a fixed-size array, which codegen
        % treats as a compile-time constant. Passing the scalar NS instead
        % makes zeros(NS,1) variable-size and it can no longer be stored
        % back into the fixed-size buffer slot.
        S.xb(:,1)   = dkf_init_state(S.xb(:,1),   z, S.Rb(:,:,1), cfg);
        S.Pb(:,:,1) = dkf_P0(        S.Pb(:,:,1), cfg);
        S.init      = 1;
        S.coast     = 0;
        do_replay   = true;
    else
        [xc, Pc, accepted] = dkf_correct(S.xb(:,1), S.Pb(:,:,1), z, ...
                                          S.Rb(:,:,1), cfg, NS);
        S.xb(:,1)   = xc;
        S.Pb(:,:,1) = Pc;
        do_replay   = accepted;
        if accepted
            S.coast = 0;
        else
            S.coast = S.coast + 1;
        end
    end

    %% ═══ 4. REPLAY k-D -> k  (Alg. 2, lines 7-9) ══════════════════
    %  Push the corrected slot forward through every stored acceleration
    %  sample. This is what buys back the 160 ms.
    if do_replay
        for i = 1:NB-1
            [xr, Pr]        = dkf_predict(S.xb(:,i), S.Pb(:,:,i), S.ab(:,i), cfg, NS);
            S.xb(:, i+1)    = xr;
            S.Pb(:, :, i+1) = Pr;
        end
    end
else
    S.coast = S.coast + 1;
end

%% ═══ 5. HEALTH / RECOVERY ═════════════════════════════════════════
health = 1;
if S.init < 0.5
    health = 0;
elseif S.coast > cfg.max_coast
    health = 0;
    if cfg.reinit_on_lost > 0.5
        S.init  = 0;                       % drop the stale track, re-acquire
        S.coast = 0;
    end
end

%% ═══ 6. PROJECT TO NOW  ═══════════════════════════════════════════
%  Re-image the current-time estimate through the CURRENT attitude.
[u_est, v_est, area_est, dist_est] = dkf_project(S.xb(:,NB), Reb_now, cfg);

if health < 0.5
    % Not trustworthy: hand the raw delayed frame straight through so the
    % controllers degrade to today's behaviour rather than to garbage.
    u_est    = u_del;
    v_est    = v_del;
    area_est = area_del;
    if area_del > 1e-9
        dist_est = cfg.fx * cfg.r_t / sqrt(area_del / pi);
    else
        dist_est = 100;
    end
end

end % ===================== dkf_core =============================


%% ─────────────────────────────────────────────────────────────────
function [x, P] = dkf_predict(x, P, a_d, cfg, NS)  %#ok<INUSD>
%  One propagation step. The drone's own acceleration a_d is a KNOWN
%  input (paper Eq. 9); only the TARGET's acceleration is noise.
%
%  CODEGEN: NS is deliberately re-derived from size(x,1) and the passed
%  argument ignored. The two branches below build F as 6x6 or 9x9, and a
%  variable may only have one size under code generation. Taking NS from
%  the passed scalar leaves the condition unknown at compile time, so the
%  coder compiles BOTH branches and reports
%      "Size mismatch (size [6 x 6] ~= size [9 x 9])" at F.
%  size() of a fixed-size input is a compile-time constant, so deriving it
%  here folds the if and deletes the dead branch outright. The argument is
%  kept so the call sites and the offline path stay unchanged.
NS = size(x, 1);

dt = cfg.DT;
I3 = eye(3);

if NS == 6
    % ── Constant-velocity relative model ──
    F = [I3, dt*I3; zeros(3), I3];
    B = [-0.5*dt*dt*I3; -dt*I3];          % minus: v_r = v_t - v_d
    x = F*x + B*a_d(:);

    Sg = diag(cfg.sigma_a(:).^2);         % target-accel PSD per axis
    G  = [0.5*dt*dt*I3; dt*I3];
    Q  = G * Sg * G.';
else
    % ── Constant-acceleration relative model ──
    F = [I3, dt*I3, 0.5*dt*dt*I3; ...
         zeros(3), I3, dt*I3;     ...
         zeros(3), zeros(3), I3];
    B = [-0.5*dt*dt*I3; -dt*I3; zeros(3)];
    x = F*x + B*a_d(:);

    Sg = diag(cfg.sigma_j(:).^2);         % target-JERK PSD per axis
    G  = [dt*dt*dt/6*I3; 0.5*dt*dt*I3; dt*I3];
    Q  = G * Sg * G.';
end

P = F*P*F.' + Q;
P = 0.5*(P + P.');                        % kill drift out of symmetry
end


%% ─────────────────────────────────────────────────────────────────
function [x, P, accepted] = dkf_correct(x, P, z, Reb, cfg, NS)  %#ok<INUSD>
%  EKF update against the delayed image measurement, evaluated in the
%  attitude that held when the shutter actually fired.
%
%  CODEGEN: NS re-derived from size(x,1), same reason as dkf_predict.
NS = size(x, 1);

accepted = false;

p_r  = x(1:3);
p_b  = Reb * p_r;                          % Earth -> Body
xb   = p_b(1);  yb = p_b(2);  zb = p_b(3); % xb is depth along optical axis
if xb < cfg.min_depth
    return;                                % target behind camera: unusable
end

fx = cfg.fx;  fy = cfg.fy;  rt = cfg.r_t;

% Predicted measurement -- identical algebra to camera_model.m steps 5 & 7.
h = [ fx*(-yb/xb) + cfg.cx ; ...
      fy*(-zb/xb) + cfg.cy ; ...
      fx*rt/xb ];

% d h / d p_b
Hpb = [  fx*yb/(xb*xb),  -fx/xb,        0     ; ...
         fy*zb/(xb*xb),      0,     -fy/xb    ; ...
        -fx*rt/(xb*xb),      0,         0     ];

H = zeros(3, NS);
H(:, 1:3) = Hpb * Reb;                     % chain rule through the rotation

Rm = diag([cfg.sigma_uv^2, cfg.sigma_uv^2, cfg.sigma_s^2]);

if cfg.use_area < 0.5
    % Bearing-only, the paper's Eq. (32)-(33).
    %
    % CODEGEN: this used to shrink H, h, z and Rm to two rows, which under
    % code generation is four size mismatches ([3 x :?] vs [2 x :?] and so
    % on) because cfg.use_area is a field of a persistent struct and so is
    % not known at compile time -- the coder must build both shapes.
    %
    % Inflating the area channel's measurement variance instead is
    % algebraically the same thing: the Kalman gain for that row goes to
    % zero, so the radius measurement contributes nothing to the state, and
    % its term in the chi-squared gate vanishes too. Every array keeps one
    % fixed size. 1e12 px^2 is ~1e6 px of assumed noise, far beyond the
    % 2464 px sensor, so the row is dead without making Sk ill-conditioned.
    Rm(3, 3) = 1e12;
end

y  = z - h;                                % innovation
Sk = H*P*H.' + Rm;
Sk = 0.5*(Sk + Sk.');

% Chi-squared innovation gate: throw out frames the model says are absurd
% (target pinned to the image edge, area glitches).
if cfg.gate_chi2 > 0
    d2 = y.' * (Sk \ y);
    if ~isfinite(d2) || d2 > cfg.gate_chi2
        return;
    end
end

K = (P * H.') / Sk;

x = x + K*y;

% Joseph form -- stays positive-definite even with a stiff gain.
IKH = eye(NS) - K*H;
P   = IKH*P*IKH.' + K*Rm*K.';
P   = 0.5*(P + P.');

accepted = true;
end


%% ─────────────────────────────────────────────────────────────────
function [u, v, area, dist] = dkf_project(x, Reb, cfg)
%  Forward camera model applied to the estimate: the delay-free image.
p_b  = Reb * x(1:3);
dist = max(p_b(1), cfg.min_depth);

u = cfg.fx * (-p_b(2)/dist) + cfg.cx;
v = cfg.fy * (-p_b(3)/dist) + cfg.cy;

% Clamp to the sensor, same as camera_model.m step 6, so downstream
% controllers never see a pixel that could not physically exist.
u = max(0, min(cfg.img_width,  u));
v = max(0, min(cfg.img_height, v));

r_px = cfg.fx * cfg.r_t / dist;
area = pi * r_px * r_px;
end


%% ─────────────────────────────────────────────────────────────────
function x0 = dkf_init_state(xt, z, Reb, cfg)
%  Cold start: invert the camera model on the first good frame.
%  Range comes from apparent radius, bearing from the pixel offsets.
%
%  xt is a size template only; its contents are discarded. See the call
%  site for why NS is taken from size() rather than passed in.
NS  = size(xt, 1);
s   = max(z(3), 1e-6);
xb  = cfg.fx * cfg.r_t / s;                    % depth from apparent size
yb  = -(z(1) - cfg.cx) * xb / cfg.fx;
zb  = -(z(2) - cfg.cy) * xb / cfg.fy;

x0        = zeros(NS, 1);
x0(1:3)   = Reb.' * [xb; yb; zb];              % Body -> Earth
% velocity (and accel) start at zero; P0 is deliberately loose so the
% first few frames pull them in quickly.
end


%% ─────────────────────────────────────────────────────────────────
function P0 = dkf_P0(Pt, cfg)
%  Pt is a size template only; contents discarded. Same codegen reason.
NS = size(Pt, 1);
P0 = zeros(NS, NS);
P0(1:3, 1:3) = cfg.P0_pos * eye(3);
P0(4:6, 4:6) = cfg.P0_vel * eye(3);
if NS == 9
    P0(7:9, 7:9) = cfg.P0_acc * eye(3);
end
end


%% ─────────────────────────────────────────────────────────────────
function Reb = dkf_Reb(euler)
%  Earth -> Body rotation. Transcribed line-for-line from camera_model.m
%  step 2 so the filter and the sensor can never disagree about geometry.
phi = euler(1);  theta = euler(2);  psi = euler(3);
cphi = cos(phi); sphi = sin(phi);
cth  = cos(theta); sth = sin(theta);
cpsi = cos(psi); spsi = sin(psi);

Reb = [ cpsi*cth,                    spsi*cth,                   -sth    ; ...
        cpsi*sth*sphi - spsi*cphi,   spsi*sth*sphi + cpsi*cphi,   cth*sphi; ...
        cpsi*sth*cphi + spsi*sphi,   spsi*sth*cphi - cpsi*sphi,   cth*cphi];
end
