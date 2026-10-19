function [u_f, v_f, area_f] = smooth_block(u_est, v_est, area_est)
%#codegen
%% SMOOTH_BLOCK — a filter placed AFTER the LSTM block
%
%  ROLE (plain words)
%  The LSTM produces a fresh prediction every 5 ms and nothing forces two
%  consecutive predictions to agree, so its output chatters. Measured on the
%  r3 flights, the step-to-step movement of v is 2.36 px against 0.392 px in
%  the real camera signal — six times noisier than physics.
%
%  That is free for the yaw loop, which is pure proportional (D = 0), and
%  ruinous for the altitude loop, which is a PD with D = 0.1 and its output
%  limit switched off. At 200 Hz a single pixel of per-step jitter becomes
%  20 N of thrust command, against 12.753 N of hover thrust.
%
%  This block smooths all three channels with a first-order low-pass, so the
%  controllers see something closer to the clean staircase a Kalman filter
%  produces, without changing the network or retraining anything.
%
%  INPUTS   u_est, v_est, area_est   the three LSTM_Block outputs
%  OUTPUTS  u_f,   v_f,   area_f     the same, smoothed
%
%  WHY AREA IS FILTERED IN RADIUS
%  The network predicts a RADIUS, and the controller turns area back into a
%  distance through  distance = 0.3 / (sqrt(area/pi)/f). Radius is the linear
%  quantity; area is its square. Smoothing area directly would bias the
%  result, because the average of a square is not the square of the average.
%  So this converts to radius, filters there, and squares back. That also
%  matches what TAU_R does inside lstm_block, so the two are comparable.
%
%  THE TRADE-OFF, STATED PLAINLY
%  A filter of TAU seconds removes jitter and adds roughly TAU of lag. The
%  network exists to remove 160 ms of lag, so TAU = 0.02 hands back about 13%
%  of that, TAU = 0.05 about 31%, TAU = 0.10 about 62%. There is a best value
%  and it is not zero and not large. Start at 0.02.
%
%  PROVE THE WIRING FIRST
%  Set BYPASS = 1 and the block passes its inputs through untouched. The
%  flight must then be bit-identical to the unfiltered one. That proves the
%  block is wired correctly before any filtering is judged — the same trick
%  the zero-weight network used to prove the LSTM wiring.
%
%  BLOCK SETTINGS
%    Sample time : 0.005   (NOT -1)
%    3 inputs, 3 outputs, all scalars.

%% ── the two numbers to tune ─────────────────────────────────
BYPASS  = 0;        % 1 = pass through untouched, to prove the wiring
TAU_UV  = 0.0200;   % seconds, filter on u and v.   0 = off
TAU_R   = 0.0200;   % seconds, filter on the radius channel. 0 = off
DT      = 0.005;    % the loop period; must match the block sample time

%% ── state ───────────────────────────────────────────────────
persistent uf vf rf started
if isempty(started)
    uf      = u_est;
    vf      = v_est;
    rf      = sqrt(max(area_est, 0) / pi);
    started = true;
end

%% ── straight through, for the wiring proof ──────────────────
if BYPASS > 0.5
    u_f    = u_est;
    v_f    = v_est;
    area_f = area_est;
    uf     = u_est;
    vf     = v_est;
    rf     = sqrt(max(area_est, 0) / pi);
    return;
end

%% ── u and v ─────────────────────────────────────────────────
if TAU_UV > 0
    a  = DT / (TAU_UV + DT);
    uf = uf + a * (u_est - uf);
    vf = vf + a * (v_est - vf);
else
    uf = u_est;
    vf = v_est;
end
u_f = uf;
v_f = vf;

%% ── area, filtered in radius ────────────────────────────────
r_in = sqrt(max(area_est, 0) / pi);
if TAU_R > 0
    b  = DT / (TAU_R + DT);
    rf = rf + b * (r_in - rf);
else
    rf = r_in;
end
if rf < 0
    rf = 0;
end
area_f = pi * rf * rf;

end
