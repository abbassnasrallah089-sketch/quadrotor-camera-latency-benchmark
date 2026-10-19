function [u_hat, v_hat, area_hat] = lstm_block(u_del, v_del, area_del, ...
                                               valid_flag, euler, omega, a_d)
%#codegen
%LSTM_BLOCK  Delay compensation for the 160 ms camera measurement.
%
%  ROLE (plain words)
%  This is the block that sits between the delay block and the controllers.
%  It takes the 160 ms-old measurement plus what the drone is doing right
%  now, and estimates where the target actually is at this instant.
%
%  It predicts a RESIDUAL — a correction that is ADDED to the delayed
%  measurement. Consequence, and this is the whole safety net: a network
%  whose weights are all zero produces exactly the delayed measurement,
%  unchanged. That is why the plumbing can be proved before any training.
%
%  INPUTS (all at 200 Hz, sample time MUST be 0.005)
%    u_del, v_del, area_del  [1x1]  delayed measurement
%    valid_flag              [1x1]  1 on the step a new delayed frame lands
%    euler                   [3x1]  attitude now [phi; theta; psi], rad
%    omega                   [3x1]  body rates now [p; q; r], rad/s
%    a_d                     [3x1]  Earth-frame accel AFTER the unit delay
%
%  OUTPUTS
%    u_hat, v_hat, area_hat  [1x1]  delay-compensated measurement
%
%  Reset between runs with:  clear lstm_block lstm_forward_core

%% ── DIAGNOSTIC SWITCH ──────────────────────────────────────
%  Which channels the network is allowed to correct. Set it here, save, and
%  re-run — no other edit needed anywhere.
%
%     0  normal      all three channels corrected
%     1  u,v only    radius passes through undelayed-uncorrected
%     2  radius only u and v pass through
%
%  WHY THIS EXISTS
%  Closed-loop testing showed the drone approaching to ~2 m and then backing
%  off to a tightly clustered ~22 m on nearly every run. A consistent final
%  distance like that means the drone is confident and wrong about range —
%  and range comes from exactly one channel, the apparent radius.
%
%  That channel is badly conditioned: distance goes as 1/radius, so at long
%  range the true radius is tiny and a small ABSOLUTE error becomes a huge
%  RELATIVE one. Offline the radius error reads 1.10 px, which looks superb.
%  The distance controller sees nothing else.
%
%  Setting this to 1 or 2 isolates the culprit in one run each.
DIAG = 0;

%% ── CORRECTION GAIN ────────────────────────────────────────
%  How much of the network's correction to actually apply.
%     1.0  full correction
%     0.5  half
%     0.0  none (identical to no compensation at all)
%
%  WHY THIS EXISTS
%  Logging the block's outputs during a failing run showed the prediction is
%  accurate — the corrected signal sits on top of the truth — yet the loop
%  oscillates at about 1.6 Hz and eventually overshoots past the target and
%  loses it.
%
%  An accurate predictor can still destabilise a loop. The network's residual
%  error is not random noise: it is a function of the drone's own states, so
%  it travels back around the loop and behaves like added gain and phase lag.
%  The controller gains were tuned against a perfect measurement, so the
%  compensated loop is not the loop they were designed for.
%
%  Reducing the gain trades some delay compensation for stability margin. If
%  0.5 converges where 1.0 oscillates, that is the diagnosis confirmed, and
%  the proper fix is retuning the controller for the compensated loop.
%  FINAL SETTING: 1.0 / 1.0 with the round-2 (r2) network. Measured over 199
%  shared starting states: gain 1.0 with the round-1 'both' network converged
%  0 times; gain 0.5 reached 44; the r2 network at gain 1.0 reached 80 with a
%  median final distance of 2.84 m. Once the network is trained on the states
%  it actually visits, the correction no longer needs to be throttled.
GAIN_UV = 1.0000;
GAIN_R  = 1.0000;

%% ── CORRECTION LOW-PASS (seconds; 0 = off) ─────────────────
%  MEASURED, 40 runs per setting: every configuration with GAIN_UV = 1.00
%  converged 0-2 times out of 40, whatever the radius gain did. With
%  GAIN_UV = 0.50 it reached 13-18. So the destabilising channels are the
%  PIXEL ones — u and v, which drive yaw and altitude — not the distance
%  channel as first suspected.
%
%  A flat gain is a blunt instrument: it discards half the correction at
%  every frequency, including the low frequencies where delay compensation
%  actually does its work. The observed limit cycle sits at 0.7-1.6 Hz.
%  A first-order low-pass keeps the correction where it helps and removes it
%  where the loop resonates.
%
%  tau = 0.15 s puts the corner near 1 Hz, just below the limit cycle.
TAU_UV = 0.000;
TAU_R  = 0.000;

%% ── Constants: must match lstm_config.m and py/common.py ───
CX       = 1640.0;
CY       = 1232.0;
FOCAL    = 289.19;
CAM_STEP = 7;      % 30 Hz camera period, in control steps
D_STEPS  = 32;     % 160 ms
NF       = 20;     % THE FEATURE CONTRACT

NB_S = CAM_STEP + 1;
NB_L = D_STEPS  + 1;

%% ── HARD SIZE GUARDS — read this before deleting them ──────
%  The previous attempt lost two days to a feature vector that was 18 long
%  instead of 20, because one 3-wide input silently arrived as a scalar and
%  the mismatch only surfaced deep inside the network, as an arithmetic
%  error with no clue which wire caused it.
%
%  These three lines are constant-folded during code generation, so a wrong
%  port width now fails AT COMPILE TIME with the offending signal named.
assert(numel(euler) == 3, 'lstm_block: euler must be 3-wide (check port 5)');
assert(numel(omega) == 3, 'lstm_block: omega must be 3-wide (check port 6)');
assert(numel(a_d)   == 3, 'lstm_block: a_d must be 3-wide (check port 7)');

persistent u_buf v_buf r_buf e_buf si li primed

if isempty(primed)
    u_buf = zeros(NB_S, 1);
    v_buf = zeros(NB_S, 1);
    r_buf = zeros(NB_S, 1);
    e_buf = zeros(NB_L, 3);
    si    = 0;
    li    = 0;
    primed = false;
end

%% ── Normalise the delayed measurement ──────────────────────
%  Radius, not area. Radius falls off as 1/distance with roughly constant
%  pixel error; area falls off as 1/distance^2, which puts a fourth power
%  on the error and makes the channel far harder to learn.
ut = (u_del - CX) / FOCAL;
vt = (v_del - CY) / FOCAL;
if area_del > 0
    rt = sqrt(area_del / pi) / FOCAL;
else
    rt = 0;
end

%% ── Prime the history with the FIRST sample, not with zeros ─
%  The Python side holds the first value when a lag reaches back before the
%  start of a run. Starting these buffers at zero instead would make d_u and
%  deul enormous for the first 32 steps and feed the network inputs it never
%  saw in training — exactly when the loop is most fragile.
if ~primed
    u_buf(:) = ut;
    v_buf(:) = vt;
    r_buf(:) = rt;
    for kk = 1:NB_L
        e_buf(kk, 1) = euler(1);
        e_buf(kk, 2) = euler(2);
        e_buf(kk, 3) = euler(3);
    end
    primed = true;
end

%% ── Short history: the value one camera period ago ─────────
si = si + 1; if si > NB_S, si = 1; end
old_s = si + 1; if old_s > NB_S, old_s = 1; end

u_prev = u_buf(old_s);
v_prev = v_buf(old_s);
r_prev = r_buf(old_s);

u_buf(si) = ut;
v_buf(si) = vt;
r_buf(si) = rt;

d_u = ut - u_prev;
d_v = vt - v_prev;
d_r = rt - r_prev;

%% ── Long history: attitude one delay window ago ────────────
li = li + 1; if li > NB_L, li = 1; end
old_l = li + 1; if old_l > NB_L, old_l = 1; end

ep1 = e_buf(old_l, 1);
ep2 = e_buf(old_l, 2);
ep3 = e_buf(old_l, 3);

e_buf(li, 1) = euler(1);
e_buf(li, 2) = euler(2);
e_buf(li, 3) = euler(3);

deul1 = euler(1) - ep1;
deul2 = euler(2) - ep2;
deul3 = euler(3) - ep3;
deul3 = atan2(sin(deul3), cos(deul3));    % wrap yaw to [-pi, pi]

%% ── Assemble the feature vector ────────────────────────────
%  Built by INDEX into a pre-sized vector, never by concatenation. x is
%  therefore 20 long by construction — it cannot come out 18 no matter what
%  arrives on the ports, and a bad port fails at the assert above instead.
x = zeros(NF, 1);
x(1)  = ut;
x(2)  = vt;
x(3)  = rt;
x(4)  = d_u;
x(5)  = d_v;
x(6)  = d_r;
x(7)  = valid_flag;
x(8)  = euler(1);
x(9)  = euler(2);
x(10) = sin(euler(3));
x(11) = cos(euler(3));
x(12) = deul1;
x(13) = deul2;
x(14) = deul3;
x(15) = omega(1);
x(16) = omega(2);
x(17) = omega(3);
x(18) = a_d(1);
x(19) = a_d(2);
x(20) = a_d(3);

%% ── Network ────────────────────────────────────────────────
y = lstm_forward_core(x);

if DIAG == 1
    y(3) = 0;                 % u, v corrected; radius left alone
elseif DIAG == 2
    y(1) = 0;  y(2) = 0;      % radius corrected; u, v left alone
end

%  Separate gains per channel. The failures at uniform gain 0.5 all share one
%  signature: the drone reaches the target, overshoots INSIDE the 2 m
%  setpoint (min 1.2-2.6 m), and then loses it. With no delay, min sits at
%  2.0 with no overshoot at all. Overshoot is a damping problem in the
%  DISTANCE loop, and distance is driven by the radius channel alone — so
%  radius and the pixel channels want different gains.
persistent yfilt
if isempty(yfilt)
    yfilt = zeros(3, 1);
end

if TAU_UV > 0
    a = 0.005 / (TAU_UV + 0.005);
    yfilt(1) = yfilt(1) + a * (y(1) - yfilt(1));
    yfilt(2) = yfilt(2) + a * (y(2) - yfilt(2));
else
    yfilt(1) = y(1);
    yfilt(2) = y(2);
end

if TAU_R > 0
    bb = 0.005 / (TAU_R + 0.005);
    yfilt(3) = yfilt(3) + bb * (y(3) - yfilt(3));
else
    yfilt(3) = y(3);
end

y(1) = GAIN_UV * yfilt(1);
y(2) = GAIN_UV * yfilt(2);
y(3) = GAIN_R  * yfilt(3);

%% ── Apply the residual ─────────────────────────────────────
u_hat = u_del + y(1) * FOCAL;
v_hat = v_del + y(2) * FOCAL;

r_px = (rt + y(3)) * FOCAL;
if r_px < 0
    r_px = 0;
end
area_hat = pi * r_px * r_px;

end
