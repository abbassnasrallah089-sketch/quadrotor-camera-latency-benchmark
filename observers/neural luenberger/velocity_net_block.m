function [du, dv, dr] = velocity_net_block(u_del, v_del, area_del, ...
                                           valid_flag, euler, omega, a_d)
%#codegen
%VELOCITY_NET_BLOCK  Block 1 of the neural observer: predict PIXEL VELOCITY.
%
%  Assembles the SAME 20 features as lstm_block.m (contract unchanged) and
%  runs whatever network lstm_weights.mat holds. The outputs are the time
%  derivatives of the three normalised image quantities, per second:
%      du = d/dt (u - cx)/f
%      dv = d/dt (v - cy)/f
%      dr = d/dt sqrt(area/pi)/f
%  It NEVER integrates and NEVER touches the measurement — that is the
%  second block's job. With zero weights the outputs are exactly zero.
%
%  BLOCK SETTINGS: sample time 0.005 (NOT -1). 7 in, 3 out.
%  Reset between runs: clear velocity_net_block lstm_forward_core

CX = 1640.0;  CY = 1232.0;  FOCAL = 289.19;
CAM_STEP = 7;  D_STEPS = 32;  NF = 20;
NB_S = CAM_STEP + 1;  NB_L = D_STEPS + 1;

assert(numel(euler) == 3, 'velocity_net_block: euler must be 3-wide (port 5)');
assert(numel(omega) == 3, 'velocity_net_block: omega must be 3-wide (port 6)');
assert(numel(a_d)   == 3, 'velocity_net_block: a_d must be 3-wide (port 7)');

persistent u_buf v_buf r_buf e_buf si li primed
if isempty(primed)
    u_buf = zeros(NB_S,1); v_buf = zeros(NB_S,1); r_buf = zeros(NB_S,1);
    e_buf = zeros(NB_L,3); si = 0; li = 0; primed = false;
end

ut = (u_del - CX) / FOCAL;
vt = (v_del - CY) / FOCAL;
if area_del > 0, rt = sqrt(area_del/pi) / FOCAL; else, rt = 0; end

%  prime with the FIRST sample, matching common.lag on the Python side
if ~primed
    u_buf(:) = ut; v_buf(:) = vt; r_buf(:) = rt;
    for kk = 1:NB_L
        e_buf(kk,1) = euler(1); e_buf(kk,2) = euler(2); e_buf(kk,3) = euler(3);
    end
    primed = true;
end

si = si + 1; if si > NB_S, si = 1; end
old_s = si + 1; if old_s > NB_S, old_s = 1; end
d_u = ut - u_buf(old_s);  d_v = vt - v_buf(old_s);  d_r = rt - r_buf(old_s);
u_buf(si) = ut; v_buf(si) = vt; r_buf(si) = rt;

li = li + 1; if li > NB_L, li = 1; end
old_l = li + 1; if old_l > NB_L, old_l = 1; end
deul1 = euler(1) - e_buf(old_l,1);
deul2 = euler(2) - e_buf(old_l,2);
deul3 = euler(3) - e_buf(old_l,3);
deul3 = atan2(sin(deul3), cos(deul3));
e_buf(li,1) = euler(1); e_buf(li,2) = euler(2); e_buf(li,3) = euler(3);

x = zeros(NF,1);
x(1)=ut; x(2)=vt; x(3)=rt;
x(4)=d_u; x(5)=d_v; x(6)=d_r;
x(7)=valid_flag;
x(8)=euler(1); x(9)=euler(2); x(10)=sin(euler(3)); x(11)=cos(euler(3));
x(12)=deul1; x(13)=deul2; x(14)=deul3;
x(15)=omega(1); x(16)=omega(2); x(17)=omega(3);
x(18)=a_d(1); x(19)=a_d(2); x(20)=a_d(3);

y = lstm_forward_core(x);
du = y(1);  dv = y(2);  dr = y(3);
end
