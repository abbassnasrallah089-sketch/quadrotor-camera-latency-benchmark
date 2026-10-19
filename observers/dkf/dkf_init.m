function S = dkf_init(cfg)  %#codegen
%% DKF_INIT — Allocate the DKF state + delay window.
%
%  Call once before the first dkf_core() step (or inside the Simulink
%  block's persistent-variable initialisation). Every array is fixed-size
%  so the whole thing is codegen-safe for the MATLAB Function block.
%
%  Buffer layout (the "window" of Fig. 7 in the paper):
%     slot 1   <- time k-D   (the instant a frame arriving NOW describes)
%     ...
%     slot NB  <- time k     (now)
%
%  For D = 32 that is 33 slots. Memory is trivial: ~11 kB in CV mode.

%% ── Sizes MUST be literals here ──────────────────────────────────
%  These two numbers decide the shape of every array below, and code
%  generation requires them to be known at compile time. Deriving them
%  from cfg (NS from cfg.model_id, NB from cfg.NBUF) makes the whole state
%  buffer variable-sized, and the failure surfaces far away, as
%  "Size mismatch (size [6 x 6] ~= size [9 x 9])" inside dkf_predict.
%
%  dkf_block.m also wraps dkf_params in coder.const, which should make cfg
%  constant on its own; these literals are the belt to that pair of braces
%  and cost nothing.
%
%  To switch to the 9-state constant-acceleration model, change NS to 9
%  AND set cfg.model_id = 2 in dkf_params.m. The assert below stops the two
%  from silently disagreeing.
NS = 6;                           % 6 = CV (model_id 1), 9 = CA (model_id 2)
NB = 33;                          % = cfg.D + 1 = round(0.160/0.005) + 1

want_NS = 6;
if isfield(cfg, 'model_id') && cfg.model_id == 2
    want_NS = 9;
end
assert(NS == want_NS, ...
    'dkf_init:modelMismatch', ...
    ['NS is hard-coded to %d but cfg.model_id asks for %d states. ' ...
     'Change NS at the top of dkf_init.m to match dkf_params.m.'], ...
    NS, want_NS);
assert(NB == cfg.NBUF, ...
    'dkf_init:bufMismatch', ...
    ['NB is hard-coded to %d but cfg.NBUF = %d. These come from ' ...
     'cfg.DELAY_S / cfg.DT; update NB in dkf_init.m to match.'], ...
    NB, cfg.NBUF);

S.xb    = zeros(NS, NB);          % state at each slot
S.Pb    = zeros(NS, NS, NB);      % covariance at each slot
S.ab    = zeros(3, NB);           % drone accel (earth frame) at each slot
S.Rb    = zeros(3, 3, NB);        % Earth->Body rotation at each slot
S.vb    = zeros(1, NB);           % in_view flag at each slot
S.n     = 0;                      % steps of history accumulated
S.init  = 0;                      % 1 once the track has been acquired
S.coast = 0;                      % consecutive steps with no fused frame

% Seed every slot's rotation with identity and covariance with P0 so the
% very first steps are numerically well-behaved before the window fills.
P0 = zeros(NS, NS);
P0(1:3, 1:3) = cfg.P0_pos * eye(3);
P0(4:6, 4:6) = cfg.P0_vel * eye(3);
if NS == 9
    P0(7:9, 7:9) = cfg.P0_acc * eye(3);
end

for i = 1:NB
    S.Rb(:, :, i) = eye(3);
    S.Pb(:, :, i) = P0;
end

end
