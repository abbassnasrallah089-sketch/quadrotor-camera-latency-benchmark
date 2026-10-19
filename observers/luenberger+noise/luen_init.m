function S = luen_init(cfg)  %#codegen
%% LUEN_INIT — Allocate observer state + delay window.
%
%  Layout matches the DKF window (paper Fig. 7):
%     slot 1   <- time k-D   (instant a frame arriving NOW describes)
%     slot NB  <- time k     (now)
%
%  No covariance P — this is a Luenberger observer, not a Kalman filter.

NS = 6;                           % [p_r(3); v_r(3)]
NB = 33;                          % = cfg.D + 1

assert(NB == cfg.NBUF, ...
    'luen_init:bufMismatch', ...
    'NB=%d but cfg.NBUF=%d. Update NB to match DELAY_S/DT.', NB, cfg.NBUF);

S.xb    = zeros(NS, NB);          % state at each slot
S.ab    = zeros(3, NB);           % drone accel (earth) at each slot
S.Rb    = zeros(3, 3, NB);        % Earth->Body rotation at each slot
S.vb    = zeros(1, NB);           % in_view at each slot
S.n     = 0;
S.init  = 0;
S.coast = 0;

for i = 1:NB
    S.Rb(:, :, i) = eye(3);
end

end
