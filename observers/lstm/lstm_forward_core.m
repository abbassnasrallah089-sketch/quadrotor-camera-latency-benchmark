function y = lstm_forward_core(x)
%#codegen
%LSTM_FORWARD_CORE  One step of a single-layer LSTM plus a linear output.
%
%  ROLE (plain words)
%  The maths of the network, written by hand so it can be code-generated and
%  run inside Simulink. It holds the LSTM's memory (h and c) between calls —
%  that memory is what lets it use the history of the flight rather than
%  just this instant.
%
%  Gate order is PyTorch's: [input, forget, cell, output] = [i, f, g, o].
%  Get that order wrong and the network still runs, still produces plausible
%  numbers, and is quietly wrong — so it is verified against PyTorch by
%  py/verify_port.py before deployment, not assumed.
%
%  Reads lstm_weights.mat, which must be on the MATLAB path.
%
%  The line below is rewritten by use_model every time the active network
%  changes. DO NOT DELETE IT.
%
%  Why it exists: coder.load folds the .mat file's CONTENTS into the
%  compiled block as constants, and Simulink's rebuild check tracks .m
%  files, not the data files they read. So replacing lstm_weights.mat alone
%  leaves the block running whatever weights it compiled first — silently.
%  Three completely different networks then produce identical trajectories,
%  and nothing anywhere reports a problem. Touching this file is what forces
%  the rebuild.
%
%#WEIGHTS: obs2  (2026-08-26 16:40:03)

persistent h c W_ih W_hh b_ih b_hh W_out b_out mu sg H loaded

if isempty(loaded)
    P     = coder.load('lstm_weights.mat');
    W_ih  = P.W_ih;      % [4H x NF]
    W_hh  = P.W_hh;      % [4H x H]
    b_ih  = P.b_ih;      % [4H x 1]
    b_hh  = P.b_hh;      % [4H x 1]
    W_out = P.W_out;     % [3  x H]
    b_out = P.b_out;     % [3  x 1]
    mu    = P.feat_mean; % [NF x 1]
    sg    = P.feat_std;  % [NF x 1]
    H     = size(W_hh, 2);
    h     = zeros(H, 1);
    c     = zeros(H, 1);
    loaded = true;
end

%% ── Contract check, with a message that names the numbers ──
%  If this ever fires, it says outright which side is wrong instead of
%  leaving a bare "sizes are incompatible" to be decoded from operand order.
if numel(x) ~= numel(mu)
    error('lstm:featureCount', ...
          ['Feature-count mismatch: lstm_block built %d features but ' ...
           'lstm_weights.mat expects %d. The .mat file and lstm_block.m ' ...
           'were built from different versions.'], numel(x), numel(mu));
end

xn = (x(:) - mu) ./ sg;

g = W_ih * xn + b_ih + W_hh * h + b_hh;

i_g = sigmoid_(g(1     : H));
f_g = sigmoid_(g(H+1   : 2*H));
g_g = tanh(    g(2*H+1 : 3*H));
o_g = sigmoid_(g(3*H+1 : 4*H));

c = f_g .* c + i_g .* g_g;
h = o_g .* tanh(c);

y = W_out * h + b_out;

end

% ─────────────────────────────────────────────────────────────
function s = sigmoid_(z)
s = 1 ./ (1 + exp(-z));
end
