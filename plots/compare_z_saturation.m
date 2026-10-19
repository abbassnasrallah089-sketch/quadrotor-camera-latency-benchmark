%% COMPARE_Z_SATURATION — z_sat_log for all runs on one graph.
%
%  The altitude channel is where the delay shows up most violently: it is
%  the axis with the least authority, so lost phase margin turns into a
%  visible limit cycle here before it does anywhere else.
%
%  Run  run_all_sims  first (or have out1..out5 in the workspace), then
%  press Run here.

if ~exist('RUN_ID_FORCE', 'var'), close all; end

%% ══ WHICH RUNS ══════════════════════════════════════════════════
%   NOISE-FREE   1 = no delay      2 = 160 ms delay        3 = DKF
%                4 = Luenberger    5 = LSTM
%   NOISY        6 = DKF+noise     7 = Luen+noise
%                8 = delay+noise (baseline)
IDS     = [8 6 7];         % <<<<<< EDIT THIS LINE (noise campaign)
SIGNAL  = 'z_sat_log';            % change if your block has another name
TAIL_S  = 10;                     % window for the steady-state numbers
%% ════════════════════════════════════════════════════════════════

R = loadruns(IDS);

keep = true(1, numel(R));
for k = 1:numel(R)
    try
        [z, t]  = getsig(SIGNAL, R(k).obj);
        R(k).z  = z(:);
        R(k).t  = t(:);
        n       = min(numel(R(k).z), numel(R(k).t));
        R(k).z  = R(k).z(1:n);
        R(k).t  = R(k).t(1:n);
    catch
        warning('compare_z_saturation:missing', ...
            '"%s" not logged in %s (%s) - skipping that run.', ...
            SIGNAL, R(k).name, R(k).label);
        keep(k) = false;
    end
end
R = R(keep);
if isempty(R)
    error('compare_z_saturation:none', ...
        ['"%s" is not logged in any run.\n' ...
         'Add a To Workspace block named %s, Save format Array, ' ...
         'Sample time -1, then re-run run_all_sims.'], SIGNAL, SIGNAL);
end

fig = figure('Name', 'Z saturation comparison', 'Color', 'w', ...
             'Position', [80 80 1220 560]);

%% ── Full run ────────────────────────────────────────────────────
subplot(1, 2, 1); hold on; grid on;
for k = 1:numel(R)
    plot(R(k).t, R(k).z, R(k).style, 'Color', R(k).color, 'LineWidth', 1.8, ...
         'DisplayName', R(k).label);
end
xlabel('t (s)', 'FontWeight', 'bold');
ylabel('Z variation', 'FontWeight', 'bold');
title('Altitude channel, full run', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best'); set(gca, 'FontSize', 11, 'LineWidth', 1.1);

%% ── Steady state, where the limit cycle lives ───────────────────
%  The transient dominates the y-axis and hides a ripple of a few percent.
%  Zooming the tail is the only way to see whether the loop actually
%  settled or just looks settled at full scale.
subplot(1, 2, 2); hold on; grid on;
t_end = max(arrayfun(@(r) r.t(end), R));
for k = 1:numel(R)
    m = R(k).t >= t_end - TAIL_S;
    plot(R(k).t(m), R(k).z(m), R(k).style, 'Color', R(k).color, 'LineWidth', 1.8, ...
         'DisplayName', R(k).label);
end
xlabel('t (s)', 'FontWeight', 'bold');
ylabel('Z variation', 'FontWeight', 'bold');
title(sprintf('Last %g s - settled, or still ringing?', TAIL_S), ...
      'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best'); set(gca, 'FontSize', 11, 'LineWidth', 1.1);

%% ── Numbers ─────────────────────────────────────────────────────
fprintf('\n=================================================================\n');
fprintf(' Z SATURATION  /  ALTITUDE CHANNEL\n');
fprintf('=================================================================\n');
fprintf(' %-32s %9s %9s %12s\n', '', 'peak', 'mean', 'ripple p-p');
fprintf(' %-32s %9s %9s %12s\n', '', '', sprintf('last %gs', TAIL_S), sprintf('last %gs', TAIL_S));
fprintf(' %s\n', repmat('-', 1, 56));
for k = 1:numel(R)
    m = R(k).t >= R(k).t(end) - TAIL_S;
    z = R(k).z(m);
    fprintf(' %-32s %9.3f %9.3f %12.4f\n', R(k).label, ...
            max(abs(R(k).z)), mean(z), max(z) - min(z));
end
fprintf('=================================================================\n');
fprintf(' ripple p-p = peak-to-peak swing once settled. A run that has\n');
fprintf(' converged sits near zero; a visibly larger value is the delay\n');
fprintf(' induced limit cycle, and it should shrink again with a compensator.\n\n');
