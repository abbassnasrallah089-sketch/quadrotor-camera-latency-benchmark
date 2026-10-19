%% COMPARE_DISTANCE — UAV-to-target distance for all runs, one graph.
%
%  The headline figure: how long each condition takes to close on the
%  target, how far it overshoots, and whether it settles cleanly or keeps
%  oscillating.
%
%  Run  run_all_sims  first (or have out1..out5 in the workspace), then
%  press Run here.

if ~exist('RUN_ID_FORCE', 'var'), close all; end

%% ══ WHICH RUNS ══════════════════════════════════════════════════
%   NOISE-FREE   1 = no delay      2 = 160 ms delay        3 = DKF
%                4 = Luenberger    5 = LSTM
%   NOISY        6 = DKF+noise     7 = Luen+noise
%                8 = delay+noise (baseline)
IDS       = [8 6 7];      % <<<<<< EDIT THIS LINE  (noise campaign)
STANDOFF  = 2.0;                  % commanded stand-off distance, m
SETTLE_TH = 2.5;                  % "arrived" threshold for the table, m
TAIL_S    = 10;                   % steady-state window for the table, s
%% ════════════════════════════════════════════════════════════════
%
%  Default is the noise comparison with the delay baseline in front of it.
%  For the original noise-free figure use  IDS = [1 2 3 4 5].
%
%  Do NOT read the gap between 3 and 6 (or 4 and 7) as "the cost of
%  noise": the observers were retuned between campaigns, so that gap is
%  noise AND a gain change at once. See the note in loadruns.m.

R = loadruns(IDS);

for k = 1:numel(R)
    [p, t]  = getsig('position',   R(k).obj);
    tg      = getsig('target_pos', R(k).obj);
    n       = min([size(p,1), size(tg,1), numel(t)]);
    R(k).t  = t(1:n);
    R(k).d  = vecnorm(tg(1:n,:) - p(1:n,:), 2, 2);
end

fig = figure('Name', 'Distance to target', 'Color', 'w', ...
             'Position', [80 80 1220 560]);

%% ── Distance ────────────────────────────────────────────────────
subplot(1, 2, 1); hold on; grid on;
for k = 1:numel(R)
    plot(R(k).t, R(k).d, R(k).style, 'Color', R(k).color, 'LineWidth', 2.0, ...
         'DisplayName', R(k).label);
end
yline(STANDOFF, ':', sprintf('%.1f m stand-off', STANDOFF), ...
      'Color', [0.35 0.35 0.35], 'LineWidth', 1.4, ...
      'LabelHorizontalAlignment', 'left', 'HandleVisibility', 'off');
xlabel('t (s)', 'FontWeight', 'bold');
ylabel('|p_t - p_d|   (m)', 'FontWeight', 'bold');
title('Distance between UAV and target', 'FontSize', 14, 'FontWeight', 'bold');
legend('Location', 'best'); set(gca, 'FontSize', 11, 'LineWidth', 1.1);

%% ── Extra distance versus the first run ─────────────────────────
%  Plotting the difference rather than a second copy of the curves is what
%  makes the claim legible: how much the delay costs at each instant, and
%  how much of that each compensator (DKF / Luenberger / LSTM) returns.
subplot(1, 2, 2); hold on; grid on;
ref = R(1);
for k = 2:numel(R)
    n = min(numel(ref.d), numel(R(k).d));
    plot(R(k).t(1:n), R(k).d(1:n) - ref.d(1:n), R(k).style, ...
         'Color', R(k).color, 'LineWidth', 1.8, 'DisplayName', R(k).label);
end
yline(0, ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
xlabel('t (s)', 'FontWeight', 'bold');
ylabel(sprintf('extra distance vs "%s"  (m)', ref.label), 'FontWeight', 'bold');
title('Cost of the delay, and what each compensator returns', ...
      'FontSize', 14, 'FontWeight', 'bold');
if numel(R) > 1, legend('Location', 'best'); end
set(gca, 'FontSize', 11, 'LineWidth', 1.1);

%% ── Numbers for the report ──────────────────────────────────────
fprintf('\n=================================================================\n');
fprintf(' DISTANCE TO TARGET\n');
fprintf('=================================================================\n');
fprintf(' %-32s %8s %8s %8s %9s %9s\n', '', 'min', 'final', 'settle', 'RMS', 'jitter');
fprintf(' %-32s %8s %8s %8s %9s %9s\n', '', '(m)', '(m)', '(s)', ...
        sprintf('last %gs', TAIL_S), sprintf('std %gs', TAIL_S));
fprintf(' %s\n', repmat('-', 1, 82));
for k = 1:numel(R)
    t = R(k).t; d = R(k).d;
    tail = t >= t(end) - TAIL_S;
    i1   = find(d < SETTLE_TH, 1, 'first');
    ts   = NaN; if ~isempty(i1), ts = t(i1); end
    fprintf(' %-32s %8.3f %8.3f %8.2f %9.3f %9.4f\n', ...
            R(k).label, min(d), d(end), ts, ...
            sqrt(mean((d(tail) - STANDOFF).^2)), std(d(tail)));
end
fprintf('=================================================================\n');
fprintf(' settle  = first time within %.1f m of the target.\n', SETTLE_TH);
fprintf(' RMS     = error vs the %.1f m stand-off over the last %g s.\n', STANDOFF, TAIL_S);
fprintf('           (This is the deviation from the setpoint, NOT the raw\n');
fprintf('           distance - a run sitting perfectly on 2 m now reads 0.)\n');
fprintf(' jitter  = std of distance over the last %g s. THIS is the column\n', TAIL_S);
fprintf('           the noise campaign turns on: detector noise does not\n');
fprintf('           move the mean, it adds spread. A fixed gain cannot\n');
fprintf('           re-weight a noisy measurement, a Kalman gain can, and\n');
fprintf('           whole-run RMSE hides the difference completely.\n\n');

if any([R.noisy]) && any(~[R.noisy])
    fprintf(' NOTE: this figure mixes noise-free (solid) and noisy (dashed)\n');
    fprintf(' runs. Compare WITHIN a campaign - the observers were retuned\n');
    fprintf(' between them, so a solid-to-dashed gap is not "the cost of\n');
    fprintf(' noise", it is noise plus a gain change.\n\n');
end
