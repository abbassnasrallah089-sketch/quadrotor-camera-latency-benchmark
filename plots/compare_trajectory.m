%% COMPARE_TRAJECTORY — UAV trajectories on one 3-D figure.
%
%  no delay / 160 ms delay / DKF / Luenberger / LSTM against the same
%  target path. Nothing else on the figure.
%
%  Run  run_all_sims  first (or have out1..out5 in the workspace), then
%  press Run here.

if ~exist('RUN_ID_FORCE', 'var'), close all; end

%% ══ SETTINGS ════════════════════════════════════════════════════
%   NOISE-FREE   1 = no delay      2 = 160 ms delay        3 = DKF
%                4 = Luenberger    5 = LSTM
%   NOISY        6 = DKF+noise     7 = Luen+noise
%                8 = delay+noise (baseline)
IDS        = [8 6 7];      % <<<<<< which runs to draw (noise campaign)
EQUAL_AXES = false;               % see the note below before turning on
VIEW_ANGLE = [-37, 18];           % [azimuth, elevation]
%% ════════════════════════════════════════════════════════════════
%
%  EQUAL_AXES deliberately defaults to FALSE.
%
%  With it on, one metre of altitude is drawn the same length as one metre
%  of X. That is geometrically honest, but in a scenario where the target
%  climbs 600 m while the chase spans about 5 m horizontally it collapses
%  every trajectory onto a single vertical line and you can see nothing at
%  all. Letting each axis scale itself distorts the shape but shows the
%  behaviour, which is the point of this figure.
%
%  Turn it on for scenarios where the three axes have comparable extent
%  (3 and 6). Leave it off for the 600 m climbs (2 and 8).

R = loadruns(IDS);

for k = 1:numel(R)
    [R(k).p, R(k).t] = getsig('position',   R(k).obj);
    R(k).tg          = getsig('target_pos', R(k).obj);
    n = min([size(R(k).p,1), size(R(k).tg,1), numel(R(k).t)]);
    R(k).p  = R(k).p(1:n, :);
    R(k).tg = R(k).tg(1:n, :);
    R(k).t  = R(k).t(1:n);
end

T = R(1).tg;                        % target path, common to all runs

figure('Name', 'UAV trajectory comparison', 'Color', 'w', ...
       'Position', [90 70 1080 780]);
ax = axes; hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');

%% ── Target path ─────────────────────────────────────────────────
plot3(T(:,1), T(:,2), T(:,3), '--', 'Color', [0.55 0.55 0.55], ...
      'LineWidth', 2.4, 'DisplayName', 'target path');

%% ── The chases ──────────────────────────────────────────────────
%  Widths decrease down the list so overlapping traces (no-delay / DKF /
%  Luenberger) stay visible instead of one hiding the other.
w = [3.2, 2.2, 1.8, 1.5, 1.3, 1.3, 1.3, 1.3, 1.3];
for k = 1:numel(R)
    plot3(R(k).p(:,1), R(k).p(:,2), R(k).p(:,3), R(k).style, ...
          'Color', R(k).color, 'LineWidth', w(min(k, numel(w))), ...
          'DisplayName', R(k).label);
end

%% ── Start and finish ────────────────────────────────────────────
plot3(T(1,1), T(1,2), T(1,3), 's', 'Color', [0.3 0.3 0.3], ...
      'MarkerFaceColor', [0.3 0.3 0.3], 'MarkerSize', 10, ...
      'DisplayName', 'target start');
plot3(T(end,1), T(end,2), T(end,3), 'p', 'Color', [0.3 0.3 0.3], ...
      'MarkerFaceColor', [0.3 0.3 0.3], 'MarkerSize', 16, ...
      'DisplayName', 'target end');
for k = 1:numel(R)
    plot3(R(k).p(end,1), R(k).p(end,2), R(k).p(end,3), 'o', ...
          'Color', R(k).color, 'MarkerFaceColor', R(k).color, ...
          'MarkerSize', 9, 'HandleVisibility', 'off');
end

%% ── Presentation ────────────────────────────────────────────────
xlabel('X  (m)', 'FontWeight', 'bold', 'FontSize', 12);
ylabel('Y  (m)', 'FontWeight', 'bold', 'FontSize', 12);
zlabel('Z  (m)', 'FontWeight', 'bold', 'FontSize', 12);
title('UAV trajectory chasing the target', 'FontSize', 15, 'FontWeight', 'bold');
legend('Location', 'northeastoutside', 'FontSize', 11);
view(VIEW_ANGLE); set(ax, 'FontSize', 11, 'LineWidth', 1.1);
if EQUAL_AXES, axis(ax, 'equal'); end

% A flat axis gets no useful range from min/max, so pad it explicitly --
% otherwise MATLAB draws a degenerate box and the trajectory sits on the
% wall of the plot.
pad_axis_(ax, 'X', [T(:,1); vertcat_(R, 1)]);
pad_axis_(ax, 'Y', [T(:,2); vertcat_(R, 2)]);
pad_axis_(ax, 'Z', [T(:,3); vertcat_(R, 3)]);

rotate3d(ax, 'on');     % drag to rotate: often the fastest way to read it

%% ── Path length: "how much extra flying did the delay cost" ─────
fprintf('\n=================================================================\n');
fprintf(' TRAJECTORY COMPARISON\n');
fprintf('=================================================================\n');
fprintf(' %-32s %12s %12s\n', '', 'path flown', 'vs first');
fprintf(' %-32s %12s %12s\n', '', '(m)', '(%)');
fprintf(' %s\n', repmat('-', 1, 50));
L0 = NaN;
for k = 1:numel(R)
    L = sum(vecnorm(diff(R(k).p), 2, 2));
    if k == 1, L0 = L; end
    fprintf(' %-32s %12.1f %12.1f\n', R(k).label, L, 100*(L/L0 - 1));
end
fprintf('=================================================================\n');
fprintf(' Drag the figure to rotate. EQUAL_AXES is off: axes are scaled\n');
fprintf(' independently so the motion is visible. Set it true for\n');
fprintf(' scenarios 3 and 6, where true proportions still read clearly.\n\n');


%% ─────────────────────────────────────────────────────────────────
function v = vertcat_(R, col)
v = [];
for k = 1:numel(R), v = [v; R(k).p(:, col)]; end %#ok<AGROW>
end

function pad_axis_(ax, which, data)
lo = min(data); hi = max(data);
if ~isfinite(lo) || ~isfinite(hi), return; end
r = hi - lo;
if r < 1e-6, r = 1; end                 % genuinely flat axis
set(ax, [which 'Lim'], [lo - 0.06*r, hi + 0.06*r]);
end
