%% plot_distance_error.m
% Plots BOTH the UAV-target distance and the distance error about the 2 m
% setpoint, on one figure, two stacked panels.
%
%   top    : distance(t)  with the commanded stand-off drawn as a reference
%   bottom : e_dist(t) = distance(t) - desired_dist,  zero-referenced
%
% Uses the existing position logs only - no new Simulink blocks needed.
% Also prints the settling time, final error, minimum separation and RMSE,
% which are the numbers the results tables need.

if ~exist('RUN_ID_FORCE','var'), close all; end


%% ══ WHICH RUN TO PLOT ═══════════════════════════════════════════
%  Edit the numbers on the RUN_ID line below, then press Run.
%      1 = no delay        2 = 160 ms delay      3 = delay + DKF
%      4 = delay + Luenberger                  5 = delay + LSTM
%      6 = DKF + noise     7 = Luenberger + noise   8 = delay + noise
%      0 = whatever the last simulation left in the workspace
%  ONE number  -> one figure.       RUN_ID = 3;
%  SEVERAL     -> one window per run, each labelled.   RUN_ID = [1 2 3 4 5];
if exist('RUN_ID_FORCE', 'var')
    RUN_ID = RUN_ID_FORCE;              % set by runmany - do not edit
else
    RUN_ID = 10;                   % <<<<<< EDIT THIS LINE
end

if numel(RUN_ID) > 1
    runmany(mfilename, RUN_ID);         % one window per run, then stop
    return;
end

RUN = pickrun(RUN_ID);


%% ══ SETTINGS ════════════════════════════════════════════════════
desired_dist = 2.0;      % commanded stand-off, m
SETTLE_TOL   = 0.10;     % half-width of the band used for settling, m
TAIL_S       = 5.0;      % length of the settled tail scored at the end, s
SHOW_TITLE   = true;     % false when the figure goes into the report
                         % (the LaTeX caption does the titling instead)
SAVE_PNG     = '';       % e.g. 'ch4_dist_s8.png' to export at 300 dpi


%% 1. Extract data
[pos_uav, t] = getsig('position',   RUN);
pos_tgt      = getsig('target_pos', RUN);

n       = min(size(pos_uav,1), size(pos_tgt,1));
pos_uav = pos_uav(1:n, :);
pos_tgt = pos_tgt(1:n, :);
t       = t(1:n);


%% 2. Distance and error
distances = sqrt(sum((pos_tgt - pos_uav).^2, 2));   % 3-D separation, m
e_dist    = distances - desired_dist;               % error about the setpoint


%% 3. Metrics for the results table
inside = abs(e_dist) <= SETTLE_TOL;
last_out = find(~inside, 1, 'last');
if isempty(last_out)
    t_set = t(1);                       % inside the band from the start
elseif last_out >= numel(inside)
    t_set = NaN;                        % never settles
else
    t_set = t(last_out + 1);
end

tail    = t >= (t(end) - TAIL_S);
d_final = mean(distances(tail));
e_final = mean(e_dist(tail));
rmse_all = sqrt(mean(e_dist.^2));
rmse_ss  = sqrt(mean(e_dist(tail).^2));
d_min    = min(distances);

fprintf('\n--- %s ---\n', runlabel(RUN_ID));
fprintf('  final distance (last %.0f s) : %8.3f m\n', TAIL_S, d_final);
fprintf('  final error                 : %8.3f m\n', e_final);
fprintf('  settling time (+/- %.2f m)  : %8s s\n', SETTLE_TOL, num2str(t_set,'%.2f'));
fprintf('  minimum separation          : %8.3f m\n', d_min);
fprintf('  RMSE, whole run             : %8.3f m\n', rmse_all);
fprintf('  RMSE, settled tail          : %8.4f m\n', rmse_ss);


%% 4. Figure
fig = figure('Name', 'Distance and distance error', ...
             'Position', [150, 120, 820, 620], 'Color', 'w');
tl = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

blue = [0.00 0.45 0.74];
band = [0.80 1.00 0.80];
xr   = [t(1), t(1), t(end), t(end)];

% ── top panel: distance vs time ──────────────────────────────────
ax1 = nexttile;
hold(ax1, 'on');

fill(ax1, xr, desired_dist + [-SETTLE_TOL, SETTLE_TOL, SETTLE_TOL, -SETTLE_TOL], ...
     band, 'EdgeColor', 'none', 'FaceAlpha', 0.5, ...
     'DisplayName', sprintf('\\pm %.2f m band', SETTLE_TOL));

plot(ax1, t, distances, '-', 'Color', blue, 'LineWidth', 2.5, ...
     'DisplayName', 'UAV--target distance');

yline(ax1, desired_dist, 'k--', sprintf('setpoint %.1f m', desired_dist), ...
      'LineWidth', 2, 'LabelHorizontalAlignment', 'right', ...
      'FontSize', 11, 'FontWeight', 'bold', 'HandleVisibility', 'off');

grid(ax1, 'on');
ylabel(ax1, 'Distance (m)', 'FontWeight', 'bold', 'FontSize', 13);
legend(ax1, 'Location', 'northeast', 'FontSize', 11);
set(ax1, 'FontSize', 11, 'LineWidth', 1.2);
ylim(ax1, [0, max(distances)*1.1 + 0.5]);
xlim(ax1, [t(1) t(end)]);

% ── bottom panel: error about the setpoint ───────────────────────
ax2 = nexttile;
hold(ax2, 'on');

fill(ax2, xr, [-SETTLE_TOL, SETTLE_TOL, SETTLE_TOL, -SETTLE_TOL], ...
     band, 'EdgeColor', 'none', 'FaceAlpha', 0.5, ...
     'DisplayName', sprintf('\\pm %.2f m band', SETTLE_TOL));

plot(ax2, t, e_dist, '-', 'Color', blue, 'LineWidth', 2.5, ...
     'DisplayName', 'Distance error e_{dist}');

yline(ax2, 0, 'k--', 'zero error', 'LineWidth', 2, ...
      'LabelHorizontalAlignment', 'right', 'FontSize', 11, ...
      'FontWeight', 'bold', 'HandleVisibility', 'off');

grid(ax2, 'on');
xlabel(ax2, 'Time (s)', 'FontWeight', 'bold', 'FontSize', 13);
ylabel(ax2, 'Distance error (m)', 'FontWeight', 'bold', 'FontSize', 13);
legend(ax2, 'Location', 'northeast', 'FontSize', 11);
set(ax2, 'FontSize', 11, 'LineWidth', 1.2);

% symmetric-ish limits, but always show the setpoint line and any overshoot
e_span = max(0.5, min(max(abs(e_dist)) * 1.1, max(abs(e_dist)) + 0.5));
ylim(ax2, [-min(e_span, desired_dist + 0.5), e_span]);
xlim(ax2, [t(1) t(end)]);

linkaxes([ax1 ax2], 'x');

if SHOW_TITLE
    title(tl, sprintf('Distance to target and error about the %.1f m setpoint  |  %s', ...
          desired_dist, runlabel(RUN_ID)), ...
          'FontSize', 15, 'FontWeight', 'bold');
end

hold(ax1, 'off'); hold(ax2, 'off');


%% 5. Optional export
if ~isempty(SAVE_PNG)
    exportgraphics(fig, SAVE_PNG, 'Resolution', 300);
    fprintf('  saved -> %s\n', SAVE_PNG);
end


%% ─────────────────────────────────────────────────────────────────
function s = runlabel(id)
switch id
    case 1, s = 'no delay';
    case 2, s = '160 ms delay';
    case 3, s = 'delay + DKF';
    case 4, s = 'delay + Luenberger';
    case 5, s = 'delay + LSTM';
    case 6, s = 'DKF + noise';
    case 7, s = 'Luenberger + noise';
    case 8, s = '160 ms delay + noise';
    case 0, s = 'last run';
    otherwise, s = sprintf('run %d', id);
end
end
