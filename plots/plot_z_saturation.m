%% plot_z_saturation.m
% Plots the Z-Saturation (Thrust or Z-Velocity command limit) over time.
% Proves the altitude controller safely respects its maximum physical limits.

if ~exist('RUN_ID_FORCE','var'), close all; end


%% ══ WHICH RUN TO PLOT ═══════════════════════════════════════════
%  Edit the numbers on the RUN_ID line below, then press Run.
%      1 = no delay        2 = 160 ms delay      3 = delay + DKF
%      4 = delay + Luenberger                  5 = delay + LSTM
%      0 = whatever the last simulation left in the workspace
%  ONE number  -> one figure.       RUN_ID = 3;
%  SEVERAL     -> one window per run, each labelled.   RUN_ID = [1 2 3 4 5];
if exist('RUN_ID_FORCE', 'var')
    RUN_ID = RUN_ID_FORCE;              % set by runmany - do not edit
else
    RUN_ID = [1 2 3 4 5];               % <<<<<< EDIT THIS LINE
end

if numel(RUN_ID) > 1
    runmany(mfilename, RUN_ID);         % one window per run, then stop
    return;
end

RUN = pickrun(RUN_ID);

%% 1. Extract Data
[z_sat_data, t] = getsig('z_sat_log', RUN);
z_sat_data = z_sat_data(:);


%% 2. Create Figure
fig = figure('Name', 'Presentation Results: Z-Saturation', ...
    'Position', [250, 250, 800, 400], 'Color', 'w');
hold on;

% Plot the saturated signal
plot(t, z_sat_data, '-', 'Color', [0.85 0.33 0.10], 'LineWidth', 2.5, 'DisplayName', 'Z Variation');

max_val = max(z_sat_data);

grid on;
xlabel('Time (s)', 'FontWeight', 'bold', 'FontSize', 13);
ylabel('Z Variation', 'FontWeight', 'bold', 'FontSize', 13);
title('Altitude Controller: Z Variation vs Time', 'FontSize', 16, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);

% Fix the Y-axis padding
ylim([0, max_val + 0.5]);

% Aesthetics
set(gca, 'FontSize', 12, 'LineWidth', 1.2);
hold off;

fprintf('Created Z-Saturation Plot! Save this figure for your slides.\n');
