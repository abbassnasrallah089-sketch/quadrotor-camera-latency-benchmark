%% visualize_3d_trajectory.m — 3D UAV pursuit with playback controls
%  Run AFTER the simulation has completed.

if ~exist('RUN_ID_FORCE','var'), close all; end

%% ── Check data ─────────────────────────────────────────────
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
    RUN_ID =[1 6 7];                   % <<<<<< EDIT THIS LINE 
end

if numel(RUN_ID) > 1
    runmany(mfilename, RUN_ID);         % one window per run, then stop
    return;
end

RUN = pickrun(RUN_ID);

%% ── Trajectory data ────────────────────────────────────────
%  Read through getsig so the To Workspace Save format (Array, Timeseries,
%  Structure) no longer matters, and so a specific run can be selected.
[pos_data, t_pos] = getsig('position', RUN);

try
    target_data = getsig('target_pos', RUN);
catch
    warning('target_pos not found - the target path will be left blank.');
    target_data = nan(numel(t_pos), 3);
end

%% ── Euler angles (for the camera FOV cone) ─────────────────
%  euler_log is checked FIRST because that is what these models actually
%  log. The original list looked only for euler_angles / euler, neither of
%  which exists here, so the FOV cone was silently never drawn.
has_euler  = false;
euler_data = [];
for nm = {'euler_log', 'euler_angles', 'euler', 'log_euler'}
    try
        euler_data = getsig(nm{1}, RUN);
        has_euler  = true;
        break;
    catch
    end
end
if ~has_euler
    warning('No euler angles found (tried euler_log, euler_angles, euler, log_euler). FOV cone will not be drawn.');
end

%% ── Put everything on one common length ────────────────────
nn = min([size(pos_data,1), size(target_data,1), numel(t_pos)]);
if has_euler, nn = min(nn, size(euler_data,1)); end
t_pos       = t_pos(1:nn);
pos_data    = pos_data(1:nn, :);
target_data = target_data(1:nn, :);
if has_euler, euler_data = euler_data(1:nn, :); end

%% ── Camera parameters ──────────────────────────────────────
fov_h_deg  = 160;           % horizontal FOV in degrees
fov_half   = fov_h_deg / 2; % half-angle = 80°
cone_len   = 5.0;           % length of FOV cone (meters)

%% ── Downsample ─────────────────────────────────────────────
N = length(t_pos);
skip_val = max(1, floor(N / 1000));
frame_indices = 1:skip_val:N;
if frame_indices(end) ~= N
    frame_indices(end+1) = N;
end
num_frames = length(frame_indices);

%% ── Axis limits ────────────────────────────────────────────
all_pts = [pos_data; target_data];
x_lim = [min(all_pts(:,1))-2, max(all_pts(:,1))+2];
y_lim = [min(all_pts(:,2))-2, max(all_pts(:,2))+2];
z_lim = [min(all_pts(:,3))-2, max(all_pts(:,3))+2];
if diff(x_lim) < 1, x_lim = x_lim + [-2, 2]; end
if diff(y_lim) < 1, y_lim = y_lim + [-2, 2]; end
if diff(z_lim) < 1, z_lim = z_lim + [-2, 2]; end

%% ── Create figure ──────────────────────────────────────────
fig = figure('Name', 'UAV Pursuit - 3D Trajectory', ...
    'Position', [80, 80, 950, 780], 'Color', 'w');

ax = axes('Parent', fig, 'Position', [0.18 0.22 0.78 0.72]);
hold(ax, 'on');

%% ── Legend dummies (named, distinct colors) ────────────────
% We only want these in the legend!
h_leg = [];
h_leg(1) = plot3(ax, NaN,NaN,NaN,'o','Color',[0.13 0.55 0.13],'MarkerSize',14, ...
    'MarkerFaceColor',[0.13 0.55 0.13],'DisplayName','Initial Point');
h_leg(2) = plot3(ax, NaN,NaN,NaN,'o','Color',[0.15 0.15 0.15],'MarkerSize',12, ...
    'MarkerFaceColor',[0.15 0.15 0.15],'DisplayName','Current UAV Position');
h_leg(3) = plot3(ax, NaN,NaN,NaN,'o','Color',[0.85 0.12 0.12],'MarkerSize',12, ...
    'MarkerFaceColor',[0.85 0.12 0.12],'DisplayName','Current Target Position');
h_leg(4) = plot3(ax, NaN,NaN,NaN,'-','Color',[0.00 0.45 0.74],'LineWidth',2, ...
    'DisplayName','UAV Trajectory');
h_leg(5) = plot3(ax, NaN,NaN,NaN,'-','Color',[0.85 0.33 0.10],'LineWidth',2, ...
    'DisplayName','Target Trajectory');
h_leg(6) = plot3(ax, NaN,NaN,NaN,'--','Color',[0.75 0.55 0.75],'LineWidth',1, ...
    'DisplayName','Planned Target Path');
h_leg(7) = plot3(ax, NaN,NaN,NaN,'-','Color',[0.49 0.18 0.56],'LineWidth',1.5, ...
    'DisplayName','LOS Vector');
% Cyan patch for FOV in legend
h_leg(8) = fill3(ax, NaN,NaN,NaN, [0.5 0.9 0.9], 'FaceAlpha', 0.5, 'EdgeColor', 'none', ...
    'DisplayName','Camera FOV');

%% ── Initial point ──────────────────────────────────────────
plot3(ax, pos_data(1,1), pos_data(1,2), pos_data(1,3), 'o', ...
    'Color', [0.13 0.55 0.13], 'MarkerSize', 16, ...
    'MarkerFaceColor', [0.13 0.55 0.13], 'LineWidth', 2, 'HandleVisibility', 'off');

%% ── Faded target path (full planned trajectory) ────────────
plot3(ax, target_data(:,1), target_data(:,2), target_data(:,3), '--', ...
    'Color', [0.75 0.55 0.75], 'LineWidth', 1, 'HandleVisibility', 'off');

%% ── Dynamic elements ──────────────────────────────────────
% IMPORTANT: HandleVisibility = 'off' prevents these from showing up in the legend!
h_uav_trail = plot3(ax, NaN,NaN,NaN,'-','Color',[0.00 0.45 0.74],'LineWidth',2.5, 'HandleVisibility', 'off');
h_tgt_trail = plot3(ax, NaN,NaN,NaN,'-','Color',[0.85 0.33 0.10],'LineWidth',2, 'HandleVisibility', 'off');
h_uav_pos   = plot3(ax, NaN,NaN,NaN,'o','Color',[0.15 0.15 0.15], ...
    'MarkerSize',14,'MarkerFaceColor',[0.15 0.15 0.15],'LineWidth',2, 'HandleVisibility', 'off');
h_tgt_pos   = plot3(ax, NaN,NaN,NaN,'o','Color',[0.85 0.12 0.12], ...
    'MarkerSize',14,'MarkerFaceColor',[0.85 0.12 0.12],'LineWidth',2, 'HandleVisibility', 'off');
h_los       = plot3(ax, NaN,NaN,NaN,'-','Color',[0.49 0.18 0.56],'LineWidth',1.5, 'HandleVisibility', 'off');

% Camera FOV cone
h_fov_L = plot3(ax, NaN,NaN,NaN,'-','Color',[0.5 0.9 0.9],'LineWidth',1.5, 'HandleVisibility', 'off');
h_fov_R = plot3(ax, NaN,NaN,NaN,'-','Color',[0.5 0.9 0.9],'LineWidth',1.5, 'HandleVisibility', 'off');
% Filled FOV sector (transparent cyan patch)
h_fov_patch = fill3(ax, [NaN NaN NaN], [NaN NaN NaN], [NaN NaN NaN], ...
    [0.5 0.9 0.9], 'FaceAlpha', 0.4, 'EdgeColor', 'none', 'HandleVisibility', 'off');

xlabel(ax, 'X (m)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel(ax, 'Y (m)', 'FontSize', 13, 'FontWeight', 'bold');
zlabel(ax, 'Z (m)', 'FontSize', 13, 'FontWeight', 'bold');
title(ax, 'Global View', 'FontSize', 16, 'FontWeight', 'bold');
xlim(ax, x_lim); ylim(ax, y_lim); zlim(ax, z_lim);
grid(ax, 'on'); view(ax, 30, 25);

% ONLY include the specific handles we created for the legend
legend(ax, h_leg, 'Location', 'northwestoutside', 'FontSize', 9);
hold(ax, 'off');

%% ── UI Controls ────────────────────────────────────────────
uicontrol(fig, 'Style', 'text', 'String', 'Time:', ...
    'Position', [20, 100, 50, 20], 'BackgroundColor', 'w', 'FontSize', 10, 'HorizontalAlignment', 'right');
h_time_slider = uicontrol(fig, 'Style', 'slider', ...
    'Min', 1, 'Max', num_frames, 'Value', 1, ...
    'Position', [75, 100, 650, 22], 'SliderStep', [1/num_frames, 10/num_frames]);
h_time_label = uicontrol(fig, 'Style', 'text', 'String', 'Time: 0.00 s', ...
    'Position', [730, 97, 200, 25], 'BackgroundColor', 'w', ...
    'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

uicontrol(fig, 'Style', 'text', 'String', 'Speed:', ...
    'Position', [20, 60, 50, 20], 'BackgroundColor', 'w', 'FontSize', 10, 'HorizontalAlignment', 'right');
h_speed_slider = uicontrol(fig, 'Style', 'slider', ...
    'Min', 0.1, 'Max', 10, 'Value', 1, ...
    'Position', [75, 60, 350, 22], 'SliderStep', [0.05, 0.2]);
h_speed_label = uicontrol(fig, 'Style', 'text', 'String', 'Speed: 1.00x', ...
    'Position', [430, 57, 200, 25], 'BackgroundColor', 'w', 'FontSize', 10, 'HorizontalAlignment', 'left');

h_play = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Play', ...
    'Position', [75, 20, 80, 30], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.3 0.8 0.3]);
h_pause = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Pause', ...
    'Position', [165, 20, 80, 30], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.9 0.7 0.2]);
h_reset = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Reset', ...
    'Position', [255, 20, 80, 30], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.7 0.7 0.7]);

%% ── Store all shared data in figure's AppData ──────────────
app = struct();
app.pos_data = pos_data;
app.target_data = target_data;
app.t_pos = t_pos;
app.frame_indices = frame_indices;
app.num_frames = num_frames;
app.h_uav_trail = h_uav_trail;
app.h_tgt_trail = h_tgt_trail;
app.h_uav_pos = h_uav_pos;
app.h_tgt_pos = h_tgt_pos;
app.h_los = h_los;
app.h_fov_L = h_fov_L;
app.h_fov_R = h_fov_R;
app.h_fov_patch = h_fov_patch;
app.h_time_slider = h_time_slider;
app.h_time_label = h_time_label;
app.h_speed_slider = h_speed_slider;
app.h_speed_label = h_speed_label;
app.is_playing = false;
app.has_euler = has_euler;
if has_euler
    app.euler_data = euler_data;
end
app.fov_half = fov_half;
app.cone_len = cone_len;
setappdata(fig, 'app', app);

%% ── Callbacks ──────────────────────────────────────────────
set(h_time_slider, 'Callback', @cb_time_slider);
set(h_speed_slider, 'Callback', @cb_speed_slider);
set(h_play,  'Callback', @cb_play);
set(h_pause, 'Callback', @cb_pause);
set(h_reset, 'Callback', @cb_reset);

% Show final frame immediately
update_3d_frame(fig, num_frames);

fprintf('3D Trajectory viewer ready.\n');
fprintf('  Drag the time slider to scrub. Click Play to animate.\n');

%% ════════════════════════════════════════════════════════════
%  CALLBACK FUNCTIONS
%% ════════════════════════════════════════════════════════════

function update_3d_frame(fig, f)
    app = getappdata(fig, 'app');
    k = app.frame_indices(f);
    
    % Update trails
    set(app.h_uav_trail, 'XData', app.pos_data(1:k,1), 'YData', app.pos_data(1:k,2), 'ZData', app.pos_data(1:k,3));
    set(app.h_tgt_trail, 'XData', app.target_data(1:k,1), 'YData', app.target_data(1:k,2), 'ZData', app.target_data(1:k,3));
    
    % Update current positions
    set(app.h_uav_pos, 'XData', app.pos_data(k,1), 'YData', app.pos_data(k,2), 'ZData', app.pos_data(k,3));
    set(app.h_tgt_pos, 'XData', app.target_data(k,1), 'YData', app.target_data(k,2), 'ZData', app.target_data(k,3));
    
    % Update LOS vector
    ux = app.pos_data(k,1); uy = app.pos_data(k,2); uz = app.pos_data(k,3);
    tx = app.target_data(k,1); ty = app.target_data(k,2); tz = app.target_data(k,3);
    set(app.h_los, 'XData', [ux tx], 'YData', [uy ty], 'ZData', [uz tz]);
    
    % Update camera FOV cone
    if app.has_euler
        psi = app.euler_data(k, 3);   % yaw angle
        
        % Left edge of FOV (yaw + half_fov)
        ang_L = psi + deg2rad(app.fov_half);
        Lx = ux + app.cone_len * cos(ang_L);
        Ly = uy + app.cone_len * sin(ang_L);
        
        % Right edge of FOV (yaw - half_fov)
        ang_R = psi - deg2rad(app.fov_half);
        Rx = ux + app.cone_len * cos(ang_R);
        Ry = uy + app.cone_len * sin(ang_R);
        
        set(app.h_fov_L, 'XData', [ux Lx], 'YData', [uy Ly], 'ZData', [uz uz]);
        set(app.h_fov_R, 'XData', [ux Rx], 'YData', [uy Ry], 'ZData', [uz uz]);
        
        % Filled FOV sector (triangle: drone → left curve → right → drone)
        n_arc = 20;
        arc_angles = linspace(ang_R, ang_L, n_arc);
        patch_x = [ux, ux + app.cone_len * cos(arc_angles), ux];
        patch_y = [uy, uy + app.cone_len * sin(arc_angles), uy];
        patch_z = ones(size(patch_x)) * uz;
        set(app.h_fov_patch, 'XData', patch_x, 'YData', patch_y, 'ZData', patch_z);
    end
    
    % Update time label
    dist = sqrt((tx-ux)^2 + (ty-uy)^2 + (tz-uz)^2);
    set(app.h_time_label, 'String', sprintf('Time: %.2f s | dist: %.2f m', app.t_pos(k), dist));
    set(app.h_time_slider, 'Value', f);
end

function cb_time_slider(src, ~)
    fig = ancestor(src, 'figure');
    f = round(get(src, 'Value'));
    update_3d_frame(fig, f);
end

function cb_speed_slider(src, ~)
    fig = ancestor(src, 'figure');
    app = getappdata(fig, 'app');
    set(app.h_speed_label, 'String', sprintf('Speed: %.2fx', get(src, 'Value')));
end

function cb_pause(src, ~)
    fig = ancestor(src, 'figure');
    app = getappdata(fig, 'app');
    app.is_playing = false;
    setappdata(fig, 'app', app);
end

function cb_reset(src, ~)
    fig = ancestor(src, 'figure');
    app = getappdata(fig, 'app');
    app.is_playing = false;
    setappdata(fig, 'app', app);
    update_3d_frame(fig, 1);
end

function cb_play(src, ~)
    fig = ancestor(src, 'figure');
    app = getappdata(fig, 'app');
    app.is_playing = true;
    setappdata(fig, 'app', app);
    
    f_start = round(get(app.h_time_slider, 'Value'));
    if f_start >= app.num_frames
        f_start = 1;
    end
    
    for f = f_start:app.num_frames
        app = getappdata(fig, 'app');
        if ~app.is_playing || ~ishandle(fig)
            return;
        end
        
        update_3d_frame(fig, f);
        
        speed = get(app.h_speed_slider, 'Value');
        set(app.h_speed_label, 'String', sprintf('Speed: %.2fx', speed));
        drawnow;
        pause(0.03 / speed);
    end
    
    app = getappdata(fig, 'app');
    app.is_playing = false;
    setappdata(fig, 'app', app);
end
