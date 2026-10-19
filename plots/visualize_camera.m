%% visualize_camera.m — Camera view with playback controls
%  Run AFTER the simulation.

if ~exist('RUN_ID_FORCE','var'), close all; clc; end
fprintf('Camera View Visualization\n');
fprintf('========================\n\n');

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
    RUN_ID = 1;               % <<<<<< EDIT THIS LINE
end

if numel(RUN_ID) > 1
    runmany(mfilename, RUN_ID);         % one window per run, then stop
    return;
end

RUN = pickrun(RUN_ID);

%% ── Extract data ───────────────────────────────────────────
[u_data, t_cam] = getsig('cam_u',    RUN);
v_data          = getsig('cam_v',    RUN);
area_data       = getsig('cam_area', RUN);
u_data = u_data(:); v_data = v_data(:); area_data = area_data(:);

nn = min([numel(t_cam), numel(u_data), numel(v_data), numel(area_data)]);
t_cam = t_cam(1:nn); u_data = u_data(1:nn);
v_data = v_data(1:nn); area_data = area_data(1:nn);

N = length(u_data);
fprintf('  Loaded %d samples over %.1f seconds\n\n', N, t_cam(end));

%% ── Downsample ─────────────────────────────────────────────
skip_val = max(1, round(N / 1000));
frame_indices = 1:skip_val:N;
if frame_indices(end) ~= N
    frame_indices(end+1) = N;
end
num_frames = length(frame_indices);

%% ── Camera parameters ──────────────────────────────────────
img_width  = 3280;
img_height = 2464;
cx = img_width / 2;
cy = img_height / 2;

%% ── Create figure ──────────────────────────────────────────
fig = figure('Name', 'Camera View', 'Position', [100, 100, 800, 700], 'Color', 'k');

ax = axes('Parent', fig, 'Position', [0.08 0.22 0.88 0.72]);
set(ax, 'Color', [0.05 0.08 0.05], 'XColor', [0.2 0.8 0.2], 'YColor', [0.2 0.8 0.2]);
hold(ax, 'on');
grid(ax, 'on');
set(ax, 'GridColor', [0.2 0.8 0.2], 'GridAlpha', 0.2);

xlim(ax, [0, img_width]); ylim(ax, [0, img_height]);
set(ax, 'YDir', 'reverse');
xlabel(ax, 'u (px)', 'Color', [0.2 0.8 0.2], 'FontSize', 12, 'FontWeight', 'bold');
ylabel(ax, 'v (px)', 'Color', [0.2 0.8 0.2], 'FontSize', 12, 'FontWeight', 'bold');
title(ax, '▶ TACTICAL UAV TRACKING CAMERA ◀', 'Color', [0.4 1.0 0.4], 'FontSize', 14, 'FontWeight', 'bold');

% Realistic Drone Crosshair (Green HUD)
plot(ax, [cx cx], [0 img_height], '--', 'Color', [0.2 0.6 0.2], 'LineWidth', 0.5);
plot(ax, [0 img_width], [cy cy], '--', 'Color', [0.2 0.6 0.2], 'LineWidth', 0.5);
plot(ax, cx, cy, '+', 'Color', [0.4 1.0 0.4], 'MarkerSize', 30, 'LineWidth', 1.5);
rectangle(ax, 'Position', [0 0 img_width img_height], 'EdgeColor', [0.4 1.0 0.4], 'LineWidth', 2);

% Center brackets
plot(ax, [cx-100 cx-100 cx-80], [cy-80 cy-100 cy-100], 'Color', [0.4 1.0 0.4], 'LineWidth', 2);
plot(ax, [cx+100 cx+100 cx+80], [cy-80 cy-100 cy-100], 'Color', [0.4 1.0 0.4], 'LineWidth', 2);
plot(ax, [cx-100 cx-100 cx-80], [cy+80 cy+100 cy+100], 'Color', [0.4 1.0 0.4], 'LineWidth', 2);
plot(ax, [cx+100 cx+100 cx+80], [cy+80 cy+100 cy+100], 'Color', [0.4 1.0 0.4], 'LineWidth', 2);

% Dynamic elements
target_dot = plot(ax, u_data(1), v_data(1), 'o', 'MarkerFaceColor', [1 0.2 0.2], 'MarkerEdgeColor', [1 0.5 0.5], 'LineWidth', 1.5);
trail = plot(ax, u_data(1), v_data(1), '-', 'Color', [1 0.2 0.2 0.3], 'LineWidth', 1.5);

% HUD Text
time_text = text(ax, 60, 80, 'T+ 0.00 s', 'Color', [0.4 1.0 0.4], 'FontSize', 14, 'FontName', 'Courier', 'FontWeight', 'bold');
err_text  = text(ax, 60, 160, '', 'Color', [0.8 1.0 0.8], 'FontSize', 12, 'FontName', 'Courier');
lock_text = text(ax, img_width-300, 80, 'TRACKING', 'Color', [0.4 1.0 0.4], 'FontSize', 14, 'FontName', 'Courier', 'FontWeight', 'bold');

hold(ax, 'off');

%% ── UI Controls ────────────────────────────────────────────
uicontrol(fig, 'Style', 'text', 'String', 'Time:', ...
    'Position', [20, 95, 50, 20], 'BackgroundColor', 'k', 'ForegroundColor', 'w', 'FontSize', 10, 'HorizontalAlignment', 'right');
h_time_slider = uicontrol(fig, 'Style', 'slider', ...
    'Min', 1, 'Max', num_frames, 'Value', 1, ...
    'Position', [75, 95, 530, 22], 'SliderStep', [1/num_frames, 10/num_frames]);
h_time_label = uicontrol(fig, 'Style', 'text', 'String', 'Time: 0.00 s', ...
    'Position', [610, 92, 180, 25], 'BackgroundColor', 'k', 'ForegroundColor', 'w', ...
    'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

uicontrol(fig, 'Style', 'text', 'String', 'Speed:', ...
    'Position', [20, 58, 50, 20], 'BackgroundColor', 'k', 'ForegroundColor', 'w', 'FontSize', 10, 'HorizontalAlignment', 'right');
h_speed_slider = uicontrol(fig, 'Style', 'slider', ...
    'Min', 0.1, 'Max', 10, 'Value', 1, ...
    'Position', [75, 58, 300, 22], 'SliderStep', [0.05, 0.2]);
h_speed_label = uicontrol(fig, 'Style', 'text', 'String', 'Speed: 1.00x', ...
    'Position', [380, 55, 150, 25], 'BackgroundColor', 'k', 'ForegroundColor', 'w', 'FontSize', 10, 'HorizontalAlignment', 'left');

h_play = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Play', ...
    'Position', [75, 18, 80, 30], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.3 0.8 0.3]);
h_pause = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Pause', ...
    'Position', [165, 18, 80, 30], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.9 0.7 0.2]);
h_reset = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Reset', ...
    'Position', [255, 18, 80, 30], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.7 0.7 0.7]);

%% ── Store shared data ──────────────────────────────────────
app = struct();
app.u_data = u_data;
app.v_data = v_data;
app.area_data = area_data;
app.t_cam = t_cam;
app.frame_indices = frame_indices;
app.num_frames = num_frames;
app.cx = cx; app.cy = cy;
app.target_dot = target_dot;
app.trail = trail;
app.time_text = time_text;
app.err_text = err_text;
app.lock_text = lock_text;
app.h_time_slider = h_time_slider;
app.h_time_label = h_time_label;
app.h_speed_slider = h_speed_slider;
app.h_speed_label = h_speed_label;
app.is_playing = false;
setappdata(fig, 'app', app);

%% ── Callbacks ──────────────────────────────────────────────
set(h_time_slider, 'Callback', @cb_cam_time);
set(h_speed_slider, 'Callback', @cb_cam_speed);
set(h_play,  'Callback', @cb_cam_play);
set(h_pause, 'Callback', @cb_cam_pause);
set(h_reset, 'Callback', @cb_cam_reset);

% Show final frame immediately
update_cam_frame(fig, num_frames);
fprintf('Camera viewer ready. Drag slider or click Play.\n');

%% ════════════════════════════════════════════════════════════

function update_cam_frame(fig, f)
    app = getappdata(fig, 'app');
    k = app.frame_indices(f);
    
    u = app.u_data(k);
    v = app.v_data(k);
    a = app.area_data(k);
    
    % Check if target is out of view (clamped to borders or area 0)
    out_of_view = (u <= 0) || (u >= 3280) || (v <= 0) || (v >= 2464) || (a == 0);
    
    if out_of_view
        set(app.target_dot, 'XData', NaN, 'YData', NaN);
        set(app.lock_text, 'String', 'LOST TARGET', 'Color', [1.0 0.2 0.2]);
    else
        set(app.target_dot, 'XData', u, 'YData', v);
        % REALISTIC TARGET SCALING: Convert pixel area directly to a visual radius
        pixel_radius = sqrt(a / pi);
        dot_size = max(2, pixel_radius * 2);
        set(app.target_dot, 'MarkerSize', dot_size);
        
        px_err = u - app.cx;
        py_err = v - app.cy;
        % Update Lock Status
        if abs(px_err) < 50 && abs(py_err) < 50
            set(app.lock_text, 'String', 'LOCKED', 'Color', [0.2 1.0 0.2]);
        else
            set(app.lock_text, 'String', 'TRACKING', 'Color', [1.0 0.8 0.2]);
        end
    end
    
    trail_start = max(1, k-200);
    % Extract valid trail points (where target is in view)
    u_trail = app.u_data(trail_start:k);
    v_trail = app.v_data(trail_start:k);
    a_trail = app.area_data(trail_start:k);
    valid_mask = (u_trail > 0) & (u_trail < 3280) & (v_trail > 0) & (v_trail < 2464) & (a_trail > 0);
    u_trail(~valid_mask) = NaN;
    v_trail(~valid_mask) = NaN;
    
    set(app.trail, 'XData', u_trail, 'YData', v_trail);
    
    set(app.time_text, 'String', sprintf('T+ %.2f s', app.t_cam(k)));
    px_err = u - app.cx;
    py_err = v - app.cy;
    set(app.err_text, 'String', sprintf('ERR_X: %+5.0f px\nERR_Y: %+5.0f px\nAREA:  %5.0f px^2', px_err, py_err, a));
    
    set(app.h_time_label, 'String', sprintf('Time: %.2f s', app.t_cam(k)));
    set(app.h_time_slider, 'Value', f);
end

function cb_cam_time(src, ~)
    fig = ancestor(src, 'figure');
    update_cam_frame(fig, round(get(src, 'Value')));
end

function cb_cam_speed(src, ~)
    fig = ancestor(src, 'figure');
    app = getappdata(fig, 'app');
    set(app.h_speed_label, 'String', sprintf('Speed: %.2fx', get(src, 'Value')));
end

function cb_cam_pause(src, ~)
    fig = ancestor(src, 'figure');
    app = getappdata(fig, 'app');
    app.is_playing = false;
    setappdata(fig, 'app', app);
end

function cb_cam_reset(src, ~)
    fig = ancestor(src, 'figure');
    app = getappdata(fig, 'app');
    app.is_playing = false;
    setappdata(fig, 'app', app);
    update_cam_frame(fig, 1);
end

function cb_cam_play(src, ~)
    fig = ancestor(src, 'figure');
    app = getappdata(fig, 'app');
    app.is_playing = true;
    setappdata(fig, 'app', app);
    
    f_start = round(get(app.h_time_slider, 'Value'));
    if f_start >= app.num_frames, f_start = 1; end
    
    for f = f_start:app.num_frames
        app = getappdata(fig, 'app');
        if ~app.is_playing || ~ishandle(fig), return; end
        update_cam_frame(fig, f);
        speed = get(app.h_speed_slider, 'Value');
        set(app.h_speed_label, 'String', sprintf('Speed: %.2fx', speed));
        drawnow;
        pause(0.03 / speed);
    end
    app = getappdata(fig, 'app');
    app.is_playing = false;
    setappdata(fig, 'app', app);
end
