%% visualize_combined.m — Combined 3D Trajectory & Camera View
%  Run AFTER the simulation has completed.

if ~exist('RUN_ID_FORCE','var'), close all; clc; end
fprintf('Combined 3D Trajectory & Camera View\n');
fprintf('====================================\n\n');

%% ── 1. Check Data for 3D Trajectory ─────────────────────────
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
    RUN_ID = 3;                   % <<<<<< EDIT THIS LINE
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

%% ── 2. Camera signals ───────────────────────────────────────
u_data    = getsig('cam_u',    RUN);
v_data    = getsig('cam_v',    RUN);
area_data = getsig('cam_area', RUN);
u_data = u_data(:); v_data = v_data(:); area_data = area_data(:);

N = min(length(t_pos), length(u_data));
fprintf('  Loaded %d common samples over %.1f seconds\n\n', N, t_pos(N));

%% ── 3. Downsample ───────────────────────────────────────────
skip_val = max(1, floor(N / 1000));
frame_indices = 1:skip_val:N;
if frame_indices(end) ~= N
    frame_indices(end+1) = N;
end
num_frames = length(frame_indices);

%% ── 4. Camera & 3D Parameters ───────────────────────────────
img_width  = 3280;
img_height = 2464;
cx = img_width / 2;
cy = img_height / 2;
fov_h_deg  = 160;           
fov_half   = fov_h_deg / 2; 
cone_len   = 5.0;           

%% ── 5. Axis limits (3D) ─────────────────────────────────────
all_pts = [pos_data(1:N,:); target_data(1:N,:)];
x_lim = [min(all_pts(:,1))-2, max(all_pts(:,1))+2];
y_lim = [min(all_pts(:,2))-2, max(all_pts(:,2))+2];
z_lim = [min(all_pts(:,3))-2, max(all_pts(:,3))+2];
if diff(x_lim) < 1, x_lim = x_lim + [-2, 2]; end
if diff(y_lim) < 1, y_lim = y_lim + [-2, 2]; end
if diff(z_lim) < 1, z_lim = z_lim + [-2, 2]; end

%% ── 6. Create Figure and Subplots ───────────────────────────
fig = figure('Name', 'Dual View: 3D Trajectory & Camera', 'Position', [50, 50, 1400, 650], 'Color', [0.1 0.1 0.1]);

% ---- Left Axes (3D View) ----
ax3d = axes('Parent', fig, 'Position', [0.03 0.22 0.45 0.72]);
set(ax3d, 'Color', 'w');
hold(ax3d, 'on');

plot3(ax3d, pos_data(1,1), pos_data(1,2), pos_data(1,3), 'o', 'Color', [0.13 0.55 0.13], 'MarkerSize', 14, 'MarkerFaceColor', [0.13 0.55 0.13]);
plot3(ax3d, target_data(:,1), target_data(:,2), target_data(:,3), '--', 'Color', [0.75 0.55 0.75], 'LineWidth', 1);

h_uav_trail = plot3(ax3d, NaN,NaN,NaN,'-','Color',[0.00 0.45 0.74],'LineWidth',2.5);
h_tgt_trail = plot3(ax3d, NaN,NaN,NaN,'-','Color',[0.85 0.33 0.10],'LineWidth',2);
h_uav_pos   = plot3(ax3d, NaN,NaN,NaN,'o','Color',[0.15 0.15 0.15], 'MarkerSize',14,'MarkerFaceColor',[0.15 0.15 0.15],'LineWidth',2);
h_tgt_pos   = plot3(ax3d, NaN,NaN,NaN,'o','Color',[0.85 0.12 0.12], 'MarkerSize',14,'MarkerFaceColor',[0.85 0.12 0.12],'LineWidth',2);
h_los       = plot3(ax3d, NaN,NaN,NaN,'-','Color',[0.49 0.18 0.56],'LineWidth',1.5);
h_fov_L     = plot3(ax3d, NaN,NaN,NaN,'-','Color',[0.5 0.9 0.9],'LineWidth',1.5);
h_fov_R     = plot3(ax3d, NaN,NaN,NaN,'-','Color',[0.5 0.9 0.9],'LineWidth',1.5);
h_fov_patch = fill3(ax3d, [NaN NaN NaN], [NaN NaN NaN], [NaN NaN NaN], [0.5 0.9 0.9], 'FaceAlpha', 0.4, 'EdgeColor', 'none');

xlabel(ax3d, 'X (m)', 'FontWeight', 'bold'); ylabel(ax3d, 'Y (m)', 'FontWeight', 'bold'); zlabel(ax3d, 'Z (m)', 'FontWeight', 'bold');
title(ax3d, 'Global 3D View', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
xlim(ax3d, x_lim); ylim(ax3d, y_lim); zlim(ax3d, z_lim);
grid(ax3d, 'on'); view(ax3d, 30, 25);
hold(ax3d, 'off');

% ---- Right Axes (Camera View) ----
axcam = axes('Parent', fig, 'Position', [0.52 0.22 0.45 0.72]);
set(axcam, 'Color', [0.05 0.08 0.05], 'XColor', [0.2 0.8 0.2], 'YColor', [0.2 0.8 0.2]);
hold(axcam, 'on');
grid(axcam, 'on'); set(axcam, 'GridColor', [0.2 0.8 0.2], 'GridAlpha', 0.2);
xlim(axcam, [0, img_width]); ylim(axcam, [0, img_height]); set(axcam, 'YDir', 'reverse');
xlabel(axcam, 'u (px)', 'Color', [0.2 0.8 0.2], 'FontWeight', 'bold');
ylabel(axcam, 'v (px)', 'Color', [0.2 0.8 0.2], 'FontWeight', 'bold');
title(axcam, '▶ TACTICAL UAV TRACKING CAMERA ◀', 'Color', [0.4 1.0 0.4], 'FontSize', 14, 'FontWeight', 'bold');

% Crosshair and borders
plot(axcam, [cx cx], [0 img_height], '--', 'Color', [0.2 0.6 0.2], 'LineWidth', 0.5);
plot(axcam, [0 img_width], [cy cy], '--', 'Color', [0.2 0.6 0.2], 'LineWidth', 0.5);
plot(axcam, cx, cy, '+', 'Color', [0.4 1.0 0.4], 'MarkerSize', 30, 'LineWidth', 1.5);
rectangle(axcam, 'Position', [0 0 img_width img_height], 'EdgeColor', [0.4 1.0 0.4], 'LineWidth', 2);
plot(axcam, [cx-100 cx-100 cx-80], [cy-80 cy-100 cy-100], 'Color', [0.4 1.0 0.4], 'LineWidth', 2);
plot(axcam, [cx+100 cx+100 cx+80], [cy-80 cy-100 cy-100], 'Color', [0.4 1.0 0.4], 'LineWidth', 2);
plot(axcam, [cx-100 cx-100 cx-80], [cy+80 cy+100 cy+100], 'Color', [0.4 1.0 0.4], 'LineWidth', 2);
plot(axcam, [cx+100 cx+100 cx+80], [cy+80 cy+100 cy+100], 'Color', [0.4 1.0 0.4], 'LineWidth', 2);

cam_trail  = plot(axcam, NaN, NaN, '-', 'Color', [1 0.2 0.2 0.3], 'LineWidth', 1.5);
target_dot = plot(axcam, NaN, NaN, 'o', 'MarkerFaceColor', [1 0.2 0.2], 'MarkerEdgeColor', [1 0.5 0.5], 'LineWidth', 1.5);

cam_time_text = text(axcam, 60, 80, 'T+ 0.00 s', 'Color', [0.4 1.0 0.4], 'FontSize', 14, 'FontName', 'Courier', 'FontWeight', 'bold');
cam_err_text  = text(axcam, 60, 160, '', 'Color', [0.8 1.0 0.8], 'FontSize', 12, 'FontName', 'Courier');
cam_lock_text = text(axcam, img_width-300, 80, 'TRACKING', 'Color', [0.4 1.0 0.4], 'FontSize', 14, 'FontName', 'Courier', 'FontWeight', 'bold');
hold(axcam, 'off');

%% ── 7. Shared UI Controls ───────────────────────────────────
uicontrol(fig, 'Style', 'text', 'String', 'Time:', 'Position', [20, 100, 50, 20], 'BackgroundColor', [0.1 0.1 0.1], 'ForegroundColor', 'w', 'FontSize', 10, 'HorizontalAlignment', 'right');
h_time_slider = uicontrol(fig, 'Style', 'slider', 'Min', 1, 'Max', num_frames, 'Value', 1, 'Position', [75, 100, 1100, 22], 'SliderStep', [1/num_frames, 10/num_frames]);
h_time_label = uicontrol(fig, 'Style', 'text', 'String', 'Time: 0.00 s', 'Position', [1180, 97, 200, 25], 'BackgroundColor', [0.1 0.1 0.1], 'ForegroundColor', 'w', 'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');

uicontrol(fig, 'Style', 'text', 'String', 'Speed:', 'Position', [20, 60, 50, 20], 'BackgroundColor', [0.1 0.1 0.1], 'ForegroundColor', 'w', 'FontSize', 10, 'HorizontalAlignment', 'right');
h_speed_slider = uicontrol(fig, 'Style', 'slider', 'Min', 0.1, 'Max', 10, 'Value', 1, 'Position', [75, 60, 350, 22], 'SliderStep', [0.05, 0.2]);
h_speed_label = uicontrol(fig, 'Style', 'text', 'String', 'Speed: 1.00x', 'Position', [430, 57, 200, 25], 'BackgroundColor', [0.1 0.1 0.1], 'ForegroundColor', 'w', 'FontSize', 10, 'HorizontalAlignment', 'left');

h_play  = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Play', 'Position', [75, 20, 80, 30], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.3 0.8 0.3]);
h_pause = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Pause', 'Position', [165, 20, 80, 30], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.9 0.7 0.2]);
h_reset = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Reset', 'Position', [255, 20, 80, 30], 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [0.7 0.7 0.7]);

%% ── 8. Store AppData ────────────────────────────────────────
app = struct();
app.pos_data = pos_data; app.target_data = target_data; app.t_pos = t_pos;
app.u_data = u_data; app.v_data = v_data; app.area_data = area_data;
app.frame_indices = frame_indices; app.num_frames = num_frames;
app.h_uav_trail = h_uav_trail; app.h_tgt_trail = h_tgt_trail;
app.h_uav_pos = h_uav_pos; app.h_tgt_pos = h_tgt_pos;
app.h_los = h_los; app.h_fov_L = h_fov_L; app.h_fov_R = h_fov_R; app.h_fov_patch = h_fov_patch;
app.cx = cx; app.cy = cy; app.target_dot = target_dot; app.cam_trail = cam_trail;
app.cam_time_text = cam_time_text; app.cam_err_text = cam_err_text; app.cam_lock_text = cam_lock_text;
app.h_time_slider = h_time_slider; app.h_time_label = h_time_label;
app.h_speed_slider = h_speed_slider; app.h_speed_label = h_speed_label;
app.is_playing = false; app.has_euler = has_euler;
if has_euler, app.euler_data = euler_data; end
app.fov_half = fov_half; app.cone_len = cone_len;
setappdata(fig, 'app', app);

%% ── 9. Callbacks ────────────────────────────────────────────
set(h_time_slider, 'Callback', @cb_time_slider);
set(h_speed_slider, 'Callback', @cb_speed_slider);
set(h_play,  'Callback', @cb_play);
set(h_pause, 'Callback', @cb_pause);
set(h_reset, 'Callback', @cb_reset);

% Show final frame immediately
update_combined_frame(fig, num_frames);
fprintf('Dual viewer ready. Drag slider or click Play.\n');

%% ════════════════════════════════════════════════════════════
function update_combined_frame(fig, f)
    app = getappdata(fig, 'app');
    k = app.frame_indices(f);
    
    % --- Update 3D View ---
    set(app.h_uav_trail, 'XData', app.pos_data(1:k,1), 'YData', app.pos_data(1:k,2), 'ZData', app.pos_data(1:k,3));
    set(app.h_tgt_trail, 'XData', app.target_data(1:k,1), 'YData', app.target_data(1:k,2), 'ZData', app.target_data(1:k,3));
    set(app.h_uav_pos, 'XData', app.pos_data(k,1), 'YData', app.pos_data(k,2), 'ZData', app.pos_data(k,3));
    set(app.h_tgt_pos, 'XData', app.target_data(k,1), 'YData', app.target_data(k,2), 'ZData', app.target_data(k,3));
    
    ux = app.pos_data(k,1); uy = app.pos_data(k,2); uz = app.pos_data(k,3);
    tx = app.target_data(k,1); ty = app.target_data(k,2); tz = app.target_data(k,3);
    set(app.h_los, 'XData', [ux tx], 'YData', [uy ty], 'ZData', [uz tz]);
    
    if app.has_euler
        psi = app.euler_data(k, 3);
        ang_L = psi + deg2rad(app.fov_half); Lx = ux + app.cone_len * cos(ang_L); Ly = uy + app.cone_len * sin(ang_L);
        ang_R = psi - deg2rad(app.fov_half); Rx = ux + app.cone_len * cos(ang_R); Ry = uy + app.cone_len * sin(ang_R);
        set(app.h_fov_L, 'XData', [ux Lx], 'YData', [uy Ly], 'ZData', [uz uz]);
        set(app.h_fov_R, 'XData', [ux Rx], 'YData', [uy Ry], 'ZData', [uz uz]);
        arc_angles = linspace(ang_R, ang_L, 20);
        set(app.h_fov_patch, 'XData', [ux, ux + app.cone_len * cos(arc_angles), ux], ...
            'YData', [uy, uy + app.cone_len * sin(arc_angles), uy], 'ZData', ones(1,22) * uz);
    end
    
    dist = sqrt((tx-ux)^2 + (ty-uy)^2 + (tz-uz)^2);
    set(app.h_time_label, 'String', sprintf('Time: %.2f s | dist: %.2f m', app.t_pos(k), dist));
    
    % --- Update Camera View ---
    u = app.u_data(k); v = app.v_data(k); a = app.area_data(k);
    out_of_view = (u <= 0) || (u >= 3280) || (v <= 0) || (v >= 2464) || (a == 0);
    
    if out_of_view
        set(app.target_dot, 'XData', NaN, 'YData', NaN);
        set(app.cam_lock_text, 'String', 'LOST TARGET', 'Color', [1.0 0.2 0.2]);
    else
        set(app.target_dot, 'XData', u, 'YData', v, 'MarkerSize', max(2, sqrt(a/pi)*2));
        if abs(u - app.cx) < 50 && abs(v - app.cy) < 50
            set(app.cam_lock_text, 'String', 'LOCKED', 'Color', [0.2 1.0 0.2]);
        else
            set(app.cam_lock_text, 'String', 'TRACKING', 'Color', [1.0 0.8 0.2]);
        end
    end
    
    trail_start = max(1, k-200);
    u_tr = app.u_data(trail_start:k); v_tr = app.v_data(trail_start:k); a_tr = app.area_data(trail_start:k);
    mask = (u_tr > 0) & (u_tr < 3280) & (v_tr > 0) & (v_tr < 2464) & (a_tr > 0);
    u_tr(~mask) = NaN; v_tr(~mask) = NaN;
    set(app.cam_trail, 'XData', u_tr, 'YData', v_tr);
    
    set(app.cam_time_text, 'String', sprintf('T+ %.2f s', app.t_pos(k)));
    set(app.cam_err_text, 'String', sprintf('ERR_X: %+5.0f px\nERR_Y: %+5.0f px\nAREA:  %5.0f px^2', u - app.cx, v - app.cy, a));
    
    set(app.h_time_slider, 'Value', f);
end

function cb_time_slider(src, ~), fig = ancestor(src, 'figure'); update_combined_frame(fig, round(get(src, 'Value'))); end
function cb_speed_slider(src, ~), fig = ancestor(src, 'figure'); app = getappdata(fig, 'app'); set(app.h_speed_label, 'String', sprintf('Speed: %.2fx', get(src, 'Value'))); end
function cb_pause(src, ~), fig = ancestor(src, 'figure'); app = getappdata(fig, 'app'); app.is_playing = false; setappdata(fig, 'app', app); end
function cb_reset(src, ~), fig = ancestor(src, 'figure'); app = getappdata(fig, 'app'); app.is_playing = false; setappdata(fig, 'app', app); update_combined_frame(fig, 1); end

function cb_play(src, ~)
    fig = ancestor(src, 'figure');
    app = getappdata(fig, 'app');
    app.is_playing = true;
    setappdata(fig, 'app', app);
    
    f_start = round(get(app.h_time_slider, 'Value'));
    if f_start >= app.num_frames, f_start = 1; end
    
    for f = f_start:app.num_frames
        app = getappdata(fig, 'app');
        if ~app.is_playing || ~ishandle(fig), return; end
        
        update_combined_frame(fig, f);
        speed = get(app.h_speed_slider, 'Value');
        drawnow;
        pause(0.03 / speed);
    end
    app = getappdata(fig, 'app');
    app.is_playing = false;
    setappdata(fig, 'app', app);
end
