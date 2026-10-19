function [d, t] = getsig(name, src)
%% GETSIG — read a logged signal no matter how Simulink stored it.
%
%  One reader for every case, so a plotting script never has to care again:
%    Save format Array / Structure / Structure-with-time / Timeseries
%    variables sitting loose in the base workspace
%    variables inside a Simulink.SimulationOutput (out1 .. out5, ...)
%
%  USAGE
%    [pos, t] = getsig('position');          % from the base workspace
%    [pos, t] = getsig('position', out2);    % from a specific run
%    tgt      = getsig('target_pos', out2);
%
%  RETURNS
%    d  [N x channels] double, always oriented with time down the rows
%    t  [N x 1] time vector. Taken from the signal if it carries one, else
%       from tout in the same source, else reconstructed at 200 Hz.
%
%  WHY THIS EXISTS
%  The plotting scripts used to read X.Time and X.Data directly, which only
%  works for Timeseries format. Switching the To Workspace blocks to Array
%  broke all of them at once. Reading through here means the format is now
%  irrelevant, and so is whether you ran the model from the GUI or through
%  sim().

if nargin < 2, src = []; end

v = fetch_(name, src);
if isempty(v)
    % Say what IS available. "not found" on its own cannot distinguish
    % "you are reading the wrong run" from "that signal was never logged",
    % and those need opposite fixes.
    avail = names_(src);
    if isempty(src), where = 'the base workspace';
    else,            where = 'this run object';
    end
    if isempty(avail), listing = '    (nothing readable found)';
    else,              listing = ['    ' strjoin(sort(avail), ', ')];
    end
    error('getsig:missing', ...
        ['Could not find "%s" in %s.\n\n' ...
         'Available there:\n%s\n\n' ...
         'If that list is empty or looks like the wrong run:\n' ...
         '  - with "Single simulation output" ON, set RUN before calling,\n' ...
         '    e.g.   RUN = out3; plot_distance_error\n' ...
         '  - with it OFF, use  clear RUN  to read the base workspace\n' ...
         '  - otherwise the model has no To Workspace block named "%s"'], ...
        name, where, listing, name);
end

[d, t] = unwrap_(v);

% ---- time base ----------------------------------------------------
if isempty(t)
    tv = fetch_('tout', src);
    if isempty(tv), tv = fetch_('t_log', src); end
    if ~isempty(tv)
        tv = unwrap_(tv);
        t  = double(tv(:));
    end
end
if isempty(t)
    t = (0:size(d,1)-1).' * 0.005;      % 200 Hz, the model's control rate
end
t = double(t(:));

% ---- orientation, decided against time, never guessed ---------------
if size(d,1) ~= numel(t) && size(d,2) == numel(t)
    d = d.';
end

n = min(size(d,1), numel(t));
d = d(1:n, :);
t = t(1:n);
end


%% ─────────────────────────────────────────────────────────────────
function v = fetch_(name, src)
%  Plain dot access in a try/catch. Guarding with isprop/isfield first
%  looks tidier but is wrong: a Simulink.SimulationOutput does not report
%  To Workspace variables through isprop on every release, so the guard
%  rejects signals that are actually right there.
v = [];
if isempty(src)
    if evalin('base', sprintf('exist(''%s'',''var'') == 1', name))
        v = evalin('base', name);
    end
    return;
end
try
    v = src.(name);
catch
    v = [];
end
end


%% ─────────────────────────────────────────────────────────────────
function n = names_(src)
%  Best-effort list of what a source actually holds, for error messages.
n = {};
if isempty(src)
    try, n = evalin('base', 'who'); catch, end
    return;
end
for f = {@() src.who, @() fieldnames(src), @() properties(src)}
    try
        c = f{1}();
        if iscell(c) && ~isempty(c), n = c(:).'; return; end
    catch
    end
end
end


%% ─────────────────────────────────────────────────────────────────
function [d, t] = unwrap_(v)
t = [];
if isa(v, 'timeseries')
    d = double(squeeze(v.Data));
    t = v.Time;
elseif isnumeric(v) || islogical(v)
    d = double(squeeze(v));             % squeeze: an Array-format 3-vector
                                        % arrives 3x1xN and must collapse
elseif isstruct(v) && isfield(v, 'signals')
    d = double(squeeze(v.signals.values));
    if isfield(v, 'time') && ~isempty(v.time), t = v.time; end
elseif isstruct(v) && isfield(v, 'Data')
    d = double(squeeze(v.Data));
    if isfield(v, 'Time') && ~isempty(v.Time), t = v.Time; end
else
    error('getsig:format', 'Cannot interpret a signal of class %s.', class(v));
end
if isvector(d), d = d(:); end
end
