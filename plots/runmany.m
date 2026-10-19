function runmany(scriptName, ids)
%% RUNMANY — run a plotting script once per simulation output.
%
%  Called automatically when a script's RUN_ID holds more than one number:
%      RUN_ID = [1 2 3 4 5];  % one window per run (noise-free)
%      RUN_ID = [6 7 8];      % the noisy-camera runs
%
%  It re-runs the script for each id, and labels every figure the script
%  creates with which run it came from -- window title and axes titles --
%  so three near-identical plots on screen can never be mixed up.
%
%  How the re-entry works: RUN_ID_FORCE is planted in the base workspace
%  before each call. The script's header reads that instead of its own
%  RUN_ID when it exists, which both selects the run and prevents infinite
%  recursion. The scripts' "close all" is guarded by the same variable, so
%  the first pass clears old figures and later passes keep theirs.

if nargin < 2 || isempty(ids), ids = 0; end
ids = ids(:).';

cleanup = onCleanup(@() evalin('base', 'clear RUN_ID_FORCE'));

for k = 1:numel(ids)
    id = ids(k);
    before = fignums_();

    assignin('base', 'RUN_ID_FORCE', id);
    evalin('base', scriptName);          % the script itself, unchanged

    new = setdiff(fignums_(), before);
    lbl = runlabel_(id);

    for n = new(:).'
        h = findobj(groot, 'Type', 'figure', 'Number', n);
        if isempty(h), continue; end
        h = h(1);

        if isempty(h.Name), h.Name = scriptName; end
        h.Name = sprintf('[%s] %s', lbl, h.Name);

        % Offset each run's windows so five sets do not land on top of
        % one another. Small step, so they stay readable side by side.
        p = h.Position;
        h.Position = [p(1) + (k-1)*46, max(60, p(2) - (k-1)*46), p(3), p(4)];

        for a = findall(h, 'Type', 'axes').'
            s = a.Title.String;
            if ~isempty(s)
                if iscell(s), s{1} = sprintf('%s  |  %s', s{1}, lbl);
                else,         s    = sprintf('%s  |  %s', s, lbl);
                end
                a.Title.String = s;
            end
        end
    end
end

fprintf('\n[runmany] %d run(s) plotted: %s\n', numel(ids), ...
        strjoin(arrayfun(@runlabel_, ids, 'UniformOutput', false), ', '));
end


%% ─────────────────────────────────────────────────────────────────
function n = fignums_()
f = findall(groot, 'Type', 'figure');
if isempty(f), n = []; else, n = [f.Number]; end
end


%% ─────────────────────────────────────────────────────────────────
function s = runlabel_(id)
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
