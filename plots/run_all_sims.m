%% RUN_ALL_SIMS — run every model, leave out1..out8 ready to plot.
%
%  USAGE
%      run_all_sims                 % scenario 8 (Scenario 3 in the paper)
%      scen = 3; run_all_sims       % any scenario
%
%  Afterwards every plotting script works straight away:
%      RUN_ID = [1 2 3 4 5];   -> one window per condition
%
%  out1 = QuadVisionControl              no delay
%  out2 = QuadVisionControl_delay        160 ms delay
%  out3 = QuadVisionControl_dkf          160 ms delay + DKF
%  out4 = QuadVisionControl_luenberger   160 ms delay + Luenberger
%  out5 = QuadVisionControl_lstm         160 ms delay + LSTM
%
%  NOISY CAMERA (1.5 px centroid, 0.25 px radius):
%  out6 = QuadVisionControl_noise_dkf    delay + DKF + noise
%  out7 = QuadVisionControl_noise_luen   delay + Luenberger + noise
%  out8 = QuadVisionControl_noise_delay  delay only + noise  (baseline)
%
%  The Luenberger noise run uses omega_n = 4, set by luen_tune.m. That is
%  the single reported Luenberger result - see loadruns.m for why.
%
%  RECORD THE SCENARIO. The out1..out4 campaign was written up without it
%  and the comparison is weaker for it.
%
%  A model that is not on the MATLAB path is skipped (warning), so you can
%  still plot the runs you have.
%
%  NOTE: you do NOT need "Single simulation output" ticked in the models.
%  Asking sim for a return value hands back a SimulationOutput whatever
%  that setting says, and capturing it here under different names is
%  what keeps the runs apart. Leaving the models untouched also means the
%  Run button in Simulink still behaves the way you are used to.

if ~exist('scen', 'var') || isempty(scen)
    scen = 8;              % 2, 5, 8 = Scenarios 1, 2, 3 of the paper
end
stopTime = '30';

models = {'QuadVisionControl', 'QuadVisionControl_delay', ...
          'QuadVisionControl_dkf', 'QuadVisionControl_luenberger', ...
          'QuadVisionControl_lstm', ...
          'QuadVisionControl_noise_dkf', 'QuadVisionControl_noise_luen', ...
          'QuadVisionControl_noise_delay'};
outs   = {'out1', 'out2', 'out3', 'out4', 'out5', 'out6', 'out7', 'out8'};
labels = {'no delay', '160 ms delay', 'delay + DKF', ...
          'delay + Luenberger', 'delay + LSTM', ...
          'DKF + noise', 'Luenberger + noise', '160 ms delay + noise'};

n = numel(models);
fprintf('\n=================================================================\n');
fprintf(' RUNNING ALL MODELS  |  scenario %d\n', scen);
fprintf('=================================================================\n');

ok = 0;
for i = 1:n
    m = models{i};
    fprintf('\n [%d/%d] %-32s %s\n', i, n, m, labels{i});

    try
        load_system(m);
    catch
        warning('run_all_sims:missingModel', ...
            'Could not open %s — skipping (%s).', m, labels{i});
        continue;
    end

    % Identical solver everywhere, or the runs are not comparable.
    set_param(m, 'SolverType','Fixed-step', 'Solver','ode4', ...
                 'FixedStep','0.005', 'StopTime',stopTime);

    % Identical scenario everywhere. Found by tracing the wire into
    % Target_Trajectory input 2 rather than assuming the Constant is
    % called the same thing in every model.
    % Every model reads the scenario from the workspace variable idx
    % (the Constant feeding Target_Trajectory input 2 has Value 'idx').
    % Set idx in the base workspace instead of overwriting that Constant
    % with a number, so that a later manual  idx = ...; sim(...)  in the
    % same session still selects the scenario it asks for.
    assignin('base', 'idx', scen);
    blk = ras_idx_block(m);
    if isempty(blk)
        warning('run_all_sims:noIdx', ...
            ['No Constant found driving Target_Trajectory input 2 in %s.\n' ...
             'The run may not use scenario %d.'], m, scen);
    elseif ~strcmp(strtrim(get_param(blk, 'Value')), 'idx')
        warning('run_all_sims:idxNotUsed', ...
            ['The scenario Constant in %s holds ''%s'' instead of ''idx''; ' ...
             'restore it to idx so the run uses scenario %d.'], ...
            m, get_param(blk, 'Value'), scen);
    end
    assignin('base', outs{i}, sim(m));
    fprintf('        -> %s\n', outs{i});
    ok = ok + 1;
end

fprintf('\n=================================================================\n');
fprintf(' Done. %d/%d models ran, scenario %d.\n', ok, n, scen);
fprintf(' WRITE THE SCENARIO DOWN: scen = %d\n', scen);

fprintf(' Open any plotting script, set  RUN_ID = [1 2 3 4 5];  and press Run.\n');
fprintf('=================================================================\n\n');


%% ─────────────────────────────────────────────────────────────────
function blk = ras_idx_block(model)
blk = '';
ws  = warning('off','all'); c = onCleanup(@() warning(ws)); %#ok<NASGU>
tt  = find_system(model, 'LookUnderMasks','all', 'FollowLinks','on', ...
                  'Type','block', 'Name','Target_Trajectory');
if isempty(tt), return; end
ph = get_param(tt{1}, 'PortHandles');
if numel(ph.Inport) < 2, return; end
ln = get_param(ph.Inport(2), 'Line');
if ln <= 0, return; end
guard = 0;
while get_param(ln,'SrcPortHandle') < 0 && guard < 50
    p = get_param(ln,'LineParent'); if p <= 0, break; end
    ln = p; guard = guard + 1;
end
sp = get_param(ln,'SrcPortHandle');
if sp <= 0, return; end
cand = get_param(sp, 'Parent');
if strcmp(get_param(cand,'BlockType'), 'Constant'), blk = cand; end
end
