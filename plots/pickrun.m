function R = pickrun(id)
%% PICKRUN — turn a run id 1..9 into the right simulation output object.
%
%  So a plotting script only ever needs ONE number changed:
%      RUN_ID = 3;
%      RUN    = pickrun(RUN_ID);
%
%    NOISE-FREE CAMERA
%      1  ->  out1   QuadVisionControl              no delay
%      2  ->  out2   QuadVisionControl_delay        160 ms delay
%      3  ->  out3   QuadVisionControl_dkf          160 ms delay + DKF
%      4  ->  out4   QuadVisionControl_luenberger   160 ms delay + Luenberger
%      5  ->  out5   QuadVisionControl_lstm         160 ms delay + LSTM
%
%    NOISY CAMERA  (1.5 px centroid, 0.25 px radius - see luen_noise\)
%      6  ->  out6   QuadVisionControl_noise_dkf    delay + DKF + noise
%      7  ->  out7   QuadVisionControl_noise_luen   + Luenberger, omega_n = 4
%      8  ->  out8   delay-only + noisy camera      noisy baseline
%
%      0  ->  []     whatever the last simulation left loose
%
%  Set the matching name in each model under
%      Model Settings -> Data Import/Export -> Single simulation output
%  and every run stays separate instead of overwriting the previous one.
%
%  NOTE on 7: omega_n = 4, set by luen_tune.m (documented sweep over
%  omega_n and zeta, 3 scenarios, 5 seeds, minimising open-loop estimator
%  RMSE). The noise-free Luenberger (out4) runs omega_n = 8 - the gain had
%  to halve once the camera became noisy, which is itself a result.

names  = {'out1', 'out2', 'out3', 'out4', 'out5', ...
          'out6', 'out7', 'out8', 'out9', 'out10'};
labels = {'QuadVisionControl (no delay)', ...
          'QuadVisionControl_delay (160 ms delay)', ...
          'QuadVisionControl_dkf (160 ms delay + DKF)', ...
          'QuadVisionControl_luenberger (160 ms delay + Luenberger)', ...
          'QuadVisionControl_lstm (160 ms delay + LSTM)', ...
          'QuadVisionControl_noise_dkf (delay + DKF + noise)', ...
          'QuadVisionControl_noise_luen (delay + Luenberger + noise)', ...
          'delay-only + noisy camera (noisy baseline)', ...
          'spare slot (out9)', ...
          'QuadVisionControl_obs (delay + Neural Luenberger Observer)'};

if isempty(id) || (isnumeric(id) && isscalar(id) && id == 0)
    R = [];
    fprintf('[pickrun] reading the base workspace (last run).\n');
    return;
end

if isnumeric(id) && ~isscalar(id)
    error('pickrun:vector', ...
        ['pickrun takes one number. For several runs at once, leave it to\n' ...
         'the script header, which routes  RUN_ID = %s  through runmany\n' ...
         'and opens one labelled window per run.'], mat2str(id));
end
if ~(isnumeric(id) && isscalar(id) && any(id == 1:numel(names)))
    error('pickrun:id', 'RUN_ID must be 1..%d, or 0. Got: %s', ...
          numel(names), mat2str(id));
end

nm = names{id};
if evalin('base', sprintf('exist(''%s'',''var'') == 1', nm))
    R = evalin('base', nm);
    fprintf('[pickrun] %d -> %s : %s\n', id, nm, labels{id});
    return;
end

% Not there. Say which ones ARE, because the usual cause is that the model
% was run before its output name was set, or only one model has been run.
have = {};
for i = 1:numel(names)
    if evalin('base', sprintf('exist(''%s'',''var'') == 1', names{i}))
        have{end+1} = names{i}; %#ok<AGROW>
    end
end
if evalin('base', 'exist(''out'',''var'') == 1'), have{end+1} = 'out'; end

if isempty(have), havetxt = '(none)'; else, havetxt = strjoin(have, ', '); end

error('pickrun:missing', ...
    ['RUN_ID = %d needs "%s", which does not exist.\n\n' ...
     'Currently in the workspace: %s\n\n' ...
     'To create it:\n' ...
     '  1. open %s\n' ...
     '  2. Model Settings -> Data Import/Export\n' ...
     '  3. tick "Single simulation output" and set the name to %s\n' ...
     '  4. save the model and run it\n\n' ...
     'Or set RUN_ID = 0 to just plot the last run.'], ...
    id, nm, havetxt, labels{id}, nm);
end
