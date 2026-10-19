function R = loadruns(ids)
%% LOADRUNS — collect out1..out8 into one struct array for plotting.
%
%  R = loadruns()              % everything that exists, skipping the rest
%  R = loadruns([2 6 7 8])     % the noise campaign + the delay baseline
%
%  R(k).obj    the SimulationOutput
%  R(k).label  short name used in legends
%  R(k).color  colour = METHOD
%  R(k).style  line style = NOISE CONDITION
%  R(k).noisy  true for the noisy-camera runs
%
%  ── RUN IDS ──────────────────────────────────────────────────────
%   NOISE-FREE CAMERA
%     1  out1  no delay                        black    solid
%     2  out2  160 ms delay                    orange   solid
%     3  out3  delay + DKF                     blue     solid
%     4  out4  delay + Luenberger              purple   solid
%     5  out5  delay + LSTM                    green    solid
%
%   NOISY CAMERA  (1.5 px centroid, 0.25 px radius - see luen_noise\)
%     6  out6  delay + DKF + noise             blue     solid
%     7  out7  delay + Luenberger + noise      purple   solid    omega_n = 4
%     8  out8  160 ms delay + noise            orange   solid    (baseline)
%
%  WHY THERE IS ONLY ONE LUENBERGER NOISE RUN
%    omega_n = 4 (out7) is THE Luenberger result. It was chosen by
%    luen_tune.m - a documented sweep over omega_n {2,3,4,6,8,12,16} x
%    zeta {0.7,1.0}, 3 scenarios, 5 noise seeds - minimising open-loop
%    estimator RMSE. That criterion was fixed BEFORE any closed-loop run
%    existed, which is what makes the number defensible.
%
%    The noise-free baseline uses omega_n = 8 (out4). The bandwidth had to
%    HALVE once the camera became noisy: a fixed gain must trade delay
%    compensation against noise rejection and cannot make that trade
%    online. That retune is itself the headline comparison result.
%
%  COLOUR = METHOD
%    The DKF is blue in both campaigns, the Luenberger purple, so "same
%    method, harder conditions" reads straight off the figure. All lines are
%    SOLID by default - within one campaign the colours are already
%    distinct, and solid lines read better in a printed thesis.
%
%  ── ON MIXING THE TWO CAMPAIGNS IN ONE FIGURE ────────────────────
%    Runs 1-5 and 6-8 are DIFFERENT EXPERIMENTS, not a before/after of one.
%    Their observers were retuned for the noise (DKF sigma_a [5 5 10] ->
%    [2 2 4], Luenberger omega_n 8 -> 4), so an arrow drawn from out3 to
%    out6 measures noise AND a gain change at once. That is why out6 can
%    score BETTER than out3 on steady-state RMSE, which otherwise looks
%    impossible. Overlay them to show context; compare within a campaign.
%    If you do overlay them, re-enable dashed styles (see below) or the two
%    blue lines become indistinguishable.
%
%  Populate the workspace first with:  run_all_sims
%  (or run each model so the outN you want exists)

if nargin < 1 || isempty(ids), ids = 1:8; end

names  = {'out1', 'out2', 'out3', 'out4', 'out5', ...
          'out6', 'out7', 'out8'};

labels = {'no delay', '160 ms delay', 'delay + DKF', ...
          'delay + Luenberger', 'delay + LSTM', ...
          'DKF + noise', 'Luenberger + noise', ...
          '160 ms delay + noise'};

cols   = [0.00 0.00 0.00       % 1 no delay        black
          0.85 0.33 0.10       % 2 delay           orange
          0.00 0.45 0.74       % 3 DKF             blue
          0.49 0.18 0.56       % 4 Luenberger      purple
          0.47 0.67 0.19       % 5 LSTM            green
          0.00 0.45 0.74       % 6 DKF + noise     blue    (dashed)
          0.49 0.18 0.56       % 7 Luen + noise    purple  (dashed)
          0.85 0.33 0.10];     % 8 delay + noise   orange

% All solid. Colour alone separates the runs, which is enough while each
% figure stays inside one campaign (e.g. IDS = [8 6 7]). If you ever put
% noise-free and noisy runs of the SAME method on one axis - say 3 and 6,
% both blue - switch the noisy ones back to '--' so the pair is readable:
%   styles = {'-','-','-','-','-','--','--','--'};
styles = {'-', '-', '-', '-', '-', '-', '-', '-'};
noisy  = [false false false false false true true true];

R = struct('id', {}, 'name', {}, 'label', {}, 'color', {}, ...
           'style', {}, 'noisy', {}, 'obj', {});
missing = {};
ok = 1:numel(names);

for id = ids(:).'
    if ~any(id == ok)
        error('loadruns:id', 'Run ids must be 1..%d. Got %d.', numel(names), id);
    end
    nm = names{id};
    if evalin('base', sprintf('exist(''%s'',''var'') == 1', nm))
        k = numel(R) + 1;
        R(k).id    = id;
        R(k).name  = nm;
        R(k).label = labels{id};
        R(k).color = cols(id, :);
        R(k).style = styles{id};
        R(k).noisy = noisy(id);
        R(k).obj   = evalin('base', nm);
    else
        missing{end+1} = sprintf('%s (%s)', nm, labels{id}); %#ok<AGROW>
    end
end

if isempty(R)
    error('loadruns:none', ...
        ['None of the requested runs exist in the workspace.\n\n' ...
         'Noise-free runs come from:\n    run_all_sims\n\n' ...
         'Noisy runs must be run by hand:\n' ...
         '    out6 = QuadVisionControl_noise_dkf\n' ...
         '    out7 = QuadVisionControl_noise_luen   (omega_n = 4)\n' ...
         '    out8 = delay-only model + noisy camera block']);
end

if ~isempty(missing)
    warning('loadruns:partial', ...
        'Plotting %d of %d runs. Missing: %s.', ...
        numel(R), numel(ids), strjoin(missing, ', '));
end

% A noise figure without a noisy baseline invites the obvious question:
% compared to WHAT? out2 is noise-free, so it is only an approximate
% reference for the noisy runs.
if any([R.noisy]) && ~any(arrayfun(@(r) r.id == 8, R))
    if any(arrayfun(@(r) r.id == 2, R))
        fprintf(['[loadruns] Baseline shown is out2 (delay, NOISE-FREE). The\n' ...
                 '           matched baseline is out8 (delay + noise).\n']);
    end
end
end
