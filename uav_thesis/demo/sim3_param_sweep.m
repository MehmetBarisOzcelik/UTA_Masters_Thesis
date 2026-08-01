function sim3_param_sweep()
% SIM3_PARAM_SWEEP
% ---------------------------------------------------------------
% Thesis parameter-sweep module.
%
% This function produces:
%
%   A) FAIR four-case comparison:
%       1) route + basic
%       2) route + PN
%       3) MPC   + basic
%       4) MPC   + PN
%
%   B) PN navigation constant sweep
%   C) Chaser-to-UAV speed-ratio sweep
%   D) MPC horizon sweep
%
% This file is written as a FUNCTION, not a script, so it can be called
% safely from the GUI callback in run_thesis_menu.m.

clc; close all;
fprintf('=== Thesis Parameter Sweeps and Mission Statistics ===\n');

%% 1. Base configuration --------------------------------------------------

cfg_base = config();

% Make all sweeps console-only.
cfg_base.plot.show         = false;
cfg_base.outputs.save_anim = false;
cfg_base.env.mode          = 'default';

% Ensure backward compatibility if an older config.m is used.
% The preferred location for k_basic is env/config.m.
if ~isfield(cfg_base.pn, 'k_basic') || isempty(cfg_base.pn.k_basic)
    cfg_base.pn.k_basic = 0.08;
end

%% 2. Sweep definitions ---------------------------------------------------

fairCases = struct( ...
    'name',      { 'route + basic',  'route + PN',  'MPC   + basic',  'MPC   + PN' }, ...
    'uav_mode',  { 'route',          'route',       'mpc',            'mpc'       }, ...
    'purs_mode', { 'basic',          'pn',          'basic',          'pn'        } ...
    );

N_values = [2 3 4 5 6];

vRatio_values = [1.00 1.10 1.20 1.25 1.30 1.40];

Np_values = [6 8 10 12 16 20];

seeds = 1001:1010;

%% 3. Run all studies -----------------------------------------------------

fprintf('\n--- Study A: Fair four-case comparison ---\n');
T_fair = run_fair_comparison(cfg_base, fairCases, seeds);

fprintf('\n--- Study B: PN gain sweep ---\n');
T_pnGain = run_pn_gain_sweep(cfg_base, N_values, seeds);

fprintf('\n\n========== STUDY C: CHASER-TO-UAV SPEED-RATIO SWEEP =======\n');
T_speed = run_speed_ratio_sweep(cfg_base, vRatio_values, seeds);

fprintf('\n--- Study D: MPC horizon sweep ---\n');
T_horizon = run_mpc_horizon_sweep(cfg_base, Np_values, seeds);

%% 4. Print summaries -----------------------------------------------------

fprintf('\n\n================ STUDY A: FAIR COMPARISON ================\n');
summarize_by_group(T_fair, 'CaseName');

fprintf('\n\n================ STUDY B: PN GAIN SWEEP ===================\n');
summarize_by_group(T_pnGain, 'PN_N');

fprintf('\n\n================ STUDY C: SPEED-RATIO SWEEP ===============\n');
summarize_by_group(T_speed, 'v_ratio');

fprintf('\n\n================ STUDY D: MPC HORIZON SWEEP ===============\n');
summarize_by_group(T_horizon, 'Np');

%% 5. Export to workspace -------------------------------------------------

assignin('base','T_fair',    T_fair);
assignin('base','T_pnGain',  T_pnGain);
assignin('base','T_speed',   T_speed);
assignin('base','T_horizon', T_horizon);

fprintf('\nTables exported to workspace:\n');
fprintf('  T_fair\n');
fprintf('  T_pnGain\n');
fprintf('  T_speed\n');
fprintf('  T_horizon\n');
fprintf('\nDone.\n');

end

% ========================================================================
function T = run_fair_comparison(cfg_base, fairCases, seeds)

rows = [];

for ic = 1:numel(fairCases)

    C = fairCases(ic);

    for is = 1:numel(seeds)

        cfg = cfg_base;
        cfg.seed = seeds(is);

        cfg.pn = cfg_base.pn;
        cfg.metrics.catch_radius = cfg_base.metrics.catch_radius;

        cfg.uav.mode = C.uav_mode;
        cfg.pn.mode  = C.purs_mode;

        cfg.uav_mode = cfg.uav.mode;
        cfg.pn_mode  = cfg.pn.mode;

        fprintf('Running fair case %-14s seed = %d ...\n', C.name, seeds(is));

        out = run_all(cfg);

        row = extract_row(out);
        row.Study      = "FairComparison";
        row.CaseName   = string(C.name);
        row.Seed       = seeds(is);
        row.PN_N       = cfg.pn.N;
        row.v_ratio    = cfg.pn.v_ratio;
        row.Np         = cfg.mpc.Np;
        row.k_basic    = cfg.pn.k_basic;

        rows = [rows; row]; %#ok<AGROW>
    end
end

T = struct2table(rows);

end

% ========================================================================
function T = run_pn_gain_sweep(cfg_base, N_values, seeds)

rows = [];

for iN = 1:numel(N_values)

    Nval = N_values(iN);

    for is = 1:numel(seeds)

        cfg = cfg_base;
        cfg.seed = seeds(is);

        cfg.uav.mode = 'route';
        cfg.pn.mode  = 'pn';

        cfg.pn = cfg_base.pn;
        cfg.pn.N = Nval;
        cfg.pn.mode = 'pn';

        cfg.metrics.catch_radius = cfg_base.metrics.catch_radius;

        cfg.uav_mode = cfg.uav.mode;
        cfg.pn_mode  = cfg.pn.mode;

        fprintf('Running PN gain N = %.2f seed = %d ...\n', Nval, seeds(is));

        out = run_all(cfg);

        row = extract_row(out);
        row.Study      = "PNGainsweep";
        row.CaseName   = "route + PN";
        row.Seed       = seeds(is);
        row.PN_N       = Nval;
        row.v_ratio    = cfg.pn.v_ratio;
        row.Np         = cfg.mpc.Np;
        row.k_basic    = cfg.pn.k_basic;

        rows = [rows; row]; %#ok<AGROW>
    end
end

T = struct2table(rows);

end

% ========================================================================
function T = run_speed_ratio_sweep(cfg_base, vRatio_values, seeds)

rows = [];

for iv = 1:numel(vRatio_values)

    vr = vRatio_values(iv);

    for is = 1:numel(seeds)

        cfg = cfg_base;
        cfg.seed = seeds(is);

        cfg.uav.mode = 'route';
        cfg.pn.mode  = 'pn';

        cfg.pn = cfg_base.pn;
        cfg.pn.v_ratio = vr;
        cfg.pn.mode = 'pn';

        cfg.pn.v_max = max(cfg_base.pn.v_max, cfg.pn.v_ratio * cfg.uav.v_nom);
        cfg.metrics.catch_radius = cfg_base.metrics.catch_radius;

        cfg.uav_mode = cfg.uav.mode;
        cfg.pn_mode  = cfg.pn.mode;

        fprintf('Running chaser-to-UAV speed ratio = %.2f, seed = %d ...\n', ...
        vr, seeds(is));

        out = run_all(cfg);

        row = extract_row(out);
        row.Study      = "SpeedRatioSweep";
        row.CaseName   = "route + PN";
        row.Seed       = seeds(is);
        row.PN_N       = cfg.pn.N;
        row.v_ratio    = vr;
        row.Np         = cfg.mpc.Np;
        row.k_basic    = cfg.pn.k_basic;

        rows = [rows; row]; %#ok<AGROW>
    end
end

T = struct2table(rows);

end

% ========================================================================
function T = run_mpc_horizon_sweep(cfg_base, Np_values, seeds)

rows = [];

for iNp = 1:numel(Np_values)

    Np = Np_values(iNp);

    for is = 1:numel(seeds)

        cfg = cfg_base;
        cfg.seed = seeds(is);

        cfg.uav.mode = 'mpc';
        cfg.pn.mode  = 'pn';

        cfg.pn = cfg_base.pn;
        cfg.pn.mode = 'pn';

        cfg.metrics.catch_radius = cfg_base.metrics.catch_radius;
        cfg.mpc.Np = Np;

        cfg.uav_mode = cfg.uav.mode;
        cfg.pn_mode  = cfg.pn.mode;

        fprintf('Running MPC horizon Np = %d seed = %d ...\n', Np, seeds(is));

        out = run_all(cfg);

        row = extract_row(out);
        row.Study      = "MPCHorizonSweep";
        row.CaseName   = "MPC + PN";
        row.Seed       = seeds(is);
        row.PN_N       = cfg.pn.N;
        row.v_ratio    = cfg.pn.v_ratio;
        row.Np         = Np;
        row.k_basic    = cfg.pn.k_basic;

        rows = [rows; row]; %#ok<AGROW>
    end
end

T = struct2table(rows);

end

% ========================================================================
function row = extract_row(out)

m = out.metrics;

row = struct();

row.final_time_s = get_field(m, 'final_time', NaN);
row.catch        = logical(get_field(m, 'catch', false));
row.miss_m       = get_field(m, 'miss_distance', NaN);
row.path_m       = get_field(m, 'path_length_m', NaN);

row.min_nfz_clearance_m   = get_field(m, 'min_nfz_clearance_m', NaN);
row.min_radar_clearance_m = get_field(m, 'min_radar_clearance_m', NaN);

row.nfz_violation_count   = get_field(m, 'nfz_violation_count', NaN);
row.radar_violation_count = get_field(m, 'radar_violation_count', NaN);

row.nfz_exposure_integral   = get_field(m, 'nfz_exposure_integral', NaN);
row.radar_exposure_integral = get_field(m, 'radar_exposure_integral', NaN);
row.total_exposure_integral = get_field(m, 'total_exposure_integral', NaN);

row.mpc_time_mean_s = get_field(m, 'mpc_time_mean_s', NaN);
row.mpc_time_max_s  = get_field(m, 'mpc_time_max_s', NaN);

end

% ========================================================================
function val = get_field(S, name, defaultVal)

if isfield(S, name)
    val = S.(name);
else
    val = defaultVal;
end

end

% ========================================================================
function summarize_by_group(T, groupVar)

G = findgroups(T.(groupVar));
u = splitapply(@(x) x(1), T.(groupVar), G);

catchRate = splitapply(@(x) 100*mean(x), T.catch, G);
tfMean    = splitapply(@(x) mean(x,'omitnan'), T.final_time_s, G);
missMean  = splitapply(@(x) mean(x,'omitnan'), T.miss_m, G);
pathMean  = splitapply(@(x) mean(x,'omitnan'), T.path_m, G);
nfzClr    = splitapply(@(x) mean(x,'omitnan'), T.min_nfz_clearance_m, G);
radClr    = splitapply(@(x) mean(x,'omitnan'), T.min_radar_clearance_m, G);
radExp    = splitapply(@(x) mean(x,'omitnan'), T.radar_exposure_integral, G);
mpcMean   = splitapply(@(x) mean(x,'omitnan'), T.mpc_time_mean_s, G);

Summary = table(u, catchRate, tfMean, missMean, pathMean, ...
                nfzClr, radClr, radExp, mpcMean, ...
    'VariableNames', {groupVar, 'catch_rate_pct', 'mean_t_final_s', ...
                      'mean_miss_m', 'mean_path_m', ...
                      'mean_min_nfz_clearance_m', ...
                      'mean_min_radar_clearance_m', ...
                      'mean_radar_exposure', ...
                      'mean_mpc_time_s'});

disp(Summary);

end