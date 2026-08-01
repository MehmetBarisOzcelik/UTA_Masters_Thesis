function out = sim3_full_mission_compare(scenSel, visSel)
% SIM3_FULL_MISSION_COMPARE
% ---------------------------------------------------------------
% Thesis Demo 3:
%   Fair full-mission comparison under threat.
%
% Cases:
%   1) Default env + route + basic
%   2) Default env + route + PN
%   3) Default env + MPC   + basic
%   4) Default env + MPC   + PN
%   5) Manual environment + user-selected modes
%
% IMPORTANT:
%   Cases 1–4 use the same chaser physical parameters:
%       - same initial position
%       - same speed ratio
%       - same turn-rate limit
%       - same catch radius
%
%   The only difference between the basic and proportional-navigation
%   chasers is the guidance law.
%
% Expected thesis behavior:
%   route + basic : UAV avoids capture
%   route + PN    : proportional-navigation chaser captures the UAV
%   MPC + basic   : MPC improves survivability and avoids capture
%   MPC + PN      : UAV avoids capture, but the proportional-navigation
%                   chaser produces a smaller miss distance

    clc; close all;
    fprintf('=== Full Mission (Single Scenario) ===\n');

    %% 1) Load base configuration
    cfg_base = config();

% Ensure backward compatibility if an older config.m is used.
% The preferred location for k_basic is env/config.m.
if ~isfield(cfg_base.pn, 'k_basic') || isempty(cfg_base.pn.k_basic)
    cfg_base.pn.k_basic = 0.08;
end

    %% 2) Scenario selection
    if nargin < 1 || isempty(scenSel)
        fprintf('\nSelect scenario:\n');
        fprintf('  1) Default env : route + basic\n');
        fprintf('  2) Default env : route + PN\n');
        fprintf('  3) Default env : MPC   + basic\n');
        fprintf('  4) Default env : MPC   + PN\n');
        fprintf('  5) MANUAL env  : you choose MPC / non-MPC and PN / basic\n');

        scenSel = input('Select 1–5 [default 4]: ');
        if isempty(scenSel), scenSel = 4; end
    end

    cfg       = cfg_base;
    useManual = false;

    switch scenSel
        case 1
            cfg.env.mode = 'default';
            uavMode      = 'route';
            pursMode     = 'basic';

        case 2
            cfg.env.mode = 'default';
            uavMode      = 'route';
            pursMode     = 'pn';

        case 3
            cfg.env.mode = 'default';
            uavMode      = 'mpc';
            pursMode     = 'basic';

        case 4
            cfg.env.mode = 'default';
            uavMode      = 'mpc';
            pursMode     = 'pn';

        case 5
            useManual    = true;
            cfg.env.mode = 'manual';

            cfg = manual_env_input(cfg);

            fprintf('\nManual environment defined.\n');
            fprintf('Now choose guidance modes:\n');

            fprintf('\nUAV guidance:\n');
            fprintf('  1) route (no MPC)\n');
            fprintf('  2) MPC\n');
            uSel = input('Select 1–2 [default 2]: ');
            if isempty(uSel), uSel = 2; end

            if uSel == 1
                uavMode = 'route';
            else
                uavMode = 'mpc';
            end

            fprintf('\nChaser guidance:\n');
            fprintf('  1) basic (pure pursuit)\n');
            fprintf('  2) proportional navigation\n');
            pSel = input('Select 1–2 [default 2]: ');
            if isempty(pSel), pSel = 2; end

            if pSel == 1
                pursMode = 'basic';
            else
                pursMode = 'pn';
            end

        otherwise
            warning('Invalid selection, defaulting to option 4: Default env, MPC + PN.');
            cfg.env.mode = 'default';
            uavMode      = 'mpc';
            pursMode     = 'pn';
    end

    %% 3) Fair chaser setup
    % Reset to baseline physical parameters for all cases.
    % The basic and proportional-navigation chasers share speed, initial
    % position, turn-rate limit, and capture radius. Only the guidance law
    % differs.
    cfg.pn = cfg_base.pn;
    cfg.metrics.catch_radius = cfg_base.metrics.catch_radius;

    %% 4) Visualization options
    if nargin < 2 || isempty(visSel)
        fprintf('\nVisualization options:\n');
        fprintf('  1) Plot only\n');
        fprintf('  2) Plot + animation (MP4/GIF)\n');
        visSel = input('Select 1–2 [default 1]: ');
        if isempty(visSel), visSel = 1; end
    end

    cfg.plot.show         = true;
    cfg.outputs.save_anim = (visSel == 2);

    %% 5) Set modes and run
    cfg.uav.mode = uavMode;
    cfg.pn.mode  = pursMode;

    cfg.uav_mode = cfg.uav.mode;
    cfg.pn_mode  = cfg.pn.mode;

    fprintf('\nRunning full mission with:\n');
    fprintf('  UAV mode   : %s\n', cfg.uav.mode);
    fprintf('  Chaser     : %s\n', cfg.pn.mode);
    fprintf('  Environment: %s\n', cfg.env.mode);

    if useManual
        fprintf('  Scenario   : MANUAL environment, baseline chaser parameters\n');
    else
        fprintf('  Scenario   : FAIR comparison, baseline chaser parameters\n');
    end

    fprintf('  Chaser parameters: v_ratio = %.3f, N = %.3f, k_basic = %.3f, k_pp = %.3f, capture radius = %.1f m\n', ...
            cfg.pn.v_ratio, cfg.pn.N, cfg.pn.k_basic, cfg.pn.k_pp, cfg.metrics.catch_radius);

    fprintf('  Visual     : %s\n\n', tern(visSel==1, 'Plot only', 'Plot + animation'));

    out = run_all(cfg);

    %% 6) Summary
    tf     = NaN;
    miss   = NaN;
    caught = false;
    path   = NaN;

    if isfield(out, 'metrics')
        m = out.metrics;

        if isfield(m, 'final_time'),    tf     = m.final_time;    end
        if isfield(m, 'miss_distance'), miss   = m.miss_distance; end
        if isfield(m, 'catch'),         caught = m.catch;         end
    end

    if isfield(out, 'metrics') && isfield(out.metrics, 'path_length_m')
        path = out.metrics.path_length_m;
    elseif isfield(out, 'uav') && isfield(out.uav, 'pos') && ~isempty(out.uav.pos)
        path = path_length(out.uav.pos);
    end

    fprintf('================ Scenario Summary ================\n');
    fprintf('Mode: UAV = %s, Chaser = %s, Env = %s\n', ...
            cfg.uav.mode, cfg.pn.mode, cfg.env.mode);

    fprintf('t_final = %s s, miss = %s m, catch = %s, path = %s m\n', ...
            fmt_num(tf), fmt_num(miss), tern(caught, 'YES', 'no'), fmt_num(path));

    fprintf('==================================================\n');
    fprintf('\nSimulation complete.\n');

    if nargout == 0
        clear out;
    end
end

% ========================================================================
function L = path_length(P)
if size(P,1) < 2
    L = 0;
else
    d = sqrt(sum(diff(P,1,1).^2, 2));
    L = sum(d);
end
end

% ========================================================================
function s = tern(c, a, b)
if c
    s = a;
else
    s = b;
end
end

% ========================================================================
function s = fmt_num(x)
if isnan(x)
    s = 'n/a';
else
    s = sprintf('%.1f', x);
end
end