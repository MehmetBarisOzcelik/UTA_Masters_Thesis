% RUN_ALL
% ---------------------------------------------------------------
% Core single-scenario engine for the full mission simulation.
%
% Usage:
%   out = run_all();        % use default config()
%   out = run_all(cfg);     % use caller-supplied cfg struct
%
% Responsibilities:
%   1) Initialize configuration and RNG.
%   2) Build the environment and risk-aware route.
%   3) Simulate the UAV using either:
%        - 'route' mode  : simple waypoint follower
%        - 'mpc' mode    : grid-search MPC with NFZ/radar penalties
%   4) Simulate the chaser using either:
%        - 'basic' mode  : pure-pursuit-style guidance
%        - 'pn' mode     : proportional-navigation guidance
%   5) Compute mission/interception/safety/computation metrics.
%   6) Generate a static plot and optional animation.
%
% Output:
%   out.env, out.route, out.cfg, out.uav, out.pn, out.metrics

function out = run_all(cfg)

clc;
fprintf('=== Full Mission Simulation (Strict Threat-Avoidance Version) ===\n');

%% 1. Config and RNG ------------------------------------------------------
if nargin < 1 || isempty(cfg)
    cfg = config();
end

% -------------------------------------------------------------------------
% Mode handling
% Preferred fields:
%   cfg.uav.mode
%   cfg.pn.mode
% Legacy fields:
%   cfg.uav_mode
%   cfg.pn_mode
% -------------------------------------------------------------------------
if ~isfield(cfg, 'uav') || ~isstruct(cfg.uav)
    cfg.uav = struct();
end

if ~isfield(cfg, 'pn') || ~isstruct(cfg.pn)
    cfg.pn = struct();
end

if ~isfield(cfg.uav, 'mode') || isempty(cfg.uav.mode)
    if isfield(cfg, 'uav_mode') && ~isempty(cfg.uav_mode)
        cfg.uav.mode = cfg.uav_mode;
    else
        cfg.uav.mode = 'mpc';
    end
end

if ~isfield(cfg.pn, 'mode') || isempty(cfg.pn.mode)
    if isfield(cfg, 'pn_mode') && ~isempty(cfg.pn_mode)
        cfg.pn.mode = cfg.pn_mode;
    else
        cfg.pn.mode = 'pn';
    end
end

cfg.uav_mode = cfg.uav.mode;
cfg.pn_mode  = cfg.pn.mode;

fprintf('UAV mode: %s | Chaser mode: %s\n', cfg.uav.mode, cfg.pn.mode);

% RNG seed
if ischar(cfg.seed) && strcmpi(cfg.seed, 'shuffle')
    rng('shuffle');
else
    rng(cfg.seed);
end

dt = cfg.dt;

%% 2. Environment and route -----------------------------------------------
env = threat_map(cfg.env);

fprintf('Environment generated with %d NFZs and %d radars.\n', ...
        size(env.nfz,1), size(env.radar,1));

G     = build_graph(env, cfg);
route = route_optimizer(G, env, cfg);

fprintf('Environment and route generated successfully.\n');

%% 3. Initialize UAV and chaser ------------------------------------------

% UAV initial state at start, heading toward the next route node.
uav.x = env.start(1);
uav.y = env.start(2);

if isfield(route, 'nodes') && size(route.nodes,1) >= 2
    next_wp = route.nodes(2,:);
else
    next_wp = env.goal;
end

uav.psi = atan2(next_wp(2) - env.start(2), ...
                next_wp(1) - env.start(1));
uav.v   = cfg.uav.v_nom;

% Route/MPC bookkeeping
mpc.route_wp     = route.nodes;
mpc.route_wp_idx = 1:size(route.nodes,1);
mpc.wp_idx       = 2;

% Chaser initial state
pn0 = init_pursuer(env, cfg);

fprintf('UAV and chaser initialized.\n');

%% 4. UAV simulation loop -------------------------------------------------

Kmax    = ceil(cfg.Tmax / dt);
uav_pos = nan(Kmax, 2);
uav_v   = nan(Kmax, 1);
uav_psi = nan(Kmax, 1);
tlog    = nan(Kmax, 1);

% Computation timing logs
mpc_time_log = nan(Kmax, 1);
ctrl_time_log = nan(Kmax, 1);

k    = 1;
t    = 0;
done = false;

while ~done && t <= cfg.Tmax && k <= Kmax

    % ---------------------------------------------------------------------
    % 4.a Guidance / controller timing
    % ---------------------------------------------------------------------
    ctrl_tic = tic;

    switch lower(cfg.uav.mode)
        case 'mpc'
            mpc_tic = tic;
            [u_cmd, mpc] = mpc_step(mpc, uav, pn0, env, cfg);
            mpc_time_log(k) = toc(mpc_tic);

        case 'route'
            [u_cmd, mpc] = simple_route_step(mpc, uav, env, cfg);
            mpc_time_log(k) = 0;

        otherwise
            error('Unknown cfg.uav.mode = %s. Use ''mpc'' or ''route''.', ...
                  cfg.uav.mode);
    end

    ctrl_time_log(k) = toc(ctrl_tic);

    % ---------------------------------------------------------------------
    % 4.b Integrate UAV dynamics
    % ---------------------------------------------------------------------
    uav = advance_states(uav, u_cmd, dt, cfg);

    % Waypoint / goal handling
    [uav, mpc] = waypoint_manager(uav, mpc, env, cfg);

    % ---------------------------------------------------------------------
    % 4.c Log UAV state
    % ---------------------------------------------------------------------
    uav_pos(k, :) = [uav.x, uav.y];
    uav_v(k)      = uav.v;
    uav_psi(k)    = uav.psi;
    tlog(k)       = t;

    % ---------------------------------------------------------------------
    % 4.d Goal check
    % ---------------------------------------------------------------------
    dist_goal = hypot(uav.x - env.goal(1), uav.y - env.goal(2));

    if dist_goal <= cfg.env.r_goal
        fprintf('Goal reached at t = %.1f s (dist = %.1f m)\n', ...
                t, dist_goal);
        done = true;
    end

    t = t + dt;
    k = k + 1;
end

% Trim logs
uav_pos      = uav_pos(1:k-1, :);
uav_v        = uav_v(1:k-1);
uav_psi      = uav_psi(1:k-1);
tlog         = tlog(1:k-1);
mpc_time_log = mpc_time_log(1:k-1);
ctrl_time_log = ctrl_time_log(1:k-1);

%% 5. Build UAV log for chaser simulation --------------------------------

uav_vel = [uav_v .* cos(uav_psi), ...
           uav_v .* sin(uav_psi)];

uavlog = struct('t',   tlog, ...
                'pos', uav_pos, ...
                'vel', uav_vel);

%% 6. Simulate chaser ----------------------------------------------------

switch lower(cfg.pn.mode)
    case 'pn'
        pnlog = simulate_pursuer_pn(pn0, uavlog, cfg, env);

    case 'basic'
        pnlog = simulate_pursuer_basic(pn0, uavlog, cfg, env);

    otherwise
        error('Unknown cfg.pn.mode = %s. Use ''pn'' or ''basic''.', ...
              cfg.pn.mode);
end

%% 7. Compute metrics -----------------------------------------------------

n_common = min(size(uavlog.pos,1), size(pnlog.pos,1));
MD       = miss_distance(uavlog.pos(1:n_common,:), pnlog.pos(1:n_common,:));

metrics.miss_distance = MD.value;
metrics.miss_index    = MD.t_index;
metrics.final_time    = tlog(end);

% Catch info
if isfield(pnlog, 'caught') && pnlog.caught
    metrics.catch      = true;
    metrics.catch_idx  = pnlog.catch_idx;
    metrics.catch_pos  = pnlog.catch_pos;

    % Truncate UAV log at catch time for plotting and path metrics
    kC = min(pnlog.catch_idx, numel(uavlog.t));

    uavlog.t   = uavlog.t(1:kC);
    uavlog.pos = uavlog.pos(1:kC, :);
    uavlog.vel = uavlog.vel(1:kC, :);

    metrics.final_time = uavlog.t(end);
else
    metrics.catch      = false;
    metrics.catch_idx  = NaN;
    metrics.catch_pos  = [NaN NaN];
end

% Safety / exposure metrics use the actually flown UAV path.
safety = compute_safety_metrics(uavlog.pos, uavlog.t, env);

metrics.path_length_m          = path_length(uavlog.pos);
metrics.min_nfz_clearance_m    = safety.min_nfz_clearance_m;
metrics.min_radar_clearance_m  = safety.min_radar_clearance_m;
metrics.nfz_violation_count    = safety.nfz_violation_count;
metrics.radar_violation_count  = safety.radar_violation_count;
metrics.nfz_exposure_integral  = safety.nfz_exposure_integral;
metrics.radar_exposure_integral = safety.radar_exposure_integral;
metrics.total_exposure_integral = safety.total_exposure_integral;

% Computation metrics
valid_mpc  = mpc_time_log(isfinite(mpc_time_log));
valid_ctrl = ctrl_time_log(isfinite(ctrl_time_log));

if isempty(valid_mpc)
    metrics.mpc_time_mean_s = 0;
    metrics.mpc_time_max_s  = 0;
else
    metrics.mpc_time_mean_s = mean(valid_mpc);
    metrics.mpc_time_max_s  = max(valid_mpc);
end

if isempty(valid_ctrl)
    metrics.ctrl_time_mean_s = 0;
    metrics.ctrl_time_max_s  = 0;
else
    metrics.ctrl_time_mean_s = mean(valid_ctrl);
    metrics.ctrl_time_max_s  = max(valid_ctrl);
end

metrics.n_mpc_steps = sum(valid_mpc > 0);

%% 8. Pack output struct --------------------------------------------------

out.env     = env;
out.route   = route;
out.cfg     = cfg;
out.uav     = struct('t', uavlog.t, 'pos', uavlog.pos);
out.pn      = pnlog;
out.metrics = metrics;

%% 9. Plot and animate ----------------------------------------------------

if isfield(cfg, 'plot') && isfield(cfg.plot, 'show') && cfg.plot.show
    plot_mission(out);
end

if isfield(cfg, 'outputs') && isfield(cfg.outputs, 'save_anim') && ...
        cfg.outputs.save_anim && ...
        isfield(cfg, 'plot') && isfield(cfg.plot, 'show') && cfg.plot.show

    basename = sprintf('mission_%s', cfg.outputs.tag);
    animate_mission(out, basename);
end

fprintf('Simulation complete.\n');

end

% ========================================================================
% Helper: path length along UAV trajectory
% ========================================================================
function L = path_length(P)

if size(P,1) < 2
    L = 0;
else
    d = diff(P,1,1);
    L = sum(hypot(d(:,1), d(:,2)));
end

end

% ========================================================================
% Helper: safety, clearance, violation, and exposure metrics
% ========================================================================
function safety = compute_safety_metrics(P, t, env)

    if nargin < 2 || isempty(t)
        t = (0:size(P,1)-1).';
    else
        t = t(:);
    end

    if size(P,1) < 2
        dt = 0;
    else
        dt = median(diff(t));
    end

    nfz   = [];
    radar = [];

    if isfield(env, 'nfz') && ~isempty(env.nfz)
        nfz = env.nfz;
    end

    if isfield(env, 'radar') && ~isempty(env.radar)
        radar = env.radar;
    end

    [minNfzClr, nfzViol, nfzExp] = zone_metrics(P, nfz, dt);
    [minRadClr, radViol, radExp] = zone_metrics(P, radar, dt);

    safety.min_nfz_clearance_m     = minNfzClr;
    safety.min_radar_clearance_m   = minRadClr;
    safety.nfz_violation_count     = nfzViol;
    safety.radar_violation_count   = radViol;
    safety.nfz_exposure_integral   = nfzExp;
    safety.radar_exposure_integral = radExp;
    safety.total_exposure_integral = nfzExp + radExp;

end

% ========================================================================
% Helper: metrics for one class of circular zones
% ========================================================================
function [minClr, violationCount, exposureIntegral] = zone_metrics(P, zones, dt)

    if isempty(zones)
        minClr           = inf;
        violationCount   = 0;
        exposureIntegral = 0;
        return;
    end

    minClr = inf;
    violationMask = false(size(P,1),1);
    exposure = zeros(size(P,1),1);

    for k = 1:size(zones,1)

        c = zones(k,1:2);
        R = zones(k,3);

        d = hypot(P(:,1)-c(1), P(:,2)-c(2));
        clr = d - R;

        minClr = min(minClr, min(clr));

        violationMask = violationMask | (clr < 0);

        % Exposure model:
        %   inside zone: exposure = 1
        %   outside zone: decays smoothly with distance from boundary
        scale = max(0.6*R, 1);
        exp_k = exp(-(max(clr,0)./scale).^2);
        exp_k(clr < 0) = 1;

        exposure = exposure + exp_k;
    end

    violationCount = sum(violationMask);

    if dt > 0
        exposureIntegral = sum(exposure) * dt;
    else
        exposureIntegral = sum(exposure);
    end

end