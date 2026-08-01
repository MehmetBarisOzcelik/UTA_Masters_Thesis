function sim2_mpc_avoid()
% SIM2_MPC_AVOID
% ---------------------------------------------------------------
% Thesis Demo 2:
%   MPC vs No-MPC avoidance around two circular no-fly zones.
%
% Purpose:
%   Isolates the effect of MPC on obstacle avoidance using the same UAV
%   kinematic model and nominal speed in both cases.
%
% Cases:
%   Case A: No-MPC guidance
%       - Direct go-to-goal steering
%       - No NFZ awareness
%
%   Case B: MPC guidance
%       - Uses the thesis MPC controller
%       - Penalizes proximity to NFZ boundaries
%       - Avoids entering keep-out regions
%
% Fairness:
%   Both cases use:
%       - same start and goal
%       - same UAV speed limits
%       - same nominal speed
%       - same turn-rate limit
%       - same time step
%
%   The only difference is the guidance strategy.
%
% Output:
%   - Figure: sim2_mpc_avoid.png
%   - Console summary of final time, path length, and minimum NFZ clearance
%
% Author:
%   Mehmet Barış Özçelik

clc; clear; close all;

fprintf('=== Demo 2: MPC vs No-MPC NFZ Avoidance ===\n');

%% ========================================================================
% 1. Scenario geometry
% ========================================================================

% Start and goal are selected so the direct no-MPC path crosses or closely
% skirts the NFZs, while MPC must bend around the keep-out regions.
p0 = [-2800,    0];      % Start [m]
pg = [ 2800, 2000];      % Goal  [m]

% Two circular no-fly zones between start and goal
c1 = [   0,  800];
R1 = 700;

c2 = [1800, 1600];
R2 = 600;

nfz = [c1 R1;
       c2 R2];

%% ========================================================================
% 2. Configuration and environment struct
% ========================================================================

cfg = config();

dt   = cfg.dt;
Tmax = min(260, cfg.Tmax);
Kmax = ceil(Tmax / dt) + 2;

% Minimal environment struct expected by mpc_step and waypoint_manager
env.start       = p0;
env.goal        = pg;
env.checkpoints = [];
env.nfz         = nfz;
env.radar       = [];
env.bounds      = [-3500 3500;
                    -500 2600];

r_goal = cfg.env.r_goal;

%% ========================================================================
% 3. Case A: No-MPC direct go-to-goal guidance
% ========================================================================

u_nm.x   = env.start(1);
u_nm.y   = env.start(2);
u_nm.psi = atan2(env.goal(2) - env.start(2), ...
                 env.goal(1) - env.start(1));
u_nm.v   = cfg.uav.v_nom;

tlog_nm = nan(Kmax,1);
pos_nm  = nan(Kmax,2);

t = 0;

for k = 1:Kmax

    % Log current state
    tlog_nm(k)  = t;
    pos_nm(k,:) = [u_nm.x, u_nm.y];

    % Direct heading command toward goal
    psi_des = atan2(env.goal(2) - u_nm.y, env.goal(1) - u_nm.x);
    err     = wrap_angle(psi_des - u_nm.psi);

    omega = cfg.trk.k_psi * err;
    omega = max(-cfg.uav.omega_max, min(cfg.uav.omega_max, omega));

    % Same kinematic model used in the MPC comparison
    u_nm.psi = wrap_angle(u_nm.psi + omega*dt);
    u_nm.x   = u_nm.x + u_nm.v * cos(u_nm.psi) * dt;
    u_nm.y   = u_nm.y + u_nm.v * sin(u_nm.psi) * dt;

    % Goal check
    if hypot(u_nm.x - env.goal(1), u_nm.y - env.goal(2)) <= r_goal
        t = t + dt;

        if k + 1 <= Kmax
            tlog_nm(k+1)  = t;
            pos_nm(k+1,:) = [u_nm.x, u_nm.y];
        end

        break;
    end

    t = t + dt;
end

valid_nm = ~isnan(tlog_nm);
tlog_nm  = tlog_nm(valid_nm);
pos_nm   = pos_nm(valid_nm,:);

%% ========================================================================
% 4. Case B: MPC avoidance guidance
% ========================================================================

u_m.x   = env.start(1);
u_m.y   = env.start(2);
u_m.psi = atan2(env.goal(2) - env.start(2), ...
                env.goal(1) - env.start(1));
u_m.v   = cfg.uav.v_nom;

% Route consists only of start and goal.
route_nodes      = [env.start; env.goal];
mpc.route_wp     = route_nodes;
mpc.route_wp_idx = 1:size(route_nodes,1);
mpc.wp_idx       = 2;

% Dummy chaser argument. Demo 2 does not include a chaser.
pn_dummy = [];

tlog_m = nan(Kmax,1);
pos_m  = nan(Kmax,2);

t    = 0;
done = false;
k    = 1;

while ~done && t <= Tmax && k <= Kmax

    % MPC guidance
    [u_cmd, mpc] = mpc_step(mpc, u_m, pn_dummy, env, cfg);

    % Same thesis UAV dynamics
    u_m = advance_states(u_m, u_cmd, dt, cfg);

    % Goal/waypoint handling
    [u_m, mpc] = waypoint_manager(u_m, mpc, env, cfg);

    % Log
    tlog_m(k)  = t;
    pos_m(k,:) = [u_m.x, u_m.y];

    % Goal check
    if hypot(u_m.x - env.goal(1), u_m.y - env.goal(2)) <= r_goal
        done = true;
    end

    t = t + dt;
    k = k + 1;
end

valid_m = ~isnan(tlog_m);
tlog_m  = tlog_m(valid_m);
pos_m   = pos_m(valid_m,:);

%% ========================================================================
% 5. Metrics
% ========================================================================

L_nm = path_length(pos_nm);
L_m  = path_length(pos_m);

[minClr_nm, which_nm] = min_clearance_to_nfz(pos_nm, nfz);
[minClr_m,  which_m ] = min_clearance_to_nfz(pos_m,  nfz);

fprintf('Fairness note: No-MPC and MPC use the same UAV dynamics and speed.\n');
fprintf('               Only the guidance strategy differs.\n\n');

fprintf('No-MPC case:\n');
if hypot(pos_nm(end,1)-env.goal(1), pos_nm(end,2)-env.goal(2)) <= r_goal
    fprintf('  Reached goal at t = %.1f s\n', tlog_nm(end));
else
    fprintf('  Did NOT reach goal. Simulation ended at t = %.1f s\n', tlog_nm(end));
end
fprintf('  Path length              : %.1f m\n', L_nm);
fprintf('  Minimum NFZ clearance    : %.1f m (NFZ #%d)\n\n', ...
        minClr_nm, which_nm);

fprintf('MPC case:\n');
if hypot(pos_m(end,1)-env.goal(1), pos_m(end,2)-env.goal(2)) <= r_goal
    fprintf('  Reached goal at t = %.1f s\n', tlog_m(end));
else
    fprintf('  Did NOT reach goal. Simulation ended at t = %.1f s\n', tlog_m(end));
end
fprintf('  Path length              : %.1f m\n', L_m);
fprintf('  Minimum NFZ clearance    : %.1f m (NFZ #%d)\n', ...
        minClr_m, which_m);

fprintf('\nComparison:\n');
fprintf('  Path-length ratio No-MPC/MPC : %.2f\n', L_nm / max(L_m,1e-6));
fprintf('  Clearance improvement        : %.1f m\n', minClr_m - minClr_nm);
fprintf('=================================================\n');

%% ========================================================================
% 6. Plot
% ========================================================================

km = 1e-3;

figure('Position',[80 80 1100 700], 'Color','w');
hold on;
grid on;
box on;
axis equal;

xlabel('x [km]');
ylabel('y [km]');

% NFZs
th = linspace(0, 2*pi, 300);
hNFZ = gobjects(size(nfz,1),1);

for i = 1:size(nfz,1)

    cx = nfz(i,1);
    cy = nfz(i,2);
    R  = nfz(i,3);

    hx = cx + R*cos(th);
    hy = cy + R*sin(th);

    hNFZ(i) = patch(hx*km, hy*km, [1 .7 .7], ...
                    'EdgeColor',[0.6 0 0], ...
                    'LineWidth',1.6, ...
                    'FaceAlpha',0.35, ...
                    'DisplayName',sprintf('NFZ #%d',i));
end

% Trajectories
hNoMPC = plot(pos_nm(:,1)*km, pos_nm(:,2)*km, '--', ...
              'LineWidth',2.3, ...
              'Color',[0.4 0.4 0.4], ...
              'DisplayName','No-MPC path');

hMPC = plot(pos_m(:,1)*km, pos_m(:,2)*km, '-', ...
            'LineWidth',2.6, ...
            'Color',[0.90 0.70 0.10], ...
            'DisplayName','MPC path');

% Start and goal
hStart = plot(env.start(1)*km, env.start(2)*km, 'ko', ...
              'MarkerFaceColor','k', ...
              'MarkerSize',7, ...
              'DisplayName','Start');

hGoal = plot(env.goal(1)*km, env.goal(2)*km, 'kp', ...
             'MarkerFaceColor',[0.95 0.75 0.15], ...
             'MarkerSize',11, ...
             'DisplayName','Goal');

title({'Demo 2: MPC vs No-MPC NFZ Avoidance', ...
       sprintf('No-MPC min clearance = %.0f m | MPC min clearance = %.0f m', ...
               minClr_nm, minClr_m)});

legend([hNFZ(:).' hStart hGoal hNoMPC hMPC], ...
       {'NFZ #1','NFZ #2','Start','Goal','No-MPC path','MPC path'}, ...
       'Location','northwest');

xlim(env.bounds(1,:)*km);
ylim(env.bounds(2,:)*km);

exportgraphics(gcf, 'sim2_mpc_avoid.png', 'Resolution', 220);

fprintf('Saved figure: sim2_mpc_avoid.png\n');

end

% ========================================================================
% Helper: wrap angle to [-pi, pi]
% ========================================================================
function a = wrap_angle(th)

a = atan2(sin(th), cos(th));

end

% ========================================================================
% Helper: path length from a sequence of points
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
% Helper: minimum clearance from a path to any NFZ
% ========================================================================
function [minClr, idx] = min_clearance_to_nfz(P, nfz)

minClr = inf;
idx    = NaN;

for k = 1:size(nfz,1)

    c = nfz(k,1:2);
    R = nfz(k,3);

    d   = sqrt((P(:,1)-c(1)).^2 + (P(:,2)-c(2)).^2);
    clr = min(d) - R;

    if clr < minClr
        minClr = clr;
        idx    = k;
    end
end

end