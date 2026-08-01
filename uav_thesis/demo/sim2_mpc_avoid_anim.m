function sim2_mpc_avoid_anim()
% SIM2_MPC_AVOID_ANIM
% ---------------------------------------------------------------
% Animated version of Thesis Demo 2:
%   MPC vs No-MPC avoidance around two circular no-fly zones.
%
% Purpose:
%   Visually demonstrates that MPC bends the UAV trajectory around NFZs,
%   while the no-MPC direct go-to-goal controller has no NFZ awareness.
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
% Outputs:
%   - sim2_mpc_avoid_anim.mp4
%   - sim2_mpc_avoid_anim.gif
%
% Author:
%   Mehmet Barış Özçelik

clc; 
close all;

fprintf('=== Demo 2 Animation: MPC vs No-MPC NFZ Avoidance ===\n');

%% ========================================================================
% 1. Scenario geometry
% ========================================================================

p0 = [-2800,    0];      % Start [m]
pg = [ 2800, 2000];      % Goal  [m]

% Two circular NFZs between start and goal
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

env.start       = p0;
env.goal        = pg;
env.checkpoints = [];
env.nfz         = nfz;
env.radar       = [];
env.bounds      = [-3500 3500;
                    -500 2600];

r_goal = cfg.env.r_goal;

%% ========================================================================
% 3. Initialize No-MPC case
% ========================================================================

u_nm.x   = env.start(1);
u_nm.y   = env.start(2);
u_nm.psi = atan2(env.goal(2) - env.start(2), ...
                 env.goal(1) - env.start(1));
u_nm.v   = cfg.uav.v_nom;

%% ========================================================================
% 4. Initialize MPC case
% ========================================================================

u_m.x   = env.start(1);
u_m.y   = env.start(2);
u_m.psi = atan2(env.goal(2) - env.start(2), ...
                env.goal(1) - env.start(1));
u_m.v   = cfg.uav.v_nom;

route_nodes      = [env.start; env.goal];
mpc.route_wp     = route_nodes;
mpc.route_wp_idx = 1:size(route_nodes,1);
mpc.wp_idx       = 2;

% Dummy chaser argument. Demo 2 does not include a chaser.
pn_dummy = [];

%% ========================================================================
% 5. Logs and metrics
% ========================================================================

tvec   = nan(Kmax,1);
pos_nm = nan(Kmax,2);
pos_m  = nan(Kmax,2);

L_nm = 0;
L_m  = 0;

t_goal_nm = NaN;
t_goal_m  = NaN;

done_nm = false;
done_m  = false;

t = 0;
k = 1;

while t <= Tmax && k <= Kmax

    % Log current states
    tvec(k)     = t;
    pos_nm(k,:) = [u_nm.x, u_nm.y];
    pos_m(k,:)  = [u_m.x,  u_m.y];

    % ---------------------------------------------------------------------
    % No-MPC update: direct go-to-goal steering
    % ---------------------------------------------------------------------
    if ~done_nm

        x_old = u_nm.x;
        y_old = u_nm.y;

        psi_des = atan2(env.goal(2) - u_nm.y, ...
                        env.goal(1) - u_nm.x);

        err = wrap_angle(psi_des - u_nm.psi);

        omega = cfg.trk.k_psi * err;
        omega = max(-cfg.uav.omega_max, min(cfg.uav.omega_max, omega));

        u_nm.psi = wrap_angle(u_nm.psi + omega*dt);
        u_nm.x   = u_nm.x + u_nm.v * cos(u_nm.psi) * dt;
        u_nm.y   = u_nm.y + u_nm.v * sin(u_nm.psi) * dt;

        L_nm = L_nm + hypot(u_nm.x - x_old, u_nm.y - y_old);

        if hypot(u_nm.x - env.goal(1), u_nm.y - env.goal(2)) <= r_goal
            done_nm   = true;
            t_goal_nm = t;
        end
    end

    % ---------------------------------------------------------------------
    % MPC update: thesis MPC controller
    % ---------------------------------------------------------------------
    if ~done_m

        x_old = u_m.x;
        y_old = u_m.y;

        [u_cmd, mpc] = mpc_step(mpc, u_m, pn_dummy, env, cfg);

        u_m = advance_states(u_m, u_cmd, dt, cfg);

        [u_m, mpc] = waypoint_manager(u_m, mpc, env, cfg);

        L_m = L_m + hypot(u_m.x - x_old, u_m.y - y_old);

        if hypot(u_m.x - env.goal(1), u_m.y - env.goal(2)) <= r_goal
            done_m   = true;
            t_goal_m = t;
        end
    end

    % Stop early if both cases are complete
    if done_nm && done_m
        break;
    end

    t = t + dt;
    k = k + 1;
end

% Trim logs
K = k;

tvec   = tvec(1:K);
pos_nm = pos_nm(1:K,:);
pos_m  = pos_m(1:K,:);

%% ========================================================================
% 6. Metrics
% ========================================================================

[minClr_nm, which_nm] = min_clearance_to_nfz(pos_nm, nfz);
[minClr_m,  which_m ] = min_clearance_to_nfz(pos_m,  nfz);

fprintf('Fairness note: No-MPC and MPC use the same UAV dynamics and speed.\n');
fprintf('               Only the guidance strategy differs.\n\n');

fprintf('No-MPC case:\n');
fprintf('  Goal time              : %s\n', ...
        ternum(~isnan(t_goal_nm), sprintf('%.1f s', t_goal_nm), 'no goal'));
fprintf('  Path length            : %.1f m\n', L_nm);
fprintf('  Minimum NFZ clearance  : %.1f m (NFZ #%d)\n\n', ...
        minClr_nm, which_nm);

fprintf('MPC case:\n');
fprintf('  Goal time              : %s\n', ...
        ternum(~isnan(t_goal_m), sprintf('%.1f s', t_goal_m), 'no goal'));
fprintf('  Path length            : %.1f m\n', L_m);
fprintf('  Minimum NFZ clearance  : %.1f m (NFZ #%d)\n', ...
        minClr_m, which_m);

fprintf('\nComparison:\n');
fprintf('  Clearance improvement  : %.1f m\n', minClr_m - minClr_nm);
fprintf('===============================================================\n');

%% ========================================================================
% 7. Figure setup
% ========================================================================

km = 1e-3;

fig = figure('Position',[80 80 1100 700], 'Color','w');
clf;

ax = axes('Parent',fig);
hold(ax,'on');
grid(ax,'on');
box(ax,'on');
axis(ax,'equal');

xlabel(ax,'x [km]');
ylabel(ax,'y [km]');

xlim(ax, env.bounds(1,:)*km);
ylim(ax, env.bounds(2,:)*km);

% NFZs
th = linspace(0, 2*pi, 300);
hNFZ = gobjects(size(nfz,1),1);

for i = 1:size(nfz,1)

    cx = nfz(i,1);
    cy = nfz(i,2);
    R  = nfz(i,3);

    hx = cx + R*cos(th);
    hy = cy + R*sin(th);

    hNFZ(i) = patch(ax, hx*km, hy*km, [1 .7 .7], ...
                    'EdgeColor',[0.6 0 0], ...
                    'LineWidth',1.6, ...
                    'FaceAlpha',0.35, ...
                    'DisplayName',sprintf('NFZ #%d',i));
end

% Direct line from start to goal for context
hLOS = plot(ax, [env.start(1), env.goal(1)]*km, ...
                [env.start(2), env.goal(2)]*km, ...
                ':', ...
                'Color',[0.4 0.4 0.4], ...
                'LineWidth',1.2, ...
                'DisplayName','Start-goal line');

% Start and goal markers
hStart = plot(ax, env.start(1)*km, env.start(2)*km, 'ko', ...
              'MarkerFaceColor','k', ...
              'MarkerSize',7, ...
              'DisplayName','Start');

hGoal = plot(ax, env.goal(1)*km, env.goal(2)*km, 'kp', ...
             'MarkerFaceColor',[0.95 0.75 0.15], ...
             'MarkerSize',11, ...
             'DisplayName','Goal');

% Trails
hTrail_nm = plot(ax, nan, nan, '--', ...
                 'LineWidth',2.3, ...
                 'Color',[0.4 0.4 0.4], ...
                 'DisplayName','No-MPC path');

hTrail_m = plot(ax, nan, nan, '-', ...
                'LineWidth',2.6, ...
                'Color',[0.90 0.70 0.10], ...
                'DisplayName','MPC path');

% Current-position markers
hDot_nm = plot(ax, nan, nan, 'o', ...
               'MarkerSize',6, ...
               'MarkerFaceColor',[0.4 0.4 0.4], ...
               'MarkerEdgeColor','k', ...
               'DisplayName','No-MPC vehicle');

hDot_m = plot(ax, nan, nan, 's', ...
              'MarkerSize',7, ...
              'MarkerFaceColor',[0.90 0.70 0.10], ...
              'MarkerEdgeColor','k', ...
              'DisplayName','MPC vehicle');

title(ax, {'Demo 2: MPC vs No-MPC NFZ Avoidance', ...
           sprintf('No-MPC min clearance = %.0f m | MPC min clearance = %.0f m', ...
                   minClr_nm, minClr_m)});

legend(ax, [hNFZ(:).' hStart hGoal hLOS hTrail_nm hTrail_m hDot_nm hDot_m], ...
       {'NFZ #1','NFZ #2','Start','Goal','Start-goal line', ...
        'No-MPC path','MPC path','No-MPC vehicle','MPC vehicle'}, ...
       'Location','northwest');

%% ========================================================================
% 8. Video writer and GIF setup
% ========================================================================

fps  = max(10, min(30, round(1/dt)));
step = max(1, round(fps/10));

make_mp4 = true;
make_gif = true;

if make_mp4
    v = VideoWriter('sim2_mpc_avoid_anim.mp4', 'MPEG-4');
    v.FrameRate = fps;
    open(v);
end

gifname = 'sim2_mpc_avoid_anim.gif';
gif_written = false;

%% ========================================================================
% 9. Animation loop
% ========================================================================

for k = 1:step:K

    set(hTrail_nm, ...
        'XData', pos_nm(1:k,1)*km, ...
        'YData', pos_nm(1:k,2)*km);

    set(hTrail_m, ...
        'XData', pos_m(1:k,1)*km, ...
        'YData', pos_m(1:k,2)*km);

    set(hDot_nm, ...
        'XData', pos_nm(k,1)*km, ...
        'YData', pos_nm(k,2)*km);

    set(hDot_m, ...
        'XData', pos_m(k,1)*km, ...
        'YData', pos_m(k,2)*km);

    title(ax, {'Demo 2: MPC vs No-MPC NFZ Avoidance', ...
               sprintf('t = %.1f s | No-MPC clr = %.0f m | MPC clr = %.0f m', ...
                       tvec(k), minClr_nm, minClr_m)});

    drawnow;

    frame = getframe(fig);

    if make_mp4
        writeVideo(v, frame);
    end

    if make_gif
        [A,map] = rgb2ind(frame2im(frame), 256);

        if ~gif_written
            imwrite(A, map, gifname, 'gif', ...
                    'LoopCount', inf, ...
                    'DelayTime', 0.08);

            gif_written = true;
        else
            imwrite(A, map, gifname, 'gif', ...
                    'WriteMode', 'append', ...
                    'DelayTime', 0.08);
        end
    end
end

%% ========================================================================
% 10. Close writer
% ========================================================================

if make_mp4
    close(v);
end

fprintf('Saved animation files:\n');

if make_mp4
    fprintf('  sim2_mpc_avoid_anim.mp4\n');
end

if make_gif
    fprintf('  %s\n', gifname);
end

end

%% ========================================================================
% Helper: wrap angle to [-pi, pi]
% ========================================================================
function a = wrap_angle(th)

a = atan2(sin(th), cos(th));

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

% ========================================================================
% Helper: ternary for strings
% ========================================================================
function s = ternum(cond, a, b)

if cond
    s = a;
else
    s = b;
end

end