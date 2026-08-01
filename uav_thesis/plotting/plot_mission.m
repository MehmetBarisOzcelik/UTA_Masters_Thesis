% PLOT_MISSION
% ---------------------------------------------------------------
% Produces a static 2-D plot of the full mission:
%   - Hexagonal NFZs and circular radar zones
%   - Start, checkpoints, and goal markers
%   - Planned risk-aware route
%   - UAV path and pursuer path
%   - Catch point, if capture occurs
%
% The title includes the UAV mode, pursuer mode, final time,
% miss distance, and catch status. This makes figures easier to use
% directly in thesis slides and the thesis document.

function plot_mission(out)

env   = out.env;
route = out.route;
uav   = out.uav;
pn    = out.pn;

figure('Color','w'); clf; hold on; grid on; box on;

%% ------------------------------------------------------------------------
% Radar zones: circular regions
% -------------------------------------------------------------------------
hRadar = [];

if isfield(env,'radar') && ~isempty(env.radar)

    th = linspace(0, 2*pi, 200);

    for i = 1:size(env.radar,1)

        cx = env.radar(i,1);
        cy = env.radar(i,2);
        r  = env.radar(i,3);

        h = fill(cx + r*cos(th), cy + r*sin(th), [1 .8 .8], ...
                 'EdgeColor',[.85 .2 .2], ...
                 'LineWidth',1.2, ...
                 'FaceAlpha',0.35);

        if isempty(hRadar)
            hRadar = h;
        end
    end
end

%% ------------------------------------------------------------------------
% No-fly zones: plotted as hexagons for visual distinction
% -------------------------------------------------------------------------
hNFZ = [];

if isfield(env,'nfz') && ~isempty(env.nfz)

    th_hex = linspace(0, 2*pi, 7);   % 6-sided polygon, closed

    for i = 1:size(env.nfz,1)

        cx = env.nfz(i,1);
        cy = env.nfz(i,2);
        r  = env.nfz(i,3);

        xpoly = cx + r*cos(th_hex);
        ypoly = cy + r*sin(th_hex);

        h = fill(xpoly, ypoly, [1 .7 .7], ...
                 'EdgeColor',[.8 0 0], ...
                 'LineWidth',1.5, ...
                 'FaceAlpha',0.35);

        if isempty(hNFZ)
            hNFZ = h;
        end
    end
end

%% ------------------------------------------------------------------------
% Start, checkpoints, and goal
% -------------------------------------------------------------------------
hStart = plot(env.start(1), env.start(2), 'p', ...
              'MarkerSize',11, ...
              'MarkerFaceColor',[0 .6 0], ...
              'MarkerEdgeColor','k');

hCP = [];

if isfield(env,'checkpoints') && ~isempty(env.checkpoints)

    hCP = plot(env.checkpoints(:,1), env.checkpoints(:,2), 'd', ...
               'MarkerSize',7, ...
               'MarkerFaceColor',[.2 .7 1], ...
               'MarkerEdgeColor','k');
end

hGoal = plot(env.goal(1), env.goal(2), 'p', ...
             'MarkerSize',11, ...
             'MarkerFaceColor',[.95 .75 .15], ...
             'MarkerEdgeColor','k');

%% ------------------------------------------------------------------------
% Planned route
% -------------------------------------------------------------------------
hRoute = [];

if isfield(route,'nodes') && size(route.nodes,1) >= 2

    hRoute = plot(route.nodes(:,1), route.nodes(:,2), '-', ...
                  'Color',[.5 .5 .5], ...
                  'LineWidth',2.0);
end

%% ------------------------------------------------------------------------
% UAV path
% -------------------------------------------------------------------------
hUAV = [];

if isfield(uav,'pos') && ~isempty(uav.pos)

    P = uav.pos;

    % Snap final plotted point to the exact goal if it is within the
    % checkpoint/goal capture radius. This makes the final plot clearer.
    if isfield(out,'cfg') && isfield(out.cfg,'env') && isfield(out.cfg.env,'rC')
        rC = out.cfg.env.rC;
    else
        rC = 60;
    end

    if norm(P(end,:) - env.goal) <= rC
        P(end,:) = env.goal;
    end

    hUAV = plot(P(:,1), P(:,2), '-', ...
                'Color',[0 .45 .74], ...
                'LineWidth',2.4);
end

%% ------------------------------------------------------------------------
% Pursuer path and pursuer start
% -------------------------------------------------------------------------
hPN      = [];
hPNstart = [];

if isfield(pn,'pos') && ~isempty(pn.pos)

    p0 = pn.pos(1,:);

    hPNstart = plot(p0(1), p0(2), 's', ...
                    'MarkerSize',8, ...
                    'MarkerFaceColor',[.9 .2 .9], ...
                    'MarkerEdgeColor','k');

    hPN = plot(pn.pos(:,1), pn.pos(:,2), '--', ...
               'Color','k', ...
               'LineWidth',2.0);
end

%% ------------------------------------------------------------------------
% Catch point
% -------------------------------------------------------------------------
hCatch = [];

if isfield(out,'metrics') && isfield(out.metrics,'catch') && ...
        out.metrics.catch && isfield(out.metrics,'catch_pos')

    cpos = out.metrics.catch_pos;

    if all(isfinite(cpos))
        hCatch = plot(cpos(1), cpos(2), 'x', ...
                      'MarkerSize',10, ...
                      'LineWidth',2.0, ...
                      'Color',[.8 0 0]);
    end

elseif isfield(pn,'catch_pos') && all(isfinite(pn.catch_pos))

    cpos = pn.catch_pos;

    hCatch = plot(cpos(1), cpos(2), 'x', ...
                  'MarkerSize',10, ...
                  'LineWidth',2.0, ...
                  'Color',[.8 0 0]);
end

%% ------------------------------------------------------------------------
% Axes and labels
% -------------------------------------------------------------------------
axis equal;

if isfield(env,'bounds') && ~isempty(env.bounds)
    xlim(env.bounds(1,:));
    ylim(env.bounds(2,:));
end

xlabel('x [m]');
ylabel('y [m]');

%% ------------------------------------------------------------------------
% Legend
% -------------------------------------------------------------------------
H = [];
L = {};

if ~isempty(hRadar)
    H(end+1) = hRadar;
    L{end+1} = 'Radar Zones';
end

if ~isempty(hNFZ)
    H(end+1) = hNFZ;
    L{end+1} = 'No-Fly Zones';
end

if ~isempty(hStart)
    H(end+1) = hStart;
    L{end+1} = 'UAV Start';
end

if ~isempty(hPNstart)
    H(end+1) = hPNstart;
    L{end+1} = 'Chaser Start';
end

if ~isempty(hCP)
    H(end+1) = hCP;
    L{end+1} = 'Checkpoints';
end

if ~isempty(hGoal)
    H(end+1) = hGoal;
    L{end+1} = 'Goal';
end

if ~isempty(hRoute)
    H(end+1) = hRoute;
    L{end+1} = 'Planned Route';
end

if ~isempty(hUAV)
    H(end+1) = hUAV;
    L{end+1} = 'UAV Path';
end

if ~isempty(hPN)
    H(end+1) = hPN;
    L{end+1} = 'Chaser Path';
end

if ~isempty(hCatch)
    H(end+1) = hCatch;
    L{end+1} = 'Catch Point';
end

if ~isempty(H)
    legend(H, L, 'Location','northeastoutside');
end

%% ------------------------------------------------------------------------
% Title using mode and metrics
% -------------------------------------------------------------------------
titlePrefix = 'Trajectory';

if isfield(out,'cfg') && ...
   isfield(out.cfg,'uav') && isfield(out.cfg.uav,'mode') && ...
   isfield(out.cfg,'pn')  && isfield(out.cfg.pn,'mode')

    titlePrefix = sprintf('Trajectory: UAV = %s, Chaser = %s', ...
                          upper(out.cfg.uav.mode), ...
                          upper(out.cfg.pn.mode));
end

if isfield(out,'metrics')

    % Final time
    if isfield(out.metrics,'final_time')
        tfin = out.metrics.final_time;
    elseif isfield(out.metrics,'t_final')
        tfin = out.metrics.t_final;
    else
        tfin = NaN;
    end

    % Miss distance
    if isfield(out.metrics,'miss_distance')
        md = out.metrics.miss_distance;
    else
        md = NaN;
    end

    % Catch status
    if isfield(out.metrics,'catch')
        catchText = tern(out.metrics.catch, 'catch = YES', 'catch = no');
    else
        catchText = 'catch = n/a';
    end

    if ~isnan(tfin) && ~isnan(md)

        title({titlePrefix, ...
               sprintf('t_{final} = %.1f s | miss = %.1f m | %s', ...
                       tfin, md, catchText)});

    elseif ~isnan(md)

        title({titlePrefix, ...
               sprintf('miss distance = %.1f m | %s', md, catchText)});

    else

        title(titlePrefix);
    end

else
    title(titlePrefix);
end

end

% ========================================================================
function s = tern(c, a, b)
% TERN
% ---------------------------------------------------------------
% Small helper for inline conditional strings.

if c
    s = a;
else
    s = b;
end

end