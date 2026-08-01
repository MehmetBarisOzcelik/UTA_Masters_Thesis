% ANIMATE_MISSION
% ---------------------------------------------------------------
% Generates a mission animation and saves:
%   - MP4 video file
%   - GIF file
%
% The animation shows:
%   - radar zones and no-fly zones
%   - start, checkpoints, and goal
%   - planned risk-aware route
%   - moving UAV and pursuer trails
%   - catch point, if capture occurs
%
% The title includes the UAV mode, pursuer mode, current time,
% miss distance, and catch status.
%
% Filename base is given by basename, for example:
%   animate_mission(out, 'mission_strictNFZ')

function animate_mission(out, basename)

if nargin < 2 || isempty(basename)
    basename = 'anim_mission';
end

env   = out.env;
route = out.route;
uav   = out.uav;

pn = [];
if isfield(out,'pn')
    pn = out.pn;
end

met = out.metrics;

t = uav.t(:);
K = numel(t);

if K < 2
    warning('animate_mission:short','Not enough samples to animate.');
    return;
end

dt   = median(diff(t));
fps  = max(10, round(1/dt));
step = max(1, round(fps/10));

%% ------------------------------------------------------------------------
% Figure setup
% -------------------------------------------------------------------------
fig = figure('Color','w','Position',[80 80 950 720]);
clf;

ax = axes('Parent',fig);
hold(ax,'on');
grid(ax,'on');
box(ax,'on');

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

        h = fill(ax, cx + r*cos(th), cy + r*sin(th), [1 .8 .8], ...
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

    th_hex = linspace(0, 2*pi, 7);  % 6-sided polygon, closed

    for i = 1:size(env.nfz,1)

        cx = env.nfz(i,1);
        cy = env.nfz(i,2);
        r  = env.nfz(i,3);

        xx = cx + r*cos(th_hex);
        yy = cy + r*sin(th_hex);

        h = fill(ax, xx, yy, [1 .7 .7], ...
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
hStart = plot(ax, env.start(1), env.start(2), 'p', ...
              'MarkerSize',11, ...
              'MarkerFaceColor',[0 .6 0], ...
              'MarkerEdgeColor','k');

hCP = [];

if isfield(env,'checkpoints') && ~isempty(env.checkpoints)

    hCP = plot(ax, env.checkpoints(:,1), env.checkpoints(:,2), 'd', ...
               'MarkerSize',7, ...
               'MarkerFaceColor',[.2 .7 1], ...
               'MarkerEdgeColor','k');
end

hGoal = plot(ax, env.goal(1), env.goal(2), 'p', ...
             'MarkerSize',11, ...
             'MarkerFaceColor',[.95 .75 .15], ...
             'MarkerEdgeColor','k');

%% ------------------------------------------------------------------------
% Planned route
% -------------------------------------------------------------------------
hRoute = [];

if isfield(route,'nodes') && size(route.nodes,1) >= 2

    hRoute = plot(ax, route.nodes(:,1), route.nodes(:,2), '-', ...
                  'Color',[.5 .5 .5], ...
                  'LineWidth',2.0);
end

%% ------------------------------------------------------------------------
% Moving trails
% -------------------------------------------------------------------------
hUtrail = plot(ax, nan, nan, '-', ...
               'Color',[0 .45 .74], ...
               'LineWidth',2.4);

hPtrail = plot(ax, nan, nan, '--', ...
               'Color','k', ...
               'LineWidth',2.0);

%% ------------------------------------------------------------------------
% Catch marker
% -------------------------------------------------------------------------
hCatch = [];

if isfield(met,'catch') && met.catch && isfield(met,'catch_pos')

    cpos = met.catch_pos;

    if all(isfinite(cpos))
        hCatch = plot(ax, cpos(1), cpos(2), 'x', ...
                      'Color',[0.85 0 0], ...
                      'LineWidth',2.2, ...
                      'MarkerSize',11);
    end
end

%% ------------------------------------------------------------------------
% Axes and labels
% -------------------------------------------------------------------------
axis(ax,'equal');

if isfield(env,'bounds') && ~isempty(env.bounds)
    xlim(ax, env.bounds(1,:));
    ylim(ax, env.bounds(2,:));
end

xlabel(ax,'x [m]');
ylabel(ax,'y [m]');

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

H(end+1) = hUtrail;
L{end+1} = 'UAV Path';

H(end+1) = hPtrail;
L{end+1} = 'Chaser Path';

if ~isempty(hCatch)
    H(end+1) = hCatch;
    L{end+1} = 'Catch Point';
end

legend(ax, H, L, 'Location','northeastoutside');

%% ------------------------------------------------------------------------
% Title prefix
% -------------------------------------------------------------------------
titlePrefix = 'Trajectory';

if isfield(out,'cfg') && ...
   isfield(out.cfg,'uav') && isfield(out.cfg.uav,'mode') && ...
   isfield(out.cfg,'pn')  && isfield(out.cfg.pn,'mode')

    titlePrefix = sprintf('Trajectory: UAV = %s, Chaser = %s', ...
                          upper(out.cfg.uav.mode), ...
                          upper(out.cfg.pn.mode));
end

if isfield(met,'catch')
    catchText = tern(met.catch, 'catch = YES', 'catch = no');
else
    catchText = 'catch = n/a';
end

if isfield(met,'miss_distance')
    missDistance = met.miss_distance;
else
    missDistance = NaN;
end

%% ------------------------------------------------------------------------
% Writers
% -------------------------------------------------------------------------
make_mp4 = true;
make_gif = true;

if make_mp4
    v = VideoWriter([basename '.mp4'], 'MPEG-4');
    v.FrameRate = min(30, fps);
    open(v);
end

gifname = [basename '.gif'];
gif_written = false;

%% ------------------------------------------------------------------------
% Animation loop
% -------------------------------------------------------------------------
for k = 1:step:K

    set(hUtrail, ...
        'XData', uav.pos(1:k,1), ...
        'YData', uav.pos(1:k,2));

    if ~isempty(pn) && isfield(pn,'pos') && size(pn.pos,1) >= k
        set(hPtrail, ...
            'XData', pn.pos(1:k,1), ...
            'YData', pn.pos(1:k,2));
    end

    if ~isnan(missDistance)
        title(ax, {titlePrefix, ...
             sprintf('t = %.1f s | miss = %.1f m | %s', ...
                     t(k), missDistance, catchText)});
    else
        title(ax, {titlePrefix, ...
             sprintf('t = %.1f s | %s', t(k), catchText)});
    end

    drawnow;

    fr = getframe(fig);

    if make_mp4
        writeVideo(v, fr);
    end

    if make_gif
        [A,map] = rgb2ind(frame2im(fr), 256);

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

%% ------------------------------------------------------------------------
% Close video writer
% -------------------------------------------------------------------------
if make_mp4
    close(v);
end

fprintf('Saved animation(s): %s.mp4 and/or %s\n', basename, gifname);

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