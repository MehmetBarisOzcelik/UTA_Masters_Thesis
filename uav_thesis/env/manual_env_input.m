function cfg = manual_env_input(cfg)
% MANUAL_ENV_INPUT
% ---------------------------------------------------------------
% Interactive helper for defining a manual mission environment.
%
% This function asks the user for:
%   - start and goal coordinates
%   - checkpoint coordinates
%   - NFZ geometry
%   - radar-zone geometry
%
% The values are stored in:
%   cfg.env.manual.start
%   cfg.env.manual.goal
%   cfg.env.manual.checkpoints
%   cfg.env.manual.nfz
%   cfg.env.manual.radar
%
% These fields are later consumed by threat_map.m when:
%   cfg.env.mode = 'manual'
%
% The default thesis geometry is shown in the prompts, so the user can
% either accept the baseline layout or perturb it manually.
%
% Note:
%   This function provides early warnings for obvious manual overlaps.
%   Final strict validation is performed inside threat_map.m.

fprintf('\n=== Manual Environment Definition ===\n');

B = cfg.env.bounds;

fprintf('Map bounds: x in [%.1f, %.1f], y in [%.1f, %.1f]\n', ...
        B(1,1), B(1,2), B(2,1), B(2,2));

%% ------------------------------------------------------------------------
% Get default thesis geometry for reference
% -------------------------------------------------------------------------
env_tmp      = cfg.env;
env_tmp.mode = 'default';

env_default = threat_map(env_tmp);

%% ------------------------------------------------------------------------
% Start and goal
% -------------------------------------------------------------------------
fprintf('\n--- Start and Goal (defaults in brackets) ---\n');

sx = input(sprintf('Start x  [default %.1f]: ', env_default.start(1)));
if isempty(sx)
    sx = env_default.start(1);
end

sy = input(sprintf('Start y  [default %.1f]: ', env_default.start(2)));
if isempty(sy)
    sy = env_default.start(2);
end

gx = input(sprintf('Goal  x  [default %.1f]: ', env_default.goal(1)));
if isempty(gx)
    gx = env_default.goal(1);
end

gy = input(sprintf('Goal  y  [default %.1f]: ', env_default.goal(2)));
if isempty(gy)
    gy = env_default.goal(2);
end

cfg.env.manual.start = [sx sy];
cfg.env.manual.goal  = [gx gy];

%% ------------------------------------------------------------------------
% Checkpoints
% -------------------------------------------------------------------------
fprintf('\n--- Checkpoints (defaults in brackets where available) ---\n');

n_cp_default = size(env_default.checkpoints,1);

n_cp = input(sprintf('Number of checkpoints [default %d]: ', cfg.env.n_cp));
if isempty(n_cp)
    n_cp = cfg.env.n_cp;
end

CP = zeros(n_cp,2);

for i = 1:n_cp

    if i <= n_cp_default

        cx_def = env_default.checkpoints(i,1);
        cy_def = env_default.checkpoints(i,2);

        prompt_x = sprintf('Checkpoint %d x  [default %.1f]: ', i, cx_def);
        prompt_y = sprintf('Checkpoint %d y  [default %.1f]: ', i, cy_def);

    else

        cx_def = [];
        cy_def = [];

        prompt_x = sprintf('Checkpoint %d x: ', i);
        prompt_y = sprintf('Checkpoint %d y: ', i);
    end

    cx = input(prompt_x);
    if isempty(cx) && ~isempty(cx_def)
        cx = cx_def;
    end

    cy = input(prompt_y);
    if isempty(cy) && ~isempty(cy_def)
        cy = cy_def;
    end

    CP(i,:) = [cx cy];
end

cfg.env.manual.checkpoints = CP;

%% ------------------------------------------------------------------------
% No-fly zones
% -------------------------------------------------------------------------
fprintf('\n--- No-Fly Zones (stored as circles) [cx cy r] ---\n');

n_nfz_default = size(env_default.nfz,1);

n_nfz = input(sprintf('Number of NFZs [default %d]: ', cfg.env.n_nfz));
if isempty(n_nfz)
    n_nfz = cfg.env.n_nfz;
end

NFZ = zeros(n_nfz,3);

for i = 1:n_nfz

    if i <= n_nfz_default

        cxd = env_default.nfz(i,1);
        cyd = env_default.nfz(i,2);
        rd  = env_default.nfz(i,3);

        px = sprintf('NFZ %d center x  [default %.1f]: ', i, cxd);
        py = sprintf('NFZ %d center y  [default %.1f]: ', i, cyd);
        pr = sprintf('NFZ %d radius r  [default %.1f]: ', i, rd);

    else

        cxd = [];
        cyd = [];
        rd  = [];

        px = sprintf('NFZ %d center x: ', i);
        py = sprintf('NFZ %d center y: ', i);
        pr = sprintf('NFZ %d radius  r: ', i);
    end

    cx = input(px);
    if isempty(cx) && ~isempty(cxd)
        cx = cxd;
    end

    cy = input(py);
    if isempty(cy) && ~isempty(cyd)
        cy = cyd;
    end

    r = input(pr);
    if isempty(r) && ~isempty(rd)
        r = rd;
    end

    NFZ(i,:) = [cx cy r];
end

cfg.env.manual.nfz = NFZ;

%% ------------------------------------------------------------------------
% Radar zones
% -------------------------------------------------------------------------
fprintf('\n--- Radar Zones [cx cy r] ---\n');

n_rad_default = size(env_default.radar,1);

n_rad = input(sprintf('Number of radar zones [default %d]: ', cfg.env.n_radar));
if isempty(n_rad)
    n_rad = cfg.env.n_radar;
end

RAD = zeros(n_rad,3);

for i = 1:n_rad

    if i <= n_rad_default

        cxd = env_default.radar(i,1);
        cyd = env_default.radar(i,2);
        rd  = env_default.radar(i,3);

        px = sprintf('Radar %d center x  [default %.1f]: ', i, cxd);
        py = sprintf('Radar %d center y  [default %.1f]: ', i, cyd);
        pr = sprintf('Radar %d radius r  [default %.1f]: ', i, rd);

    else

        cxd = [];
        cyd = [];
        rd  = [];

        px = sprintf('Radar %d center x: ', i);
        py = sprintf('Radar %d center y: ', i);
        pr = sprintf('Radar %d radius  r: ', i);
    end

    cx = input(px);
    if isempty(cx) && ~isempty(cxd)
        cx = cxd;
    end

    cy = input(py);
    if isempty(cy) && ~isempty(cyd)
        cy = cyd;
    end

    r = input(pr);
    if isempty(r) && ~isempty(rd)
        r = rd;
    end

    RAD(i,:) = [cx cy r];
end

cfg.env.manual.radar = RAD;

%% ------------------------------------------------------------------------
% Early warnings
% -------------------------------------------------------------------------
warn_manual_overlap(cfg);

fprintf('\nManual environment stored in cfg.env.manual.\n');
fprintf('Final strict geometry validation will be performed by threat_map.m.\n\n');

end

% ========================================================================
function warn_manual_overlap(cfg)
% WARN_MANUAL_OVERLAP
% ---------------------------------------------------------------
% Issues early warnings if manual start/checkpoint/goal points lie inside
% any manual NFZ or radar zone. These are warnings only; final strict
% validation is handled by threat_map.m.

if ~isfield(cfg,'env') || ~isfield(cfg.env,'manual')
    return;
end

start = cfg.env.manual.start;
goal  = cfg.env.manual.goal;
CP    = cfg.env.manual.checkpoints;
nfz   = cfg.env.manual.nfz;
radar = cfg.env.manual.radar;

pts = [start; CP; goal];

names = ["Start"; ...
         repmat("CP", size(CP,1), 1); ...
         "Goal"];

%% Check against NFZs ------------------------------------------------------

if ~isempty(nfz)

    for i = 1:size(pts,1)

        p = pts(i,:);

        for k = 1:size(nfz,1)

            cx = nfz(k,1);
            cy = nfz(k,2);
            R  = nfz(k,3);

            d = hypot(p(1)-cx, p(2)-cy);

            if d <= R
                warning(['manual_env_input:OverlapNFZ\n' ...
                         '%s point (%.1f, %.1f) lies inside manual NFZ #%d ' ...
                         '(center = [%.1f, %.1f], r = %.1f).'], ...
                        names(i), p(1), p(2), k, cx, cy, R);
            end
        end
    end
end

%% Check against radar zones ----------------------------------------------

if ~isempty(radar)

    for i = 1:size(pts,1)

        p = pts(i,:);

        for k = 1:size(radar,1)

            cx = radar(k,1);
            cy = radar(k,2);
            R  = radar(k,3);

            d = hypot(p(1)-cx, p(2)-cy);

            if d <= R
                warning(['manual_env_input:OverlapRadar\n' ...
                         '%s point (%.1f, %.1f) lies inside manual Radar #%d ' ...
                         '(center = [%.1f, %.1f], r = %.1f).'], ...
                        names(i), p(1), p(2), k, cx, cy, R);
            end
        end
    end
end

end