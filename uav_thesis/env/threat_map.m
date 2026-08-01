function env = threat_map(env_cfg)
% THREAT_MAP
% ---------------------------------------------------------------
% Constructs the mission environment used in the thesis.
%
% Modes (env_cfg.mode):
%   'default' : fixed thesis layout (used for all default runs)
%   'manual'  : use cfg.env.manual.* (from manual_env_input.m)
%
% For all modes, this function ensures:
%   - Start / goal / checkpoints are outside all NFZ/radar disks.
%   - NFZ and radar disks do NOT overlap each other.
%
% Outputs a struct "env" with fields:
%   env.start, env.goal, env.checkpoints, env.rC, env.r_goal
%   env.nfz [cx cy r], env.radar [cx cy r]
%   env.bounds, env.strict, env.nfz_tol, env.cp_clear, env.edge_margin

    % Determine mode (default if missing)
    if isfield(env_cfg,'mode') && ~isempty(env_cfg.mode)
        mode = lower(env_cfg.mode);
    else
        mode = 'default';
    end

    % Dispatch by mode (only default + manual now)
    switch mode
        case 'manual'
            env = build_manual_env(env_cfg);

        otherwise
            % 'default' or anything else → fixed thesis layout
            env = build_default_env(env_cfg);
    end

    % Common flags / defaults
    if ~isfield(env,'strict')   || isempty(env.strict)
        env.strict = true;
    end
    if ~isfield(env,'nfz_tol')  || isempty(env.nfz_tol)
        env.nfz_tol = 10;
    end
    if ~isfield(env,'cp_clear') || isempty(env.cp_clear)
        env.cp_clear = 150;
    end
    if ~isfield(env,'edge_margin') || isempty(env.edge_margin)
        env.edge_margin = 0.4 * env.cp_clear;
    end
    if ~isfield(env,'bounds') || isempty(env.bounds)
        env.bounds = [-1200 1200; -900 900];
    end

    % Ensure zone–zone non-overlap
    check_zone_overlaps(env);

    % Final safety check: no mission point inside NFZ or radar
    check_zero_overlap(env);
end

% ========================================================================
% DEFAULT LAYOUT (thesis environment)
% ========================================================================
function env = build_default_env(env_cfg)
    env = env_cfg;

    % Map bounds
    if ~isfield(env,'bounds') || isempty(env.bounds)
        env.bounds = [-1200 1200; -900 900];
    end

    % Core geometry (as in the thesis figures)
    env.start = [-600,   0];     % UAV start
    env.goal  = [ 600,  200];    % goal

    env.checkpoints = [
        -200,  700;              % CP1
         700, -600               % CP2
    ];

    env.rC     = env_cfg.rC;
    env.r_goal = env_cfg.r_goal;
    env.n_cp   = size(env.checkpoints,1);

    % No-fly zones (stored as circles; plotted as hexagons)
    env.nfz = [
       -400,  350, 180;          % NFZ #1 (near CP1 leg)
        650, -250, 180           % NFZ #2 (near CP2 leg)
    ];
    env.n_nfz = size(env.nfz,1);

    % Radar zones (circles)
    env.radar = [
        250, 120, 160            % [cx cy r] near middle leg
    ];
    env.n_radar = size(env.radar,1);
end

% ========================================================================
% MANUAL LAYOUT (from cfg.env.manual.*)
% ========================================================================
function env = build_manual_env(env_cfg)
    % Expect fields populated by manual_env_input.m
    if ~isfield(env_cfg,'manual')
        error('threat_map:manualMissing', ...
              'env_cfg.mode = ''manual'' but env_cfg.manual.* is missing.');
    end

    env = env_cfg;

    % Bounds
    if ~isfield(env,'bounds') || isempty(env.bounds)
        env.bounds = [-1200 1200; -900 900];
    end

    % Mission geometry from manual struct
    env.start       = env_cfg.manual.start;
    env.goal        = env_cfg.manual.goal;
    env.checkpoints = env_cfg.manual.checkpoints;
    env.nfz         = env_cfg.manual.nfz;
    env.radar       = env_cfg.manual.radar;

    env.rC      = env_cfg.rC;
    env.r_goal  = env_cfg.r_goal;
    env.n_cp    = size(env.checkpoints,1);
    env.n_nfz   = size(env.nfz,1);
    env.n_radar = size(env.radar,1);
end

% ========================================================================
% MISSION-POINT SAFETY CHECK: Start / checkpoints / goal vs NFZ / radar
% ========================================================================
function check_zero_overlap(env)
% Hard condition:
%   Mission points must lie outside every NFZ and radar disk.
%
% Soft warning:
%   If a mission point is outside a zone but closer than cp_clear to its
%   boundary, issue a warning.

    pts   = [env.start; env.checkpoints; env.goal];
    names = ["Start"; ...
             repmat("CP", size(env.checkpoints,1), 1); ...
             "Goal"];

    if isfield(env,'cp_clear') && ~isempty(env.cp_clear)
        cp_clear = env.cp_clear;
    else
        cp_clear = 150;
    end

    % ---- Against NFZs --------------------------------------------------
    if isfield(env,'nfz') && ~isempty(env.nfz)
        for i = 1:size(pts,1)
            p = pts(i,:);
            for k = 1:size(env.nfz,1)
                cx = env.nfz(k,1);
                cy = env.nfz(k,2);
                R  = env.nfz(k,3);

                d = hypot(p(1)-cx, p(2)-cy);

                if d <= R
                    error(['threat_map:OverlapNFZ\n' ...
                           '%s point (%.1f, %.1f) lies INSIDE NFZ #%d ' ...
                           '(center = [%.1f, %.1f], r = %.1f).'], ...
                          names(i), p(1), p(2), k, cx, cy, R);
                elseif d <= R + cp_clear
                    warning(['threat_map:NearNFZ\n' ...
                             '%s point (%.1f, %.1f) is only %.1f m outside NFZ #%d ' ...
                             '(center = [%.1f, %.1f], r = %.1f).'], ...
                            names(i), p(1), p(2), d - R, k, cx, cy, R);
                end
            end
        end
    end

    % ---- Against radar -------------------------------------------------
    if isfield(env,'radar') && ~isempty(env.radar)
        for i = 1:size(pts,1)
            p = pts(i,:);
            for k = 1:size(env.radar,1)
                cx = env.radar(k,1);
                cy = env.radar(k,2);
                R  = env.radar(k,3);

                d = hypot(p(1)-cx, p(2)-cy);

                if d <= R
                    error(['threat_map:OverlapRadar\n' ...
                           '%s point (%.1f, %.1f) lies INSIDE Radar #%d ' ...
                           '(center = [%.1f, %.1f], r = %.1f).'], ...
                          names(i), p(1), p(2), k, cx, cy, R);
                elseif d <= R + cp_clear
                    warning(['threat_map:NearRadar\n' ...
                             '%s point (%.1f, %.1f) is only %.1f m outside Radar #%d ' ...
                             '(center = [%.1f, %.1f], r = %.1f).'], ...
                            names(i), p(1), p(2), d - R, k, cx, cy, R);
                end
            end
        end
    end
end

% ========================================================================
% ZONE–ZONE NON-OVERLAP: NFZ vs NFZ, radar vs radar, NFZ vs radar
% ========================================================================
function check_zone_overlaps(env)
% check_zone_overlaps
%   Hard condition:
%     For any disks A, B (NFZ or radar), require
%         dist(centerA, centerB) > rA + rB

    nfz   = [];
    radar = [];

    if isfield(env,'nfz')   && ~isempty(env.nfz),   nfz   = env.nfz;   end
    if isfield(env,'radar') && ~isempty(env.radar), radar = env.radar; end

    % NFZ–NFZ
    for i = 1:size(nfz,1)
        for j = i+1:size(nfz,1)
            d  = hypot(nfz(i,1)-nfz(j,1), nfz(i,2)-nfz(j,2));
            R  = nfz(i,3) + nfz(j,3);
            if d <= R
                error(['threat_map:NFZOverlap\n' ...
                       'NFZ #%d and NFZ #%d overlap or touch (d = %.1f, sumR = %.1f).'], ...
                       i, j, d, R);
            end
        end
    end

    % Radar–Radar
    for i = 1:size(radar,1)
        for j = i+1:size(radar,1)
            d  = hypot(radar(i,1)-radar(j,1), radar(i,2)-radar(j,2));
            R  = radar(i,3) + radar(j,3);
            if d <= R
                error(['threat_map:RadarOverlap\n' ...
                       'Radar #%d and Radar #%d overlap or touch (d = %.1f, sumR = %.1f).'], ...
                       i, j, d, R);
            end
        end
    end

    % NFZ–Radar
    for i = 1:size(nfz,1)
        for j = 1:size(radar,1)
            d  = hypot(nfz(i,1)-radar(j,1), nfz(i,2)-radar(j,2));
            R  = nfz(i,3) + radar(j,3);
            if d <= R
                error(['threat_map:NFZRadarOverlap\n' ...
                       'NFZ #%d and Radar #%d overlap or touch (d = %.1f, sumR = %.1f).'], ...
                       i, j, d, R);
            end
        end
    end
end
