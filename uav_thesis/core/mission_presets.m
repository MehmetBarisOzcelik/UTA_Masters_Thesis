function cfg_out = mission_presets(preset_id, cfg_base)
% MISSION_PRESETS
% ---------------------------------------------------------------
% Frozen mission configurations for the thesis demonstrations.
%
% Accepted values:
%   1, 'default1', 'safe', 'baseline'
%       -> Default fair baseline mission with basic-chaser mode
%
%   2, 'default2', 'pn', 'fairpn'
%       -> Default fair mission with PN pursuer mode
%
%   3, 'manual'
%       -> Manual geometry with baseline chaser parameters
%
% IMPORTANT:
%   This file does NOT strengthen or tune the proportional-navigation chaser.
%
%   For the main thesis comparison, the basic and proportional-navigation
%   chasers use the same physical parameters:
%       - same initial position
%       - same speed ratio
%       - same maximum turn rate
%       - same catch radius
%
%   The only difference between the two chasers is the guidance law.
%
% Output:
%   cfg_out : configuration struct with preset-specific fields set.

    %% 0. Base configuration
    if nargin < 2 || isempty(cfg_base)
        cfg_base = config();
    end

    if nargin < 1 || isempty(preset_id)
        preset_id = 1;
    end

    %% 1. Normalize string inputs
    if ischar(preset_id) || isstring(preset_id)

        pid = lower(string(preset_id));

        if pid == "default1" || pid == "safe" || pid == "baseline"
            preset_id = 1;

        elseif pid == "default2" || pid == "pn" || pid == "fairpn"
            preset_id = 2;

        elseif pid == "manual"
            preset_id = 3;

        else
            warning('mission_presets:unknownId', ...
                'Unknown preset "%s". Using DEFAULT FAIR baseline preset.', pid);
            preset_id = 1;
        end
    end

    %% 2. Start from base configuration
    cfg = cfg_base;

    %% 3. Select preset
    switch preset_id

        case 1
            % DEFAULT FAIR BASELINE MISSION
            % Geometry: default thesis environment.
            % Chaser: baseline physical parameters, basic guidance.
            cfg.env.mode = 'default';

            cfg.pn = cfg_base.pn;
            cfg.pn.mode = 'basic';

            cfg.metrics.catch_radius = cfg_base.metrics.catch_radius;

        case 2
            % DEFAULT FAIR PN MISSION
            % Same geometry and same physical chaser parameters as preset 1.
            % Only the guidance mode changes to PN.
            cfg.env.mode = 'default';

            cfg.pn = cfg_base.pn;
            cfg.pn.mode = 'pn';

            cfg.metrics.catch_radius = cfg_base.metrics.catch_radius;

        case 3
            % MANUAL GEOMETRY
            % Geometry is filled later by manual_env_input(cfg).
            % Chaser remains at baseline physical parameters.
            cfg.env.mode = 'manual';

            cfg.pn = cfg_base.pn;
            cfg.metrics.catch_radius = cfg_base.metrics.catch_radius;

        otherwise
            warning('mission_presets:unknownNumericId', ...
                'Unknown numeric preset. Using DEFAULT FAIR baseline preset.');

            cfg.env.mode = 'default';

            cfg.pn = cfg_base.pn;
            cfg.pn.mode = 'basic';

            cfg.metrics.catch_radius = cfg_base.metrics.catch_radius;
    end

    %% 4. Synchronize legacy fields
    if isfield(cfg, 'uav') && isfield(cfg.uav, 'mode')
        cfg.uav_mode = cfg.uav.mode;
    end

    if isfield(cfg, 'pn') && isfield(cfg.pn, 'mode')
        cfg.pn_mode = cfg.pn.mode;
    end

    cfg_out = cfg;

end