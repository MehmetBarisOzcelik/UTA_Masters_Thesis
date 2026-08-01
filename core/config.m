function cfg = config()
% CONFIG
% ---------------------------------------------------------------
% Central configuration file for the thesis simulation:
%
%   Autonomous UAV Mission Planning Under Threat Using
%   Model Predictive Control with Proportional-Navigation Pursuers
%
% This file defines:
%   - simulation timing
%   - environment parameters
%   - UAV dynamics limits
%   - route-following gains
%   - MPC parameters
%   - chaser parameters
%   - metric settings
%   - plotting/output options
%
% IMPORTANT FOR FAIR COMPARISON:
%   The basic and proportional-navigation chasers use the same physical parameters:
%       - same start offset
%       - same speed ratio
%       - same maximum turn rate
%       - same catch radius
%
%   The only difference is the guidance law.
%
%   The basic chaser uses cfg.pn.k_basic.
%   The proportional-navigation chaser uses cfg.pn.N and cfg.pn.k_pp.

%% ========================================================================
% 1. General simulation settings
% ========================================================================

cfg.seed = 1001;

cfg.dt   = 0.1;      % [s] simulation time step
cfg.Tmax = 300;      % [s] maximum simulation time

%% ========================================================================
% 2. Environment settings
% ========================================================================

cfg.env.mode = 'default';   % 'default' or 'manual'

% Map bounds:
%   row 1 = x bounds
%   row 2 = y bounds
cfg.env.bounds = [-1200 1200;
                  -900   900];

% Capture radius for checkpoints
cfg.env.rC = 60;            % [m]

% Goal capture radius
cfg.env.r_goal = 60;        % [m]

% Minimum desired clearance for placing mission points
cfg.env.cp_clear = 120;     % [m]

% Strict keep-out logic flag
cfg.env.strict = true;

% Numerical tolerance for NFZ checking/plotting
cfg.env.nfz_tol = 10;       % [m]

% Edge margin used by visibility graph planning
cfg.env.edge_margin = 0.4 * cfg.env.cp_clear;

% Default counts used by environment generator if needed
cfg.env.n_cp    = 2;
cfg.env.n_nfz   = 2;
cfg.env.n_radar = 1;

%% ========================================================================
% 3. Route planning settings
% ========================================================================

% Route edge cost:
%   cost = w_t * path_length + w_r * integrated_risk
cfg.route.w_t = 1.0;
cfg.route.w_r = 3.0;

%% ========================================================================
% 4. Blue UAV model and limits
% ========================================================================

cfg.uav.mode = 'mpc';       % 'route' or 'mpc'

cfg.uav.v_min = 10;         % [m/s]
cfg.uav.v_nom = 25;         % [m/s]
cfg.uav.v_max = 38;         % [m/s]

cfg.uav.omega_max = deg2rad(45);   % [rad/s]
cfg.uav.a_max     = 6;             % [m/s^2]

%% ========================================================================
% 5. Simple route-following controller
% ========================================================================

% Heading proportional gain for route mode
cfg.trk.k_psi = 3.0;

% Speed proportional gain for route mode
cfg.trk.k_v = 1.2;

%% ========================================================================
% 6. MPC controller settings
% ========================================================================

cfg.mpc.Np = 12;    % prediction horizon in time steps

cfg.mpc.omega_grid = linspace(-cfg.uav.omega_max, ...
                               cfg.uav.omega_max, 9);

cfg.mpc.a_grid = linspace(-0.5 * cfg.uav.a_max, ...
                            0.5 * cfg.uav.a_max, 3);

% Cost weights
cfg.mpc.w_wp    = 1.2;      % waypoint tracking
cfg.mpc.w_v     = 0.05;     % speed tracking
cfg.mpc.w_u     = 0.02;     % control effort
cfg.mpc.w_nfz   = 1800;     % NFZ proximity penalty
cfg.mpc.w_radar = 1000;     % radar proximity penalty
cfg.mpc.w_coll  = 1e6;      % hard collision/inside-zone penalty

% Soft safety margin outside keep-out boundaries
cfg.mpc.safe_margin = 90;   % [m]

%% ========================================================================
% 7. Avoidance parameters
% ========================================================================

cfg.avoid.margin      = 40;      % [m] added to obstacle radius in MPC
cfg.avoid.influence   = 200;     % [m] influence distance
cfg.avoid.k_repulse   = 2.8;     % repulsion strength
cfg.avoid.slow_radius = 100;     % [m] slowdown distance near obstacles

%% ========================================================================
% 8. Chaser settings
% ========================================================================

cfg.pn.mode = 'pn';         % 'basic' or 'pn'

% Shared physical chaser parameters
% This value gives the chaser a modest speed advantage while keeping
% the engagement realistic for the full-mission comparison.
cfg.pn.v_ratio = 1.075;             % chaser-to-UAV speed ratio
cfg.pn.v_max     = 45;              % [m/s]
cfg.pn.omega_max = deg2rad(55);     % [rad/s]

% Chaser initial position relative to UAV start
cfg.pn.start_dx = -300;     % [m]
cfg.pn.start_dy = 0;        % [m]

% PN guidance parameters
% N = 1.0 produces a less aggressive proportional-navigation chaser that
% still demonstrates
% improved interception capability compared with basic pursuit.
cfg.pn.N = 1.0;

% PN line-of-sight heading-blend gain
% Set to zero so the PN case is governed by the LOS-rate term rather than
% an additional pure-pursuit heading-error blend.
cfg.pn.k_pp = 0.0;

% Basic pursuit heading-error gain
% IMPORTANT:
%   Basic pursuit should not use cfg.pn.N.
%   cfg.pn.N is reserved for the PN navigation constant.
%   The basic chaser uses a lower-gain heading-error law while preserving
%   the same start point, speed ratio, turn-rate limit, and catch radius.
cfg.pn.k_basic = 0.08;

%% ========================================================================
% 9. Mission metrics
% ========================================================================

cfg.metrics.catch_radius = 20;   % [m]

%% ========================================================================
% 10. Plotting and outputs
% ========================================================================

cfg.plot.show = true;

cfg.outputs.save_anim = false;
cfg.outputs.tag       = 'strictNFZ';

%% ========================================================================
% 11. Legacy compatibility fields
% ========================================================================

% These are kept for older scripts that may still read cfg.uav_mode or
% cfg.pn_mode. The preferred fields are cfg.uav.mode and cfg.pn.mode.
cfg.uav_mode = cfg.uav.mode;
cfg.pn_mode  = cfg.pn.mode;

end