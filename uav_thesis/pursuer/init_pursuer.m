% INIT_PURSUER
% ---------------------------------------------------------------
% Initializes the chaser state for the full-mission simulation.
%
% The chaser is placed relative to the UAV start location:
%
%   x_p0 = env.start(1) + cfg.pn.start_dx
%   y_p0 = env.start(2) + cfg.pn.start_dy
%
% The chaser speed is shared by both basic and proportional-navigation
% guidance modes:
%
%   v_p = min(cfg.pn.v_ratio * cfg.uav.v_nom, cfg.pn.v_max)
%
% The initial heading is aimed approximately toward the goal.
%
% Fair-comparison note:
%   The basic and proportional-navigation chasers use the same initial
%   position, speed, turn-rate limit, and capture radius. The only
%   difference is the guidance law used later in
%   simulate_pursuer_basic.m or simulate_pursuer_pn.m.

function pn0 = init_pursuer(env, cfg)

% -------------------------------------------------------------------------
% Initial position relative to UAV start
% -------------------------------------------------------------------------
px0 = env.start(1) + cfg.pn.start_dx;
py0 = env.start(2) + cfg.pn.start_dy;

% -------------------------------------------------------------------------
% Shared chaser speed
% -------------------------------------------------------------------------
v_des = min(cfg.pn.v_ratio * cfg.uav.v_nom, cfg.pn.v_max);

% -------------------------------------------------------------------------
% Initial heading aimed toward goal
% -------------------------------------------------------------------------
psi0 = atan2(env.goal(2) - py0, env.goal(1) - px0);
psi0 = wrapToPi(psi0);

% -------------------------------------------------------------------------
% Pack chaser state
% -------------------------------------------------------------------------
pn0.x   = px0;
pn0.y   = py0;
pn0.v   = v_des;
pn0.psi = psi0;

end