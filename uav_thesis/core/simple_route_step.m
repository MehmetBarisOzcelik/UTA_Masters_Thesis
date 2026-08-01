% SIMPLE_ROUTE_STEP
% ---------------------------------------------------------------
% Non-MPC waypoint-following controller for the UAV.
%
% This controller is used as the route-following baseline in the
% fair full-mission comparison. It does not predict future obstacle
% encounters and does not optimize over a finite horizon. Instead, it:
%
%   - points the UAV toward the current waypoint
%   - applies proportional heading-error control
%   - applies proportional speed control toward cfg.uav.v_nom
%   - saturates turn-rate and acceleration commands
%
% Inputs:
%   mpc : route/MPC struct containing route_wp and wp_idx
%   uav : current UAV state
%   env : environment struct, currently unused but kept for interface consistency
%   cfg : configuration struct
%
% Output:
%   u_cmd : control vector [omega_cmd; a_cmd]
%   mpc   : returned unchanged for interface consistency

function [u_cmd, mpc] = simple_route_step(mpc, uav, env, cfg) %#ok<INUSD>

% -------------------------------------------------------------------------
% Current target waypoint
% -------------------------------------------------------------------------
if mpc.wp_idx <= size(mpc.route_wp,1)
    wp = mpc.route_wp(mpc.wp_idx, :);
else
    wp = mpc.route_wp(end, :);
end

dx = wp(1) - uav.x;
dy = wp(2) - uav.y;

% -------------------------------------------------------------------------
% Heading control
% -------------------------------------------------------------------------
psi_des = atan2(dy, dx);
e_psi   = wrapToPi(psi_des - uav.psi);

omega_cmd = cfg.trk.k_psi * e_psi;
omega_cmd = max(min(omega_cmd, cfg.uav.omega_max), -cfg.uav.omega_max);

% -------------------------------------------------------------------------
% Speed control
% -------------------------------------------------------------------------
e_v   = cfg.uav.v_nom - uav.v;
a_cmd = cfg.trk.k_v * e_v;
a_cmd = max(min(a_cmd, cfg.uav.a_max), -cfg.uav.a_max);

% -------------------------------------------------------------------------
% Pack command vector expected by advance_states.m
% -------------------------------------------------------------------------
u_cmd = [omega_cmd; a_cmd];

end