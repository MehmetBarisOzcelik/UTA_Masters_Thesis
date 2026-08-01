% WAYPOINT_MANAGER
% ---------------------------------------------------------------
% Handles waypoint progression for the UAV.
%
% Responsibilities:
%   - checks distance to the current waypoint
%   - advances to the next waypoint when within capture radius
%   - snaps the UAV position to captured waypoints for clean plots
%   - snaps to the exact goal location after the final waypoint is reached
%
% Inputs:
%   uav : UAV state struct with fields x, y, v, psi
%   mpc : route/MPC struct containing route_wp and wp_idx
%   env : environment struct containing goal
%   cfg : configuration struct, uses cfg.env.rC
%
% Outputs:
%   uav : updated UAV state
%   mpc : updated waypoint index

function [uav, mpc] = waypoint_manager(uav, mpc, env, cfg)

% -------------------------------------------------------------------------
% Check for valid waypoint index
% -------------------------------------------------------------------------
if ~isfield(mpc, 'route_wp') || isempty(mpc.route_wp)
    return;
end

if ~isfield(mpc, 'wp_idx') || isempty(mpc.wp_idx)
    mpc.wp_idx = 1;
end

if mpc.wp_idx > size(mpc.route_wp,1)
    return;
end

% -------------------------------------------------------------------------
% Current target waypoint
% -------------------------------------------------------------------------
rc = cfg.env.rC;

wp = mpc.route_wp(mpc.wp_idx,:);
d  = hypot(uav.x - wp(1), uav.y - wp(2));

% -------------------------------------------------------------------------
% Advance when waypoint is captured
% -------------------------------------------------------------------------
if d <= rc

    % Snap to current waypoint for clean trajectory plots and unambiguous
    % checkpoint completion.
    uav.x = wp(1);
    uav.y = wp(2);

    % Advance to next waypoint
    mpc.wp_idx = mpc.wp_idx + 1;

    % If the final waypoint has been completed, snap exactly to the goal.
    if mpc.wp_idx > size(mpc.route_wp,1)
        uav.x = env.goal(1);
        uav.y = env.goal(2);
        return;
    end
end

end