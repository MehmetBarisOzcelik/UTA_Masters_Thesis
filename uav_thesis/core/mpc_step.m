% MPC_STEP
% ---------------------------------------------------------------
% Grid-search Model Predictive Control step for the UAV.
%
% The controller evaluates constant candidate control inputs:
%
%   u = [omega; a]
%
% over a finite prediction horizon of length cfg.mpc.Np. The selected
% command minimizes a cost function containing:
%
%   J = waypoint tracking
%     + speed tracking
%     + soft NFZ/radar proximity penalties
%     + hard penalties for entering NFZ/radar keep-out regions
%     + control effort penalty
%
% Inputs:
%   mpc  : MPC/route struct containing route waypoints and current index
%   uav  : current UAV state, with fields x, y, psi, v
%   pn0  : chaser initial state, currently unused but kept for extension
%   env  : environment struct containing NFZ and radar geometry
%   cfg  : configuration struct containing MPC weights and control grids
%
% Output:
%   u_cmd : selected control input [omega; a]
%   mpc   : returned for interface consistency

function [u_cmd, mpc] = mpc_step(mpc, uav, pn0, env, cfg) %#ok<INUSD>

dt    = cfg.dt;
v_nom = cfg.uav.v_nom;
Np    = cfg.mpc.Np;

omega_grid = cfg.mpc.omega_grid;
a_grid     = cfg.mpc.a_grid;

%% Current target waypoint ------------------------------------------------

if mpc.wp_idx <= size(mpc.route_wp,1)
    wp = mpc.route_wp(mpc.wp_idx, :);
else
    wp = env.goal;
end

%% Grid-search over constant candidate controls ---------------------------

bestJ = inf;
bestU = [0; 0];

for io = 1:numel(omega_grid)

    for ia = 1:numel(a_grid)

        omega = omega_grid(io);
        a     = a_grid(ia);

        % Start prediction from current UAV state
        up = uav;
        J  = 0;

        for k = 1:Np

            % Predict one step forward using the candidate command
            up = predict_step(up, omega, a, dt, cfg);

            % -------------------------------------------------------------
            % 1) Waypoint tracking penalty
            % -------------------------------------------------------------
            dx = up.x - wp(1);
            dy = up.y - wp(2);
            dist2 = dx*dx + dy*dy;

            J = J + cfg.mpc.w_wp * dist2;

            % -------------------------------------------------------------
            % 2) Speed tracking penalty
            % -------------------------------------------------------------
            dv = up.v - v_nom;
            J = J + cfg.mpc.w_v * (dv*dv);

            % -------------------------------------------------------------
            % 3) NFZ keep-out and proximity penalties
            % -------------------------------------------------------------
            if isfield(env, 'nfz') && ~isempty(env.nfz)

                for i = 1:size(env.nfz,1)

                    cx = env.nfz(i,1);
                    cy = env.nfz(i,2);
                    R  = env.nfz(i,3) + cfg.avoid.margin;

                    d  = hypot(up.x - cx, up.y - cy);
                    de = d - R;    % signed distance from expanded boundary

                    if de < 0
                        % Inside expanded NFZ boundary
                        J = J + cfg.mpc.w_coll;

                    elseif de < cfg.mpc.safe_margin
                        % Smooth proximity penalty near the NFZ boundary
                        s = (cfg.mpc.safe_margin - de) / cfg.mpc.safe_margin;
                        J = J + cfg.mpc.w_nfz * s^2;
                    end
                end
            end

            % -------------------------------------------------------------
            % 4) Radar keep-out and proximity penalties
            % -------------------------------------------------------------
            if isfield(env, 'radar') && ~isempty(env.radar)

                for i = 1:size(env.radar,1)

                    cx = env.radar(i,1);
                    cy = env.radar(i,2);
                    R  = env.radar(i,3) + cfg.avoid.margin;

                    d  = hypot(up.x - cx, up.y - cy);
                    de = d - R;    % signed distance from expanded boundary

                    if de < 0
                        % Inside expanded radar boundary
                        J = J + cfg.mpc.w_coll;

                    elseif de < cfg.mpc.safe_margin
                        % Smooth proximity penalty near radar boundary
                        s = (cfg.mpc.safe_margin - de) / cfg.mpc.safe_margin;
                        J = J + cfg.mpc.w_radar * s^2;
                    end
                end
            end
        end

        % -------------------------------------------------------------
        % 5) Control effort penalty
        % -------------------------------------------------------------
        J = J + cfg.mpc.w_u * (omega^2 + a^2) * Np;

        % Store best candidate
        if J < bestJ
            bestJ = J;
            bestU = [omega; a];
        end
    end
end

%% Return saturated selected command --------------------------------------

omega = bestU(1);
a     = bestU(2);

omega = min(max(omega, -cfg.uav.omega_max), cfg.uav.omega_max);
a     = min(max(a,     -cfg.uav.a_max),     cfg.uav.a_max);

u_cmd = [omega; a];

end

% ========================================================================
function s = predict_step(s, omega, a, dt, cfg)
% PREDICT_STEP
% ---------------------------------------------------------------
% One-step 2-D kinematic prediction used internally by the MPC.
%
% State fields:
%   s.x   : x-position [m]
%   s.y   : y-position [m]
%   s.psi : heading [rad]
%   s.v   : speed [m/s]
%
% Inputs:
%   omega : commanded turn rate [rad/s]
%   a     : commanded acceleration [m/s^2]

    % Saturate candidate controls
    omega = min(max(omega, -cfg.uav.omega_max), cfg.uav.omega_max);
    a     = min(max(a,     -cfg.uav.a_max),     cfg.uav.a_max);

    % Update heading and speed
    psi = wrapToPi(s.psi + omega * dt);
    v   = s.v + a * dt;

    % Enforce speed limits
    v = min(max(v, cfg.uav.v_min), cfg.uav.v_max);

    % Update position
    s.x   = s.x + v * cos(psi) * dt;
    s.y   = s.y + v * sin(psi) * dt;
    s.v   = v;
    s.psi = psi;

end