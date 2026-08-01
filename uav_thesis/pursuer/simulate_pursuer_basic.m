% SIMULATE_PURSUER_BASIC
% ---------------------------------------------------------------
% Basic pure-pursuit-style chaser.
%
% The basic chaser:
%   - computes the line-of-sight angle to the current UAV position
%   - turns toward that angle using a proportional heading-error law
%   - uses a separate basic-pursuit gain cfg.pn.k_basic
%   - uses the same physical parameters as the proportional-navigation
%     chaser: chaser-to-UAV speed ratio, maximum turn rate, initial
%     position, and capture radius
%
% IMPORTANT:
%   This function intentionally does NOT use cfg.pn.N as its gain.
%   cfg.pn.N is the PN navigation constant and should only be used by
%   the proportional-navigation chaser. This keeps the comparison fair.
%
%   The only difference between the basic and proportional-navigation
%   chasers is the guidance law. Physical limits remain the same.

function pnlog = simulate_pursuer_basic(pn0, uavlog, cfg, ~)

t = uavlog.t(:);
N = numel(t);

if N < 2
    pnlog.t         = t;
    pnlog.pos       = [pn0.x pn0.y];
    pnlog.vel       = [0 0];
    pnlog.caught    = false;
    pnlog.catch_idx = NaN;
    pnlog.catch_pos = [NaN NaN];
    return;
end

dt = mean(diff(t));

%% Preallocate logs -------------------------------------------------------

pn_pos = nan(N,2);
pn_vel = nan(N,2);

%% Initial chaser state --------------------------------------------------

x   = pn0.x;
y   = pn0.y;
psi = wrapToPi(pn0.psi);

%% Shared physical chaser parameters ------------------------------------

v_des = min(cfg.pn.v_ratio * cfg.uav.v_nom, cfg.pn.v_max);
wmax  = cfg.pn.omega_max;

rCatch = cfg.metrics.catch_radius;

%% Basic guidance gain ----------------------------------------------------

% Preferred source is env/config.m. Fallback is kept for compatibility.
if isfield(cfg, 'pn') && isfield(cfg.pn, 'k_basic') && ~isempty(cfg.pn.k_basic)
    k_basic = cfg.pn.k_basic;
else
    k_basic = 0.08;
end

%% Catch bookkeeping ------------------------------------------------------

caught    = false;
catch_idx = NaN;
catch_pos = [NaN NaN];

%% Main simulation loop ---------------------------------------------------

for k = 1:N

    % Log current state
    pn_pos(k,:) = [x y];
    pn_vel(k,:) = [v_des*cos(psi), v_des*sin(psi)];

    % Current UAV position
    up = uavlog.pos(k,:);

    % Check catch condition
    dist = hypot(up(1) - x, up(2) - y);

    if ~caught && dist <= rCatch
        caught    = true;
        catch_idx = k;
        catch_pos = [x y];
    end

    if caught
        % Freeze after catch
        pn_pos(k,:) = catch_pos;
        pn_vel(k,:) = [0 0];

        if k < N
            pn_pos(k+1:end,:) = repmat(catch_pos, N-k, 1);
            pn_vel(k+1:end,:) = zeros(N-k, 2);
        end

        break;
    end

    % Pure-pursuit desired heading
    psi_des = atan2(up(2) - y, up(1) - x);
    e_psi   = wrapToPi(psi_des - psi);

    % Basic proportional heading-error guidance law
    omega = k_basic * e_psi;
    omega = max(min(omega, wmax), -wmax);

    % Integrate chaser kinematics
    psi = wrapToPi(psi + omega*dt);
    x   = x + v_des*cos(psi)*dt;
    y   = y + v_des*sin(psi)*dt;
end

%% Pack output ------------------------------------------------------------

pnlog.t         = t;
pnlog.pos       = pn_pos;
pnlog.vel       = pn_vel;
pnlog.caught    = caught;
pnlog.catch_idx = catch_idx;
pnlog.catch_pos = catch_pos;

end