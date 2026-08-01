% SIMULATE_PURSUER_PN
% ---------------------------------------------------------------
% Proportional-navigation chaser following the flown UAV trajectory
% from the simulation log.
%
% The chaser uses a proportional-navigation heading-rate command with
% an optional line-of-sight heading-error blend:
%
%   omega = N * lambda_dot + k_pp * e_psi
%
% where:
%   lambda     : line-of-sight angle from chaser to UAV
%   lambda_dot : line-of-sight angular rate
%   e_psi      : heading error between chaser heading and LOS direction
%   N          : PN navigation constant, cfg.pn.N
%   k_pp       : LOS-heading blend gain, cfg.pn.k_pp
%
% The proportional-navigation and basic chasers use the same physical
% limits:
%   - same start position
%   - same speed ratio
%   - same maximum turn rate
%   - same catch radius
%
% The only difference is the guidance law.
%
% The chaser:
%   - flies at constant speed based on cfg.pn.v_ratio, capped by cfg.pn.v_max
%   - ignores NFZ and radar zones
%   - declares a catch when separation <= cfg.metrics.catch_radius
%   - holds its position for the remainder of the simulation after capture

function log = simulate_pursuer_pn(pn0, uavlog, cfg, env) %#ok<INUSD>

t = uavlog.t(:);
K = numel(t);

log.t = t;

if K < 2
    log.pos       = repmat([pn0.x pn0.y], 1, 1);
    log.vel       = [0 0];
    log.caught    = false;
    log.catch_idx = NaN;
    log.catch_pos = [NaN NaN];
    return;
end

dt = median(diff(t));

%% PN parameters ----------------------------------------------------------

Npn    = cfg.pn.N;
wmax   = cfg.pn.omega_max;
rCatch = cfg.metrics.catch_radius;

if isfield(cfg, 'pn') && isfield(cfg.pn, 'k_pp') && ~isempty(cfg.pn.k_pp)
    k_pp = cfg.pn.k_pp;
else
    k_pp = 0.8;
end

% Shared physical chaser speed
v_des = min(cfg.pn.v_ratio * cfg.uav.v_nom, cfg.pn.v_max);

%% Initial chaser state --------------------------------------------------

x   = pn0.x;
y   = pn0.y;
psi = pn0.psi;

pos = zeros(K,2);
vel = zeros(K,2);

pos(1,:) = [x y];
vel(1,:) = [v_des*cos(psi), v_des*sin(psi)];

% Initial LOS angle for finite-difference LOS-rate calculation
pU1 = uavlog.pos(1,:);
rel = pU1 - [x y];
lambda_last = atan2(rel(2), rel(1));

caught    = false;
catch_idx = NaN;
catch_pos = [NaN NaN];

%% Main simulation loop ---------------------------------------------------

for k = 2:K

    if caught
        pos(k,:) = catch_pos;
        vel(k,:) = [0 0];
        continue;
    end

    % Current UAV position
    pU = uavlog.pos(k,:);

    % Line-of-sight angle and LOS rate
    rel = pU - [x y];

    lambda     = atan2(rel(2), rel(1));
    lambda_dot = wrapToPi(lambda - lambda_last) / dt;

    % Heading error to LOS
    e_psi = wrapToPi(lambda - psi);

    % PN + LOS-heading blend
    omega = Npn * lambda_dot + k_pp * e_psi;
    omega = max(min(omega, wmax), -wmax);

    % Integrate chaser kinematics
    psi = wrapToPi(psi + omega * dt);
    x   = x + v_des * cos(psi) * dt;
    y   = y + v_des * sin(psi) * dt;

    pos(k,:) = [x y];
    vel(k,:) = [v_des*cos(psi), v_des*sin(psi)];

    % Update LOS memory
    lambda_last = lambda;

    % Catch check after motion update
    if norm(pU - [x y]) <= rCatch
        caught    = true;
        catch_idx = k;
        catch_pos = [x y];

        pos(k,:) = catch_pos;
        vel(k,:) = [0 0];
    end
end

%% Pack output ------------------------------------------------------------

log.t         = t;
log.pos       = pos;
log.vel       = vel;
log.caught    = caught;
log.catch_idx = catch_idx;
log.catch_pos = catch_pos;

end