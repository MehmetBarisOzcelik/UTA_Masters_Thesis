function sim1_pn_vs_pp_crossing()
% SIM1_PN_VS_PP_CROSSING
% ---------------------------------------------------------------
% Thesis Demo 1:
%   Proportional Navigation (PN) vs Pure Pursuit (PP) in a simple
%   crossing-target engagement.
%
% Purpose:
%   Demonstrates that PN guidance can intercept more efficiently than
%   pure pursuit under the same physical limits.
%
% Fairness:
%   Both chasers:
%       - start from the same initial position
%       - start with the same initial heading
%       - fly with the same constant speed
%       - use the same lateral acceleration limit
%       - use the same hit radius
%
%   The only difference is the guidance law:
%       PN: a_lat = N * Vc * lambda_dot
%       PP: turn toward the instantaneous line of sight
%
% Outputs:
%   - Figure: sim1_pn_vs_pp_crossing.png
%   - Console summary of intercept time, minimum separation, and path length
%
% Author:
%   Mehmet Barış Özçelik

clc; clear; close all;

%% ========================================================================
% 1. Scenario and parameters
% ========================================================================

% Time integration
DT = 0.02;      % [s] integration time step
T  = 60;        % [s] total simulation time

% Speeds
vm = 320;       % [m/s] chaser speed, shared by PN and PP
vt = 260;       % [m/s] target speed

% Guidance and physical limits
N     = 4.0;    % [-] PN navigation constant
amax  = 80;     % [m/s^2] lateral acceleration limit, shared by PN and PP
Rhit  = 20;     % [m] hit radius
Kpp   = 1.6;    % [-] PP heading-error gain

% Crossing geometry
xm   = 0;
ym   = 0;
thm  = 0;

xmPP  = xm;
ymPP  = ym;
thmPP = thm;

% Target starts ahead/right and moves upward
xt  = 8000;
yt  = -2000;
tht = pi/2;

%% ========================================================================
% 2. Helper functions and storage
% ========================================================================

LOS  = @(x1,y1,x2,y2) atan2(y2-y1, x2-x1);
wrap = @(a) atan2(sin(a), cos(a));
clip = @(x,lo,hi) min(max(x,lo),hi);

K = round(T/DT);

Mm = zeros(K+1,2);   % Proportional-navigation chaser positions
Mp = zeros(K+1,2);   % Pure-pursuit chaser positions
Tt = zeros(K+1,2);   % Target positions

Mm(1,:) = [xm,   ym];
Mp(1,:) = [xmPP, ymPP];
Tt(1,:) = [xt,   yt];

% Hit and minimum-separation bookkeeping
hitPN  = false;
hitPP  = false;

tPN = NaN;
tPP = NaN;

dminPN = inf;
dminPP = inf;

pminPN = [NaN NaN];
pminPP = [NaN NaN];

%% ========================================================================
% 3. Simulation loop
% ========================================================================

for k = 1:K

    % ---------------------------------------------------------------------
    % Target motion: constant velocity, straight line
    % ---------------------------------------------------------------------
    xtn = xt + vt*DT*cos(tht);
    ytn = yt + vt*DT*sin(tht);

    % ---------------------------------------------------------------------
    % Proportional-navigation chaser
    % ---------------------------------------------------------------------
    R = [xt - xm;
         yt - ym];

    Vrel = [vt*cos(tht) - vm*cos(thm);
            vt*sin(tht) - vm*sin(thm)];

    R2 = max(1e-9, R.'*R);

    lambda_dot = (R(1)*Vrel(2) - R(2)*Vrel(1)) / R2;
    Vc         = -(R.'*Vrel) / sqrt(R2);

    a_lat = clip(N * Vc * lambda_dot, -amax, amax);

    thm = thm + (a_lat/vm)*DT;

    xmn = xm + vm*DT*cos(thm);
    ymn = ym + vm*DT*sin(thm);

    % ---------------------------------------------------------------------
    % Pure-pursuit chaser
    % ---------------------------------------------------------------------
    lambdaPP = LOS(xmPP, ymPP, xt, yt);
    errPP    = wrap(lambdaPP - thmPP);

    w_cmd   = Kpp * errPP;
    a_latPP = clip(vm*w_cmd, -amax, amax);

    thmPP = thmPP + (a_latPP/vm)*DT;

    xmPPn = xmPP + vm*DT*cos(thmPP);
    ymPPn = ymPP + vm*DT*sin(thmPP);

    % ---------------------------------------------------------------------
    % Logging
    % ---------------------------------------------------------------------
    Mm(k+1,:) = [xmn,   ymn];
    Mp(k+1,:) = [xmPPn, ymPPn];
    Tt(k+1,:) = [xtn,   ytn];

    % Separation relative to updated target position
    dPN = hypot(xmn   - xtn, ymn   - ytn);
    dPP = hypot(xmPPn - xtn, ymPPn - ytn);

    if dPN < dminPN
        dminPN = dPN;
        pminPN = [xmn, ymn]; %#ok<NASGU>
    end

    if dPP < dminPP
        dminPP = dPP;
        pminPP = [xmPPn, ymPPn]; %#ok<NASGU>
    end

    if ~hitPN && dPN < Rhit
        hitPN = true;
        tPN   = k*DT;
    end

    if ~hitPP && dPP < Rhit
        hitPP = true;
        tPP   = k*DT;
    end

    % Advance states
    xm   = xmn;
    ym   = ymn;

    xmPP = xmPPn;
    ymPP = ymPPn;

    xt = xtn;
    yt = ytn;
end

%% ========================================================================
% 4. End indices and path lengths
% ========================================================================

kPNend = K+1;
if hitPN
    kPNend = min(K+1, round(tPN/DT)+1);
end

kPPend = K+1;
if hitPP
    kPPend = min(K+1, round(tPP/DT)+1);
end

plPN = traj_length(Mm(1:kPNend,:));
plPP = traj_length(Mp(1:kPPend,:));

%% ========================================================================
% 5. Console summary
% ========================================================================

fprintf('=== Demo 1: PN vs Pure Pursuit Crossing Engagement ===\n');
fprintf('Chaser speed     : v_m = %.1f m/s\n', vm);
fprintf('Target speed     : v_t = %.1f m/s\n', vt);
fprintf('Shared a_max     : %.1f m/s^2\n', amax);
fprintf('Hit radius       : %.1f m\n', Rhit);
fprintf('Fairness note    : PN and PP share speed, initial condition, and acceleration limit.\n');
fprintf('                   Only the guidance law differs.\n');

fprintf('\nIntercept times:\n');
fprintf('  PN : %s\n', tern(hitPN, sprintf('hit at t = %.2f s', tPN), 'no hit'));
fprintf('  PP : %s\n', tern(hitPP, sprintf('hit at t = %.2f s', tPP), 'no hit'));

fprintf('\nMinimum separation:\n');
fprintf('  PN : %.1f m\n', dminPN);
fprintf('  PP : %.1f m\n', dminPP);

fprintf('\nPath lengths:\n');
fprintf('  PN : %.1f m\n', plPN);
fprintf('  PP : %.1f m\n', plPP);
fprintf('  PP/PN path-length ratio: %.2f\n', plPP / max(plPN,1e-9));
fprintf('======================================================\n\n');

%% ========================================================================
% 6. Plotting
% ========================================================================

km = 1e-3;

allX = [Mm(:,1); Mp(:,1); Tt(:,1)];
allY = [Mm(:,2); Mp(:,2); Tt(:,2)];

dx = max(allX) - min(allX);
dy = max(allY) - min(allY);

margin = 0.05;

xlims = [min(allX)-margin*dx, max(allX)+margin*dx];
ylims = [min(allY)-margin*dy, max(allY)+margin*dy];

figure('Position',[110 110 980 640], 'Color','w');
hold on;
grid on;
axis equal;

% Target path
hTarget = plot(Tt(:,1)*km, Tt(:,2)*km, ':', ...
    'LineWidth',1.6, ...
    'Color',[0.2 0.5 0.9], ...
    'DisplayName','Target path');

% Pure pursuit path
hPPpath = plot(Mp(1:kPPend,1)*km, Mp(1:kPPend,2)*km, '--', ...
    'LineWidth',2.3, ...
    'Color',[0.85 0.3 0.2], ...
    'DisplayName','PP path');

% PN path
hPNpath = plot(Mm(1:kPNend,1)*km, Mm(1:kPNend,2)*km, '-', ...
    'LineWidth',2.6, ...
    'Color',[0.90 0.70 0.10], ...
    'DisplayName','PN path');

% End markers
hPPend = plot(Mp(kPPend,1)*km, Mp(kPPend,2)*km, 'x', ...
    'Color',[0.55 0.15 0.1], ...
    'LineWidth',2.0, ...
    'MarkerSize',10, ...
    'DisplayName','PP end');

hPNend = plot(Mm(kPNend,1)*km, Mm(kPNend,2)*km, 'x', ...
    'Color',[0.5 0.4 0.1], ...
    'LineWidth',2.0, ...
    'MarkerSize',10, ...
    'DisplayName','PN end');

% LOS from PN to target at PN end time
hLOS = plot([Mm(kPNend,1), Tt(kPNend,1)]*km, ...
            [Mm(kPNend,2), Tt(kPNend,2)]*km, '-', ...
            'Color',[0.2 0.2 0.2], ...
            'LineWidth',1.6, ...
            'DisplayName','LOS at PN end');

xlabel('x [km]');
ylabel('y [km]');

if hitPN
    ttlPN = sprintf('PN: hit, t = %.1f s, d_{min} = %.1f m', tPN, dminPN);
else
    ttlPN = sprintf('PN: no hit, d_{min} = %.1f m', dminPN);
end

if hitPP
    ttlPP = sprintf('PP: hit, t = %.1f s, d_{min} = %.1f m', tPP, dminPP);
else
    ttlPP = sprintf('PP: no hit, d_{min} = %.1f m', dminPP);
end

title({'Demo 1: Proportional Navigation vs Pure Pursuit', ...
       [ttlPN, '   |   ', ttlPP]});

xlim(xlims*km);
ylim(ylims*km);

legend([hTarget, hPPpath, hPNpath, hPPend, hPNend, hLOS], ...
       {'Target path', 'PP path', 'PN path', ...
        'PP end', 'PN end', 'LOS at PN end'}, ...
       'Location','northwest');

exportgraphics(gcf, 'sim1_pn_vs_pp_crossing.png', 'Resolution', 220);

end

%% ========================================================================
% Helper functions
% ========================================================================
function s = tern(c, a, b)

if c
    s = a;
else
    s = b;
end

end

% -------------------------------------------------------------------------
function L = traj_length(P)

if size(P,1) < 2
    L = 0;
else
    d = diff(P,1,1);
    L = sum(hypot(d(:,1), d(:,2)));
end

end