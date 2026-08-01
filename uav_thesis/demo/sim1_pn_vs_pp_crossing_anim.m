function sim1_pn_vs_pp_crossing_anim()
% SIM1_PN_VS_PP_CROSSING_ANIM
% ---------------------------------------------------------------
% Animated version of Thesis Demo 1:
%   Proportional Navigation (PN) vs Pure Pursuit (PP) in a simple
%   crossing-target engagement.
%
% Purpose:
%   Shows the time evolution of the PN and PP chasers under the
%   same physical limits.
%
% Fairness:
%   Both chasers:
%       - start from the same initial position
%       - start with the same initial heading
%       - fly with the same constant speed
%       - use the same lateral acceleration limit
%       - use the same hit radius
%
%   The only difference is the guidance law.
%
% Outputs:
%   - sim1_pn_vs_pp_crossing_anim.mp4
%   - sim1_pn_vs_pp_crossing_anim.gif
%
% Author:
%   Mehmet Barış Özçelik

clc; clear; close all;

%% ========================================================================
% 1. Scenario and parameters
% ========================================================================

DT = 0.02;      % [s] integration time step
T  = 60;        % [s] total simulation time

vm = 320;       % [m/s] chaser speed, shared by PN and PP
vt = 260;       % [m/s] target speed

N     = 4.0;    % [-] PN navigation constant
amax  = 80;     % [m/s^2] lateral acceleration limit, shared by PN and PP
Rhit  = 20;     % [m] hit radius
Kpp   = 1.6;    % [-] PP heading-error gain

% Crossing geometry
xm  = 0;
ym  = 0;
thm = 0;

xmPP  = xm;
ymPP  = ym;
thmPP = thm;

% Target starts ahead/right and moves upward.
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

hitPN = false;
hitPP = false;

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
    % Proportional Navigation chaser
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
    % Pure Pursuit chaser
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
    xm = xmn;
    ym = ymn;

    xmPP = xmPPn;
    ymPP = ymPPn;

    xt = xtn;
    yt = ytn;
end

%% ========================================================================
% 4. Metrics
% ========================================================================

tvec = (0:K)*DT;

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

fprintf('=== Demo 1 Animation: PN vs Pure Pursuit Crossing Engagement ===\n');
fprintf('Chaser speed: v_m = %.1f m/s\n', vm);
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
fprintf('===============================================================\n\n');

%% ========================================================================
% 5. Animation setup
% ========================================================================

km = 1e-3;

allX = [Mm(:,1); Mp(:,1); Tt(:,1)];
allY = [Mm(:,2); Mp(:,2); Tt(:,2)];

dx = max(allX) - min(allX);
dy = max(allY) - min(allY);

margin = 0.05;

xlims = [min(allX)-margin*dx, max(allX)+margin*dx];
ylims = [min(allY)-margin*dy, max(allY)+margin*dy];

fig = figure('Color','w','Position',[110 110 980 640]);
ax  = axes('Parent',fig);

hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');

xlabel(ax,'x [km]');
ylabel(ax,'y [km]');

xlim(ax, xlims*km);
ylim(ax, ylims*km);

% Full target path
hTarget = plot(ax, Tt(:,1)*km, Tt(:,2)*km, ':', ...
    'LineWidth',1.6, ...
    'Color',[0.2 0.5 0.9], ...
    'DisplayName','Target path');

% PN and PP paths
hPPpath = plot(ax, nan, nan, '--', ...
    'LineWidth',2.3, ...
    'Color',[0.85 0.3 0.2], ...
    'DisplayName','PP path');

hPNpath = plot(ax, nan, nan, '-', ...
    'LineWidth',2.6, ...
    'Color',[0.90 0.70 0.10], ...
    'DisplayName','PN path');

% Current markers
hPPend = plot(ax, nan, nan, 'x', ...
    'Color',[0.55 0.15 0.1], ...
    'LineWidth',2.0, ...
    'MarkerSize',10, ...
    'DisplayName','PP current');

hPNend = plot(ax, nan, nan, 'x', ...
    'Color',[0.5 0.4 0.1], ...
    'LineWidth',2.0, ...
    'MarkerSize',10, ...
    'DisplayName','PN current');

% LOS line from PN to target
hLOS = plot(ax, nan, nan, '-', ...
    'Color',[0.25 0.25 0.25], ...
    'LineWidth',1.4, ...
    'DisplayName','LOS at PN end');

legend(ax, [hTarget, hPPpath, hPNpath, hPPend, hPNend, hLOS], ...
       {'Target path', 'PP path', 'PN path', ...
        'PP current', 'PN current', 'LOS to target'}, ...
       'Location','northwest');

%% ========================================================================
% 6. Writers
% ========================================================================

fps    = 40;
stride = 2;

make_mp4 = true;
make_gif = true;

if make_mp4
    vw = VideoWriter('sim1_pn_vs_pp_crossing_anim.mp4', 'MPEG-4');
    vw.FrameRate = fps;
    open(vw);
end

gifname = 'sim1_pn_vs_pp_crossing_anim.gif';
gif_written = false;

%% ========================================================================
% 7. Animation loop
% ========================================================================

for k = 1:stride:(K+1)

    kPN = min(k, kPNend);
    kPP = min(k, kPPend);

    set(hPPpath, ...
        'XData', Mp(1:kPP,1)*km, ...
        'YData', Mp(1:kPP,2)*km);

    set(hPNpath, ...
        'XData', Mm(1:kPN,1)*km, ...
        'YData', Mm(1:kPN,2)*km);

    set(hPPend, ...
        'XData', Mp(kPP,1)*km, ...
        'YData', Mp(kPP,2)*km);

    set(hPNend, ...
        'XData', Mm(kPN,1)*km, ...
        'YData', Mm(kPN,2)*km);

    set(hLOS, ...
        'XData', [Mm(kPN,1), Tt(kPN,1)]*km, ...
        'YData', [Mm(kPN,2), Tt(kPN,2)]*km);

    tk = tvec(min(k,K+1));

    if hitPN && tk >= tPN
        statusPN = sprintf('PN hit at %.1f s', tPN);
    else
        statusPN = sprintf('PN tracking, t = %.1f s', tk);
    end

    if hitPP && tk >= tPP
        statusPP = sprintf('PP hit at %.1f s', tPP);
    else
        statusPP = 'PP tracking';
    end

    title(ax, { ...
        'Demo 1: Proportional Navigation vs Pure Pursuit', ...
        [statusPN, '   |   ', statusPP]});

    drawnow;

    fr  = getframe(fig);
    img = frame2im(fr);

    if make_mp4
        writeVideo(vw, fr);
    end

    if make_gif
        [A,map] = rgb2ind(img,256);

        if ~gif_written
            imwrite(A, map, gifname, 'gif', ...
                    'LoopCount', inf, ...
                    'DelayTime', 1/fps);

            gif_written = true;
        else
            imwrite(A, map, gifname, 'gif', ...
                    'WriteMode', 'append', ...
                    'DelayTime', 1/fps);
        end
    end
end

%% ========================================================================
% 8. Close writers
% ========================================================================

if make_mp4
    close(vw);
end

fprintf('Saved animation files:\n');

if make_mp4
    fprintf('  sim1_pn_vs_pp_crossing_anim.mp4\n');
end

if make_gif
    fprintf('  %s\n', gifname);
end

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