% PLOT_DEMO3_SEPARATION
% ---------------------------------------------------------------
% Creates a separation-vs-time plot for the four Demo 3
% full-mission cases:
%
%   1. Route + Basic
%   2. Route + PN
%   3. MPC + Basic
%   4. MPC + PN
%
% Output:
%   figures/demo3_separation_time.png
%
% This figure shows UAV survivability more clearly than the
% trajectory plots alone.

clear;
clc;
close all;

addpath(genpath(pwd));

%% ------------------------------------------------------------------------
% Output folder
% -------------------------------------------------------------------------
figDir = fullfile(pwd, 'figures');

if ~exist(figDir, 'dir')
    mkdir(figDir);
end

%% ------------------------------------------------------------------------
% Run Demo 3 cases
% -------------------------------------------------------------------------
fprintf('\n=== Generating Demo 3 separation-vs-time plot ===\n');

caseIDs = [1 2 3 4];

caseNames = { ...
    'Route + Basic', ...
    'Route + PN', ...
    'MPC + Basic', ...
    'MPC + PN'};

outs = cell(numel(caseIDs), 1);

for i = 1:numel(caseIDs)
    fprintf('Running case %d: %s\n', caseIDs(i), caseNames{i});

    % Second argument 0 suppresses the individual mission plot,
    % if supported by sim3_full_mission_compare.
    outs{i} = sim3_full_mission_compare(caseIDs(i), 0);
end

%% ------------------------------------------------------------------------
% Create separation figure
% -------------------------------------------------------------------------
fig = figure( ...
    'Color', 'w', ...
    'Position', [120 120 1000 650]);

hold on;
grid on;
box on;

lineStyles = {'-', '--', '-.', ':'};
lineWidths = [2.4 2.4 2.4 2.8];

for i = 1:numel(outs)

    out = outs{i};

    % UAV and chaser trajectories
    uavPos = out.uav.pos;
    pnPos  = out.pn.pos;

    % Time vector
    t = out.uav.t(:);

    % Use the common trajectory length
    n = min([size(uavPos,1), size(pnPos,1), numel(t)]);

    uavPos = uavPos(1:n, :);
    pnPos  = pnPos(1:n, :);
    t      = t(1:n);

    % Euclidean UAV-chaser separation
    sep = sqrt(sum((uavPos - pnPos).^2, 2));

    plot( ...
        t, ...
        sep, ...
        'LineStyle', lineStyles{i}, ...
        'LineWidth', lineWidths(i), ...
        'DisplayName', caseNames{i});
end

%% ------------------------------------------------------------------------
% Capture-radius line
% -------------------------------------------------------------------------
cfg = config();

yline( ...
    cfg.metrics.catch_radius, ...
    'k-', ...
    'LineWidth', 1.8, ...
    'DisplayName', sprintf( ...
        'Capture Radius = %.0f m', ...
        cfg.metrics.catch_radius));

%% ------------------------------------------------------------------------
% Labels, title, and legend
% -------------------------------------------------------------------------
xlabel('Time [s]');
ylabel('UAV--Chaser Separation [m]');

title({ ...
    'Demo 3: UAV--Chaser Separation History', ...
    ['Comparison of Route-Following and MPC Guidance Against ' ...
     'Basic and Proportional-Navigation Chasers']});

legend( ...
    'Location', 'northeast', ...
    'FontSize', 10);

ylim([0 inf]);

set(gca, ...
    'FontSize', 11, ...
    'LineWidth', 1.0);

%% ------------------------------------------------------------------------
% Save figure
% -------------------------------------------------------------------------
outfile = fullfile(figDir, 'demo3_separation_time.png');

print(fig, outfile, '-dpng', '-r220');

fprintf('\nSaved figure:\n  %s\n', outfile);
fprintf('Done.\n');