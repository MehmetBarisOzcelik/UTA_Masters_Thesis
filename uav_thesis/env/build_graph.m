% BUILD_GRAPH
% ---------------------------------------------------------------
% Builds an augmented visibility graph for mission routing.
%
% Required mission nodes:
%   [start; checkpoints; goal]
%
% Auxiliary detour nodes:
%   Automatically generated around NFZ and radar keep-out regions.
%
% Each candidate edge is:
%   - rejected if the segment intersects an NFZ or radar keep-out disk
%     expanded by cfg.env.edge_margin
%   - assigned a cost based on:
%       * Euclidean length
%       * integrated soft risk near NFZ/radar boundaries
%
% Output struct G:
%   G.nodes        : all graph node coordinates [required; auxiliary]
%   G.required_idx : indices of required nodes in G.nodes
%   G.aux_idx      : indices of auxiliary detour nodes
%   G.n_required   : number of required mission nodes
%   G.edges        : [i j cost] undirected weighted visibility edges
%   G.edge_margin  : edge keep-out expansion margin
%
% Used by route_optimizer.m to compute:
%
%   start -> checkpoints in optimized order -> goal
%
% with shortest/risk-aware paths through the visibility graph.

function G = build_graph(env, cfg)

%% ------------------------------------------------------------------------
% 1. Required mission nodes
% -------------------------------------------------------------------------

required_pts = [env.start; env.checkpoints; env.goal];
nReq         = size(required_pts, 1);

%% ------------------------------------------------------------------------
% 2. Extract threat zones
% -------------------------------------------------------------------------

nfz = [];
rad = [];

if isfield(env, 'nfz') && ~isempty(env.nfz)
    nfz = env.nfz;
end

if isfield(env, 'radar') && ~isempty(env.radar)
    rad = env.radar;
end

% For hard geometric edge rejection, NFZ and radar zones are both treated
% as circular keep-out regions.
zones = [nfz;
         rad];

%% ------------------------------------------------------------------------
% 3. Edge margin / safety buffer
% -------------------------------------------------------------------------

if isfield(cfg, 'env') && isfield(cfg.env, 'edge_margin')
    edge_margin = cfg.env.edge_margin;
elseif isfield(env, 'edge_margin')
    edge_margin = env.edge_margin;
elseif isfield(cfg, 'env') && isfield(cfg.env, 'cp_clear')
    edge_margin = 0.4 * cfg.env.cp_clear;
else
    edge_margin = 50;
end

%% ------------------------------------------------------------------------
% 4. Generate auxiliary detour nodes
% -------------------------------------------------------------------------

aux_pts = generate_detour_nodes(required_pts, zones, env, cfg, edge_margin);

%% ------------------------------------------------------------------------
% 5. Full graph node set
% -------------------------------------------------------------------------

pts = [required_pts;
       aux_pts];

n = size(pts, 1);

required_idx = 1:nReq;
aux_idx      = (nReq+1):n;

%% ------------------------------------------------------------------------
% 6. Build visibility edges
% -------------------------------------------------------------------------

E = [];

for i = 1:n
    for j = i+1:n

        p1 = pts(i,:);
        p2 = pts(j,:);

        % Reject edge if it intersects any expanded keep-out zone.
        if segment_hits_any_zone(p1, p2, zones, edge_margin)
            continue;
        end

        % Compute edge cost.
        L    = hypot(p2(1)-p1(1), p2(2)-p1(2));
        risk = integrated_risk_along(p1, p2, nfz, rad, edge_margin, cfg);

        cost = cfg.route.w_t * L + cfg.route.w_r * risk;

        if isfinite(cost)
            E = [E; i j cost]; %#ok<AGROW>
        end
    end
end

%% ------------------------------------------------------------------------
% 7. Pack output
% -------------------------------------------------------------------------

G.nodes        = pts;
G.required_idx = required_idx;
G.aux_idx      = aux_idx;
G.n_required   = nReq;
G.edges        = E;
G.edge_margin  = edge_margin;

end

% ========================================================================
% Generate detour nodes around all threat zones
% ========================================================================
function aux_pts = generate_detour_nodes(required_pts, zones, env, cfg, edge_margin)

aux_pts = [];

if isempty(zones)
    return;
end

% Number of candidate nodes around each obstacle.
% More nodes improve route flexibility but increase graph size.
nTheta = 16;

% Additional radial clearance beyond the expanded obstacle radius.
% This reduces the chance that edges between adjacent detour nodes cut
% through an expanded keep-out region.
detour_extra = 80;

if isfield(cfg, 'env') && isfield(cfg.env, 'cp_clear')
    detour_extra = max(detour_extra, 0.5 * cfg.env.cp_clear);
end

theta = linspace(0, 2*pi, nTheta+1);
theta(end) = [];

for k = 1:size(zones,1)

    c = zones(k,1:2);
    R = zones(k,3);

    rho = R + edge_margin + detour_extra;

    cand = [c(1) + rho*cos(theta(:)), ...
            c(2) + rho*sin(theta(:))];

    for i = 1:size(cand,1)

        p = cand(i,:);

        % Keep only nodes inside map bounds.
        if isfield(env, 'bounds') && ~isempty(env.bounds)
            if p(1) < env.bounds(1,1) || p(1) > env.bounds(1,2) || ...
               p(2) < env.bounds(2,1) || p(2) > env.bounds(2,2)
                continue;
            end
        end

        % Reject nodes inside any expanded keep-out zone.
        if point_inside_any_zone(p, zones, edge_margin)
            continue;
        end

        % Reject nodes too close to required mission points.
        if ~isempty(required_pts)
            dReq = sqrt(sum((required_pts - p).^2, 2));
            if min(dReq) < 20
                continue;
            end
        end

        aux_pts = [aux_pts; p]; %#ok<AGROW>
    end
end

% Remove near-duplicate auxiliary nodes.
aux_pts = unique_rows_tol(aux_pts, 1e-6);

end

% ========================================================================
% Segment-zone collision check
% ========================================================================
function hit = segment_hits_any_zone(p1, p2, zones, margin)

hit = false;

if isempty(zones)
    return;
end

for k = 1:size(zones,1)

    c = zones(k,1:2);
    r = zones(k,3) + margin;

    if segment_circle_intersect(p1, p2, c, r)
        hit = true;
        return;
    end
end

end

% ========================================================================
% Point-zone collision check
% ========================================================================
function inside = point_inside_any_zone(p, zones, margin)

inside = false;

if isempty(zones)
    return;
end

for k = 1:size(zones,1)

    c = zones(k,1:2);
    r = zones(k,3) + margin;

    if hypot(p(1)-c(1), p(2)-c(2)) <= r
        inside = true;
        return;
    end
end

end

% ========================================================================
% Integrated risk along a segment
% ========================================================================
function riskInt = integrated_risk_along(p1, p2, nfz, rad, margin, cfg)

M  = 35;
xs = linspace(p1(1), p2(1), M);
ys = linspace(p1(2), p2(2), M);

riskSum = 0;

for m = 1:M
    p = [xs(m), ys(m)];
    riskSum = riskSum + point_risk(p, nfz, rad, margin, cfg);
end

L = hypot(p2(1)-p1(1), p2(2)-p1(2));

riskInt = (riskSum / M) * L;

end

% ========================================================================
% Pointwise risk near NFZ/radar boundaries
% ========================================================================
function r = point_risk(p, nfz, rad, margin, cfg)

r = 0;

% Normalized NFZ risk weight
if isfield(cfg, 'mpc') && isfield(cfg.mpc, 'w_nfz')
    wNFZ = cfg.mpc.w_nfz;
else
    wNFZ = 1800;
end

% Normalized radar risk weight
if isfield(cfg, 'mpc') && isfield(cfg.mpc, 'w_radar')
    wRAD = cfg.mpc.w_radar;
else
    wRAD = 1000;
end

% Normalize to keep routing risk numerically moderate.
wNFZ = wNFZ / 1800;
wRAD = wRAD / 1800;

% NFZ contribution
if ~isempty(nfz)
    for k = 1:size(nfz,1)

        c = nfz(k,1:2);
        R = nfz(k,3);
        d = hypot(p(1)-c(1), p(2)-c(2));

        if d <= R
            r = r + 1e3;

        elseif d < R + margin
            s = (d - R) / max(margin, 1e-6);
            r = r + wNFZ * exp(-4*s^2);
        end
    end
end

% Radar contribution
if ~isempty(rad)
    for k = 1:size(rad,1)

        c = rad(k,1:2);
        R = rad(k,3);
        d = hypot(p(1)-c(1), p(2)-c(2));

        if d <= R
            r = r + 1e3;

        elseif d < R + margin
            s = (d - R) / max(margin, 1e-6);
            r = r + wRAD * exp(-4*s^2);
        end
    end
end

end

% ========================================================================
% Remove duplicate rows with tolerance
% ========================================================================
function Aout = unique_rows_tol(A, tol)

if isempty(A)
    Aout = A;
    return;
end

Aout = A(1,:);

for i = 2:size(A,1)

    d = sqrt(sum((Aout - A(i,:)).^2, 2));

    if all(d > tol)
        Aout = [Aout; A(i,:)]; %#ok<AGROW>
    end
end

end