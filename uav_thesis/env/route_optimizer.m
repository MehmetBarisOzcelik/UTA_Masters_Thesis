% ROUTE_OPTIMIZER
% ---------------------------------------------------------------
% Computes a mission route:
%
%   Start -> all checkpoints in optimized order -> Goal
%
% using the augmented visibility graph from build_graph.m.
%
% The optimizer can route through auxiliary detour nodes around NFZ/radar
% zones. The checkpoint order is optimized over the required mission nodes,
% while each leg cost is computed as the shortest path through the full
% augmented graph.
%
% Required nodes are assumed to be:
%   1              = Start
%   2..nReq-1      = Checkpoints
%   nReq           = Goal
%
% Output route:
%   route.nodes          : full waypoint sequence, including detour nodes
%   route.order          : required-node visiting order
%   route.required_order : same as route.order
%   route.leg_paths      : cell array of node-index paths for each leg
%   route.cost           : total route cost
%   route.used_fallback  : true/false

function route = route_optimizer(G, env, cfg) %#ok<INUSD>

    %% --------------------------------------------------------------------
    % 1. Basic checks
    % ---------------------------------------------------------------------
    if ~isfield(G, 'nodes') || isempty(G.nodes)
        error('route_optimizer:emptyNodes', ...
              'Graph has no nodes. Cannot optimize route.');
    end

    nodes = G.nodes;

    if isfield(G, 'n_required') && ~isempty(G.n_required)
        nReq = G.n_required;
    else
        % Backward compatibility: assume only required nodes exist.
        nReq = size([env.start; env.checkpoints; env.goal], 1);
    end

    required_idx = 1:nReq;

    iS = 1;
    iG = nReq;
    CP = 2:(nReq-1);

    %% --------------------------------------------------------------------
    % 2. Build adjacency matrix
    % ---------------------------------------------------------------------
    N = size(nodes, 1);

    C = inf(N,N);
    C(1:N+1:end) = 0;

    if isfield(G, 'edges') && ~isempty(G.edges)
        E = G.edges;

        for e = 1:size(E,1)
            i = E(e,1);
            j = E(e,2);
            c = E(e,3);

            C(i,j) = min(C(i,j), c);
            C(j,i) = min(C(j,i), c);
        end
    else
        warning('route_optimizer:emptyGraph', ...
            'Visibility graph has no edges; using fallback S->CPs->G order.');

        route = fallback_route(env);
        return;
    end

    %% --------------------------------------------------------------------
    % 3. Precompute shortest paths between required nodes
    % ---------------------------------------------------------------------
    legCost = inf(nReq,nReq);
    legPath = cell(nReq,nReq);

    for a = 1:nReq
        src = required_idx(a);

        [dist, prev] = dijkstra_dense(C, src);

        for b = 1:nReq
            dst = required_idx(b);

            legCost(a,b) = dist(dst);

            if isfinite(dist(dst))
                legPath{a,b} = reconstruct_path(prev, src, dst);
            else
                legPath{a,b} = [];
            end
        end
    end

    %% --------------------------------------------------------------------
    % 4. No-checkpoint case
    % ---------------------------------------------------------------------
    if isempty(CP)
        if isfinite(legCost(iS,iG))
            pIdx = legPath{iS,iG};

            route.nodes          = nodes(pIdx,:);
            route.graph_indices  = pIdx;
            route.order          = [iS iG];
            route.required_order = [iS iG];
            route.leg_paths      = {pIdx};
            route.cost           = legCost(iS,iG);
            route.used_fallback  = false;
        else
            route = fallback_route(env);
        end

        return;
    end

    %% --------------------------------------------------------------------
    % 5. Enumerate checkpoint orders
    % ---------------------------------------------------------------------
    PermCP = perms(CP);

    bestCost  = inf;
    bestOrder = [];
    bestLegs  = {};

    for r = 1:size(PermCP,1)

        seq = [iS, PermCP(r,:), iG];

        cost     = 0;
        feasible = true;
        legs     = cell(1, numel(seq)-1);

        for k = 1:(numel(seq)-1)

            a = seq(k);
            b = seq(k+1);

            if ~isfinite(legCost(a,b)) || isempty(legPath{a,b})
                feasible = false;
                break;
            end

            cost = cost + legCost(a,b);
            legs{k} = legPath{a,b};
        end

        if feasible && cost < bestCost
            bestCost  = cost;
            bestOrder = seq;
            bestLegs  = legs;
        end
    end

    %% --------------------------------------------------------------------
    % 6. Pack best route or fallback
    % ---------------------------------------------------------------------
    if isempty(bestOrder)

        warning('route_optimizer:noFeasibleRoute', ...
            'No feasible augmented route found; using S->CPs->G in index order.');

        route = fallback_route(env);
        return;
    end

    % Concatenate graph-index paths for each leg.
    fullIdx = [];

    for k = 1:numel(bestLegs)

        p = bestLegs{k};

        if isempty(fullIdx)
            fullIdx = p;
        else
            % Avoid duplicating the connecting required node.
            fullIdx = [fullIdx, p(2:end)]; %#ok<AGROW>
        end
    end

    route.nodes          = nodes(fullIdx,:);
    route.graph_indices  = fullIdx;
    route.order          = bestOrder;
    route.required_order = bestOrder;
    route.leg_paths      = bestLegs;
    route.cost           = bestCost;
    route.used_fallback  = false;

end

% ========================================================================
% Fallback route
% ========================================================================
function route = fallback_route(env)

    P = [env.start; env.checkpoints; env.goal];
    m = size(P,1);

    route.nodes          = P;
    route.graph_indices  = 1:m;
    route.order          = 1:m;
    route.required_order = 1:m;
    route.leg_paths      = {};
    route.cost           = NaN;
    route.used_fallback  = true;

end

% ========================================================================
% Dense Dijkstra implementation
% ========================================================================
function [dist, prev] = dijkstra_dense(C, src)

    n = size(C,1);

    dist    = inf(1,n);
    prev    = zeros(1,n);
    visited = false(1,n);

    dist(src) = 0;

    for iter = 1:n %#ok<NASGU>

        % Find unvisited node with smallest distance.
        tmp = dist;
        tmp(visited) = inf;

        [dmin, u] = min(tmp);

        if ~isfinite(dmin)
            break;
        end

        visited(u) = true;

        neighbors = find(isfinite(C(u,:)) & ~visited);

        for v = neighbors

            alt = dist(u) + C(u,v);

            if alt < dist(v)
                dist(v) = alt;
                prev(v) = u;
            end
        end
    end

end

% ========================================================================
% Reconstruct path from Dijkstra predecessor array
% ========================================================================
function path = reconstruct_path(prev, src, dst)

    if src == dst
        path = src;
        return;
    end

    path = dst;
    u    = dst;

    while u ~= src

        u = prev(u);

        if u == 0
            path = [];
            return;
        end

        path = [u, path]; %#ok<AGROW>
    end

end