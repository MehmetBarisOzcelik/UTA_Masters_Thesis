% MISS_DISTANCE
% ---------------------------------------------------------------
% Computes the minimum separation, or miss distance, between the
% UAV and chaser trajectories.
%
% Inputs:
%   trajU : [N x 2] UAV positions [x y]
%   trajP : [N x 2] chaser positions [x y]
%
% Output struct:
%   md.value     : minimum separation distance [m]
%   md.t_index   : index at which the minimum occurs
%   md.series    : separation history over the shared time interval [m]
%
% Notes:
%   If the two trajectories have different lengths, only the shared
%   portion is used.

function md = miss_distance(trajU, trajP)

% -------------------------------------------------------------------------
% Basic validation
% -------------------------------------------------------------------------
if isempty(trajU) || isempty(trajP)
    md.value   = NaN;
    md.t_index = NaN;
    md.series  = [];
    return;
end

if size(trajU,2) < 2 || size(trajP,2) < 2
    error('miss_distance:badInput', ...
          'trajU and trajP must both have at least two columns: [x y].');
end

% -------------------------------------------------------------------------
% Use only the shared portion of the two trajectories
% -------------------------------------------------------------------------
n = min(size(trajU,1), size(trajP,1));

if n < 1
    md.value   = NaN;
    md.t_index = NaN;
    md.series  = [];
    return;
end

PU = trajU(1:n,1:2);
PP = trajP(1:n,1:2);

% -------------------------------------------------------------------------
% Separation history and minimum miss distance
% -------------------------------------------------------------------------
d = sqrt(sum((PU - PP).^2, 2));

[md.value, idx] = min(d);

md.t_index = idx;
md.series  = d;

end