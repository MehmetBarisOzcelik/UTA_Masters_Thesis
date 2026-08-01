function ang = wrapToPi(ang)
% WRAPTOPI
% ---------------------------------------------------------------
% Wraps angles in radians to the principal interval [-pi, pi].
%
% Input:
%   ang : scalar, vector, or array of angles [rad]
%
% Output:
%   ang : wrapped angle values [rad]
%
% This local utility avoids dependence on the Mapping Toolbox version of
% wrapToPi and keeps the thesis code portable.

ang = mod(ang + pi, 2*pi);

ang(ang < 0) = ang(ang < 0) + 2*pi;

ang = ang - pi;

end