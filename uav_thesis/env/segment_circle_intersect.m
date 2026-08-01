function hit = segment_circle_intersect(p1, p2, c, r)
% SEGMENT_CIRCLE_INTERSECT
% ---------------------------------------------------------------
% Returns true if the line segment from p1 to p2 intersects or touches
% a circle centered at c with radius r.
%
% Inputs:
%   p1 : first segment endpoint [x y]
%   p2 : second segment endpoint [x y]
%   c  : circle center [x y]
%   r  : circle radius
%
% Output:
%   hit : true if the segment intersects, touches, or starts/ends inside
%         the circle
%
% This utility is used by the route planner to reject visibility-graph
% edges that would pass through NFZ or radar keep-out regions.

% -------------------------------------------------------------------------
% Basic input formatting
% -------------------------------------------------------------------------
p1 = p1(:).';
p2 = p2(:).';
c  = c(:).';

% -------------------------------------------------------------------------
% Check endpoint containment first
% -------------------------------------------------------------------------
if point_inside_circle(p1, c, r) || point_inside_circle(p2, c, r)
    hit = true;
    return;
end

% -------------------------------------------------------------------------
% Degenerate segment case
% -------------------------------------------------------------------------
d = p2 - p1;
a = dot(d,d);

if a < eps
    hit = point_inside_circle(p1, c, r);
    return;
end

% -------------------------------------------------------------------------
% Solve quadratic intersection with infinite line, then check segment
% parameter range 0 <= t <= 1.
% -------------------------------------------------------------------------
f  = p1 - c;
b  = 2 * dot(f,d);
cc = dot(f,f) - r^2;

disc = b*b - 4*a*cc;

if disc < 0
    hit = false;
    return;
end

sqrtD = sqrt(disc);

t1 = (-b - sqrtD) / (2*a);
t2 = (-b + sqrtD) / (2*a);

hit = (t1 >= 0 && t1 <= 1) || ...
      (t2 >= 0 && t2 <= 1);

end

% ========================================================================
function inside = point_inside_circle(p, c, r)
% POINT_INSIDE_CIRCLE
% ---------------------------------------------------------------
% Returns true if point p lies inside or on the circle.

inside = hypot(p(1)-c(1), p(2)-c(2)) <= r;

end