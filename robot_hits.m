function [hit, C] = robot_hits(pose, W, others)
% ROBOT_HITS  True if a robot at pose [x y yaw] leaves the floor or overlaps an obstacle / another robot.
%   others = one row [x y yaw] per other robot (optional)
%   C      = 2xK centres of everything it touches (only collected when asked for)
if nargin < 3, others = zeros(0,3); end
all_ = nargout > 1; C = zeros(2,0);
fp = W.footprint + W.margin*[-1 1 -1 1];
P  = corners(pose, fp);
f  = W.floor;
hit = any(P(1,:) < f(1) | P(1,:) > f(2) | P(2,:) < f(3) | P(2,:) > f(4));
if hit                                             % a point far beyond the crossed floor edge
    out = [any(P(1,:) > f(2)) - any(P(1,:) < f(1)); any(P(2,:) > f(4)) - any(P(2,:) < f(3))];
    C(:,end+1) = [pose(1); pose(2)] + 10*out;
end
r = norm(fp([2 4])) + 0.1;                         % quick distance pre-check radius
for k = 1:size(W.allBoxes,1)
    if hit && ~all_, return; end
    b = W.allBoxes(k,:);
    if norm(b(1:2) - pose(1:2)) > r + norm(b(3:4))/2, continue; end
    if overlap(P, corners([b(1:2) b(6)*pi/180], [-b(3) b(3) -b(4) b(4)]/2))
        hit = true; C(:,end+1) = b(1:2)'; %#ok<AGROW>
    end
end
for k = 1:size(others,1)
    if hit && ~all_, return; end
    if norm(others(k,1:2) - pose(1:2)) > 2*r, continue; end
    if overlap(P, corners(others(k,:), W.footprint))
        hit = true; C(:,end+1) = others(k,1:2)'; %#ok<AGROW>
    end
end
end

function P = corners(pose, fp)
a = pose(3);
P = [cos(a) -sin(a); sin(a) cos(a)]*[fp([1 2 2 1]); fp([3 3 4 4])] + [pose(1); pose(2)];
end

function o = overlap(A, B)
% separating-axis test for two convex quads (2x4 corner lists)
o = true;
for q = {A, B}
    Q = q{1};
    for i = 1:2
        e = Q(:,i+1) - Q(:,i); n = [-e(2); e(1)];
        a = n'*A; b = n'*B;
        if max(a) < min(b) || max(b) < min(a), o = false; return; end
    end
end
end
