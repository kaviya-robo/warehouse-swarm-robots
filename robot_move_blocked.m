function b = robot_move_blocked(p, q, W, others)
% ROBOT_MOVE_BLOCKED  May a robot go from pose p to pose q ([x y yaw]) without collision?
%   Normal case: blocked if q is inside the safety margin of an obstacle / robot / floor edge.
%   Already inside the margin at p: only moves that don't approach what it touches are allowed
%   (turning on the spot, backing away), and never into a real overlap.
[hq, Cq] = robot_hits(q, W, others);
if ~hq, b = false; return; end
if ~robot_hits(p, W, others), b = true; return; end
W0 = W; W0.margin = 0;
if robot_hits(q, W0, others) && ~robot_hits(p, W0, others), b = true; return; end
b = any(vecnorm(Cq - q(1:2)', 2, 1) < vecnorm(Cq - p(1:2)', 2, 1) - 1e-9);
end
