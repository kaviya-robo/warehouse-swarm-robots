function out = robot_collision_gate(u)
% ROBOT_COLLISION_GATE  Stops your robot just before it hits an obstacle, a swarm robot or the floor edge.
%   u = [v w x z yaw  swarmPoses]  ->  out = [v; w]   (any part of the command that would collide is 0)
% Called every step by the CollisionGate block in Assem2_Drive.  Run  clear robot_collision_gate
% after editing robot_world.m (drive_robot does this for you).
persistent W
if isempty(W), W = robot_world; end
v = u(1); w = u(2); pose = u(3:5); pose = pose(:)';
others = reshape(u(6:end),3,[])';
out = [v; w];
if v == 0 && w == 0, return; end

look = [0.03 0.06];                             % look-ahead: 3 cm of travel, ~3.4 deg of turn
ok = @(a,b) ~robot_move_blocked(pose, pose + [a*look(1)*cos(pose(3)), a*look(1)*sin(pose(3)), b*look(2)], W, others);
sv = sign(v); sw = sign(w);
if ok(sv,sw),                 return;            % full command is free
elseif sv ~= 0 && ok(sv,0),   out = [v; 0];      % drive, but don't turn
elseif sw ~= 0 && ok(0,sw),   out = [0; w];      % turn, but don't drive
else,                         out = [0; 0];
end
end
