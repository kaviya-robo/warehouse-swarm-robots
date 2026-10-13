function add_world(m)
% ADD_WORLD  Put the floor + obstacles from robot_world.m into Assem2_Drive and add the collision gate.
%
%   add_world                 -> updates Assem2_Drive.slx (safe to run again after editing robot_world.m)
%
% * Environment subsystem: floor and obstacle solids, fixed to the robot's floor plane (ToPlane frame)
% * CollisionGate: sits between CmdMux and Demux, zeroes speed / yaw rate that would cause a collision

if nargin < 1, m = 'Assem2_Drive'; end
load_system(m); load_system('sm_lib'); load_system('nesl_utility'); load_system('simulink');
W = robot_world;

%% ---------------- Environment subsystem (built once, contents refreshed every call) ----------------
env = [m '/Environment'];
if getSimulinkBlockHandle(env) < 0
    tp = get_param([m '/ToPlane'],'Position');
    add_block('simulink/Ports & Subsystems/Subsystem',env,'Position',tp + [0 120 30 140]);
    Simulink.SubSystem.deleteContents(env);
    add_block('nesl_utility/Connection Port',[env '/Plane'],'Position',[20 100 50 120]);
    add_line(m, getPort([m '/ToPlane'],'RConn',1), getPort(env,'LConn',1),'autorouting','on');
else
    delete_line(find_system(env,'SearchDepth',1,'FindAll','on','Type','line'));
    blks = find_system(env,'SearchDepth',1,'Type','block');
    delete_block(setdiff(blks,{env,[env '/Plane']}));
end
pp = get_param([env '/Plane'],'PortHandles'); plane = [pp.LConn pp.RConn];

h = W.floorDepth;   % ToPlane frame: x,y = floor plane, +z points DOWN (towards the floor)
f = W.floor; t = W.floorThick;
addBox(env, plane, 'Floor', [mean(f(1:2)) mean(f(3:4)) h+t/2], 0, [f(2)-f(1) f(4)-f(3) t], W.floorColor, 0);
for k = 1:size(W.boxes,1)
    b = W.boxes(k,:);
    addBox(env, plane, sprintf('Obstacle%d',k), [b(1) b(2) h-b(5)/2], b(6), b(3:5), W.boxColor, k);
end
for k = 1:size(W.walls,1)
    b = W.walls(k,:);
    addBox(env, plane, sprintf('Wall%d',k), [b(1) b(2) h-b(5)/2], b(6), b(3:5), W.wallColor, size(W.boxes,1)+k);
end

%% ---------------- collision gate (inserted once) ----------------
if getSimulinkBlockHandle([m '/CollisionGate']) < 0
    delete_line(m,'CmdMux/1','Demux/1');
    p = get_param([m '/CmdMux'],'Position');      % place the gate above the command blocks
    add_block('simulink/Signal Routing/Mux',[m '/GateMux'],'Inputs','4','Position',[p(1)+20 p(2)-170 p(1)+25 p(2)-90]);
    add_block('simulink/User-Defined Functions/Interpreted MATLAB Function',[m '/CollisionGate'], ...
        'MATLABFcn','robot_collision_gate(u)','OutputDimensions','2','Position',[p(1)+60 p(2)-145 p(1)+170 p(2)-115]);
    add_line(m,'CmdMux/1','GateMux/1','autorouting','on');
    add_line(m,'X/1','GateMux/2','autorouting','on');
    add_line(m,'Z/1','GateMux/3','autorouting','on');
    add_line(m,'Yaw/1','GateMux/4','autorouting','on');
    add_line(m,'GateMux/1','CollisionGate/1','autorouting','on');
    add_line(m,'CollisionGate/1','Demux/1','autorouting','on');
end
% discrete gate: its on/off switching must not happen inside the continuous solver's minor steps
set_param([m '/CollisionGate'],'SampleTime','0.01');

save_system(m);
fprintf('%s: floor + %d obstacles + %d walls, collision gate on.\n', m, size(W.boxes,1), size(W.walls,1));
end

%% ---------- helpers ----------
function addBox(sys, plane, name, xyz, yawDeg, dims, color, row)
y = 40 + row*90;
rt = add_block('sm_lib/Frames and Transforms/Rigid Transform',[sys '/' name 'Pose'],'Position',[120 y 190 y+40]);
set_param(rt,'TranslationMethod','Cartesian','TranslationCartesianOffset',mat2str(xyz,6),'TranslationCartesianOffsetUnits','m');
if yawDeg ~= 0
    set_param(rt,'RotationMethod','StandardAxis','RotationStandardAxis','+Z', ...
        'RotationAngle',num2str(yawDeg),'RotationAngleUnits','deg');
end
bs = add_block('sm_lib/Body Elements/Brick Solid',[sys '/' name],'Position',[250 y 310 y+40]);
set_param(bs,'BrickDimensions',mat2str(dims,6),'BrickDimensionsUnits','m', ...
    'GraphicVisPropType','SimpleVisualProperties','GraphicDiffuseColor',mat2str(color,4));
add_line(sys, plane, getPort(rt,'LConn',1),'autorouting','on');
bp = get_param(bs,'PortHandles');
add_line(sys, getPort(rt,'RConn',1), [bp.LConn bp.RConn],'autorouting','on');
end

function h = getPort(blk,type,n)
ph = get_param(blk,'PortHandles'); h = ph.(type)(n);
end
