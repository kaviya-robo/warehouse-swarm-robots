function add_swarm(m)
% ADD_SWARM  Add the swarm robots from robot_world.m to Assem2_Drive.
%
%   add_swarm                 -> updates Assem2_Drive.slx (safe to run again after editing robot_world.m)
%
% Builds the subsystem  Swarm  with
%   * Robot2, Robot3, ...  : full copies of your robot (chassis, wheels, planar joint, drive kinematics)
%   * SwarmController      : swarm_controller.m, gives every copy its speed / yaw rate
% and feeds the swarm poses into the CollisionGate so your teleoperated robot can't drive into them.

if nargin < 1, m = 'Assem2_Drive'; end
load_system(m); load_system('sm_lib'); load_system('nesl_utility'); load_system('simulink');
W = robot_world; N = numel(W.swarm);

%% ---------------- Swarm subsystem (wired once, contents rebuilt every call) ----------------
sw = [m '/Swarm'];
if getSimulinkBlockHandle(sw) < 0
    tp = get_param([m '/ToPlane'],'Position');
    add_block('simulink/Ports & Subsystems/Subsystem',sw,'Position',tp + [0 220 30 260]);
    Simulink.SubSystem.deleteContents(sw);
    add_block('nesl_utility/Connection Port',[sw '/Plane'],'Position',[20 30 50 50]);
    add_block('simulink/Sources/In1',[sw '/Robot1Pose'],'Position',[20 100 50 114]);
    add_block('simulink/Sinks/Out1',[sw '/SwarmPoses'],'Position',[900 100 930 114]);
    add_line(m, getPort([m '/ToPlane'],'RConn',1), getPort(sw,'LConn',1),'autorouting','on');
    add_line(m,'PoseMux/1','Swarm/1','autorouting','on');
    set_param([m '/GateMux'],'Inputs','5');
    add_line(m,'Swarm/1','GateMux/5','autorouting','on');
else
    delete_line(find_system(sw,'SearchDepth',1,'FindAll','on','Type','line'));
    blks = find_system(sw,'SearchDepth',1,'Type','block');
    delete_block(setdiff(blks,{sw,[sw '/Plane'],[sw '/Robot1Pose'],[sw '/SwarmPoses']}));
end

if N == 0                                       % no swarm: one dummy robot far away
    add_block('simulink/Sources/Constant',[sw '/NoRobots'],'Value','[1e3;1e3;0]','Position',[700 95 760 125]);
    add_line(sw,'NoRobots/1','SwarmPoses/1');
    save_system(m); return;
end

add_block('simulink/Sources/Clock',[sw '/Clock'],'Position',[20 160 50 180]);
add_block('simulink/Signal Routing/Mux',[sw '/CtrlIn'],'Inputs',num2str(N+2),'Position',[120 60 125 60+40*(N+2)]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function',[sw '/SwarmController'], ...
    'MATLABFcn','swarm_controller(u)','OutputDimensions',num2str(2*N), ...
    'SampleTime',num2str(W.swarmPeriod),'Position',[170 100 300 140]);
add_block('simulink/Signal Routing/Demux',[sw '/CmdSplit'],'Outputs',mat2str(2*ones(1,N)),'Position',[340 60 345 60+60*N]);
add_block('simulink/Signal Routing/Mux',[sw '/Poses'],'Inputs',num2str(N),'Position',[800 60 805 60+60*N]);
add_line(sw,'Clock/1','CtrlIn/1','autorouting','on');
add_line(sw,'Robot1Pose/1','CtrlIn/2','autorouting','on');
add_line(sw,'CtrlIn/1','SwarmController/1','autorouting','on');
add_line(sw,'SwarmController/1','CmdSplit/1','autorouting','on');
add_line(sw,'Poses/1','SwarmPoses/1','autorouting','on');
plane = getPort([sw '/Plane'],'RConn',1);

for k = 1:N
    name = sprintf('Robot%d',k+1); y = 40 + 110*(k-1);
    r = makeRobot(m, [sw '/' name], [450 y 600 y+80], W.swarm(k));
    add_line(sw, sprintf('CmdSplit/%d',k), [name '/1'],'autorouting','on');
    add_line(sw, [name '/1'], sprintf('CtrlIn/%d',k+2),'autorouting','on');
    add_line(sw, [name '/1'], sprintf('Poses/%d',k),'autorouting','on');
    add_line(sw, plane, getPort(r,'LConn',1),'autorouting','on');
end

save_system(m);
fprintf('%s: %d swarm robots added (Swarm subsystem).\n', m, N);
end

%% ---------- one robot = copy of the teleoperated robot's blocks ----------
function r = makeRobot(m, r, pos, spec)
add_block('simulink/Ports & Subsystems/Subsystem', r, 'Position', pos);
Simulink.SubSystem.deleteContents(r);
add_block('nesl_utility/Connection Port',[r '/Plane'],'Position',[-200 -300 -170 -280]);
add_block('simulink/Sources/In1',[r '/cmd'],'Position',[-60 300 -30 314]);
add_block('simulink/Sinks/Out1',[r '/pose'],'Position',[800 450 830 464]);

wheels = {'Transform','Revolute','Part2_1_RIGID','wGain1','wInt1','wPS1'
          'Transform1','Revolute1','Part2_2_RIGID','wGain2','wInt2','wPS2'
          'Transform2','Revolute2','Part2_3_RIGID','wGain3','wInt3','wPS3'
          'Transform3','Revolute3','Part2_4_RIGID','wGain4','wInt4','wPS4'};
names = [{'ChassisPlanar','FromPlane','Transform4','Part1_1_RIGID','X_ps','Z_ps','Yaw_ps', ...
          'Demux','Yaw','cosYaw','sinYaw','vx','vz','X','Z','HalfTrack','vL','vR','PoseMux'}, wheels(:)'];
for i = 1:numel(names)
    add_block([m '/' names{i}], [r '/' names{i}], 'Position', get_param([m '/' names{i}],'Position'));
end

% drive kinematics (same wiring as build_robot_motion)
L = {'cmd/1','Demux/1'; 'Demux/1','vx/1'; 'Demux/1','vz/1'; 'Demux/1','vL/1'; 'Demux/1','vR/1'
     'Demux/2','Yaw/1'; 'Demux/2','HalfTrack/1'; 'Yaw/1','cosYaw/1'; 'Yaw/1','sinYaw/1'
     'Yaw/1','Yaw_ps/1'; 'Yaw/1','PoseMux/3'; 'cosYaw/1','vx/2'; 'sinYaw/1','vz/2'
     'vx/1','X/1'; 'vz/1','Z/1'; 'X/1','X_ps/1'; 'X/1','PoseMux/1'; 'Z/1','Z_ps/1'; 'Z/1','PoseMux/2'
     'HalfTrack/1','vL/2'; 'HalfTrack/1','vR/2'; 'vL/1','wGain1/1'; 'vL/1','wGain2/1'
     'vR/1','wGain3/1'; 'vR/1','wGain4/1'; 'PoseMux/1','pose/1'};
for i = 1:4, L = [L; {[wheels{i,4} '/1'],[wheels{i,5} '/1']; [wheels{i,5} '/1'],[wheels{i,6} '/1']}]; end %#ok<AGROW>
for i = 1:size(L,1), add_line(r, L{i,1}, L{i,2}, 'autorouting','on'); end

% multibody chain:  Plane -> planar joint -> chassis centre -> chassis + 4 wheel joints
P = @(b,t,n) getPort([r '/' b],t,n);
add_line(r, P('Plane','RConn',1),         P('ChassisPlanar','LConn',1));
add_line(r, P('X_ps','RConn',1),          P('ChassisPlanar','LConn',2));
add_line(r, P('Z_ps','RConn',1),          P('ChassisPlanar','LConn',3));
add_line(r, P('Yaw_ps','RConn',1),        P('ChassisPlanar','LConn',4));
add_line(r, P('ChassisPlanar','RConn',1), P('FromPlane','LConn',1));
add_line(r, P('FromPlane','RConn',1),     P('Transform4','LConn',1));
add_line(r, P('Transform4','RConn',1),    bodyPort([r '/Part1_1_RIGID']));
for i = 1:4
    add_line(r, P('FromPlane','RConn',1), P(wheels{i,1},'LConn',1));
    add_line(r, P(wheels{i,1},'RConn',1), P(wheels{i,2},'RConn',1));
    add_line(r, P(wheels{i,2},'LConn',1), bodyPort([r '/' wheels{i,3}]));
    add_line(r, P(wheels{i,6},'RConn',1), P(wheels{i,2},'LConn',2));
end

% start pose + colour
set_param([r '/X'],'InitialCondition',num2str(spec.start(1)));
set_param([r '/Z'],'InitialCondition',num2str(spec.start(2)));
set_param([r '/Yaw'],'InitialCondition',num2str(spec.start(3)*pi/180,17));
try, set_param([r '/Part1_1_RIGID/Solid'],'GraphicDiffuseColor',mat2str(spec.color,4)); catch, end
end

function h = bodyPort(blk)
% the imported part subsystems have a single frame port, on either side
ph = get_param(blk,'PortHandles'); h = [ph.LConn ph.RConn]; h = h(1);
end

function h = getPort(blk,type,n)
ph = get_param(blk,'PortHandles'); h = ph.(type)(n);
end
