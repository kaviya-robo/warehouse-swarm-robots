function build_robot_motion(varargin)
% BUILD_ROBOT_MOTION  Turn the imported SolidWorks model Assem2 into a driveable robot.
%
%   build_robot_motion            -> creates Assem2_Drive.slx (Assem2.slx is left untouched)
%
% What it does
%   * frees the chassis (Part1) from World with a Planar joint (x, z, yaw on the floor)
%   * hangs the 4 wheel joints off the chassis instead of off World
%   * adds a drive-command block (forward / reverse / turns / arcs / spin) and
%     differential-drive kinematics that feed the planar joint AND spin the wheels
%
% Edit the parameters below, then run:  build_robot_motion; sim('Assem2_Drive')

%% ---------------- parameters you can change ----------------
p.wheelRadius = 0.1877;   % m  (estimated from wheel mass/inertia - replace with the real value)
p.stopTime    = inf;      % s  (runs until you quit drive_robot)
% sign: +1 if +X of the model is "forward"
p.fwdSign     = 1;
% joint-velocity -> wheel world-Z rotation sign for [Revolute Revolute1 Revolute2 Revolute3]
% (derived from base/follower frame orientation of each joint; flip an entry if a wheel spins backwards)
p.wheelSign   = [-1 +1 +1 +1];
% wheels on the +Z side of the chassis = "left" (Part2-1, Part2-2); -Z side = "right" (Part2-3, Part2-4)
p.leftWheels  = [1 2];
p.rightWheels = [3 4];

%% ---------------- load model + data ----------------
here = fileparts(mfilename('fullpath')); cd(here);
run('Assem2_DataFile.m');                       % creates smiData
assignin('base','smiData',smiData);
src = 'Assem2'; dst = 'Assem2_Drive';
if bdIsLoaded(dst), close_system(dst,0); end
load_system(src); load_system('sm_lib'); load_system('simulink');
save_system(src,dst); load_system(dst); m = dst;
set_param(m,'StopTime',num2str(p.stopTime));

C   = smiData.RigidTransform(9).translation;   % chassis centre in World [mm]
RT  = smiData.RigidTransform;
track = abs(RT(2).translation(3) - RT(6).translation(3))/1000;   % m, left-right wheel spacing

%% ---------------- re-wire: World -> planar joint -> chassis-centre node ----------------
% The old World net (wheel transforms, chassis transform, config blocks) becomes the chassis-centre node:
% we replace World with a World -> ToPlane -> Planar -> FromPlane chain whose output lands on that net.
rtLib = 'sm_lib/Frames and Transforms/Rigid Transform';
wpos = get_param([m '/World'],'Position');
pc   = get_param([m '/World'],'PortConnectivity'); wp = pc(1).Position;     % World RConn1 location
add_block([src '/World'],[m '/WorldNew'],'Position',wpos + [0 -260 0 -260]);
delete_block([m '/World']);
set_param([m '/WorldNew'],'Name','World');

fromPlane = add_block(rtLib,[m '/FromPlane'],'Position',[wp(1)-70 wp(2)-20 wp(1) wp(2)+20]);   % re-attaches to old World branch
set_param(fromPlane,'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','-90','RotationAngleUnits','deg');
x0 = wp(1)-700; y0 = wp(2)-300;
toPlane = add_block(rtLib,[m '/ToPlane'],'Position',[x0 y0 x0+70 y0+40]);
set_param(toPlane,'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90','RotationAngleUnits','deg', ...
    'TranslationMethod','Cartesian','TranslationCartesianOffset',mat2str(C),'TranslationCartesianOffsetUnits','mm');
pj = add_block('sm_lib/Joints/Planar Joint',[m '/ChassisPlanar'],'Position',[x0+150 y0-30 x0+250 y0+70]);
set_param(pj,'PxMotionActuationMode','InputMotion','PyMotionActuationMode','InputMotion','RzMotionActuationMode','InputMotion', ...
    'PxTorqueActuationMode','ComputedTorque','PyTorqueActuationMode','ComputedTorque','RzTorqueActuationMode','ComputedTorque', ...
    'PxSensePosition','on','PySensePosition','on','RzSensePosition','on');
wd = get_param([m '/World'],'PortHandles');
% SolidWorks is Y-up, Mechanics Explorer / gravity are Z-up: tilt the whole robot so it stands upright
tilt = add_block(rtLib,[m '/SolidWorksToZup'],'Position',[x0-110 y0 x0-40 y0+40]);
set_param(tilt,'RotationMethod','StandardAxis','RotationStandardAxis','+X','RotationAngle','90','RotationAngleUnits','deg');
add_line(m, wd.RConn(1), getPort(tilt,'LConn',1),'autorouting','on');
add_line(m, getPort(tilt,'RConn',1), getPort(toPlane,'LConn',1),'autorouting','on');
add_line(m, getPort(toPlane,'RConn',1), getPort(pj,'LConn',1),'autorouting','on');
add_line(m, getPort(pj,'RConn',1), getPort(fromPlane,'LConn',1),'autorouting','on');

% chassis transform: Part1 frame == chassis centre frame
set_param([m '/Transform4'],'RotationMethod','None','TranslationMethod','None');

%% ---------------- wheel frames are now relative to the chassis centre ----------------
wheelT = {'Transform','Transform1','Transform2','Transform3'};
wheelJ = {'Revolute','Revolute1','Revolute2','Revolute3'};
wheelIdx = [2 3 6 8];                         % RigidTransform entries for F[PartX]
for i = 1:4
    r = RT(wheelIdx(i));
    set_param([m '/' wheelT{i}],'TranslationCartesianOffset',mat2str(r.translation - C));
    set_param([m '/' wheelJ{i}],'MotionActuationMode','InputMotion','TorqueActuationMode','ComputedTorque','SensePosition','on');
end

%% ---------------- drive command + kinematics ----------------
% live commands: drive_robot.m changes these two Constant blocks while the simulation runs
set_param(m,'InitFcn','Assem2_DataFile');
bx = 40; by = 300;
add_block('simulink/Sources/Constant',[m '/CmdSpeed'],'Value','0','Position',[bx by-30 bx+50 by-10]);   % m/s, +forward
add_block('simulink/Sources/Constant',[m '/CmdYaw'],  'Value','0','Position',[bx by+20 bx+50 by+40]);   % rad/s, +left
add_block('simulink/Signal Routing/Mux',[m '/CmdMux'],'Inputs','2','Position',[bx+70 by-30 bx+75 by+40]);
add_line(m,'CmdSpeed/1','CmdMux/1'); add_line(m,'CmdYaw/1','CmdMux/2');
dmx = add_block('simulink/Signal Routing/Demux',[m '/Demux'],'Outputs','2','Position',[bx+140 by bx+145 by+40]);
add_line(m,'CmdMux/1','Demux/1');

% planar pose from  v, omega
add_block('simulink/Continuous/Integrator',[m '/Yaw'],'Position',[bx+230 by+25 bx+260 by+55]);
add_line(m,'Demux/2','Yaw/1');
add_block('simulink/Math Operations/Trigonometric Function',[m '/cosYaw'],'Operator','cos','Position',[bx+290 by-40 bx+330 by-10]);
add_block('simulink/Math Operations/Trigonometric Function',[m '/sinYaw'],'Operator','sin','Position',[bx+290 by+80 bx+330 by+110]);
add_line(m,'Yaw/1','cosYaw/1'); add_line(m,'Yaw/1','sinYaw/1');
add_block('simulink/Math Operations/Product',[m '/vx'],'Position',[bx+360 by-40 bx+390 by-10]);
add_block('simulink/Math Operations/Product',[m '/vz'],'Position',[bx+360 by+80 bx+390 by+110]);
add_line(m,'Demux/1','vx/1'); add_line(m,'cosYaw/1','vx/2');
add_line(m,'Demux/1','vz/1'); add_line(m,'sinYaw/1','vz/2');
add_block('simulink/Continuous/Integrator',[m '/X'],'Position',[bx+420 by-40 bx+450 by-10]);
add_block('simulink/Continuous/Integrator',[m '/Z'],'Position',[bx+420 by+80 bx+450 by+110]);
add_line(m,'vx/1','X/1'); add_line(m,'vz/1','Z/1');

% pose scope + workspace log  [x z yaw]
add_block('simulink/Signal Routing/Mux',[m '/PoseMux'],'Inputs','3','Position',[bx+480 by+120 bx+485 by+190]);
add_block('simulink/Sinks/Scope',[m '/PoseScope'],'Position',[bx+520 by+120 bx+560 by+160]);
add_block('simulink/Sinks/To Workspace',[m '/PoseLog'],'VariableName','pose','SaveFormat','Timeseries','Position',[bx+520 by+180 bx+600 by+210]);
add_line(m,'X/1','PoseMux/1'); add_line(m,'Z/1','PoseMux/2'); add_line(m,'Yaw/1','PoseMux/3');
add_line(m,'PoseMux/1','PoseScope/1'); add_line(m,'PoseMux/1','PoseLog/1');

% Simulink -> physical converters (position input, derivatives filtered automatically)
cx = bx+500;
sx = psConv(m,'X_ps', [cx by-40], 'm');
sz = psConv(m,'Z_ps', [cx by+80], 'm');
sy = psConv(m,'Yaw_ps',[cx by+200],'rad');
add_line(m,'X/1','X_ps/1'); add_line(m,'Z/1','Z_ps/1'); add_line(m,'Yaw/1','Yaw_ps/1');
connectInput(m, pj, 2, sx); connectInput(m, pj, 3, sz); connectInput(m, pj, 4, sy);   % LConn2..4 = Px, Py, Rz

% differential drive -> wheel joint angles
%   vL = v - omega*track/2 ; vR = v + omega*track/2   (+Z side = left, yaw about -Y)
%   q_dot_i = -vFwd_i / (r * wheelSign_i)   (forward = +X, wheel axis = World Z)
add_block('simulink/Math Operations/Gain',[m '/HalfTrack'],'Gain',num2str(track/2),'Position',[bx+230 by+150 bx+270 by+180]);
add_line(m,'Demux/2','HalfTrack/1');
add_block('simulink/Math Operations/Sum',[m '/vL'],'Inputs','+-','Position',[bx+300 by+150 bx+330 by+180]);
add_block('simulink/Math Operations/Sum',[m '/vR'],'Inputs','++','Position',[bx+300 by+200 bx+330 by+230]);
add_line(m,'Demux/1','vL/1'); add_line(m,'HalfTrack/1','vL/2');
add_line(m,'Demux/1','vR/1'); add_line(m,'HalfTrack/1','vR/2');
for i = 1:4
    if ismember(i,p.leftWheels), src_ = 'vL'; else, src_ = 'vR'; end
    g = sprintf('wGain%d',i); ig = sprintf('wInt%d',i); cv = sprintf('wPS%d',i);
    yy = by + 330 + (i-1)*70;
    add_block('simulink/Math Operations/Gain',[m '/' g],'Gain',num2str(-1/(p.wheelRadius*p.wheelSign(i)),17),'Position',[bx+360 yy bx+400 yy+30]);
    add_block('simulink/Continuous/Integrator',[m '/' ig],'Position',[bx+430 yy bx+460 yy+30]);
    add_line(m,[src_ '/1'],[g '/1']); add_line(m,[g '/1'],[ig '/1']);
    s = psConv(m,cv,[cx yy],'rad');
    add_line(m,[ig '/1'],[cv '/1']);
    connectInput(m, [m '/' wheelJ{i}], 2, s);
end

%% ---------------- stand-in shapes if the SolidWorks STEP files are missing ----------------
% Assem2 draws its parts from Part1_Default_sldprt.STEP / Part2_Default_sldprt.STEP. Without them
% Mechanics Explorer is empty. Put the STEP files next to the model and this block does nothing.
if isempty(which('Part2_Default_sldprt.STEP')) || isempty(which('Part1_Default_sldprt.STEP'))
    warning('STEP files not found - using box/cylinder stand-ins (sizes derived from mass & inertia).');
    stand = {'Part1_1_RIGID','Brick Solid',      {'BrickDimensions','[0.81 0.26 0.79]'}
             'Part2_1_RIGID','Cylindrical Solid',{'CylinderRadius',num2str(p.wheelRadius),'CylinderLength','0.275'}
             'Part2_2_RIGID','Cylindrical Solid',{'CylinderRadius',num2str(p.wheelRadius),'CylinderLength','0.275'}
             'Part2_3_RIGID','Cylindrical Solid',{'CylinderRadius',num2str(p.wheelRadius),'CylinderLength','0.275'}
             'Part2_4_RIGID','Cylindrical Solid',{'CylinderRadius',num2str(p.wheelRadius),'CylinderLength','0.275'}};
    keep = {'Mass','MassUnits','CenterOfMass','CenterOfMassUnits','MomentsOfInertia','MomentsOfInertiaUnits', ...
            'ProductsOfInertia','ProductsOfInertiaUnits','GraphicDiffuseColor','GraphicOpacity'};
    for k = 1:size(stand,1)
        old = [m '/' stand{k,1} '/Solid'];
        vals = cellfun(@(f) get_param(old,f), keep, 'UniformOutput', false);
        pos_ = get_param(old,'Position');
        delete_block(old);
        nb = add_block(['sm_lib/Body Elements/' stand{k,2}],old,'Position',pos_);   % re-attaches to the old block's lines
        set_param(nb,'InertiaType','Custom','GraphicVisPropType','SimpleVisualProperties');
        for q = 1:numel(keep), set_param(nb,keep{q},vals{q}); end
        set_param(nb,stand{k,3}{:});
    end
end

%% ---------------- solver / display ----------------
sc = [m '/Solver Configuration'];
set_param(sc,'UseLocalSolver','on','LocalSolverChoice','NE_BACKWARD_EULER_ADVANCER','DoFixedCost','off');
set_param(m,'Solver','ode23t','MaxStep','0.01','EnablePacing','on','PacingRate','1');
save_system(m);
fprintf('Created %s.slx.  Run  sim(''%s'')  then open Mechanics Explorer (it opens on run) to watch the robot.\n',dst,dst);
end

%% ---------- helpers ----------
function h = getPort(blk,type,n)
if ischar(blk) || isstring(blk) || isnumeric(blk) && ~isscalar(blk), blk = get_param(blk,'Handle'); end
ph = get_param(blk,'PortHandles'); h = ph.(type)(n);
end

function connectPhys(m,a,b)
% a new block placed where the old one was re-attaches to the old block's line, so it may already be wired
la = get_param(a,'Line'); lb = get_param(b,'Line');
if la > 0 && la == lb, return; end
% delete dangling stubs left by the removed Transform blocks
for p = [a b]
    l = get_param(p,'Line');
    if l > 0 && (get_param(l,'SrcPortHandle') < 0 || any(get_param(l,'DstPortHandle') < 0))
        delete_line(l);     % only dangling stubs, never a live connection
    end
end
add_line(m,a,b,'autorouting','on');
end

function blk = psConv(m,name,pos,unit)
blk = add_block('nesl_utility/Simulink-PS Converter',[m '/' name],'Position',[pos(1) pos(2) pos(1)+60 pos(2)+30]);
set_param(blk,'Unit',unit);
try, set_param(blk,'FilteringAndDerivatives','filter','SimscapeFilterOrder','2','InputFilterTimeConstant','0.01'); catch, end
end

function connectInput(m,joint,n,conv)
% motion-input ports appear on the joint's left side once MotionActuationMode = InputMotion
ph = get_param(joint,'PortHandles'); cp = get_param(conv,'PortHandles');
add_line(m, cp.RConn(1), ph.LConn(n), 'autorouting','on');
end





