function drive_robot
% DRIVE_ROBOT  Drive Assem2_Drive live from the keyboard (or the buttons).
%
%   W / Up     forward          S / Down   backward
%   A / Left   rotate left      D / Right  rotate right
%   Space      stop             Q / Esc    quit
%
% Hold a key to move; release to stop. Keys can be combined (W + A = forward arc left).
% Click the figure first so it has keyboard focus.
% Floor + obstacles come from robot_world.m; the robot stops by itself before it hits one.

m      = 'Assem2_Drive';
speed  = 0.5;     % m/s
yawRate = 0.8;    % rad/s

cd(fileparts(mfilename('fullpath')));
if ~exist([m '.slx'],'file'), build_robot_motion; end
load_system(m);
add_world(m);                      % floor, obstacles, collision gate (picks up edits to robot_world.m)
add_swarm(m);                      % the other warehouse robots
clear robot_collision_gate swarm_controller   % forget the cached world
W = robot_world; N = numel(W.swarm);
set_param(m,'SimMechanicsOpenEditorOnUpdate','on','EnablePacing','on','PacingRate','1','StopTime','inf');
set_param([m '/CmdSpeed'],'Value','0'); set_param([m '/CmdYaw'],'Value','0');

fig = figure('Name','Robot control','NumberTitle','off','MenuBar','none','ToolBar','none', ...
    'Position',[100 100 900 460],'KeyPressFcn',@(~,e) keyChange(e.Key,true), ...
    'KeyReleaseFcn',@(~,e) keyChange(e.Key,false),'CloseRequestFcn',@(~,~) quitApp());
pressed = {};   % held keys
latch = [0 0];  % button command [fwd yaw]
cmd = [0 0];    % last command sent [v w]
uicontrol(fig,'Style','text','Position',[10 420 340 25],'String','Click here, then hold W/A/S/D (Space = stop, Q = quit)');
mk = @(txt,pos,c) uicontrol(fig,'Style','pushbutton','String',txt,'Position',pos,'Callback',@(~,~) setLatch(c));
mk('Forward',  [135 260 90 40],[ 1  0]);
mk('Rotate L', [35  210 90 40],[ 0  1]);
mk('STOP',     [135 210 90 40],[ 0  0]);
mk('Rotate R', [235 210 90 40],[ 0 -1]);
mk('Backward', [135 160 90 40],[-1  0]);
status  = uicontrol(fig,'Style','text','Position',[10 100 340 25],'String','speed 0  yaw 0');
blocked = uicontrol(fig,'Style','text','Position',[10 60 340 30],'String','','FontWeight','bold','FontSize',12,'ForegroundColor',[0.8 0 0]);

% top-down map (same view as looking down on Mechanics Explorer, World Z up)
ax = axes(fig,'Units','pixels','Position',[400 40 480 400],'YDir','reverse','HitTest','off');
hold(ax,'on'); axis(ax,'equal'); box(ax,'on'); xlabel(ax,'x (m)'); ylabel(ax,'z (m)');
title(ax,sprintf('Top view - you: black/red, %d swarm robots (dotted = path, solid = 2 s plan)',N));
f = W.floor;
patch(ax,f([1 2 2 1]),f([3 3 4 4]),W.floorColor,'EdgeColor','none','HitTest','off');
for k = 1:size(W.allBoxes,1)
    c = boxXY(W.allBoxes(k,:));
    clr = W.boxColor; if k > size(W.boxes,1), clr = W.wallColor; end
    patch(ax,c(1,:),c(2,:),clr,'EdgeColor','k','HitTest','off');
end
fp = W.footprint;
swP = gobjects(1,N); swH = gobjects(1,N); swL = gobjects(1,N);
try, setappdata(groot,'swarm_plan',{}); catch, end
for k = 1:N                        % swarm robots: path (dotted), current 2 s plan (solid), body
    rt = W.swarm(k).route([1:end 1],:);
    plot(ax,rt(:,1),rt(:,2),':','Color',W.swarm(k).color,'HitTest','off');
    swL(k) = plot(ax,nan,nan,'-','Color',W.swarm(k).color,'LineWidth',2.5,'HitTest','off');
    swP(k) = patch(ax,fp([1 2 2 1]),fp([3 3 4 4]),W.swarm(k).color,'FaceAlpha',0.8,'HitTest','off');
    swH(k) = plot(ax,[0 0.6],[0 0],'k-','LineWidth',1,'HitTest','off');
    drawRobot(swP(k),swH(k),[W.swarm(k).start(1:2) W.swarm(k).start(3)*pi/180]);
end
robotP = patch(ax,fp([1 2 2 1]),fp([3 3 4 4]),[0.15 0.15 0.15],'FaceAlpha',0.9,'EdgeColor','r','LineWidth',2,'HitTest','off');
headL  = plot(ax,[0 0.6],[0 0],'r-','LineWidth',2,'HitTest','off');
axis(ax,f + [-0.2 0.2 -0.2 0.2]);
mapTimer = timer('ExecutionMode','fixedSpacing','Period',0.1,'TimerFcn',@(~,~) updateMap());

set_param(m,'SimulationCommand','start');
start(mapTimer);

    function setLatch(c), latch = c; pressed = {}; apply(); end
    function keyChange(k,down)
        if strcmp(k,'space'), pressed = {}; latch = [0 0]; apply(); return; end
        if any(strcmp(k,{'q','escape'})) && down, quitApp(); return; end
        if down, pressed = union(pressed,{k}); else, pressed = setdiff(pressed,{k}); end
        latch = [0 0]; apply();
    end
    function apply()
        fw = latch(1); w = latch(2);
        if any(ismember(pressed,{'w','uparrow'})),    fw = fw + 1; end
        if any(ismember(pressed,{'s','downarrow'})),  fw = fw - 1; end
        if any(ismember(pressed,{'a','leftarrow'})),  w = w + 1; end
        if any(ismember(pressed,{'d','rightarrow'})), w = w - 1; end
        fw = max(-1,min(1,fw)); w = max(-1,min(1,w));
        cmd = [fw*speed w*yawRate];
        try
            set_param([m '/CmdSpeed'],'Value',num2str(cmd(1)));
            set_param([m '/CmdYaw'],  'Value',num2str(cmd(2)));
        catch, end
        if isvalid(status), status.String = sprintf('speed %.2f m/s   yaw %.2f rad/s',cmd(1),cmd(2)); end
    end
    function updateMap()
        if ~isvalid(fig), return; end
        try
            x = blockOut('X'); z = blockOut('Z'); yaw = blockOut('Yaw'); g = blockOut('CollisionGate');
        catch
            return;                % simulation not running (yet)
        end
        drawRobot(robotP,headL,[x z yaw]);
        plans = {}; try, plans = getappdata(groot,'swarm_plan'); catch, end
        for j = 1:N
            r = sprintf('Swarm/Robot%d/',j+1);
            try, drawRobot(swP(j),swH(j),[blockOut([r 'X']) blockOut([r 'Z']) blockOut([r 'Yaw'])]); catch, end
            if numel(plans) >= j && ~isempty(plans{j}), set(swL(j),'XData',plans{j}(:,1),'YData',plans{j}(:,2)); end
        end
        if any(abs(cmd) > 0 & abs(g(:)') < 1e-9), blocked.String = 'BLOCKED - obstacle / robot ahead';
        else, blocked.String = ''; end
    end
    function drawRobot(body,head,p)
        R = [cos(p(3)) -sin(p(3)); sin(p(3)) cos(p(3))];
        P = R*[fp([1 2 2 1]); fp([3 3 4 4])] + p(1:2)';
        H = R*[0 0.6; 0 0] + p(1:2)';
        set(body,'XData',P(1,:),'YData',P(2,:)); set(head,'XData',H(1,:),'YData',H(2,:));
    end
    function y = blockOut(name)
        rto = get_param([m '/' name],'RuntimeObject');
        y = rto.OutputPort(1).Data;
    end
    function quitApp()
        try, stop(mapTimer); delete(mapTimer); catch, end
        try, set_param(m,'SimulationCommand','stop'); catch, end
        if isvalid(fig), delete(fig); end
    end
end

function c = boxXY(b)
% corners of an obstacle row [x y length width height yaw(deg)]
a = b(6)*pi/180;
c = [cos(a) -sin(a); sin(a) cos(a)]*[b(3)/2*[-1 1 1 -1]; b(4)/2*[-1 -1 1 1]] + b(1:2)';
end
