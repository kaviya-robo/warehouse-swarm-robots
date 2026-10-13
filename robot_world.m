function W = robot_world
% ROBOT_WORLD  Warehouse floor, shelves and swarm robots for Assem2_Drive.
%              Edit here, then run drive_robot again.
%
% Coordinates are the robot's floor plane in metres: your (teleoperated) robot starts at (0,0)
% facing +x (same x / z values as the X, Z integrators and PoseScope in Assem2_Drive).

%% ---------------- floor ----------------
W.floor      = [-9 9 -7 7];          % [xmin xmax ymin ymax]  m
W.floorThick = 0.05;                 % m
W.floorColor = [0.70 0.72 0.75];

%% ---------------- obstacles: shelf racks + pallets ----------------
% one row per box:  [x  y  length(x)  width(y)  height  yaw(deg)]
shelf = [5 1 1.8];                   % rack length / depth / height
W.boxes = [];
for y = [-5 -2 2 5]                  % 4 rows of racks, aisles at y = 0 and y = +-3.5
    for x = [-4.5 4.5]               % cross aisle at x = 0, end aisles at x = +-8
        W.boxes(end+1,:) = [x y shelf 0];
    end
end
W.boxes = [W.boxes
           0    5.6  1.0 1.0 0.6  0   % pallets
           0   -5.6  1.0 1.0 0.6 20
          -8    5.6  1.0 1.0 0.6  0
           4.5  1.15 0.5 0.5 0.4  0   % dropped boxes in the main aisle, on swarm paths
          -4.5 -1.15 0.5 0.5 0.4  0 ];
W.boxColor   = [0.85 0.40 0.10];
W.wallHeight = 0.5;                  % perimeter wall around the floor (0 = no wall)
W.wallThick  = 0.15;
W.wallColor  = [0.35 0.35 0.40];
W.margin     = 0.05;                 % m clearance kept from obstacles / other robots

%% ---------------- swarm robots (copies of your robot, driven by swarm_controller) ----------------
% one entry per extra robot: start pose [x y yaw(deg)] and a closed path (corner points [x y]) it
% follows, in driving order. Delete / add entries to change the number of robots.
% Traffic is one-way like in a real warehouse: both lanes of the main aisle (y = +-0.75) run towards
% +x, and no cross / end aisle carries traffic in opposite directions - so robots never meet head-on.
W.swarm = struct('start',{},'route',{},'color',{});
W.swarm(end+1) = struct('start',[ 1.5  0.75   0], 'route',[ 8  0.75;  8  3.5;  1.2  3.5;  1.2  0.75], 'color',[0.10 0.60 0.20]);
W.swarm(end+1) = struct('start',[-4.5  0.75   0], 'route',[-1.2 0.75; -1.2 3.5; -8  3.5; -8  0.75],   'color',[0.95 0.75 0.10]);
W.swarm(end+1) = struct('start',[ 4.5 -0.75   0], 'route',[ 8 -0.75;  8 -3.5;  1.2 -3.5;  1.2 -0.75], 'color',[0.10 0.40 0.90]);
W.swarm(end+1) = struct('start',[-6.5 -0.75   0], 'route',[-1.2 -0.75; -1.2 -3.5; -8 -3.5; -8 -0.75], 'color',[0.80 0.10 0.60]);
W.swarmSpeed     = 0.4;              % m/s   max forward speed
W.swarmYawRate   = 0.8;              % rad/s max turn rate
W.swarmPeriod    = 0.1;              % s     controller update period
W.swarmLookahead = 2.0;             % m     pure-pursuit look-ahead distance along the path
W.swarmHorizon   = 2.0;              % s     how far ahead each candidate motion is simulated

%% ---------------- robot geometry (from the SolidWorks data) ----------------
wheelRadius = 0.1877;                % m  - keep equal to p.wheelRadius in build_robot_motion
wheelWidth  = 0.275;                 % m
chassis     = [0.81 0.79];           % m  chassis length (x) / width (y)
here = fileparts(mfilename('fullpath'));
run(fullfile(here,'Assem2_DataFile.m'));          % creates smiData
RT = smiData.RigidTransform;
C  = RT(9).translation/1000;                       % chassis centre (SolidWorks frame)
wc = vertcat(RT([2 3 6 8]).translation)/1000;      % wheel centres
d  = wc - C;                                       % body frame: x = SW x, y = SW z
W.footprint = [min([d(:,1)-wheelRadius; -chassis(1)/2]) max([d(:,1)+wheelRadius; chassis(1)/2]) ...
               min([d(:,3)-wheelWidth/2; -chassis(2)/2]) max([d(:,3)+wheelWidth/2; chassis(2)/2])];
W.floorDepth = mean(C(2) - wc(:,2)) + wheelRadius; % floor surface below the chassis centre

%% perimeter walls are just more boxes
if W.wallHeight > 0
    f = W.floor; t = W.wallThick; Lx = f(2)-f(1); Ly = f(4)-f(3);
    W.walls = [ mean(f(1:2))  f(4)-t/2      Lx  t   W.wallHeight 0
                mean(f(1:2))  f(3)+t/2      Lx  t   W.wallHeight 0
                f(2)-t/2      mean(f(3:4))  t   Ly  W.wallHeight 0
                f(1)+t/2      mean(f(3:4))  t   Ly  W.wallHeight 0 ];
else
    W.walls = zeros(0,6);
end
W.allBoxes = [W.boxes; W.walls];
end
