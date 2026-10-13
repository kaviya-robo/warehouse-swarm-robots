function cmd = robot_drive_cmd
% ROBOT_DRIVE_CMD  Drive schedule for Assem2_Drive:  rows are  [time(s)  forward speed(m/s)  yaw rate(rad/s)]
% Each row holds until the next one. Positive yaw rate = turn left (towards the +Z side of the robot).
v = 0.5;      % m/s
w = 0.6;      % rad/s
seq = [  0   0    0       % rest
         1   v    0       % forward
         5   0    0
         6  -v    0       % backward
        10   0    0
        11   0    w       % spin left on the spot
        16   0   -w       % spin right on the spot
        21   0    0
        22   v    0.6*w   % forward arc, left
        26   v   -0.6*w   % forward arc, right
        30   0    0 ];
cmd = timeseries(seq(:,2:3), seq(:,1));
end
