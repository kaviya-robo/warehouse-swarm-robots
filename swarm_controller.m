function out = swarm_controller(u)
% SWARM_CONTROLLER  Path following + obstacle avoidance for the swarm robots in Assem2_Drive/Swarm.
%   u = [t  x1 z1 yaw1  x2 z2 yaw2 ...]   pose 1 = your teleoperated robot, then the swarm robots
%   out = [v2 w2 v3 w3 ...]               speed (m/s) / yaw rate (rad/s) per swarm robot
%
% Path following  : pure pursuit - each robot aims at a point W.swarmLookahead metres ahead of its
%                   closest point on its closed path (robot_world.m).
% Lane shift      : if the path ahead is blocked (dropped box, parked robot ...) the pursuit point is
%                   slid sideways to the smallest offset whose corridor is free; the robot drives round
%                   the blockage and rejoins its path. No free corridor -> it waits 0.5 m before it.
% Obstacle avoid. : dynamic window approach - ~70 (v, w) candidates are simulated W.swarmHorizon
%                   seconds ahead, unsafe ones are rejected, the rest are scored on reaching the
%                   pursuit point, staying on the (shifted) path, clearance and speed.
% Priorities      : robots plan in order - you first, then Robot2, Robot3, ... Each robot treats the
%                   2 s plans of higher-priority robots as moving obstacles and gets out of their way;
%                   lower-priority robots only count where they are now. A robot stuck facing a
%                   higher-priority one reverses to make room.
% The chosen plans are left in getappdata(groot,'swarm_plan') for the drive_robot map.
persistent W S
t = u(1); P = reshape(u(2:end),3,[])'; R = size(P,1);
if isempty(W) || t < S.t                          % first call or a new simulation run
    W = robot_world; S = init(W, P, t);
end
N = numel(W.swarm); K = S.K; h = W.swarmHorizon/K;
dt = t - S.t;
if dt > 0                                         % measured velocity of every robot
    S.vel = 0.5*S.vel + 0.5*(P(:,1:2) - S.prevP)/dt; S.prevP = P(:,1:2); S.t = t;
end
% predicted poses over the horizon (K x 3 per robot): you = constant velocity, the swarm = current
% pose until its plan for this cycle is made
pred = cell(1,R);
for j = 1:R, pred{j} = repmat(P(j,:), K, 1); end
v1 = S.vel(1,:); if norm(v1) > 0.8, v1 = 0.8*v1/norm(v1); end
pred{1}(:,1:2) = P(1,1:2) + (1:K)'*h*v1;
out = zeros(2*N,1);

for k = 1:N
    i = k + 1; p = P(i,:); route = W.swarm(k).route;
    hi = 1:i-1; lo = i+1:R;                       % higher / lower priority robots
    [S.seg(k), pp] = pursuit(route, S.seg(k), p(1:2), W.swarmLookahead);
    [tgt, S.off(k), waiting] = shiftTarget(S, W, p, pp, [P(hi,:); vertcat(pred{hi})], S.off(k));

    j = S.yieldTo(k);                             % giving way: stop once that robot is moving / gone
    if S.back(k) > 0 && j > 0 && (norm(S.vel(j,:)) > 0.1 || norm(P(j,1:2) - p(1:2)) > 4*norm(S.rh))
        S.back(k) = 0;
    end
    backing = S.back(k) > 0;
    if ~backing, S.yieldTo(k) = 0; end
    if backing                                    % reversing out of a deadlock
        S.back(k) = S.back(k) - dt;
        V = -0.2*[1;1;1]; Wv = [S.turn(k); 0; -S.turn(k)]*0.4;
    else
        [V, Wv] = meshgrid([-0.15 -0.05 linspace(0, W.swarmSpeed, 5)], linspace(-W.swarmYawRate, W.swarmYawRate, 11));
        V = V(:); Wv = Wv(:);
    end
    [X, Y, TH, cImm, cPred, c0, cInt, cS0] = rollout(S, W, P, i, V, Wv, pred(hi), lo);
    ok = cImm >= min(W.margin, c0 - 1e-3);       % safe now (never closer than allowed, or than now)

    intent = [];
    if ~any(ok)
        best = [];
    elseif backing
        best = find(ok, 1);
    else
        pe = [X(:,end) Y(:,end)]; dir0 = pp(min(2,end),:) - pp(1,:);
        dTgt = vecnorm(pe - tgt, 2, 2);
        if waiting || norm(tgt - p(1:2)) < 0.3    % holding position: keep facing along the path
            hdg = abs(angle(exp(1i*(atan2(dir0(2), dir0(1)) - TH(:,end)))));
        else
            hdg = abs(angle(exp(1i*(atan2(tgt(2)-pe(:,2), tgt(1)-pe(:,1)) - TH(:,end)))));
        end
        xte  = pathDist(route, S.seg(k), pe, S.off(k));
        base = 1.0*dTgt + 0.5*hdg + 0.8*xte - 0.3*V/W.swarmSpeed + 0.8*(V < 0);
        score = @(c) base - 0.6*min(c, 0.8) + 4*(c < W.margin) - 4*min(c, 0).*(c < W.margin);
        cost = score(cPred); cost(~ok) = inf;
        [~, best] = min(cost);
        % intent: what I would do if lower-priority robots weren't there (they read it and make room)
        costI = score(cInt); costI(cInt < min(W.margin, cS0 - 1e-3)) = inf;
        [mI, intent] = min(costI); if ~isfinite(mI), intent = []; end
    end
    if isempty(best), v = 0; w = 0; else, v = V(best); w = Wv(best); end

    if norm(p(1:2) - S.anchor(k,:)) > 0.15 || backing  % made progress: reset the stuck clock
        if ~backing, S.waits(k) = 0; end
        S.anchor(k,:) = p(1:2); S.anchorT(k) = t; S.lastMove(k) = t;
    end
    if ~backing && t - S.anchorT(k) > 2.5 + rand + 2*(abs(w) > 1e-6 && ~waiting)
        S.anchorT(k) = t;                         % stuck: < 15 cm progress for ~3 s (jitter included)
        near = find(vecnorm(P(:,1:2) - p(1:2), 2, 2) < 4*norm(S.rh) - 0.9)';
        near(near == i) = [];
        hiStuck = near(near > 1 & near < i);      % higher-priority swarm robots that are stuck too
        hiStuck = hiStuck(t - S.lastMove(hiStuck - 1) > 1.5);
        if ~isempty(hiStuck)                      % they need room: reverse until they move
            S.back(k) = 6; S.turn(k) = 0; S.yieldTo(k) = min(hiStuck);
        elseif waiting && S.waits(k) < 2          % waiting for a blockage to clear
            S.waits(k) = S.waits(k) + 1;
        elseif isempty(near) || S.waits(k) >= 2   % still stuck: random back-off, then try again
            S.back(k) = 0.8 + 0.6*rand; S.turn(k) = sign(rand - 0.5); S.waits(k) = 0;
        else                                      % blocked by lower-priority robots: they make room
            S.waits(k) = S.waits(k) + 1;
        end
    end
    out(2*k-1:2*k) = [v; w];
    if ~isempty(best), S.plan{k} = [p(1:2); X(best,:)' Y(best,:)']; else, S.plan{k} = p(1:2); end
    if isempty(intent), intent = best; end        % my intended plan, for the robots after me
    if ~isempty(intent), pred{i} = [X(intent,:)' Y(intent,:)' TH(intent,:)']; end
end
try, setappdata(groot,'swarm_plan',S.plan); catch, end
end

%% ---------------- setup ----------------
function S = init(W, P, t)
N = numel(W.swarm); fp = W.footprint;
S.K = 8; S.t = t; S.prevP = P(:,1:2); S.vel = zeros(size(P,1),2);
S.back = zeros(1,N); S.turn = ones(1,N); S.waits = zeros(1,N);
S.anchor = P(2:end,1:2); S.anchorT = t*ones(1,N); S.lastMove = t*ones(1,N);
S.off = zeros(1,N); S.plan = cell(1,N); S.yieldTo = zeros(1,N);
% robot outline sampled every ~12 cm (body frame)
nx = ceil(diff(fp(1:2))/0.12); ny = ceil(diff(fp(3:4))/0.12);
xs = linspace(fp(1),fp(2),nx+1); ys = linspace(fp(3),fp(4),ny+1);
S.pts = unique([xs' fp(3)+0*xs'; xs' fp(4)+0*xs'; fp(1)+0*ys' ys'; fp(2)+0*ys' ys'],'rows');
S.rc = [mean(fp(1:2)) mean(fp(3:4))]; S.rh = [diff(fp(1:2)) diff(fp(3:4))]/2;   % robot as a box
b = W.allBoxes; S.boxes = [b(:,1:2) b(:,3:4)/2 b(:,6)*pi/180];                 % [cx cy hx hy yaw]
S.seg = zeros(1,N);
for k = 1:N                                       % start on the closest path segment going our way
    r = W.swarm(k).route; n = size(r,1); c = inf(1,n); hd = [cos(P(k+1,3)) sin(P(k+1,3))];
    for s = 1:n
        e = r(mod(s,n)+1,:) - r(s,:);
        c(s) = segDist(r, s, P(k+1,1:2)) + 2*(dot(e,hd) < 0);
    end
    [~, S.seg(k)] = min(c);
end
end

function B = robotBoxes(S, poses)
% robots (rows [x y yaw]) as oriented boxes [cx cy hx hy yaw]
a = poses(:,3);
B = [poses(:,1) + S.rc(1)*cos(a) - S.rc(2)*sin(a), poses(:,2) + S.rc(1)*sin(a) + S.rc(2)*cos(a), ...
     repmat(S.rh, size(poses,1), 1), a];
end

%% ---------------- dynamic window: simulate candidates, measure clearance ----------------
function [X, Y, TH, cImm, cPred, c0, cInt, cS0] = rollout(S, W, P, i, V, Wv, predHi, lo)
% cImm  : clearance to static obstacles over the horizon and to every robot's CURRENT spot over the
%         first 0.5 s (hard safety)
% cPred : clearance to static obstacles + higher-priority robots' plans + lower ones' current spots
K = S.K; h = W.swarmHorizon/K; nc = numel(V); p = P(i,:);
X = zeros(nc,K); Y = X; TH = X;
x = p(1)*ones(nc,1); y = p(2)*ones(nc,1); th = p(3)*ones(nc,1);
for k = 1:K
    thm = th + Wv*h/2; x = x + V.*cos(thm)*h; y = y + V.*sin(thm)*h; th = th + Wv*h;
    X(:,k) = x; Y(:,k) = y; TH(:,k) = th;
end
px = S.pts(:,1)'; py = S.pts(:,2)';
c = cos(TH(:)); s = sin(TH(:));
QX = X(:) + c*px - s*py; QY = Y(:) + s*px + c*py;            % (nc*K) x points
reach = W.swarmSpeed*W.swarmHorizon + norm(S.rh) + 0.5;
B = S.boxes(vecnorm(S.boxes(:,1:2) - p(1:2),2,2) < reach + vecnorm(S.boxes(:,3:4),2,2), :);
f = W.floor;
Ds = min(min(min(QX - f(1), f(2) - QX), min(QY - f(3), f(4) - QY)), boxDist(QX, QY, B));
Ds = reshape(min(Ds,[],2), nc, K);                           % static clearance per candidate/step
others = setdiff(1:size(P,1), i);
near = others(vecnorm(P(others,1:2) - p(1:2), 2, 2) < reach + 2*norm(S.rh));
Bnow = robotBoxes(S, P(near,:));
Dnow = inf(nc, K); Dh = inf(nc, K); Dl = inf(nc, K);
Blo = robotBoxes(S, P(intersect(lo, near),:));               % lower priority: where they are
for k = 1:K
    rows = (k-1)*nc + (1:nc);
    if k <= 2 && ~isempty(Bnow), Dnow(:,k) = min(boxDist(QX(rows,:), QY(rows,:), Bnow), [], 2); end
    Bk = zeros(0,5);
    for j = 1:numel(predHi)                                  % higher priority: where they want to be
        if norm(predHi{j}(k,1:2) - p(1:2)) < reach + 2*norm(S.rh), Bk(end+1,:) = robotBoxes(S, predHi{j}(k,:)); end %#ok<AGROW>
    end
    if ~isempty(Bk),  Dh(:,k) = min(boxDist(QX(rows,:), QY(rows,:), Bk), [], 2); end
    if ~isempty(Blo), Dl(:,k) = min(boxDist(QX(rows,:), QY(rows,:), Blo), [], 2); end
end
cImm  = min(min(Ds, Dnow), [], 2);
cPred = min(min(min(Ds, Dh), Dl), [], 2);
cInt  = min(min(Ds, Dh), [], 2);                             % ignoring lower-priority robots
% clearance right now (to allow moving away when already close)
c0_ = cos(p(3)); s0 = sin(p(3));
qx = p(1) + c0_*px - s0*py; qy = p(2) + s0*px + c0_*py;
dS = min(min(min(qx - f(1), f(2) - qx), min(qy - f(3), f(4) - qy)), boxDist(qx, qy, B));
cS0 = min(dS);                                               % static clearance now
c0 = min([dS boxDist(qx, qy, Bnow)]);
end

function d = boxDist(qx, qy, B)
% signed distance from points to oriented boxes B = [cx cy hx hy yaw] (negative = inside)
d = inf(size(qx));
for b = 1:size(B,1)
    c = cos(B(b,5)); s = sin(B(b,5));
    dx = qx - B(b,1); dy = qy - B(b,2);
    lx = abs(c*dx + s*dy) - B(b,3); ly = abs(-s*dx + c*dy) - B(b,4);
    d = min(d, sqrt(max(lx,0).^2 + max(ly,0).^2) + min(max(lx,ly),0));
end
end

%% ---------------- lane shift: keep the pursuit point reachable ----------------
function [tgt, off, waiting] = shiftTarget(S, W, p, pp, Q, offPrev)
% Try the path ahead (pp: closest path point -> pursuit point) shifted sideways and take the smallest
% shift whose corridor is free of obstacles and higher-priority robots (Q: their poses now and over
% their 2 s plans). The corridor blends from the robot's current sideways offset into the shift over
% the first 0.5 m and follows path corners. Nothing free -> follow the detour that gets furthest
% and wait 0.5 m before its blockage.
offs = reshape([1; -1]*(0.125:0.125:1.5), 1, []); offs = [0 offs];   % 0, +-12.5 cm ... +-1.5 m
[~, order] = sort(abs(offs) + 0.25*abs(offs - offPrev));
B = S.boxes(vecnorm(S.boxes(:,1:2) - p(1:2),2,2) < W.swarmLookahead + 3, :);
Q = Q(vecnorm(Q(:,1:2) - p(1:2), 2, 2) < W.swarmLookahead + 2, :);
B = [B; robotBoxes(S, Q)];
% sample the path ahead every 0.15 m: centre, direction, distance travelled
C = zeros(0,2); D = zeros(0,2); dist = zeros(0,1); run = 0;
for q = 1:size(pp,1)-1
    e = pp(q+1,:) - pp(q,:); len = norm(e);
    if len < 1e-6, continue; end
    st = (0:0.15:len)';
    C = [C; pp(q,:) + st*e/len]; D = [D; repmat(e/len, numel(st), 1)]; dist = [dist; run + st]; %#ok<AGROW>
    run = run + len;
end
waiting = false;
if isempty(C), tgt = pp(end,:); off = 0; return; end
N0 = [-D(:,2) D(:,1)];                             % left normal of the path at each sample
e0 = dot(p(1:2) - pp(1,:), N0(1,:));               % robot's current sideways offset
f = W.floor; off = 0; tgt = p(1:2); bestReach = -inf; blend = 0.5;
% required clearance: W.margin + 4 cm, relaxed near the robot if it is already closer than that
c0 = cos(p(3)); s0 = sin(p(3));
qx = p(1) + c0*S.pts(:,1)' - s0*S.pts(:,2)'; qy = p(2) + s0*S.pts(:,1)' + c0*S.pts(:,2)';
cNow = min(min(min(min(qx - f(1), f(2) - qx), min(qy - f(3), f(4) - qy)), boxDist(qx, qy, B)));
thr = W.margin + 0.04; thr0 = min(W.margin, cNow - 0.01);   % DWA keeps W.margin itself
thrS = thr0 + (thr - thr0)*min(max(dist - blend, 0)/0.7, 1); % per path sample
for o = offs(order)
    ok_ = o + (e0 - o)*max(1 - dist/blend, 0);     % blend current offset -> o (it can turn on the spot)
    cx = C(:,1) + ok_.*N0(:,1); cy = C(:,2) + ok_.*N0(:,2);
    keep = hypot(cx - p(1), cy - p(2)) >= 0.3;     % the robot's own spot doesn't count
    c = D(keep,1); s = D(keep,2); cx = cx(keep); cy = cy(keep); dk = dist(keep);
    qx = cx + c*S.pts(:,1)' - s*S.pts(:,2)'; qy = cy + s*S.pts(:,1)' + c*S.pts(:,2)';
    dd = min(min(min(qx - f(1), f(2) - qx), min(qy - f(3), f(4) - qy)), boxDist(qx, qy, B));
    blk = find(min(dd,[],2) <= thrS(keep), 1);
    if isempty(blk)                                % free: aim closer when the shift is large
        off = o; Lt = min(max(W.swarmLookahead - abs(e0 - o), 1.0), W.swarmLookahead);
        q = find(dist >= Lt, 1); if isempty(q), q = numel(dist); end
        tgt = C(q,:) + o*N0(q,:); return;
    end
    if dk(blk) > bestReach + 0.05                  % remember the detour that gets furthest
        bestReach = dk(blk); off = o;
        stop = find(dk <= dk(blk) - 0.5, 1, 'last');
        if isempty(stop), tgt = p(1:2); else, tgt = [cx(stop) cy(stop)]; end
    end
end
waiting = true;      % no free detour: follow the best one and wait 0.5 m before the blockage
end

%% ---------------- pure pursuit on a closed polyline ----------------
function [seg, pp] = pursuit(route, seg, pos, L)
% pp = path ahead as a polyline: closest point on the path -> corners -> pursuit point (L metres on)
n = size(route,1);
for it = 1:n                                      % move on to the next segment when it's closer
    nx = mod(seg,n) + 1;
    [d1, t1] = segDist(route, seg, pos);
    if t1 >= 1 || segDist(route, nx, pos) < d1, seg = nx; else, break; end
end
[~, t1] = segDist(route, seg, pos);
a = route(seg,:); b = route(mod(seg,n)+1,:);
q = a + min(max(t1,0),1)*(b - a); left = L; s = seg; pp = q;
for it = 1:2*n                                    % walk L metres along the path
    b = route(mod(s,n)+1,:); d = norm(b - q);
    if d >= left, break; end
    left = left - d; q = b; s = mod(s,n) + 1; pp(end+1,:) = q; %#ok<AGROW>
end
pp(end+1,:) = q + left*(b - q)/max(d,1e-9);
end

function [d, t] = segDist(route, s, pos)
% distance from pos to path segment s, and the (unclamped) position t along it
n = size(route,1); a = route(s,:); e = route(mod(s,n)+1,:) - a;
t = dot(pos - a, e)/dot(e,e);
d = norm(pos - (a + min(max(t,0),1)*e));
end

function d = pathDist(route, s, pts, off)
% distance from candidate end points to the current and next path segment (shifted sideways by off)
n = size(route,1); d = inf(size(pts,1),1);
for q = [s mod(s,n)+1]
    a = route(q,:); e = route(mod(q,n)+1,:) - a;
    a = a + off*[-e(2) e(1)]/norm(e);
    t = min(max(((pts - a)*e')/dot(e,e), 0), 1);
    d = min(d, vecnorm(pts - (a + t*e), 2, 2));
end
end
