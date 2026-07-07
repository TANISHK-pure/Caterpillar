clear; clc; close all;
CFG.use_ros          = true;
CFG.solver_period    = 1.0;
CFG.safety_period    = 0.1;
CFG.corridor_buffer  = 12.0;
CFG.critical_zone    = 5.0;
CFG.service_base     = 25.0;
CFG.service_per_m    = 0.4;
CFG.entry_xy         = [0, 0];
CFG.bay_xy           = [-30, -10; -30, -25; -30, -40];
CFG.num_bays         = 3;
CFG.max_concurrent   = 2;
CFG.max_makespan     = 600;
CFG.max_queue        = 12;
ros_ok = false;
if CFG.use_ros
    try
        node = ros2node("/milp_traffic_cop");
        sub_plans  = ros2subscriber(node, "/dump_plans",        "std_msgs/Float64MultiArray");
        sub_pos    = ros2subscriber(node, "/truck_positions",   "geometry_msgs/PoseArray");
        sub_done   = ros2subscriber(node, "/truck_completed",   "std_msgs/Int32");
        sub_path   = ros2subscriber(node, "/path_status",       "std_msgs/Float64MultiArray");
        sub_perc   = ros2subscriber(node, "/perception_events", "std_msgs/String");
        pub_bay    = ros2publisher(node, "/bay_assignments",    "std_msgs/Float64MultiArray");
        pub_grant  = ros2publisher(node, "/entry_grants",       "std_msgs/Float64MultiArray");
        pub_lock   = ros2publisher(node, "/milp_lock",          "std_msgs/Int32");
        pub_stat   = ros2publisher(node, "/milp_status",        "std_msgs/String");
        msg_bay    = ros2message(pub_bay);
        msg_grant  = ros2message(pub_grant);
        msg_lock   = ros2message(pub_lock);
        msg_stat   = ros2message(pub_stat);
        ros_ok     = true;
        fprintf('ROS 2 link [ONLINE]\n');
        fprintf('  SUB /dump_plans /truck_positions /truck_completed\n');
        fprintf('  SUB /path_status /perception_events  (future hooks)\n');
        fprintf('  PUB /bay_assignments /entry_grants /milp_lock /milp_status\n');
    catch ME
        fprintf('ROS 2 unavailable: %s\n', ME.message);
    end
end
has_intlinprog = ~isempty(which('intlinprog'));
if ~has_intlinprog
    warning('intlinprog not found — will fall back to FCFS');
end
queue   = zeros(0, 6);
bayed   = zeros(0, 6);
active  = zeros(0, 6);
schedule = zeros(0, 2);
locked  = [];
trigger_resolve = false;
fig = figure('Name','MILP Traffic Cop v2','Color','w','Position',[60 60 1280 660]);
ax_gantt = subplot(1,2,1); hold(ax_gantt,'on'); grid on;
xlabel(ax_gantt,'time (s)'); ylabel(ax_gantt,'truck id');
title(ax_gantt,'Bay → Entry → Service schedule');
ax_map = subplot(1,2,2); hold(ax_map,'on'); axis equal; grid on;
xlabel(ax_map,'x (m)'); ylabel(ax_map,'y (m)');
title(ax_map,'Corridors & state');
t_start = tic;
last_solve  = -inf;
last_safety = -inf;
log = struct('solves',0,'kills',0,'grants',0,'bays',0,'concurrent_peaks',0,'milp_times',[]);
while true
    t_now = toc(t_start);
    if ros_ok
        while true
            try
                m = receive(sub_plans, 0.01);
                d = m.data;
                if numel(d) ~= 5, continue; end
                if any(queue(:,1)==d(1)) || any(bayed(:,1)==d(1)) || any(active(:,1)==d(1))
                    continue;
                end
                queue(end+1,:) = [d(1) d(2) d(3) d(4) d(5) t_now];
                trigger_resolve = true;
                fprintf('[+plan] T%d cls%d → (%.1f,%.1f) r=%.1f\n', d(1),d(2),d(3),d(4),d(5));
            catch
                break;
            end
        end
        while true
            try
                m = receive(sub_done, 0.01);
                tid = double(m.data);
                active(active(:,1)==tid, :) = [];
                locked(locked==tid) = [];
                trigger_resolve = true;
                fprintf('[-done] T%d completed\n', tid);
            catch
                break;
            end
        end
        while true
            try
                m = receive(sub_path, 0.01);
                d = m.data;
                if numel(d) >= 2 && d(2) == 0
                    fprintf('[path] T%d reported infeasible — re-solving\n', d(1));
                    trigger_resolve = true;
                end
            catch
                break;
            end
        end
        try
            m = receive(sub_perc, 0.01);
            fprintf('[perc] event: %s — re-solving\n', m.data);
            trigger_resolve = true;
        catch
        end
    end
    if (trigger_resolve || (t_now - last_solve) > CFG.solver_period) ...
            && ~isempty(queue)
        [sched_new, bay_assign, concurrent_set, info] = ...
            solveScheduler(queue, bayed, active, t_now, CFG, has_intlinprog);
        last_solve = t_now;
        trigger_resolve = false;
        log.solves = log.solves + 1;
        log.milp_times(end+1) = info.solve_time;
        log.concurrent_peaks = max(log.concurrent_peaks, info.peak_concurrency);
        fprintf('[MILP] N=%d  solve=%.3fs  makespan=%.1f  status=%s  peak_concurrent=%d\n', ...
                size(queue,1), info.solve_time, info.makespan, info.status, info.peak_concurrency);
        if ros_ok
            for k = 1:size(bay_assign,1)
                msg_bay.data = bay_assign(k,:);
                send(pub_bay, msg_bay);
            end
            log.bays = log.bays + size(bay_assign,1);
            if ~isempty(sched_new)
                msg_grant.data = reshape(sched_new', [], 1);
                send(pub_grant, msg_grant);
                log.grants = log.grants + size(sched_new,1);
            end
            msg_stat.data = sprintf('solve N=%d ms=%.1f peak=%d', ...
                                     size(queue,1), info.makespan, info.peak_concurrency);
            send(pub_stat, msg_stat);
        end
        for k = 1:size(bay_assign,1)
            tid = bay_assign(k,1); b = bay_assign(k,2);
            row = queue(queue(:,1)==tid, :);
            if ~isempty(row)
                bayed(end+1,:) = [row(1:5), b];
                queue(queue(:,1)==tid, :) = [];
            end
        end
        schedule = sched_new;
        updateGantt(ax_gantt, queue, bayed, active, schedule, t_now, CFG);
    end
    k = 1;
    while k <= size(schedule,1)
        tid = schedule(k,1); gt = schedule(k,2);
        if gt <= t_now + 0.01
            row = bayed(bayed(:,1)==tid, :);
            if ~isempty(row)
                active(end+1,:) = [row(1:5), gt];
                bayed(bayed(:,1)==tid, :) = [];
                fprintf('[grant] T%d released from bay %d\n', tid, row(6));
            end
        end
        k = k + 1;
    end
    if (t_now - last_safety) > CFG.safety_period
        last_safety = t_now;
        positions = zeros(0,3);
        if ros_ok && ~isempty(sub_pos.LatestMessage)
            positions = poseArrayToXY(sub_pos.LatestMessage);
        end
        if ~isempty(positions)
            kill_ids = checkSafety(positions, CFG.critical_zone, active);
            for kid = kill_ids(:)'
                if ~ismember(kid, locked)
                    locked(end+1) = kid;
                    if ros_ok
                        msg_lock.data = int32(kid); send(pub_lock, msg_lock);
                    end
                    fprintf('!! [SAFETY] milp_lock T%d\n', kid);
                    log.kills = log.kills + 1;
                end
            end
        end
        updateMap(ax_map, queue, bayed, active, locked, CFG);
    end
    drawnow limitrate;
    pause(0.05);
end
function [sched, bay_assign, concurrent_set, info] = ...
        solveScheduler(queue, bayed, active, t_now, CFG, has_int)
    info = struct('solve_time',0,'makespan',0,'status','','peak_concurrency',0);
    N = size(queue,1);
    if N == 0
        sched = zeros(0,2); bay_assign = zeros(0,2);
        concurrent_set = zeros(0,2); return;
    end
    if N > CFG.max_queue
        queue = queue(1:CFG.max_queue,:); N = CFG.max_queue;
    end
    target_xy = queue(:,3:4);
    dpath = vecnorm(target_xy - CFG.entry_xy, 2, 2);
    s = CFG.service_base + dpath * CFG.service_per_m;
    earliest = max(queue(:,6), t_now);
    if ~isempty(active)
        for k = 1:size(active,1)
            if segmentDist(CFG.entry_xy, active(k,3:4), CFG.entry_xy, target_xy) ...
                    < CFG.corridor_buffer
                s_act = CFG.service_base + ...
                        norm(active(k,3:4) - CFG.entry_xy) * CFG.service_per_m;
                earliest = max(earliest, active(k,6) + s_act);
            end
        end
    end
    conflict = false(N,N);
    for i = 1:N-1
        for j = i+1:N
            d_ij = segmentDist(CFG.entry_xy, target_xy(i,:), ...
                               CFG.entry_xy, target_xy(j,:));
            if d_ij < CFG.corridor_buffer
                conflict(i,j) = true;
            end
        end
    end
    [I, J] = find(conflict);
    C = numel(I);
    nBays = CFG.num_bays;
    bays_free = nBays - size(bayed,1);
    nAssign   = min(N, max(0, bays_free));
    if ~has_int || N <= 1 || C == 0
        [~, ord] = sort(earliest);
        t = zeros(N,1); cur = 0;
        for k = ord'
            t(k) = max(earliest(k), cur);
            cur  = t(k) + s(k);
        end
        sched = [queue(:,1), t];
        free_bays = setdiff(1:nBays, bayed(:,6));
        ba = []; bi = 1;
        for k = ord'
            if bi > numel(free_bays) || bi > nAssign, break; end
            ba = [ba; queue(k,1), free_bays(bi)];
            bi = bi + 1;
        end
        bay_assign = ba;
        concurrent_set = zeros(0,2);
        info.solve_time = 0;
        info.makespan = max(t + s) - t_now;
        info.status = 'FCFS';
        info.peak_concurrency = 1;
        return;
    end
    M_big = CFG.max_makespan;
    nT = N;
    nB = N*nBays;
    nY = C;
    nZ = C;
    nM = 1;
    nVar = nT + nB + nY + nZ + nM;
    idxT = @(i)       i;
    idxB = @(i,j)     nT + (i-1)*nBays + j;
    idxY = @(c)       nT + nB + c;
    idxZ = @(c)       nT + nB + nY + c;
    idxMs = nVar;
    alpha = 1.0; beta = 0.05; gamma = 0.5;
    f = zeros(nVar,1);
    f(idxMs)      = alpha;
    f(1:nT)       = beta;
    f(idxZ(1):idxZ(C)) = -gamma;
    Aeq = []; beq = []; A = []; b = [];
    free_bays = setdiff(1:nBays, bayed(:,6));
    nFree = numel(free_bays);
    if nFree == 0
        [~, ord] = sort(earliest);
        t = zeros(N,1); cur = 0;
        for k = ord'
            t(k) = max(earliest(k), cur); cur = t(k) + s(k);
        end
        sched = [queue(:,1), t];
        bay_assign = zeros(0,2);
        concurrent_set = zeros(0,2);
        info.solve_time = 0;
        info.makespan = max(t + s) - t_now;
        info.status = 'no-free-bays-FCFS';
        info.peak_concurrency = 1;
        return;
    end
    nAssignable = min(N, nFree);
    occupied = setdiff(1:nBays, free_bays);
    for i = 1:N
        for j = occupied
            row = zeros(1,nVar); row(idxB(i,j)) = 1;
            Aeq = [Aeq; row]; beq = [beq; 0];
        end
    end
    for i = 1:nAssignable
        row = zeros(1,nVar);
        for j = 1:nBays, row(idxB(i,j)) = 1; end
        Aeq = [Aeq; row]; beq = [beq; 1];
    end
    for i = nAssignable+1:N
        row = zeros(1,nVar);
        for j = 1:nBays, row(idxB(i,j)) = 1; end
        Aeq = [Aeq; row]; beq = [beq; 0];
    end
    for j = 1:nBays
        row = zeros(1,nVar);
        for i = 1:N, row(idxB(i,j)) = 1; end
        A = [A; row]; b = [b; 1];
    end
    for c = 1:C
        i = I(c); j = J(c); yk = idxY(c); zk = idxZ(c);
        row = zeros(1,nVar);
        row(idxT(i)) = 1; row(idxT(j)) = -1;
        row(yk) = M_big; row(zk) = -M_big;
        A = [A; row]; b = [b; M_big - s(i)];
        row = zeros(1,nVar);
        row(idxT(i)) = -1; row(idxT(j)) = 1;
        row(yk) = -M_big; row(zk) = -M_big;
        A = [A; row]; b = [b; -s(j)];
    end
    for i = 1:N
        row = zeros(1,nVar);
        row(idxT(i)) = 1; row(idxMs) = -1;
        A = [A; row]; b = [b; -s(i)];
    end
    if CFG.max_concurrent >= 2 && C > 0
        row = zeros(1,nVar);
        for c = 1:C, row(idxZ(c)) = 1; end
        A = [A; row]; b = [b; C * (CFG.max_concurrent - 1)];
    elseif C > 0
        for c = 1:C
            row = zeros(1,nVar); row(idxZ(c)) = 1;
            A = [A; row]; b = [b; 0];
        end
    end
    lb = zeros(nVar,1); ub = ones(nVar,1) * M_big;
    lb(1:nT) = earliest;
    ub(1:nT) = M_big;
    ub(nT+1 : nT+nB+nY+nZ) = 1;
    ub(idxMs) = M_big;
    intcon = (nT+1):(nVar-1);
    opts = optimoptions('intlinprog','Display','off', ...
                        'MaxTime', 5.0, 'IntegerTolerance',1e-4);
    tic;
    [x, ~, flag] = intlinprog(f, intcon, A, b, Aeq, beq, lb, ub, opts);
    info.solve_time = toc;
    if isempty(x) || flag <= 0
        warning('MILP failed flag=%d, FCFS fallback', flag);
        [~, ord] = sort(earliest);
        t = zeros(N,1); cur = 0;
        for k = ord'
            t(k) = max(earliest(k), cur); cur = t(k) + s(k);
        end
        sched = [queue(:,1), t];
        ba = [];
        for k = ord(1:min(nAssignable,end))'
            ba = [ba; queue(k,1), free_bays(numel(ba)+1)];
        end
        bay_assign = ba;
        concurrent_set = zeros(0,2);
        info.makespan = max(t + s) - t_now;
        info.status = sprintf('MILP-fail(%d)-FCFS', flag);
        info.peak_concurrency = 1;
        return;
    end
    t = x(1:nT);
    sched = [queue(:,1), t];
    ba = [];
    for i = 1:nAssignable
        for j = 1:nBays
            if x(idxB(i,j)) > 0.5
                ba = [ba; queue(i,1), j];
            end
        end
    end
    bay_assign = ba;
    cs = [];
    for c = 1:C
        if x(idxZ(c)) > 0.5
            cs = [cs; queue(I(c),1), queue(J(c),1)];
        end
    end
    concurrent_set = cs;
    info.makespan = x(idxMs) - t_now;
    info.status = 'optimal';
    info.peak_concurrency = peakConcurrency(t, s);
end
function pc = peakConcurrency(t, s)
    events = [t, ones(size(t)); t+s, -ones(size(t))];
    events = sortrows(events, 1);
    pc = max(cumsum(events(:,2)));
end
function d = segmentDist(A, B, C, Drow)
    if size(Drow,1) > 1
        d = zeros(size(Drow,1),1);
        for k = 1:size(Drow,1)
            d(k) = segDistScalar(A, B, C, Drow(k,:));
        end
    else
        d = segDistScalar(A, B, C, Drow);
    end
end
function d = segDistScalar(A, B, C, D)
    u = B - A; v = D - C; w = A - C;
    a = dot(u,u); b = dot(u,v); c = dot(v,v);
    de = dot(u,w); e_ = dot(v,w);
    DD = a*c - b*b;
    if DD < 1e-9
        sc = 0; tc = (b > c) * (de/b) + (b <= c) * (e_/c);
    else
        sc = (b*e_ - c*de) / DD;
        tc = (a*e_ - b*de) / DD;
    end
    sc = min(max(sc,0),1); tc = min(max(tc,0),1);
    dP = w + sc*u - tc*v;
    d  = norm(dP);
end
function kill_ids = checkSafety(positions, crit, active)
    kill_ids = [];
    if size(positions,1) < 2 || isempty(active), return; end
    for i = 1:size(positions,1)-1
        for j = i+1:size(positions,1)
            d = norm(positions(i,2:3) - positions(j,2:3));
            if d < crit
                id_i = positions(i,1); id_j = positions(j,1);
                gi = active(active(:,1)==id_i,6);
                gj = active(active(:,1)==id_j,6);
                if isempty(gi) || isempty(gj), continue; end
                if gi > gj, kill_ids(end+1) = id_i;
                else,        kill_ids(end+1) = id_j;
                end
            end
        end
    end
    kill_ids = unique(kill_ids);
end
function P = poseArrayToXY(msg)
    N = numel(msg.poses);
    P = zeros(N,3);
    for k = 1:N
        P(k,:) = [k, msg.poses(k).position.x, msg.poses(k).position.y];
    end
end
function updateGantt(ax, queue, bayed, active, schedule, t_now, CFG)
    cla(ax);
    for k = 1:size(bayed,1)
        rectangle(ax,'Position',[t_now-2, bayed(k,1)-0.4, 2, 0.8], ...
                  'FaceColor',[1.0 0.85 0.2],'EdgeColor','k');
        text(ax, t_now-1, bayed(k,1), sprintf('T%d bay%d',bayed(k,1),bayed(k,6)), ...
             'HorizontalAlignment','center','FontSize',7);
    end
    for k = 1:size(active,1)
        dp = norm(active(k,3:4) - CFG.entry_xy);
        sv = CFG.service_base + dp * CFG.service_per_m;
        rectangle(ax,'Position',[active(k,6), active(k,1)-0.4, sv, 0.8], ...
                  'FaceColor',[0.3 0.85 0.3],'EdgeColor','k');
        text(ax, active(k,6)+sv/2, active(k,1), sprintf('T%d',active(k,1)), ...
             'HorizontalAlignment','center','FontSize',8);
    end
    for k = 1:size(schedule,1)
        tid = schedule(k,1);
        row = bayed(bayed(:,1)==tid,:);
        if isempty(row), row = queue(queue(:,1)==tid,:); end
        if isempty(row), continue; end
        dp = norm(row(3:4) - CFG.entry_xy);
        sv = CFG.service_base + dp * CFG.service_per_m;
        rectangle(ax,'Position',[schedule(k,2), tid-0.4, sv, 0.8], ...
                  'FaceColor',[0.6 0.75 1.0],'EdgeColor','k');
        text(ax, schedule(k,2)+sv/2, tid, sprintf('T%d',tid), ...
             'HorizontalAlignment','center','FontSize',8);
    end
    yl = ylim(ax);
    if yl(2) <= yl(1), ylim(ax,[0 5]); yl = ylim(ax); end
    plot(ax, [t_now t_now], yl, 'r--','LineWidth',1.5);
    title(ax, sprintf('Schedule (t = %.1fs)', t_now));
end
function updateMap(ax, queue, bayed, active, locked, CFG)
    cla(ax);
    plot(ax, CFG.entry_xy(1), CFG.entry_xy(2), 'k*', 'MarkerSize', 14, 'LineWidth', 2);
    text(ax, CFG.entry_xy(1)+2, CFG.entry_xy(2)-2, 'GATE','FontWeight','bold');
    for b = 1:CFG.num_bays
        plot(ax, CFG.bay_xy(b,1), CFG.bay_xy(b,2), 'ks', 'MarkerSize', 12, 'LineWidth', 1.2);
        text(ax, CFG.bay_xy(b,1), CFG.bay_xy(b,2)-3, sprintf('B%d',b), ...
             'HorizontalAlignment','center','FontSize',9);
    end
    for k = 1:size(queue,1)
        viscircles(ax, queue(k,3:4), queue(k,5),'Color',[0.55 0.55 0.55],'LineWidth',1);
        text(ax, queue(k,3), queue(k,4), sprintf('T%d',queue(k,1)),'HorizontalAlignment','center','FontSize',8);
    end
    for k = 1:size(bayed,1)
        viscircles(ax, bayed(k,3:4), bayed(k,5),'Color',[1.0 0.85 0.2],'LineWidth',1.4);
        plot(ax, [CFG.bay_xy(bayed(k,6),1) bayed(k,3)], ...
                 [CFG.bay_xy(bayed(k,6),2) bayed(k,4)], 'y:','LineWidth',1);
    end
    for k = 1:size(active,1)
        col = [0.2 0.7 0.2];
        if ismember(active(k,1), locked), col = [0.9 0.1 0.1]; end
        viscircles(ax, active(k,3:4), active(k,5),'Color',col,'LineWidth',1.8);
        viscircles(ax, active(k,3:4), CFG.critical_zone,'Color',col,'LineStyle',':','LineWidth',0.8);
        plot(ax, [CFG.entry_xy(1) active(k,3)], [CFG.entry_xy(2) active(k,4)], ...
             'Color', [col 0.5], 'LineWidth',1);
        text(ax, active(k,3), active(k,4), sprintf('T%d',active(k,1)), ...
             'HorizontalAlignment','center','FontSize',8,'FontWeight','bold');
    end
end
