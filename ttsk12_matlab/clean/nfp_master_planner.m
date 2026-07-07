clear; clc; close all;
CFG.use_ros          = true;
CFG.air_gap          = 1.0;
CFG.cand_res         = 0.5;
CFG.score_top_K      = 50;
CFG.confirm_timeout  = 2.0;
CFG.arrival_timeout  = 30.0;
CFG.demo_pause       = 0.05;
CFG.N_max            = 250;
FLEET.class  = ["100T","250T","400T"];
FLEET.radius = [3.0, 5.0, 7.5];
FLEET.color  = ['b','g','r'];
paddock_x = [0, 80, 100, 60, 10, -20];
paddock_y = [0,  5,  50, 80, 60,  20];
paddock   = polyshape(paddock_x, paddock_y);
EXIT      = [0, 0];
A_total   = area(paddock);
fprintf('Caching Inner-Fit Polygons...\n');
IFR = cell(1, numel(FLEET.radius));
for k = 1:numel(FLEET.radius)
    IFR{k} = polybuffer(paddock, -(FLEET.radius(k) + CFG.air_gap/2));
end
[xb, yb] = boundingbox(paddock);
[Xg, Yg] = meshgrid(xb(1):CFG.cand_res:xb(2), yb(1):CFG.cand_res:yb(2));
CAND     = [Xg(:), Yg(:)];
[~, idx] = sort(vecnorm(CAND - EXIT, 2, 2), 'descend');
CAND     = CAND(idx, :);
ros_ok = false;
if CFG.use_ros
    try
        node     = ros2node("/nfp_master_planner");
        pub_tgt  = ros2publisher(node, "/dump_targets",      "geometry_msgs/Point");
        pub_cls  = ros2publisher(node, "/dump_target_class", "std_msgs/Int32");
        pub_plan = ros2publisher(node, "/dump_plans",        "std_msgs/Float64MultiArray");
        sub_arr  = ros2subscriber(node, "/truck_arrivals",   "std_msgs/Float64MultiArray");
        sub_cnf  = ros2subscriber(node, "/dump_confirm",     "geometry_msgs/Point");
        msg_tgt  = ros2message(pub_tgt);
        msg_cls  = ros2message(pub_cls);
        msg_plan = ros2message(pub_plan);
        ros_ok   = true;
        fprintf('ROS 2 link [ONLINE]\n');
        fprintf('  PUB  /dump_targets        geometry_msgs/Point\n');
        fprintf('  PUB  /dump_target_class   std_msgs/Int32\n');
        fprintf('  PUB  /dump_plans          Float64MultiArray  [id,cls,x,y,r]\n');
        fprintf('  SUB  /truck_arrivals      Float64MultiArray  [id,cls]\n');
        fprintf('  SUB  /dump_confirm        geometry_msgs/Point\n');
    catch ME
        fprintf('ROS 2 unavailable: %s\n', ME.message);
    end
end
fig = figure('Name','NFP Master Planner v2.1','Color','w','Position',[100 100 980 720]);
ax  = axes(fig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
plot(paddock,'FaceColor',[0.82 0.82 0.82],'FaceAlpha',0.5);
plot(EXIT(1),EXIT(2),'k*','MarkerSize',18,'LineWidth',2);
text(EXIT(1)+2, EXIT(2)-2,'EXIT','FontWeight','bold');
for k = 1:numel(IFR)
    plot(IFR{k},'EdgeColor',FLEET.color(k),'LineStyle','--', ...
                'LineWidth',1.3,'FaceAlpha',0);
end
xlabel('x (m)'); ylabel('y (m)');
dumps      = zeros(0, 5);
counts     = zeros(1, numel(FLEET.radius));
A_dump     = 0;
local_uid  = 0;
drift_log  = [];
for n = 1:CFG.N_max
    if ros_ok
        try
            arr = receive(sub_arr, CFG.arrival_timeout);
            d = arr.data;
            if numel(d) < 2
                fprintf('[WARN] arrival message malformed\n'); continue;
            end
            truck_id = double(d(1));
            t_idx    = double(d(2));
        catch
            fprintf('[idle] still waiting for trucks...\n');
            continue;
        end
        if t_idx < 1 || t_idx > numel(FLEET.radius)
            fprintf('[WARN] invalid class %d\n', t_idx); continue;
        end
    else
        local_uid = local_uid + 1;
        truck_id  = local_uid;
        t_idx     = randi(numel(FLEET.radius));
    end
    r_t = FLEET.radius(t_idx);
    col = FLEET.color(t_idx);
    cls = FLEET.class(t_idx);
    [best_xy, ok] = pickBestSlot(CAND, IFR{t_idx}, dumps, r_t, CFG);
    if ~ok
        fprintf('[%3d] T%d %s : NO SLOT\n', n, truck_id, cls);
        continue;
    end
    if ros_ok
        msg_cls.data = int32(t_idx);            send(pub_cls, msg_cls);
        msg_tgt.x = best_xy(1);
        msg_tgt.y = best_xy(2);
        msg_tgt.z = r_t;                        send(pub_tgt, msg_tgt);
        msg_plan.data = [truck_id; t_idx; best_xy(1); best_xy(2); r_t];
        send(pub_plan, msg_plan);
        fprintf('[plan] T%d %s → (%.1f, %.1f) r=%.1f\n', truck_id, cls, best_xy(1), best_xy(2), r_t);
    end
    placed_xy = best_xy;
    if ros_ok
        try
            cnf = receive(sub_cnf, CFG.confirm_timeout);
            placed_xy = [cnf.x, cnf.y];
            drift_log(end+1) = norm(placed_xy - best_xy);
        catch
            fprintf('  [timeout] using commanded position\n');
        end
    end
    dumps(end+1,:) = [placed_xy(1), placed_xy(2), r_t, t_idx, truck_id];
    counts(t_idx)  = counts(t_idx) + 1;
    A_dump         = A_dump + pi*r_t^2;
    eff            = 100 * A_dump / A_total;
    viscircles(placed_xy, r_t, 'Color', col, 'LineWidth', 1.2);
    plot(placed_xy(1), placed_xy(2),'k.','MarkerSize',8);
    text(placed_xy(1), placed_xy(2), sprintf('%d', truck_id), ...
         'HorizontalAlignment','center','FontSize',7);
    title(ax, sprintf('Pack %.1f%% | 100T:%d  250T:%d  400T:%d | total %d', ...
        eff, counts(1), counts(2), counts(3), sum(counts)),'FontWeight','bold');
    drawnow limitrate;
end
function [xy, ok] = pickBestSlot(CAND, ifr, dumps, r_t, CFG)
    xy = [0 0]; ok = false;
    inside = isinterior(ifr, CAND(:,1), CAND(:,2));
    feas   = CAND(inside, :);
    if isempty(feas), return; end
    if ~isempty(dumps)
        dx = feas(:,1) - dumps(:,1)';
        dy = feas(:,2) - dumps(:,2)';
        d  = sqrt(dx.^2 + dy.^2);
        req = (r_t + dumps(:,3)' + CFG.air_gap);
        feas = feas(all(d >= req, 2), :);
        if isempty(feas), return; end
    end
    K = min(CFG.score_top_K, size(feas,1));
    cand = feas(1:K, :);
    if isempty(dumps)
        xy = cand(1,:); ok = true; return;
    end
    dx = cand(:,1) - dumps(:,1)';
    dy = cand(:,2) - dumps(:,2)';
    d  = sqrt(dx.^2 + dy.^2);
    [~, bi] = min(min(d, [], 2));
    xy = cand(bi, :);
    ok = true;
end
