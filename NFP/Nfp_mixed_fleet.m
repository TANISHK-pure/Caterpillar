clear; clc; close all;

disp('Initializing ROS 2 Master Node...');
try
    node = ros2node("/nfp_master_planner");
    target_pub = ros2publisher(node, "/gazebo_truck_targets", "geometry_msgs/Point");
    ros_msg = ros2message(target_pub);
    disp('ROS 2 Publisher [ONLINE]: Broadcasting on /gazebo_truck_targets');
catch e
    disp('ROS 2 Network not found. Running in Math-Only Simulation Mode.');
end
x_poly = [0, 80, 100, 60, 10, -20];
y_poly = [0, 5, 50, 80, 60, 20];
paddock = polyshape(x_poly, y_poly);
exit_point = [0, 0]; 
total_paddock_area = area(paddock);

fleet_classes = ["100-Ton", "250-Ton", "400-Ton"];
fleet_radii = [3.0, 5.0, 7.5]; 
fleet_colors = ['b', 'g', 'r']; 

resolution = 0.2; 
[xlim_b, ylim_b] = boundingbox(paddock);
[X, Y] = meshgrid(xlim_b(1):resolution:xlim_b(2), ylim_b(1):resolution:ylim_b(2));
valid_points = [X(:), Y(:)];

dist_to_exit = vecnorm(valid_points - exit_point, 2, 2);
[~, sort_idx] = sort(dist_to_exit, 'descend');
prioritized_points = valid_points(sort_idx, :);

fig = figure('Name', 'High-Density Mixed Fleet Packing', 'Color', 'w');
hold on; axis equal;
title('Packing Efficiency: 0.00%'); 
plot(paddock, 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.5);
plot(exit_point(1), exit_point(2), 'k*', 'MarkerSize', 15, 'LineWidth', 2);

for k = 1:3
    ifr_poly = polybuffer(paddock, -fleet_radii(k));
    plot(ifr_poly, 'EdgeColor', fleet_colors(k), 'LineStyle', '--', 'LineWidth', 1.5, 'FaceAlpha', 0);
end

num_trucks_to_simulate = 200; 
dumps = []; 
total_dump_area = 0;
failed_attempts = 0; 

for i = 1:num_trucks_to_simulate
    t_idx = randi(length(fleet_radii));
    current_radius = fleet_radii(t_idx);
    current_color = fleet_colors(t_idx);
    current_ifr = polybuffer(paddock, -current_radius);
    
    placed = false;
    for j = 1:length(prioritized_points)
        cx = prioritized_points(j, 1);
        cy = prioritized_points(j, 2);
        
        if isinterior(current_ifr, cx, cy)
            
            collision = false;
            if ~isempty(dumps)
                dists = vecnorm(dumps(:, 1:2) - [cx, cy], 2, 2);
                required_spacing = current_radius + dumps(:, 3); 
                if any(dists < required_spacing)
                    collision = true;
                end
            end
            
            if ~collision
                dumps = [dumps; cx, cy, current_radius];
                placed = true;
                total_dump_area = total_dump_area + (pi * current_radius^2);
                efficiency = (total_dump_area / total_paddock_area) * 100;
                title(sprintf('High-Density Packing Efficiency: %.2f%%', efficiency));
                
                viscircles([cx, cy], current_radius, 'Color', current_color, 'LineWidth', 1);
                plot(cx, cy, 'k.', 'MarkerSize', 10);
                if exist('target_pub', 'var')
                    ros_msg.x = cx; ros_msg.y = cy; ros_msg.z = 0;
                    send(target_pub, ros_msg);
                end
                
                pause(0.05); 
                failed_attempts = 0; 
                break; 
            end
        end
    end
    
    if ~placed
        failed_attempts = failed_attempts + 1;
        if failed_attempts > 30
            disp('Paddock is fully optimized! Simulation complete.');
            break;
        end
    end
end
hold off;