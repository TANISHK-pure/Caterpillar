% =========================================================================
% STAGE 3.4: The Ultimate Demo - Fully Randomized Mixed-Fleet NFP
% =========================================================================

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

% =========================================================================
% 1. RANDOMIZED PADDOCK GENERATOR
% Every time you run the script, it picks a random mine shape
% =========================================================================
shape_selector = randi(4); % Randomly picks 1, 2, 3, or 4

switch shape_selector
    case 1 % The Classic Paddock
        x_poly = [0, 80, 100, 60, 10, -20];
        y_poly = [0, 5, 50, 80, 60, 20];
        exit_point = [0, 0];
        shape_name = 'Classic Polygon';
    case 2 % The "L-Shape" Paddock (Tests cornering)
        x_poly = [0, 80, 80, 40, 40, 0];
        y_poly = [0, 0, 100, 100, 30, 30];
        exit_point = [0, 0];
        shape_name = 'L-Shape Paddock';
    case 3 % The "Hourglass" (Tests chokepoints)
        x_poly = [0, 40, 25, 40, 0, 15];
        y_poly = [0, 0, 50, 100, 100, 50];
        exit_point = [20, 0]; % Exit is in the bottom middle
        shape_name = 'Hourglass Chokepoint';
    case 4 % The "Sharp Triangle" (Tests extreme corners)
        x_poly = [0, 100, 10];
        y_poly = [0, 20, 90];
        exit_point = [0, 0];
        shape_name = 'Sharp Triangle';
end

paddock = polyshape(x_poly, y_poly);
total_paddock_area = area(paddock); 

% 2. Mixed Fleet Definition
fleet_classes = ["100-Ton", "250-Ton", "400-Ton"];
fleet_radii = [3.0, 5.0, 7.5]; 
fleet_colors = ['b', 'g', 'r']; 

% 3. HIGH-RESOLUTION Grid Generation (0.2m for tight packing)
resolution = 0.2; 
[xlim_b, ylim_b] = boundingbox(paddock);
[X, Y] = meshgrid(xlim_b(1):resolution:xlim_b(2), ylim_b(1):resolution:ylim_b(2));
valid_points = [X(:), Y(:)];

% Sort the grid by retreating wavefront
dist_to_exit = vecnorm(valid_points - exit_point, 2, 2);
[~, sort_idx] = sort(dist_to_exit, 'descend');
prioritized_points = valid_points(sort_idx, :);

% 4. Visualization Setup
fig = figure('Name', ['Live Demo: ', shape_name], 'Color', 'w');
hold on; axis equal;
title(sprintf('%s - Efficiency: 0.00%%', shape_name)); 
plot(paddock, 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.5);
plot(exit_point(1), exit_point(2), 'k*', 'MarkerSize', 15, 'LineWidth', 2);

% Draw Safe Driving Boundaries
for k = 1:3
    ifr_poly = polybuffer(paddock, -fleet_radii(k));
    plot(ifr_poly, 'EdgeColor', fleet_colors(k), 'LineStyle', '--', 'LineWidth', 1.5, 'FaceAlpha', 0);
end

% 5. The High-Density Packing Loop
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
                
                % Update Efficiency Tracking
                total_dump_area = total_dump_area + (pi * current_radius^2);
                efficiency = (total_dump_area / total_paddock_area) * 100;
                title(sprintf('%s - Efficiency: %.2f%%', shape_name, efficiency));
                
                viscircles([cx, cy], current_radius, 'Color', current_color, 'LineWidth', 1);
                plot(cx, cy, 'k.', 'MarkerSize', 10);
                
                % Broadcast to Gazebo
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
            disp(['Paddock [', shape_name, '] is fully optimized! Simulation complete.']);
            break;
        end
    end
end
hold off;