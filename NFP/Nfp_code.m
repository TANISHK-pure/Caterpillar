% =========================================================================
% NFP Stage 2: Retreating Wavefront (Preventing Polygon Isolation)
% =========================================================================

clear; clc; close all;

% 1. Define the Paddock and Payload (Same as before)
x_poly = [0, 50, 70, 45, 10, -10];
y_poly = [0, 5, 40, 60, 45, 20];
paddock = polyshape(x_poly, y_poly);
payload_radius = 5; 
ifr_polygon = polybuffer(paddock, -payload_radius);

% 2. Define the Paddock Exit/Entrance
% Let's assume the trucks enter and exit at the bottom left (0,0)
exit_point = [0, 0]; 

% 3. Generate a Dense Grid of all possible coordinates
[xlim, ylim] = boundingbox(ifr_polygon);
resolution = 1.0; % 1 meter grid resolution
[X, Y] = meshgrid(xlim(1):resolution:xlim(2), ylim(1):resolution:ylim(2));
grid_points = [X(:), Y(:)];

% Filter out the points that are not inside the safe IFR boundary
in_ifr = isinterior(ifr_polygon, grid_points(:,1), grid_points(:,2));
valid_points = grid_points(in_ifr, :);

% 4. The Retreating Wavefront Logic (Crucial Step)
% Calculate the straight-line distance from every valid point to the exit
dist_to_exit = vecnorm(valid_points - exit_point, 2, 2);

% Sort the points so the FURTHEST points are at the top of the list
[~, sort_idx] = sort(dist_to_exit, 'descend');
prioritized_points = valid_points(sort_idx, :);

% 5. Visualization Setup
figure('Name', 'NFP Wavefront Packing', 'Color', 'w');
hold on; axis equal;
title('Retreating Wavefront: Furthest to Closest Packing');
plot(paddock, 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.5);
plot(ifr_polygon, 'EdgeColor', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
plot(exit_point(1), exit_point(2), 'g*', 'MarkerSize', 15, 'LineWidth', 2); % Mark the exit

% 6. Packing Logic: Systematic placement instead of random
dump_centers = [];

% Iterate through our prioritized list (starting at the furthest back corner)
for i = 1:length(prioritized_points)
    cx = prioritized_points(i, 1);
    cy = prioritized_points(i, 2);
    
    % NFP Collision Check (Don't overlap existing piles)
    collision = false;
    if ~isempty(dump_centers)
        % Calculate distance to all existing dumps
        dists = vecnorm(dump_centers - [cx, cy], 2, 2);
        if any(dists < (2 * payload_radius))
            collision = true;
        end
    end
    
    if ~collision
        dump_centers = [dump_centers; cx, cy];
        viscircles([cx, cy], payload_radius, 'Color', 'b', 'LineWidth', 1);
        plot(cx, cy, 'k.', 'MarkerSize', 10);
        pause(0.1); % Animate to show the wavefront moving backward!
    end
end

disp(['Successfully placed ', num2str(size(dump_centers, 1)), ' dump spots using Wavefront logic.']);
hold off;