clear; clc;

disp('Initializing Mine Site Air Traffic Control...');
fleet_data = [
    1, 400, 500, 10;  
    2, 100, 520, 12;  
    3, 250, 800, 10   
];

num_trucks = size(fleet_data, 1);
safe_headway_time = 20; 

etas = fleet_data(:, 3) ./ fleet_data(:, 4);

TruckID = fleet_data(:, 1);
Payload = fleet_data(:, 2);
Distance = fleet_data(:, 3);
Speed = fleet_data(:, 4);
Initial_ETA = etas;

traffic_table = table(TruckID, Payload, Distance, Speed, Initial_ETA);
disp('--- INITIAL HAUL ROAD STATUS ---');
disp(sortrows(traffic_table, 'Initial_ETA'));

disp('--- RUNNING CONFLICT RESOLUTION ---');

[sorted_etas, sort_idx] = sort(etas);
cleared_etas = zeros(num_trucks, 1);
cleared_etas(1) = sorted_etas(1); 

for i = 2:num_trucks
    current_truck_idx = sort_idx(i);
    truck_payload = fleet_data(current_truck_idx, 2);
    
    time_gap = sorted_etas(i) - cleared_etas(i-1);
    if time_gap < safe_headway_time
        disp(['⚠️ CONFLICT: Truck ', num2str(fleet_data(current_truck_idx, 1)), ...
              ' will arrive too close to the truck ahead!']);
          
        new_eta = cleared_etas(i-1) + safe_headway_time;
        cleared_etas(i) = new_eta;
        
        new_speed = fleet_data(current_truck_idx, 3) / new_eta;
        
        disp(['   -> RESOLUTION: Truck ', num2str(fleet_data(current_truck_idx, 1)), ...
              ' (', num2str(truck_payload), '-Ton) commanded to slow down to ', ...
              num2str(round(new_speed, 2)), ' m/s.']);
    else
        cleared_etas(i) = sorted_etas(i);
    end
end

Final_ETA = zeros(num_trucks, 1);
Final_ETA(sort_idx) = cleared_etas;
Commanded_Speed = Distance ./ Final_ETA;

final_table = table(TruckID, Payload, Initial_ETA, Final_ETA, Commanded_Speed);
disp('--- FINAL CLEARED TRAFFIC SCHEDULE ---');
disp(sortrows(final_table, 'Final_ETA'));