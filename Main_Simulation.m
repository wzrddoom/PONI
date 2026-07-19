% Main_Simulation.m
% Space-Based Interceptor Feasibility Study: SATURATION & MKV SCENARIO
clear; clc; close all;

%% 1. Define the Constellation (Walker-Delta / Near-Polar)
num_planes = 30;
sats_per_plane = 20;
altitude_km = 500;
inclination_deg = 88;
max_delta_v_km_s = 8.0;
earth_radius_km = 6371;
mu_earth = 3.986004418e5;

% Multiple Kill Vehicles (MKV) per carrier bus
kvs_per_carrier = 5;

disp('Generating constellation...');
constellation = generate_constellation(num_planes, sats_per_plane, altitude_km, inclination_deg, max_delta_v_km_s);
total_sats = length(constellation);

% Initialize KV inventory for every satellite in the constellation
kv_inventory = kvs_per_carrier * ones(total_sats, 1);

%% 2. Resilience and Scenario Testing
simulate_random_failures = false;
failure_rate = 0.15;
simulate_plane_loss = false;
lost_plane_idx = 3;

failed_sats = [];
if simulate_random_failures
    num_failures = round(total_sats * failure_rate);
    failed_sats = randperm(total_sats, num_failures);
end
if simulate_plane_loss
    start_idx = (lost_plane_idx - 1) * sats_per_plane + 1;
    end_idx = lost_plane_idx * sats_per_plane;
    failed_sats = unique([failed_sats, start_idx:end_idx]);
end

% Set failed satellites' inventory to zero
kv_inventory(failed_sats) = 0;
disp(['Constellation initialized. ', num2str(length(failed_sats)), ' satellites offline.']);

%% 3. Generate a Simultaneous "Silo Field" Salvo Threat
disp('Generating coordinated retaliatory salvo...');
num_threats = 6; % Simulate 6 ICBMs launching simultaneously
threat_profiles = cell(num_threats, 1);
boost_durations_s = zeros(num_threats, 1);

% Base location for the silo field (e.g., Central Eurasia)
base_lat = 60.0; base_lon = 90.0; base_heading = 15.0;

for tr = 1:num_threats
    % Add slight geographical spread to simulate different silos in a field
    lat = base_lat + (randn() * 0.5);
    lon = base_lon + (randn() * 0.5);
    hdg = base_heading + (randn() * 2.0);
    
    threat_profiles{tr} = generate_threat(lat, lon, hdg);
    boost_durations_s(tr) = threat_profiles{tr}(end).time;
end
max_boost_duration = max(boost_durations_s);

%% 4. Apply the Response Timeline
detection_latency_s = 15;
decision_latency_s = 30;
total_latency_s = detection_latency_s + decision_latency_s;
interceptor_release_time = total_latency_s;

%% 5. Time-Stepped Engagement Loop
disp('Calculating engagement feasibility against salvo...');
time_step = 1;

% Track which threats have been successfully assigned a Kill Vehicle
threat_intercepted = false(num_threats, 1);
feasible_intercepts = [];

release_constellation = propagate_orbits(constellation, interceptor_release_time, mu_earth);

% Loop through FUTURE candidate points
for t_candidate = (interceptor_release_time + 1):time_step:max_boost_duration
    time_of_flight = t_candidate - interceptor_release_time;
    
    % Loop through each active threat
    for tr = 1:num_threats
        if threat_intercepted(tr) || t_candidate > boost_durations_s(tr)
            continue; % Threat already destroyed or burnt out
        end
        
        threat_index = t_candidate + 1; 
        threat_pos = threat_profiles{tr}(threat_index).position; 
        
        for i = 1:total_sats
            % Skip if satellite has no remaining Kill Vehicles
            if kv_inventory(i) <= 0
                continue;
            end
            
            sat_pos = release_constellation(i).position;
            sat_vel = release_constellation(i).velocity; 
            
            % Rigorous, fast kinematic pre-filter (Max Closing Speed)
            max_closing_speed = norm(sat_vel) + max_delta_v_km_s;
            if norm(threat_pos - sat_pos) > (max_closing_speed * time_of_flight)
                continue; % Physically impossible to reach in time
            end
            
            if check_line_of_sight(sat_pos, threat_pos, earth_radius_km)
                required_delta_v = calculate_reachability(sat_pos, sat_vel, threat_pos, time_of_flight, mu_earth);
                
                if required_delta_v < max_delta_v_km_s
                    % INTERCEPT ACHIEVED!
                    threat_intercepted(tr) = true;
                    kv_inventory(i) = kv_inventory(i) - 1; % Expend one Kill Vehicle
                    
                    new_intercept.sat_idx = i;
                    new_intercept.threat_idx = tr;
                    new_intercept.sat_start_pos = sat_pos;
                    new_intercept.threat_intercept_pos = threat_pos;
                    new_intercept.intercept_time = t_candidate;
                    new_intercept.delta_v_used = required_delta_v;
                    
                    feasible_intercepts = [feasible_intercepts, new_intercept];
                    break; % Move to the next threat since this one is dead
                end
            end
        end
    end
end

%% 6. Calculate Metrics and Sensitivity
disp('----------------------------------------------------');
disp('ENGAGEMENT METRICS REPORT (RUSI PONI)');
disp('----------------------------------------------------');
total_intercepted = sum(threat_intercepted);
survival_rate = ((num_threats - total_intercepted) / num_threats) * 100;

disp(['THREATS LAUNCHED:                ', num2str(num_threats)]);
disp(['THREATS DESTROYED:               ', num2str(total_intercepted)]);
disp(['LEAKAGE RATE (MISSILES SURVIVED):', num2str(survival_rate), '%']);

if ~isempty(feasible_intercepts)
    fired_sats = unique([feasible_intercepts.sat_idx]);
    disp(['UNIQUE CARRIER BUSES FIRED:      ', num2str(length(fired_sats))]);
    disp(['TOTAL KILL VEHICLES EXPENDED:    ', num2str(length(feasible_intercepts))]);
end
disp('----------------------------------------------------');

%% 7. Live Engagement Animation
disp('Starting live animation...');
figure('Name', 'Live MKV Engagement', 'Color', 'w', 'Position', [50, 100, 900, 700]);
hold on; grid on; view(3);

[X, Y, Z] = sphere(50);
surf(X * earth_radius_km, Y * earth_radius_km, Z * earth_radius_km, ...
    'EdgeColor', 'none', 'FaceColor', [0.1 0.4 0.7], 'FaceAlpha', 0.8);

h_idle = scatter3(NaN, NaN, NaN, 15, [0.6 0.6 0.6], 'filled', 'DisplayName', 'Idle Active Interceptors');
h_carrier = scatter3(NaN, NaN, NaN, 25, 'c', 'filled', 'DisplayName', 'Carrier Bus (Active)');
h_offline = scatter3(NaN, NaN, NaN, 40, 'r', 'x', 'LineWidth', 1.5, 'DisplayName', 'Offline/Destroyed');
h_engaging = scatter3(NaN, NaN, NaN, 50, 'g', 'filled', 'MarkerEdgeColor', 'k', 'DisplayName', 'Kill Vehicles');

h_missile_trails = gobjects(num_threats, 1);
h_missile_currents = gobjects(num_threats, 1);
for tr = 1:num_threats
    h_missile_trails(tr) = plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    h_missile_currents(tr) = plot3(NaN, NaN, NaN, 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'HandleVisibility', 'off');
end
% Single legend entry for threats
plot3(NaN, NaN, NaN, 'r-', 'LineWidth', 1.5, 'DisplayName', 'ICBM Salvo Trails');

axis equal;
max_range = earth_radius_km + altitude_km + 1000;
xlim([-max_range max_range]); ylim([-max_range max_range]); zlim([-max_range max_range]);
title('Salvo Launch vs. Multiple Kill Vehicle (MKV) Constellation');
legend('Location', 'bestoutside');

for t = 0:time_step:max_boost_duration
    current_idx = t + 1;
    
    % Update all threats
    for tr = 1:num_threats
        if t <= boost_durations_s(tr)
            t_mat = [threat_profiles{tr}.position];
            set(h_missile_trails(tr), 'XData', t_mat(1, 1:current_idx), 'YData', t_mat(2, 1:current_idx), 'ZData', t_mat(3, 1:current_idx));
            
            % Hide current marker if destroyed
            destroyed = false;
            if ~isempty(feasible_intercepts)
                tr_intercepts = feasible_intercepts([feasible_intercepts.threat_idx] == tr);
                if ~isempty(tr_intercepts) && t >= tr_intercepts(1).intercept_time
                    destroyed = true;
                end
            end
            
            if ~destroyed
                set(h_missile_currents(tr), 'XData', t_mat(1, current_idx), 'YData', t_mat(2, current_idx), 'ZData', t_mat(3, current_idx));
            else
                set(h_missile_currents(tr), 'XData', NaN, 'YData', NaN, 'ZData', NaN);
            end
        end
    end
    
    % Update Constellation
    curr_const = propagate_orbits(constellation, t, mu_earth);
    sat_pos_matrix = [curr_const.position];
    
    all_sats = 1:total_sats;
    fired_sats = [];
    if t >= interceptor_release_time && ~isempty(feasible_intercepts)
        fired_sats = unique([feasible_intercepts.sat_idx]);
    end
    idle_sats_idx = setdiff(all_sats, [fired_sats, failed_sats]);
    
    if ~isempty(failed_sats)
        set(h_offline, 'XData', sat_pos_matrix(1, failed_sats), 'YData', sat_pos_matrix(2, failed_sats), 'ZData', sat_pos_matrix(3, failed_sats));
    end
    if ~isempty(idle_sats_idx)
        set(h_idle, 'XData', sat_pos_matrix(1, idle_sats_idx), 'YData', sat_pos_matrix(2, idle_sats_idx), 'ZData', sat_pos_matrix(3, idle_sats_idx));
    end
    if ~isempty(fired_sats)
        set(h_carrier, 'XData', sat_pos_matrix(1, fired_sats), 'YData', sat_pos_matrix(2, fired_sats), 'ZData', sat_pos_matrix(3, fired_sats));
    end
    
    % Update KVs
    engaging_x = []; engaging_y = []; engaging_z = [];
    if ~isempty(feasible_intercepts)
        for k = 1:length(feasible_intercepts)
            intercept = feasible_intercepts(k);
            if t <= intercept.intercept_time && t >= interceptor_release_time
                fraction = (t - interceptor_release_time) / (intercept.intercept_time - interceptor_release_time);
                c_pos = intercept.sat_start_pos + fraction * (intercept.threat_intercept_pos - intercept.sat_start_pos);
            elseif t > intercept.intercept_time
                c_pos = intercept.threat_intercept_pos;
            else
                continue;
            end
            engaging_x(end+1) = c_pos(1); engaging_y(end+1) = c_pos(2); engaging_z(end+1) = c_pos(3);
        end
    end
    
    if ~isempty(engaging_x)
        set(h_engaging, 'XData', engaging_x, 'YData', engaging_y, 'ZData', engaging_z);
    else
        set(h_engaging, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    end
    
    drawnow; pause(0.01); 
end
disp('Simulation complete.');

%% 8. Static Engagement Summary (Poster Visual)
disp('Generating static engagement summary visual for export...');
figure('Name', 'Static Engagement Summary', 'Color', 'w', 'Position', [950, 100, 900, 700]);
hold on; grid on; view(3);

[X, Y, Z] = sphere(50);
surf(X * earth_radius_km, Y * earth_radius_km, Z * earth_radius_km, ...
    'EdgeColor', 'none', 'FaceColor', [0.7 0.8 0.9], 'FaceAlpha', 0.4);

% Plot all threat trajectories
for tr = 1:num_threats
    t_mat = [threat_profiles{tr}.position];
    if tr == 1
        plot3(t_mat(1, 1), t_mat(2, 1), t_mat(3, 1), 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Silo Field (Launch Sites)');
        plot3(t_mat(1, :), t_mat(2, :), t_mat(3, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'ICBM Salvo Trails');
    else
        plot3(t_mat(1, 1), t_mat(2, 1), t_mat(3, 1), 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'HandleVisibility', 'off');
        plot3(t_mat(1, :), t_mat(2, :), t_mat(3, :), 'r-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
end

% Sort out constellation groups
all_sats = 1:total_sats;
fired_sats_idx = [];
if ~isempty(feasible_intercepts)
    fired_sats_idx = unique([feasible_intercepts.sat_idx]);
end
idle_sats_idx = setdiff(all_sats, [fired_sats_idx, failed_sats]);

% 1. Plot Idle Satellites
if ~isempty(idle_sats_idx)
    idle_pos = [release_constellation(idle_sats_idx).position];
    scatter3(idle_pos(1, :), idle_pos(2, :), idle_pos(3, :), ...
        15, [0.6 0.6 0.6], 'filled', 'DisplayName', 'Idle Satellites');
end

% 2. Plot Offline/Destroyed Satellites
if ~isempty(failed_sats)
    failed_pos = [release_constellation(failed_sats).position];
    scatter3(failed_pos(1, :), failed_pos(2, :), failed_pos(3, :), ...
        40, 'r', 'x', 'LineWidth', 1.5, 'DisplayName', 'Offline/Destroyed');
end

% 3. Plot Firing Carrier Buses & MKV Transfer Paths
if ~isempty(fired_sats_idx)
    fired_pos = [release_constellation(fired_sats_idx).position];
    scatter3(fired_pos(1, :), fired_pos(2, :), fired_pos(3, :), ...
        40, 'c', 'filled', 'MarkerEdgeColor', 'b', 'DisplayName', 'Carrier Buses (Fired MKVs)');
    
    for k = 1:length(feasible_intercepts)
        intercept = feasible_intercepts(k);
        
        % Plot KV transfer path
        plot3([intercept.sat_start_pos(1), intercept.threat_intercept_pos(1)], ...
              [intercept.sat_start_pos(2), intercept.threat_intercept_pos(2)], ...
              [intercept.sat_start_pos(3), intercept.threat_intercept_pos(3)], ...
              'g--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
             
        % Plot Intercept Point
        if k == 1
            scatter3(intercept.threat_intercept_pos(1), intercept.threat_intercept_pos(2), intercept.threat_intercept_pos(3), ...
                     120, 'p', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y', 'DisplayName', 'Intercept Points');
        else
            scatter3(intercept.threat_intercept_pos(1), intercept.threat_intercept_pos(2), intercept.threat_intercept_pos(3), ...
                     120, 'p', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
        end
    end
end

axis equal;
max_range = earth_radius_km + altitude_km + 1000;
xlim([-max_range max_range]); ylim([-max_range max_range]); zlim([-max_range max_range]);
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('Static Engagement Summary: Multiple Kill Vehicle (MKV) Salvo Defense');
legend('Location', 'bestoutside');
set(gca, 'FontSize', 12);