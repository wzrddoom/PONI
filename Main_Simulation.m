% Main_Simulation.m
% Space-Based Interceptor Feasibility Study: SATURATION & MKV SCENARIO
clear; clc; close all;

%% 0. Simulation Mode Control
% 'Single_Visual' = Runs once, shows Live 3D Animation AND Static Plot.
% 'Monte_Carlo'   = Runs N times, shows statistical histograms (Leakage & TTK).
SIMULATION_MODE = 'Single_Visual'; 
num_mc_runs = 50; % Number of iterations for Monte Carlo mode

%% 1. Define the Constellation (Walker-Delta / Near-Polar)
num_planes = 30;
sats_per_plane = 20;
altitude_km = 500;
inclination_deg = 88;
max_delta_v_km_s = 8.0;
earth_radius_km = 6371;
mu_earth = 3.986004418e5;

kvs_per_carrier = 5;

disp('Generating base constellation...');
base_constellation = generate_constellation(num_planes, sats_per_plane, altitude_km, inclination_deg, max_delta_v_km_s);
total_sats = length(base_constellation);

%% 2. Define Threat Parameters (Multi-Field Capability)
num_threats = 12; % Size of the enemy salvo

% Define multiple distinct launch regions [Latitude, Longitude, Base Heading]
silo_fields = [
    60.0,  90.0,  15.0;  % Field A: Central Eurasia
    50.0, 115.0,  45.0;  % Field B: East Asia
    65.0,  45.0, 340.0;  % Field C: Western Russia
];

% Interceptor Performance & Doctrine
probability_of_kill = 0.80; % 80% chance a KV physically destroys the target
kill_assessment_delay_s = 5; % Seconds before system realizes a shot missed and fires again

detection_latency_s = 15;
decision_latency_s = 30;
interceptor_release_time = detection_latency_s + decision_latency_s;

% Data storage for Monte Carlo statistics
mc_leakage_rates = zeros(num_mc_runs, 1);
mc_kvs_expended = zeros(num_mc_runs, 1);
mc_avg_ttk = zeros(num_mc_runs, 1);

runs_to_execute = 1;
if strcmp(SIMULATION_MODE, 'Monte_Carlo')
    runs_to_execute = num_mc_runs;
    disp(['Starting Monte Carlo Simulation (', num2str(num_mc_runs), ' iterations)...']);
end

%% 3. Main Engagement Loop
for run = 1:runs_to_execute
    if strcmp(SIMULATION_MODE, 'Monte_Carlo') && mod(run, 10) == 0
        disp(['  Processing run ', num2str(run), ' of ', num2str(runs_to_execute), '...']);
    end
    
    % --- A. Apply Random Failures ---
    failure_rate = 0.15;
    num_failures = round(total_sats * failure_rate);
    failed_sats = randperm(total_sats, num_failures);
    
    kv_inventory = kvs_per_carrier * ones(total_sats, 1);
    kv_inventory(failed_sats) = 0; % Offline satellites have 0 KVs
    
    % --- B. Generate Threats Across Multiple Fields ---
    threat_profiles = cell(num_threats, 1);
    boost_durations_s = zeros(num_threats, 1);
    
    for tr = 1:num_threats
        % Randomly assign this missile to one of the silo fields
        field_idx = randi(size(silo_fields, 1));
        lat = silo_fields(field_idx, 1) + (randn() * 0.5);
        lon = silo_fields(field_idx, 2) + (randn() * 0.5);
        hdg = silo_fields(field_idx, 3) + (randn() * 2.0);
        
        threat_profiles{tr} = generate_threat(lat, lon, hdg);
        boost_durations_s(tr) = threat_profiles{tr}(end).time;
    end
    max_boost_duration = max(boost_durations_s);
    
    % --- C. Time-Stepped Engagement Logic ---
    threat_intercepted = false(num_threats, 1);
    threat_immune_until = zeros(num_threats, 1); % Handles Shoot-Look-Shoot delay
    feasible_intercepts = [];
    
    % Propagate constellation to fire order time
    release_constellation = propagate_orbits(base_constellation, interceptor_release_time, mu_earth);
    
    for t_candidate = (interceptor_release_time + 1):1:max_boost_duration
        time_of_flight = t_candidate - interceptor_release_time;
        
        for tr = 1:num_threats
            % Skip if already destroyed, burnt out, or immune due to a recent miss being assessed
            if threat_intercepted(tr) || t_candidate > boost_durations_s(tr) || t_candidate < threat_immune_until(tr)
                continue; 
            end
            
            threat_index = t_candidate + 1; 
            threat_pos = threat_profiles{tr}(threat_index).position; 
            
            for i = 1:total_sats
                if kv_inventory(i) <= 0
                    continue;
                end
                
                sat_pos = release_constellation(i).position;
                sat_vel = release_constellation(i).velocity; 
                
                % Fast kinematic pre-filter
                max_closing_speed = norm(sat_vel) + max_delta_v_km_s;
                if norm(threat_pos - sat_pos) > (max_closing_speed * time_of_flight)
                    continue; 
                end
                
                if check_line_of_sight(sat_pos, threat_pos, earth_radius_km)
                    required_delta_v = calculate_reachability(sat_pos, sat_vel, threat_pos, time_of_flight, mu_earth);
                    
                    if required_delta_v < max_delta_v_km_s
                        % KV FIRED! Expend inventory
                        kv_inventory(i) = kv_inventory(i) - 1; 
                        
                        % Evaluate Probability of Kill
                        is_kill = rand() <= probability_of_kill;
                        
                        new_intercept.sat_idx = i;
                        new_intercept.threat_idx = tr;
                        new_intercept.sat_start_pos = sat_pos;
                        new_intercept.threat_intercept_pos = threat_pos;
                        new_intercept.intercept_time = t_candidate;
                        new_intercept.ttk = time_of_flight;
                        new_intercept.is_kill = is_kill;
                        
                        feasible_intercepts = [feasible_intercepts, new_intercept];
                        
                        if is_kill
                            threat_intercepted(tr) = true;
                        else
                            % Missed! Target survives. Wait for kill assessment before firing again.
                            threat_immune_until(tr) = t_candidate + kill_assessment_delay_s;
                        end
                        
                        break; % Move to the next threat for this time step
                    end
                end
            end
        end
    end
    
    % --- D. Log Metrics for this Run ---
    total_intercepted = sum(threat_intercepted);
    mc_leakage_rates(run) = ((num_threats - total_intercepted) / num_threats) * 100;
    
    if ~isempty(feasible_intercepts)
        mc_kvs_expended(run) = length(feasible_intercepts);
        kill_shots = feasible_intercepts([feasible_intercepts.is_kill] == true);
        if ~isempty(kill_shots)
            mc_avg_ttk(run) = mean([kill_shots.ttk]);
        end
    end
end

%% 4. Results & Visualisation
disp('----------------------------------------------------');
disp('ENGAGEMENT METRICS REPORT (RUSI PONI)');
disp('----------------------------------------------------');

if strcmp(SIMULATION_MODE, 'Monte_Carlo')
    % --- Monte Carlo Statistics Output ---
    disp(['MONTE CARLO RESULTS (', num2str(num_mc_runs), ' Runs)']);
    disp(['Threat Salvo Size:               ', num2str(num_threats), ' ICBMs (Multi-Field)']);
    disp(['Assumed Probability of Kill:     ', num2str(probability_of_kill * 100), '%']);
    disp(['Average Leakage Rate:            ', num2str(mean(mc_leakage_rates), '%.1f'), '%']);
    disp(['Max Leakage Rate (Worst Case):   ', num2str(max(mc_leakage_rates), '%.1f'), '%']);
    disp(['Average KVs Expended:            ', num2str(mean(mc_kvs_expended), '%.1f')]);
    disp(['Average Time-To-Kill (TTK):      ', num2str(mean(mc_avg_ttk(mc_avg_ttk > 0)), '%.1f'), ' seconds']);
    disp('----------------------------------------------------');
    
    % Plot Histograms for Poster
    figure('Name', 'Monte Carlo Analysis', 'Color', 'w', 'Position', [100, 200, 1000, 400]);
    
    subplot(1, 2, 1);
    histogram(mc_leakage_rates, 'BinWidth', max(1, 100/num_threats), 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'k');
    title(['System Leakage Rate Distribution (P_k = ', num2str(probability_of_kill*100), '%)']);
    xlabel('Leakage Rate (%)'); ylabel('Frequency (Runs)');
    grid on;
    
    subplot(1, 2, 2);
    valid_ttk = mc_avg_ttk(mc_avg_ttk > 0);
    if ~isempty(valid_ttk)
        histogram(valid_ttk, 10, 'FaceColor', [0.2 0.6 0.8], 'EdgeColor', 'k');
    end
    title('Average Time-To-Kill (TTK) Distribution');
    xlabel('Time-To-Kill (Seconds post-release)'); ylabel('Frequency (Runs)');
    grid on;

else
% --- Single Run Statistics & 3D Visualization ---
disp(['THREATS LAUNCHED:                ', num2str(num_threats)]);
disp(['THREATS DESTROYED:               ', num2str(sum(threat_intercepted))]);
disp(['LEAKAGE RATE:                    ', num2str(mc_leakage_rates(1)), '%']);
disp(['TOTAL KVs FIRED:                 ', num2str(mc_kvs_expended(1))]);
if mc_avg_ttk(1) > 0
disp(['AVERAGE TIME-TO-KILL (TTK):      ', num2str(mc_avg_ttk(1), '%.1f'), ' seconds']);
end
disp('----------------------------------------------------');

% STREAMING_CHUNK: Restoring live 3D animation sequence...
%% 5. Live Engagement Animation
disp('Starting live animation...');
figure('Name', 'Live MKV Engagement', 'Color', 'w', 'Position', [50, 100, 900, 700]);
hold on; grid on; view(3);

[X, Y, Z] = sphere(50);
surf(X * earth_radius_km, Y * earth_radius_km, Z * earth_radius_km, ...
    'EdgeColor', 'none', 'FaceColor', [0.1 0.4 0.7], 'FaceAlpha', 0.8, 'HandleVisibility', 'off');

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
title('Live Simulation: Multi-Field MKV Defense');
legend('Location', 'bestoutside');

for t = 0:1:max_boost_duration
    current_idx = t + 1;
    
    % Update threats
    for tr = 1:num_threats
        if t <= boost_durations_s(tr)
            t_mat = [threat_profiles{tr}.position];
            set(h_missile_trails(tr), 'XData', t_mat(1, 1:current_idx), 'YData', t_mat(2, 1:current_idx), 'ZData', t_mat(3, 1:current_idx));
            
            destroyed = false;
            if ~isempty(feasible_intercepts)
                tr_intercepts = feasible_intercepts([feasible_intercepts.threat_idx] == tr & [feasible_intercepts.is_kill] == true);
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
    curr_const = propagate_orbits(base_constellation, t, mu_earth);
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
                engaging_x(end+1) = c_pos(1); engaging_y(end+1) = c_pos(2); engaging_z(end+1) = c_pos(3);
            elseif t > intercept.intercept_time && intercept.is_kill
                % Keep KV visible at impact point if it was a successful kill
                c_pos = intercept.threat_intercept_pos;
                engaging_x(end+1) = c_pos(1); engaging_y(end+1) = c_pos(2); engaging_z(end+1) = c_pos(3);
            end
        end
    end
    
    if ~isempty(engaging_x)
        set(h_engaging, 'XData', engaging_x, 'YData', engaging_y, 'ZData', engaging_z);
    else
        set(h_engaging, 'XData', NaN, 'YData', NaN, 'ZData', NaN);
    end
    
    drawnow; pause(0.01); 
end
disp('Animation complete.');

% STREAMING_CHUNK: Generating static poster visual with fixed legends...
%% 6. Generate Static Poster Visual
disp('Generating 3D Static Visual...');
figure('Name', 'Static Engagement Summary', 'Color', 'w', 'Position', [950, 100, 900, 700]);
hold on; grid on; view(3);

surf(X * earth_radius_km, Y * earth_radius_km, Z * earth_radius_km, ...
    'EdgeColor', 'none', 'FaceColor', [0.7 0.8 0.9], 'FaceAlpha', 0.4, 'HandleVisibility', 'off');

% Plot threats cleanly for the legend
for tr = 1:num_threats
    t_mat = [threat_profiles{tr}.position];
    if tr == 1
        plot3(t_mat(1, 1), t_mat(2, 1), t_mat(3, 1), 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Launch Site (Silo Field)');
        plot3(t_mat(1, :), t_mat(2, :), t_mat(3, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'ICBM Trajectory');
    else
        plot3(t_mat(1, 1), t_mat(2, 1), t_mat(3, 1), 'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'HandleVisibility', 'off');
        plot3(t_mat(1, :), t_mat(2, :), t_mat(3, :), 'r-', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
end

all_sats = 1:total_sats;
fired_sats_idx = [];
if ~isempty(feasible_intercepts)
    fired_sats_idx = unique([feasible_intercepts.sat_idx]);
end
idle_sats_idx = setdiff(all_sats, [fired_sats_idx, failed_sats]);

% Plot constellation state
if ~isempty(idle_sats_idx)
    idle_pos = [release_constellation(idle_sats_idx).position];
    scatter3(idle_pos(1, :), idle_pos(2, :), idle_pos(3, :), 15, [0.6 0.6 0.6], 'filled', 'DisplayName', 'Idle Satellites');
end
if ~isempty(failed_sats)
    failed_pos = [release_constellation(failed_sats).position];
    scatter3(failed_pos(1, :), failed_pos(2, :), failed_pos(3, :), 40, 'r', 'x', 'LineWidth', 1.5, 'DisplayName', 'Offline/Destroyed');
end

% Plot Firing Carrier Buses & Transfer Paths with unique legend entries
if ~isempty(fired_sats_idx)
    fired_pos = [release_constellation(fired_sats_idx).position];
    scatter3(fired_pos(1, :), fired_pos(2, :), fired_pos(3, :), 40, 'c', 'filled', 'MarkerEdgeColor', 'b', 'DisplayName', 'Carrier Bus (Active)');
    
    added_hit = false;
    added_miss = false;
    
    for k = 1:length(feasible_intercepts)
        intercept = feasible_intercepts(k);
        
        if intercept.is_kill
            if ~added_hit
                plot3([intercept.sat_start_pos(1), intercept.threat_intercept_pos(1)], ...
                      [intercept.sat_start_pos(2), intercept.threat_intercept_pos(2)], ...
                      [intercept.sat_start_pos(3), intercept.threat_intercept_pos(3)], ...
                      'g--', 'LineWidth', 1.5, 'DisplayName', 'Successful KV Path');
                scatter3(intercept.threat_intercept_pos(1), intercept.threat_intercept_pos(2), intercept.threat_intercept_pos(3), ...
                         120, 'p', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y', 'DisplayName', 'Target Destroyed');
                added_hit = true;
            else
                plot3([intercept.sat_start_pos(1), intercept.threat_intercept_pos(1)], ...
                      [intercept.sat_start_pos(2), intercept.threat_intercept_pos(2)], ...
                      [intercept.sat_start_pos(3), intercept.threat_intercept_pos(3)], ...
                      'g--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
                scatter3(intercept.threat_intercept_pos(1), intercept.threat_intercept_pos(2), intercept.threat_intercept_pos(3), ...
                         120, 'p', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
            end
        else
            if ~added_miss
                plot3([intercept.sat_start_pos(1), intercept.threat_intercept_pos(1)], ...
                      [intercept.sat_start_pos(2), intercept.threat_intercept_pos(2)], ...
                      [intercept.sat_start_pos(3), intercept.threat_intercept_pos(3)], ...
                      'm:', 'LineWidth', 1.5, 'DisplayName', 'Missed Intercept (Pk Failure)');
                added_miss = true;
            else
                plot3([intercept.sat_start_pos(1), intercept.threat_intercept_pos(1)], ...
                      [intercept.sat_start_pos(2), intercept.threat_intercept_pos(2)], ...
                      [intercept.sat_start_pos(3), intercept.threat_intercept_pos(3)], ...
                      'm:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
            end
        end
    end
end

axis equal;
max_range = earth_radius_km + altitude_km + 1000;
xlim([-max_range max_range]); ylim([-max_range max_range]); zlim([-max_range max_range]);
title('Static Engagement Summary: Multi-Field Defense');
legend('Location', 'bestoutside');
set(gca, 'FontSize', 12);
end