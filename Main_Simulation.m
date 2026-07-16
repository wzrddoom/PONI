% Main_Simulation.m
% Space-Based Interceptor Feasibility Study

%% 1. Define the Constellation (Walker-Delta)
num_planes = 4;
sats_per_plane = 10;
altitude_km = 500;
inclination_deg = 53;
max_delta_v_km_s = 6.0; % Added maximum delta-v budget in kilometres per second
earth_radius_km = 6371;
mu_earth = 3.986004418e5; % Earth's standard gravitational parameter

% Generate initial state vectors for all interceptors
constellation = generate_constellation(num_planes, sats_per_plane, altitude_km, inclination_deg, max_delta_v_km_s);

%% 2. Generate the Threat Trajectory
% Define launch coordinates, heading, and boost duration
launch_lat = 55.75; % e.g., Plesetsk or generic mid-latitude
launch_lon = 37.61;
boost_duration_s = 180; 

threat_profile = generate_threat(launch_lat, launch_lon, boost_duration_s);

%% 3. Apply the Response Timeline
detection_latency_s = 15;
decision_latency_s = 30;
total_latency_s = detection_latency_s + decision_latency_s;

% The time at which interceptors can actually begin their burn
interceptor_release_time = total_latency_s; 

%% 4. Time-Stepped Engagement Loop
time_step = 1; % 1-second resolution
successful_engagements = 0;

% Variables to store the trajectory of the first successful intercept for visualisation
best_interceptor_start = [];
best_interceptor_end = [];

% We start checking from the moment interceptors are released until burnout
for t = interceptor_release_time:time_step:boost_duration_s
    
    % Propagate orbits to current time step (simplified 2-body ECI coordinates)
    current_constellation = propagate_orbits(constellation, t, mu_earth);
    
    % Get threat position at this specific time step 
    % (Adding 1 because MATLAB uses 1-based indexing and time starts at 0)
    threat_index = t + 1; 
    threat_pos = threat_profile(threat_index).position; 
    
    % 5. Test Line of Sight & 6. Test Reachability
    for i = 1:length(current_constellation)
        sat_pos = current_constellation(i).position;
        sat_vel = current_constellation(i).velocity; % Needed for reachability math
        
        % Check if Earth is blocking the view
        if check_line_of_sight(sat_pos, threat_pos, earth_radius_km)
            
            % Calculate required Delta-V to reach the threat position
            time_to_intercept = boost_duration_s - t;
            
            % Only calculate if we have time left
            if time_to_intercept > 0
                required_delta_v = calculate_reachability(sat_pos, sat_vel, threat_pos, time_to_intercept, mu_earth);
                
                % Compare against interceptor limits
                if required_delta_v < current_constellation(i).max_delta_v
                    successful_engagements = successful_engagements + 1;
                    
                    % Log the successful intercept geometry for plotting
                    if isempty(best_interceptor_start)
                        best_interceptor_start = sat_pos;
                        best_interceptor_end = threat_pos;
                    end
                end
            end
        end
    end
end

%% 7 & 8. Calculate Metrics and Sensitivity
% Output access probability, time margins, and gaps
disp(['Total kinematically feasible intercept solutions during boost phase: ', num2str(successful_engagements)]);

%% 9. Visualisation
figure('Name', 'Interceptor Engagement Visualisation', 'Color', 'w');
hold on; grid on; view(3);

% Plot the Earth
[X, Y, Z] = sphere(50);
surf(X * earth_radius_km, Y * earth_radius_km, Z * earth_radius_km, ...
    'EdgeColor', 'none', 'FaceColor', [0.1 0.4 0.7], 'FaceAlpha', 0.8);

% Plot the Threat Trajectory
threat_pos_matrix = [threat_profile.position];
plot3(threat_pos_matrix(1,:), threat_pos_matrix(2,:), threat_pos_matrix(3,:), ...
    'r-', 'LineWidth', 2.5, 'DisplayName', 'Threat Trajectory');

% Mark Launch and Burnout
plot3(threat_pos_matrix(1,1), threat_pos_matrix(2,1), threat_pos_matrix(3,1), ...
    'r^', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Launch Site');
plot3(threat_pos_matrix(1,end), threat_pos_matrix(2,end), threat_pos_matrix(3,end), ...
    'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Burnout');

% Plot the Constellation
% Plot a snapshot of the satellites at the release time
snapshot_constellation = propagate_orbits(constellation, interceptor_release_time, mu_earth);
sat_pos_matrix = [snapshot_constellation.position];
scatter3(sat_pos_matrix(1,:), sat_pos_matrix(2,:), sat_pos_matrix(3,:), ...
    20, 'k', 'filled', 'DisplayName', 'Interceptors');

% Plot Interceptor Trajectory
% Draw a green line from the successful interceptor to the threat
if ~isempty(best_interceptor_start)
    plot3([best_interceptor_start(1), best_interceptor_end(1)], ...
          [best_interceptor_start(2), best_interceptor_end(2)], ...
          [best_interceptor_start(3), best_interceptor_end(3)], ...
          'g--', 'LineWidth', 2, 'DisplayName', 'Kinematic Intercept Path');
      
    plot3(best_interceptor_start(1), best_interceptor_start(2), best_interceptor_start(3), ...
        'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'Firing Interceptor');
end

% Formatting
axis equal;
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('Space-Based Interceptor Constellation and Threat Engagement');
legend('Location', 'bestoutside');