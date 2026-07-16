% Main_Simulation.m
% Space-Based Interceptor Feasibility Study

%% 1. Define the Constellation (Walker-Delta)
num_planes = 24;
sats_per_plane = 10;
altitude_km = 500;
inclination_deg = 53;
earth_radius_km = 6371;
mu_earth = 3.986004418e5; % Earth's standard gravitational parameter

% Generate initial state vectors for all interceptors
constellation = generate_constellation(num_planes, sats_per_plane, altitude_km, inclination_deg);

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
candidate_intercepts = [];
successful_engagements = 0;

for t = interceptor_release_time:time_step:boost_duration_s
    
    % Propagate orbits to current time step (simplified 2-body ECI coordinates)
    current_constellation = propagate_orbits(constellation, t, mu_earth);
    
    % Get threat position at this specific time step
    threat_pos = threat_profile(t).position; 
    
    % 5. Test Line of Sight & 6. Test Reachability
    for i = 1:length(current_constellation)
        sat_pos = current_constellation(i).position;
        
        % Check if Earth is blocking the view
        if check_line_of_sight(sat_pos, threat_pos, earth_radius_km)
            
            % Calculate required Delta-V to reach the threat position
            time_to_intercept = boost_duration_s - t;
            required_delta_v = calculate_reachability(sat_pos, threat_pos, time_to_intercept, mu_earth);
            
            % Compare against interceptor limits
            if required_delta_v < current_constellation(i).max_delta_v
                successful_engagements = successful_engagements + 1;
                % Log the successful intercept geometry for metrics
            end
        end
    end
end

%% 7 & 8. Calculate Metrics and Sensitivity
% Output access probability, time margins, and gaps
disp(['Total feasible intercepts during boost phase: ', num2str(successful_engagements)]);