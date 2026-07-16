function threat_profile = generate_threat(launch_lat, launch_lon, boost_duration_s)
    % GENERATE_THREAT Creates a generic ballistic missile boost trajectory.
    % Models a simplified launch profile rotated into the ECI frame.
    %
    % Inputs:
    %   launch_lat       - Launch latitude in degrees
    %   launch_lon       - Launch longitude in degrees
    %   boost_duration_s - Total time of powered flight in seconds
    %
    % Outputs:
    %   threat_profile   - Struct array containing position and velocity vectors
    %                      at 1-second intervals in the ECI frame.
    
    % Generic ICBM parametres
    earth_radius_km = 6371;
    burnout_alt_km = 300;       % Typical burnout altitude
    burnout_range_km = 600;     % Typical downrange distance covered during boost
    heading_deg = 45;           % Generic launch heading (e.g., North-East)
    earth_rotation_rate = 7.2921159e-5; % Radians per second
    
    % Preallocate the output structure for performance
    time_steps = 0:1:boost_duration_s;
    num_steps = length(time_steps);
    threat_profile(num_steps) = struct('time', [], 'position', [], 'velocity', []);
    
    % Convert inputs to radians
    lat1 = deg2rad(launch_lat);
    lon1 = deg2rad(launch_lon);
    heading = deg2rad(heading_deg);
    
    % Arrays to hold ECI positions for velocity calculations
    pos_eci = zeros(3, num_steps);
    
    for i = 1:num_steps
        t = time_steps(i);
        
        % 1. Kinematic shape of the trajectory
        % Altitude increases quadratically (accelerating upwards)
        h = burnout_alt_km * (t / boost_duration_s)^2;
        
        % Downrange distance increases cubically (starts slow, pitches over)
        s = burnout_range_km * (t / boost_duration_s)^3;
        
        % Angular distance covered on the Earth's surface
        delta = s / earth_radius_km;
        
        % 2. Great Circle calculations for current Latitude and Longitude
        lat2 = asin(sin(lat1)*cos(delta) + cos(lat1)*sin(delta)*cos(heading));
        lon2 = lon1 + atan2(sin(heading)*sin(delta)*cos(lat1), cos(delta) - sin(lat1)*sin(lat2));
        
        % 3. Convert to Earth-Centred Earth-Fixed (ECEF) Cartesian coordinates
        r = earth_radius_km + h;
        x_ecef = r * cos(lat2) * cos(lon2);
        y_ecef = r * cos(lat2) * sin(lon2);
        z_ecef = r * sin(lat2);
        
        % 4. Rotate to Earth-Centred Inertial (ECI) frame
        % Assuming the ECEF and ECI frames align exactly at t = 0
        theta_g = earth_rotation_rate * t;
        
        x_eci = x_ecef * cos(theta_g) - y_ecef * sin(theta_g);
        y_eci = x_ecef * sin(theta_g) + y_ecef * cos(theta_g);
        z_eci = z_ecef; % Z-axis is shared between ECEF and ECI
        
        % Store the position
        pos_eci(:, i) = [x_eci; y_eci; z_eci];
        
        threat_profile(i).time = t;
        threat_profile(i).position = pos_eci(:, i);
    end
    
    % 5. Calculate Velocity via Central Difference
    for i = 1:num_steps
        if i == 1
            % Forward difference for the first step (launch pad)
            vel = (pos_eci(:, i+1) - pos_eci(:, i)) / 1; 
        elseif i == num_steps
            % Backward difference for the final step (burnout)
            vel = (pos_eci(:, i) - pos_eci(:, i-1)) / 1;
        else
            % Central difference for the remaining flight path
            vel = (pos_eci(:, i+1) - pos_eci(:, i-1)) / 2;
        end
        
        threat_profile(i).velocity = vel;
    end
end