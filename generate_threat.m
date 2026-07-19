function threat_profile = generate_threat(launch_lat, launch_lon, heading_deg)
% GENERATE_THREAT Creates a modern solid-fuel ICBM boost trajectory.
%
% Optional Inputs (allows for simulating concentrated silo fields):
%   launch_lat  - Latitude of launch site (degrees)
%   launch_lon  - Longitude of launch site (degrees)
%   heading_deg - Launch azimuth (degrees)

% Randomise launch location and heading if not provided
if nargin < 3
    launch_lat = (rand() * 180) - 90;    % -90 to 90 degrees
    launch_lon = (rand() * 360) - 180;   % -180 to 180 degrees
    heading_deg = rand() * 360;          % 0 to 360 degrees
end

% Modern solid-fuel ICBM characteristics (e.g., Minuteman III, RS-24)
boost_duration_s = round(150 + rand() * 30); % 150 to 180 seconds
burnout_alt_km = 200 + rand() * 150;         % 200 to 350 kilometres
burnout_range_km = 600 + rand() * 300;       % 600 to 900 kilometres

earth_radius_km = 6371;
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
    
    % 1. Kinematic shape of the trajectory (Staged Acceleration Approximation)
    h = burnout_alt_km * (t / boost_duration_s)^2.5;
    s = burnout_range_km * (t / boost_duration_s)^3.5;
    
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
    theta_g = earth_rotation_rate * t;
    
    x_eci = x_ecef * cos(theta_g) - y_ecef * sin(theta_g);
    y_eci = x_ecef * sin(theta_g) + y_ecef * cos(theta_g);
    z_eci = z_ecef; 
    
    % Store the position
    pos_eci(:, i) = [x_eci; y_eci; z_eci];
    
    threat_profile(i).time = t;
    threat_profile(i).position = pos_eci(:, i);
end

% 5. Calculate Velocity via Central Difference
for i = 1:num_steps
    if i == 1
        vel = (pos_eci(:, i+1) - pos_eci(:, i)) / 1; 
    elseif i == num_steps
        vel = (pos_eci(:, i) - pos_eci(:, i-1)) / 1;
    else
        vel = (pos_eci(:, i+1) - pos_eci(:, i-1)) / 2;
    end
    
    threat_profile(i).velocity = vel;
end


end