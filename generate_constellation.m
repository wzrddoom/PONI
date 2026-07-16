function constellation = generate_constellation(num_planes, sats_per_plane, altitude_km, inclination_deg, max_delta_v)
    % GENERATE_CONSTELLATION Creates a Walker-Delta orbital constellation.
    % 
    % Inputs:
    %   num_planes      - Number of orbital planes
    %   sats_per_plane  - Number of interceptors per plane
    %   altitude_km     - Orbital altitude in kilometres
    %   inclination_deg - Orbital inclination in degrees
    %   max_delta_v     - Maximum delta-v capacity of the interceptors (km/s)
    %
    % Outputs:
    %   constellation   - Struct array containing the initial Keplerian elements
    
    earth_radius_km = 6371;
    semi_major_axis = earth_radius_km + altitude_km;
    inclination_rad = deg2rad(inclination_deg);
    
    total_sats = num_planes * sats_per_plane;
    
    % Preallocate struct array for speed
    constellation(total_sats) = struct('a', [], 'inc', [], 'RAAN', [], 'theta_0', [], 'max_delta_v', []);
    
    sat_index = 1;
    
    % Walker-Delta phasing factor (typically 1 for standard uniform distribution)
    phasing_factor = 1; 
    phase_shift = 2 * pi / total_sats * phasing_factor;
    
    for i = 1:num_planes
        % Distribute Right Ascension of the Ascending Node (RAAN) evenly across 360 degrees
        raan = (i - 1) * (2 * pi / num_planes);
        
        for j = 1:sats_per_plane
            % Distribute True Anomaly evenly within the plane, adding the phase shift
            true_anomaly = (j - 1) * (2 * pi / sats_per_plane) + (i - 1) * phase_shift;
            
            % Store the orbital elements
            constellation(sat_index).a = semi_major_axis;
            constellation(sat_index).inc = inclination_rad;
            constellation(sat_index).RAAN = raan;
            constellation(sat_index).theta_0 = true_anomaly;
            constellation(sat_index).max_delta_v = max_delta_v;
            
            sat_index = sat_index + 1;
        end
    end
end