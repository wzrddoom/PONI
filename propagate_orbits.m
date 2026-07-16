function current_constellation = propagate_orbits(constellation, time_s, mu_earth)
    % PROPAGATE_ORBITS Updates the positions and velocities of the constellation.
    % Uses simplified circular two-body mechanics.
    %
    % Inputs:
    %   constellation - Initial constellation struct array
    %   time_s        - Current simulation time in seconds
    %   mu_earth      - Earth's standard gravitational parameter (km^3/s^2)
    %
    % Outputs:
    %   current_constellation - Constellation struct updated with ECI Cartesian vectors
    
    current_constellation = constellation;
    
    for i = 1:length(constellation)
        a = constellation(i).a;
        inc = constellation(i).inc;
        RAAN = constellation(i).RAAN;
        theta_0 = constellation(i).theta_0;
        
        % Calculate Mean Motion (n) in radians per second
        n = sqrt(mu_earth / a^3);
        
        % Update true anomaly for circular orbit (Mean Anomaly = True Anomaly)
        theta_current = theta_0 + n * time_s;
        
        % Position and velocity in the orbital plane (Perifocal coordinate system)
        r_pqw = [a * cos(theta_current); a * sin(theta_current); 0];
        v_pqw = [-a * n * sin(theta_current); a * n * cos(theta_current); 0];
        
        % Rotation matrices to transform from orbital plane to ECI frame
        % R3(-RAAN) * R1(-inc) * R3(-omega) -- note: omega is 0 for circular
        R_raan = [cos(RAAN), -sin(RAAN), 0; 
                  sin(RAAN),  cos(RAAN), 0; 
                  0,          0,         1];
              
        R_inc = [1, 0,         0; 
                 0, cos(inc), -sin(inc); 
                 0, sin(inc),  cos(inc)];
             
        Transformation_Matrix = R_raan * R_inc;
        
        % Convert to Earth-Centred Inertial (ECI) coordinates
        current_constellation(i).position = Transformation_Matrix * r_pqw;
        current_constellation(i).velocity = Transformation_Matrix * v_pqw;
    end
end