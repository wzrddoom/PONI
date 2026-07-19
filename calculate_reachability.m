function required_delta_v = calculate_reachability(sat_pos, sat_vel, threat_pos, time_of_flight, mu_earth)
    % CALCULATE_REACHABILITY Solves Lambert's Problem to find the required Delta-V.
    % Uses a Universal Variable formulation to ensure robustness.
    %
    % Inputs:
    %   sat_pos        - Current interceptor ECI position [x;y;z] (km)
    %   sat_vel        - Current interceptor ECI velocity [x;y;z] (km/s)
    %   threat_pos     - Target intercept position [x;y;z] (km)
    %   time_of_flight - Time available to reach the target (seconds)
    %   mu_earth       - Gravitational parameter (km^3/s^2)
    %
    % Outputs:
    %   required_delta_v - The impulsive velocity change needed (km/s)
    
    r1 = norm(sat_pos);
    r2 = norm(threat_pos);
    
    % Cross product to find direction of motion
    cross_12 = cross(sat_pos, threat_pos);
    
    % Change in true anomaly
    dtheta = acos(dot(sat_pos, threat_pos) / (r1 * r2));
    
    % Assume prograde transfer
    if cross_12(3) < 0
        dtheta = 2 * pi - dtheta;
    end
    
    A = sin(dtheta) * sqrt((r1 * r2) / (1 - cos(dtheta)));
    
    % Bisection search limits for universal variable z
    z = 0; 
    z_up = 4 * pi^2;
    z_low = -4 * pi^2;
    
    tolerance = 1e-6;
    max_iter = 1000;
    iter = 0;
    
    % Iterative solver for Universal Variable 'z'
    while iter < max_iter
        iter = iter + 1; % Increment here to prevent infinite loops on continue
        
        [C, S] = stumpff_functions(z);
        
        y = r1 + r2 - A * (1 - z*S) / sqrt(C);
        
        if A > 0 && y < 0
            % Readjust boundaries if y is negative
            z_up = z;
            z = (z_up + z_low) / 2;
            continue;
        end
        
        chi = sqrt(y / C);
        t_calculated = (chi^3 * S + A * sqrt(y)) / sqrt(mu_earth);
        
        if abs(time_of_flight - t_calculated) < tolerance
            break;
        end
        
        if t_calculated <= time_of_flight
            z_low = z;
        else
            z_up = z;
        end
        
        z = (z_up + z_low) / 2;
    end
    
    % Calculate Lagrange coefficients
    f = 1 - y / r1;
    g = A * sqrt(y / mu_earth);
    
    % Calculate required transfer velocity vector at departure
    v_transfer = (threat_pos - f * sat_pos) / g;
    
    % Delta-V is the difference between current velocity and required transfer velocity
    required_delta_v = norm(v_transfer - sat_vel);
end

% --- Helper function for Stumpff functions ---
function [C, S] = stumpff_functions(z)
    if z > 0
        S = (sqrt(z) - sin(sqrt(z))) / (sqrt(z))^3;
        C = (1 - cos(sqrt(z))) / z;
    elseif z < 0
        S = (sinh(sqrt(-z)) - sqrt(-z)) / (sqrt(-z))^3;
        C = (cosh(sqrt(-z)) - 1) / (-z);
    else
        S = 1/6;
        C = 1/2;
    end
end