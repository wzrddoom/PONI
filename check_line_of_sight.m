function is_visible = check_line_of_sight(sat_pos, threat_pos, earth_radius_km)
% CHECK_LINE_OF_SIGHT Determines if the Earth obstructs the view between interceptor and threat.
%
% Inputs:
%   sat_pos         - Interceptor position vector [x; y; z] in km
%   threat_pos      - Missile/threat position vector [x; y; z] in km
%   earth_radius_km - Radius of the Earth in km
%
% Outputs:
%   is_visible      - Boolean: true if path is clear, false if obstructed

% Add a buffer to account for the atmosphere (e.g., 100 km)
keep_out_radius = earth_radius_km + 20;

% Define the line segment S (Satellite) to T (Threat)
% Parametric equation of the line: P(t) = S + t*(T - S)
S = sat_pos;
D = threat_pos - sat_pos;

% We are finding the intersection of the line with a sphere of keep_out_radius
% Equation: |P(t)|^2 = R^2  -> (S + tD).(S + tD) = R^2
% Quadratic: (D.D)t^2 + 2(S.D)t + (S.S - R^2) = 0

a = dot(D, D);
b = 2 * dot(S, D);
c = dot(S, S) - keep_out_radius^2;

discriminant = b^2 - 4*a*c;

% If the discriminant is less than zero, the line completely misses the Earth
if discriminant < 0
    is_visible = true;
    return;
end

% Calculate the points of intersection (t1 and t2)
t1 = (-b - sqrt(discriminant)) / (2*a);
t2 = (-b + sqrt(discriminant)) / (2*a);

% If either intersection point falls between 0 and 1, the Earth blocks the line segment
if (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)
    is_visible = false;
else
    % The line intersects the Earth's mathematical sphere, but not strictly 
    % between the satellite and the threat (i.e., it intersects behind them)
    is_visible = true;
end


end