% Method of Characteristics (MoC) Bell Nozzle Contour Generator
% Generates 2D wall coordinates and exports to CSV for CAD integration.

clear; clc; close all;

%% 1. Input Parameters
% Using Stage 1 properties optimised for 3000 m altitude
gamma = 1.18;               % Specific heat ratio
Me = 3.95;                  % Target exit Mach number
Rt = 31.45;                 % Throat radius in mm (Target At = 0.00311 m^2)
N = 15;                     % Number of characteristic lines (resolution)

%% 2. Prandtl-Meyer Function Definition
% Anonymous function for PM angle (in radians) given Mach number
nu_func = @(M) sqrt((gamma+1)/(gamma-1)) * ...
    atan(sqrt((gamma-1)/(gamma+1)*(M.^2-1))) - atan(sqrt(M.^2-1));

% Calculate total required expansion angle to reach Me
nu_e = nu_func(Me);

% Maximum turning angle at the throat for a minimum length nozzle
theta_max = nu_e / 2;

%% 3. Discretise the Expansion Fan
% Create an array of N characteristic lines originating from the throat tip
% Each line has a specific flow angle (theta) and PM angle (nu)
theta_fan = linspace(0.01, theta_max, N); 
nu_fan = theta_fan; % For the initial expansion fan, nu = theta

%% 4. Mesh Generation Matrices
% Preallocate coordinate and property arrays
X = zeros(N, N);
Y = zeros(N, N);
Theta = zeros(N, N);
Nu = zeros(N, N);
Mach = zeros(N, N);

%% 5. Solve the Internal Flow Field
for i = 1:N
    for j = 1:N
        if i == 1 % Points along the axis of symmetry (y = 0)
            Theta(i,j) = 0;
            Nu(i,j) = nu_fan(j) + theta_fan(j);
            Y(i,j) = 0;
            
            % Initial point X-coordinate calculation
            Mach(i,j) = fzero(@(M) nu_func(M) - Nu(i,j), [1.001, 10]);
            mu = asin(1 / Mach(i,j));
            
            if j == 1
                X(i,j) = Rt / tan(theta_fan(j) - mu);
            else
                % Intersect with previous right-running characteristic
                m_minus = tan(Theta(i,j-1) - asin(1/Mach(i,j-1)));
                X(i,j) = X(i,j-1) - Y(i,j-1) / m_minus;
            end
            
        else % Internal mesh points
            if j >= i
                % Riemann invariant intersections
                Theta(i,j) = ( (Nu(i-1,j) + Theta(i-1,j)) - (Nu(i,j-1) - Theta(i,j-1)) ) / 2;
                Nu(i,j) = ( (Nu(i-1,j) + Theta(i-1,j)) + (Nu(i,j-1) - Theta(i,j-1)) ) / 2;
                
                Mach(i,j) = fzero(@(M) nu_func(M) - Nu(i,j), [1.001, 10]);
                mu = asin(1 / Mach(i,j));
                
                % Slopes for intersection calculation
                s_plus = tan(Theta(i,j-1) + asin(1/Mach(i,j-1)));
                s_minus = tan(Theta(i-1,j) - asin(1/Mach(i-1,j)));
                
                % Calculate X and Y physical coordinates
                X(i,j) = (Y(i-1,j) - Y(i,j-1) + s_plus * X(i,j-1) - s_minus * X(i-1,j)) / (s_plus - s_minus);
                Y(i,j) = Y(i,j-1) + s_plus * (X(i,j) - X(i,j-1));
            end
        end
    end
end

%% 6. Calculate Wall Contour Coordinates
% Preallocate wall arrays (starting point is the throat edge)
X_wall = zeros(1, N+1);
Y_wall = zeros(1, N+1);
X_wall(1) = 0;
Y_wall(1) = Rt;

for j = 1:N
    % Average slope between the wall and the closest internal mesh point
    theta_wall = Theta(j,j);
    if j == 1
        s_wall = tan((theta_max + theta_wall) / 2);
    else
        s_wall = tan((Theta(j-1,j-1) + theta_wall) / 2);
    end
    
    % Slope of the right-running characteristic hitting the wall
    s_char = tan(Theta(j,j) + asin(1/Mach(j,j)));
    
    % Intersection to find the wall point
    X_wall(j+1) = (Y(j,j) - Y_wall(j) + s_wall * X_wall(j) - s_char * X(j,j)) / (s_wall - s_char);
    Y_wall(j+1) = Y_wall(j) + s_wall * (X_wall(j+1) - X_wall(j));
end

%% 7. Visualisation
figure('Name', 'Method of Characteristics Nozzle Contour', 'Color', 'w');
hold on; grid on; axis equal;

% Plot characteristic mesh
for i = 1:N
    for j = i:N
        plot(X(i,j), Y(i,j), 'k.', 'MarkerSize', 5);
    end
end

% Plot final wall contour
plot(X_wall, Y_wall, 'r-', 'LineWidth', 2);
plot([0 max(X_wall)], [0 0], 'b-.', 'LineWidth', 1.5); % Centreline

xlabel('Axial Position X (mm)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Radial Position Y (mm)', 'FontSize', 12, 'FontWeight', 'bold');
title('Stage 1 Bell Nozzle Contour (MoC)', 'FontSize', 14);
legend('Mesh Nodes', 'Wall Contour', 'Centreline', 'Location', 'Best');

%% 8. Export to CAD
% Format as Z, Y, X for standard CAD import (axial direction along Z)
cad_data = [zeros(length(X_wall), 1), Y_wall', X_wall'];
csvwrite('stage1_nozzle_contour.csv', cad_data);
disp('Contour successfully saved to stage1_nozzle_contour.csv');
