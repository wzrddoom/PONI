% Corrected Method of Characteristics (MoC) Bell Nozzle Contour
% Fixes the matrix initialization and wave-reflection indexing logic.

clear; clc; close all;

%% 1. Input Parameters
gamma = 1.18;               % Specific heat ratio
Me = 3.95;                  % Target exit Mach number
Rt = 31.45;                 % Throat radius in mm 
N = 15;                     % Number of characteristic lines (resolution)

%% 2. Prandtl-Meyer Function
nu_func = @(M) sqrt((gamma+1)/(gamma-1)) * ...
    atan(sqrt((gamma-1)/(gamma+1)*(M.^2-1))) - atan(sqrt(M.^2-1));

nu_e = nu_func(Me);
theta_max = nu_e / 2;

%% 3. Discretise the Throat Expansion Fan
theta_fan = linspace(0.01, theta_max, N); 
nu_fan = theta_fan; 
mu_fan = zeros(1, N);

% Pre-calculate throat Mach angles for the fan
for k = 1:N
    M_fan = fzero(@(M) nu_func(M) - nu_fan(k), [1.001, 10]);
    mu_fan(k) = asin(1 / M_fan);
end

%% 4. Mesh Generation Matrices
X = zeros(N, N); Y = zeros(N, N);
Theta = zeros(N, N); Nu = zeros(N, N); Mach = zeros(N, N);

%% 5. Solve the Internal Flow Field
for i = 1:N
    for j = i:N % Strictly upper triangular domain
        
        if i == 1 
            % DOMAIN A: Points hitting the axis of symmetry
            Theta(i,j) = 0;
            Nu(i,j) = theta_fan(j) + nu_fan(j);
            Mach(i,j) = fzero(@(M) nu_func(M) - Nu(i,j), [1.001, 10]);
            Y(i,j) = 0;
            
            % Intersection of straight line from throat (0, Rt) to axis
            m_minus = tan(theta_fan(j) - mu_fan(j));
            X(i,j) = -Rt / m_minus;
            
        elseif i == j 
            % DOMAIN B: Diagonal points (intersecting waves from throat and axis)
            J_plus = Theta(i-1, i) - Nu(i-1, i);
            J_minus = theta_fan(i) + nu_fan(i);
            
            Theta(i,i) = (J_minus + J_plus) / 2;
            Nu(i,i)   = (J_minus - J_plus) / 2;
            Mach(i,i) = fzero(@(M) nu_func(M) - Nu(i,i), [1.001, 10]);
            
            s_plus  = tan(Theta(i-1, i) + asin(1/Mach(i-1, i)));
            s_minus = tan(theta_fan(i) - mu_fan(i));
            
            % Intersection of line from previous internal node and the throat
            X(i,i) = (Rt - Y(i-1, i) + s_plus * X(i-1, i)) / (s_plus - s_minus);
            Y(i,i) = Rt + s_minus * X(i,i);
            
        else 
            % DOMAIN C: Internal Riemann mesh points
            J_plus  = Theta(i-1, j) - Nu(i-1, j);
            J_minus = Theta(i, j-1) + Nu(i, j-1);
            
            Theta(i,j) = (J_minus + J_plus) / 2;
            Nu(i,j)   = (J_minus - J_plus) / 2;
            Mach(i,j) = fzero(@(M) nu_func(M) - Nu(i,j), [1.001, 10]);
            
            s_plus  = tan(Theta(i-1, j) + asin(1/Mach(i-1, j)));
            s_minus = tan(Theta(i, j-1) - asin(1/Mach(i, j-1)));
            
            X(i,j) = (Y(i-1, j) - Y(i, j-1) + s_plus * X(i-1, j) - s_minus * X(i, j-1)) / (s_plus - s_minus);
            Y(i,j) = Y(i, j-1) + s_minus * (X(i,j) - X(i, j-1));
        end
    end
end

%% 6. Calculate Wall Contour Coordinates
X_wall = zeros(1, N+1); Y_wall = zeros(1, N+1);
X_wall(1) = 0; Y_wall(1) = Rt;
current_wall_angle = theta_max;

for j = 1:N
    % Average flow angle to smoothly curve the wall
    s_wall = tan((current_wall_angle + Theta(j,j)) / 2);
    s_char = tan(Theta(j,j) + asin(1/Mach(j,j)));
    
    X_wall(j+1) = (Y(j,j) - Y_wall(j) + s_wall * X_wall(j) - s_char * X(j,j)) / (s_wall - s_char);
    Y_wall(j+1) = Y_wall(j) + s_wall * (X_wall(j+1) - X_wall(j));
    
    current_wall_angle = Theta(j,j); 
end

%% 7. Visualisation
figure('Name', 'Corrected MoC Nozzle Contour', 'Color', 'w');
hold on; grid on; axis equal;

for i = 1:N
    for j = i:N
        plot(X(i,j), Y(i,j), 'k.', 'MarkerSize', 5);
    end
end

plot(X_wall, Y_wall, 'r-', 'LineWidth', 2);
plot([0 max(X_wall)], [0 0], 'b-.', 'LineWidth', 1.5); 

xlabel('Axial Position X (mm)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Radial Position Y (mm)', 'FontSize', 12, 'FontWeight', 'bold');
title('Stage 1 Bell Nozzle Contour (MoC)', 'FontSize', 14);
legend('Mesh Nodes', 'Wall Contour', 'Centreline', 'Location', 'Best');

%% 8. Export to CAD
cad_data = [zeros(length(X_wall), 1), Y_wall', X_wall'];
csvwrite('stage1_nozzle_contour.csv', cad_data);
disp('Contour successfully saved to stage1_nozzle_contour.csv');
