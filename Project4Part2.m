%% Project 4 Part 2
clear all; close all; clc;
%% Initialize Robot
%% Robot Parameters
% Setup
sympref('FloatingPointOutput', true) % This allows floating point coefficient
format short
syms L1 L2 t1 t2

% DH parameters for robot
alpha = [0  0   0]; % Create a symbolic representation of a constant
a   =   [0  L1  L2];
d   =   [0  0   0];
theta = [t1 t2  0];

% Define the robot with Robot Toolbox (MUST BE THERE FRAMES HERE)
L(1) = Link('revolute','d',0,'a',0,'alpha',0,'modified'); % base frame
L(2) = Link('revolute','d',0,'a',L1,'alpha',0,'modified'); % frame 1
L(3) = Link('revolute','d',0,'a',L2,'alpha',0,'modified'); % frame 2
Robot = SerialLink(L, 'name', '2R SCARA Arm') % Combine Link objects together to form a Robot Object
%% FK Calculations
[T, Tfinal] = FK(alpha, a, d, theta);
%% Jacobian Calculation Explicit Method
% Forward Kinematics - Get T matrices from T structure
T01 = T(1).matrix;
T12 = T(2).matrix;
T23 = T(3).matrix;
T02 = simplify(T01 * T12);
T03 = simplify(T01 * T12 * T23);

[R01,P01] = tr2rt(T01);
[R02,P02] = tr2rt(T02);
[R03,P03] = tr2rt(T03);

Pe = P03;

% Direct Differentiation
% 3 col of the transformation matrix
Z01 = T01(1:3, 3); % z-axis of frame 1 (Joint 1 axis)
Z02 = T02(1:3, 3); % z-axis of frame 2 (Joint 2 axis)
% Z03 = T03(1:3, 3); % z-axis of frame 3 (Joint 3 axis)

J1 = [cross(Z01,(Pe-P01)); Z01]; % Revolute Joint Equation
J2 = [cross(Z02,(Pe-P02)); Z02]; 
J_DD = simplify([J1, J2])

%% Brute Force Approach
% row = 1; col = 1;
goal = zeros(50);
cm1 = cell(50,50,15,11);
N = 10;
l = linspace(10,100,N);
alpha = linspace(0,pi/2,N);

for l1 = 10:10:500
    for l2 = 10:10:500
        % We want the selection of l1 and l2 to be within the workspace
        if (l1 + l2 > sqrt((200+140)^2 + 50^2)) && (abs(l1 - l2) < 200)% Fill this out
            % Calculate the goal at this configurtaion using calGoal function. Store that into a matrix, Also store K 
            [cm1{l1/10, l2/10}, goal(l1/10, l2/10)] = calGoal(l1,l2);
        end
    end
end

%% Plot Goal Function
figure (1)

hold off;
[L2_grid,L1_grid] = meshgrid(10:10:500,10:10:500);
surf(L2_grid, L1_grid, goal);

xlabel('L1 Length (mm)'); % X-axis label
ylabel('L2 Length (mm)'); % Y-axis label
zlabel('Goal Function Value'); % Z-axis label
title('3D Surface Plot of Goal Function');
xlim([0, 500]);
ylim([0, 500]);

%colormap jet; % Apply a color scheme
colorbar; % Add a color scale

% Find the maximum value and its index
[maxValue, maxIndex] = max(goal(:)); % Find the max value in the goal matrix

% Convert linear index to (row, col) indices
[row, col] = ind2sub(size(goal), maxIndex);

% Get corresponding L1 and L2 values
L1_max = L1_grid(row, col);
L2_max = L2_grid(row, col);

% Display the max value and location
disp(['Max Goal Value: ', num2str(maxValue)]);
disp(['L1 at Max: ', num2str(L1_max), ' mm']);
disp(['L2 at Max: ', num2str(L2_max), ' mm']);

%% Eigens (Best Scenario)
L1_max = 300;
L2_max = 210;

% Define workspace grid
[Px_grid, Py_grid] = meshgrid(linspace(200, 340, 15), linspace(-50, 50, 11));

% Initialize ki matrix
ki_values = zeros(size(Px_grid));

% Compute ki for each workspace point
for i = 1:size(Px_grid, 1)
    for j = 1:size(Px_grid, 2)
        Px = Px_grid(i, j);
        Py = Py_grid(i, j);
        
        % Solve inverse kinematics
        [t1, t2] = IK(Px, Py, L1_max, L2_max);
        
        % Check for valid solutions
        if isnan(t1) || isnan(t2)
            ki_values(i, j) = NaN;  % Mark unreachable points
        else
            % Compute Jacobian at this configuration
            J = calJ(L1_max, L2_max, t1, t2);
            
            % Compute JJ
            JJt = J*J.';
            I = JJt(1:2,1:2);
            
            % Compute eigenvalues
            [V, D] = eig(I);
            eigenvalues = real(diag(D));  % Extract only real parts
            %disp(JJt);
            %disp(eigenvalues);
            
            % Ensure eigenvalues are non-negative
            %eigenvalues(eigenvalues < 0) = 0;
            
            % Get min and max eigenvalues
            lambda_min = min(eigenvalues);
            lambda_max = max(eigenvalues);

            %disp(['Px:', num2str(Px), ', Py:', num2str(Py)]);

            
            % Compute ki safely
            if lambda_max > 1e-30  % Avoid division by zero
                ki_values(i, j) = sqrt(lambda_min / lambda_max);
            else
                ki_values(i, j) = 0; 
            end
            %disp(ki_values(i,j));
        end
    end
end

% Ensure ki_values are real
ki_values = real(ki_values);

% Plot ki values over the workspace
figure;
surf(Px_grid, Py_grid, ki_values);
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
zlabel('Manipulability Index k_i');
title('Manipulability Index k_i Across Workspace (Max Optimization)');
colorbar;

%% Eigens (Worst Scenario)
L1_min = 210;
L2_min = 300;

% Define workspace grid
[Px_grid, Py_grid] = meshgrid(linspace(200, 340, 15), linspace(-50, 50, 11));

% Initialize ki matrix
ki_values = zeros(size(Px_grid));

% Compute ki for each workspace point
for i = 1:size(Px_grid, 1)
    for j = 1:size(Px_grid, 2)
        Px = Px_grid(i, j);
        Py = Py_grid(i, j);
        
        % Solve inverse kinematics
        [t1, t2] = IK(Px, Py, L1_min, L2_min);
        
        % Check for valid solutions
        if isnan(t1) || isnan(t2)
            ki_values(i, j) = NaN;  % Mark unreachable points
        else
            % Compute Jacobian at this configuration
            J = calJ(L1_min, L2_min, t1, t2);
            
            % Compute JJ
            JJt = J*J.';
            I = JJt(1:2,1:2);
            
            % Compute eigenvalues
            [V, D] = eig(I);
            eigenvalues = real(diag(D));  % Extract only real parts
            %disp(JJt);
            %disp(eigenvalues);
            
            % Ensure eigenvalues are non-negative
            %eigenvalues(eigenvalues < 0) = 0;
            
            % Get min and max eigenvalues
            lambda_min = min(eigenvalues);
            lambda_max = max(eigenvalues);

            %disp(['Px:', num2str(Px), ', Py:', num2str(Py)]);

            
            % Compute ki safely
            if lambda_max > 1e-30  % Avoid division by zero
                ki_values(i, j) = sqrt(lambda_min / lambda_max);
            else
                ki_values(i, j) = 0; 
            end
            %disp(ki_values(i,j));
        end
    end
end

% Ensure ki_values are real
ki_values = real(ki_values);

% Plot ki values over the workspace
figure;
surf(Px_grid, Py_grid, ki_values);
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
zlabel('Manipulability Index k_i');
title('Manipulability Index k_i Across Workspace (Worst Optimization)');
colorbar;

%%

function J = calJ(L1,L2,T1,T2)
    % This function will be used to calculate the
    % Jacobian Matrix using given input

    J = [   -L2*sin(T1+T2)-L1*sin(T1), -L2*sin(T1+T2);
            L2*cos(T1+T2)+L1*cos(T1),   L2*cos(T1+T2);
            0,                          0;
            0,                          0;
            0,                          0;
            1,                          1;];
end

function [K,C] = calGoal(l1,l2)
    % This function will calculate the goal function C
    % Define and initialize parameters
    L3 = l1^3 + l2^3;
    ki_sum = 0;
    K = [];
    % Initialize kmin to be very large such that we 
    % Can keep updating this values as long as there
    % Is a smaller value pops out
    ki_min = 20000; 
    % We want to cover all workspace in x and y direction
    for Px = linspace(200,340,15)
        for Py = linspace(-50,50,11)
            % Call our IK solution function to solve inverse kinematic
            [t1, t2] = IK(Px, Py, l1, l2);
            % Calculate Jacobian with our IK solution angles
            J = calJ(l1,l2,t1,t2);
            % We then need to calculate the eigen values for JJ'
            JJt = J*J';
            % Use Matlab function: eig
            [V,D] = eig(JJt);
            % Extract all eigen valeus from the matrix
            D = diag(D);
            D = D(5:6);
            % Get max and min value
            lambda_min = min(D);
            lambda_max = max(D);
            % Calculate ki
            ki = sqrt(lambda_min/lambda_max);
            % Calculate sum of ki
            ki_sum = ki_sum + ki;
            % Compare wether this ki is the smallest among all 
            % Workspace
            ki_min = min(ki_min, sqrt(lambda_min/lambda_max));
            % Record the corresponding location of Ki within
            % The workspace
            K = [K, ki];
    
        end
    end
    % Calculate goal function C
    C = ki_sum*ki_min/L3;
    % Reshape K to match the size of our workspace
    K = reshape(K,15,11);
end