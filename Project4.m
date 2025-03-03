%% Project 4
% Can change the variable under the FK function where we define simplify
% steps
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
%% IK Notes
x = L2*cos(t1+t2)+L1*cos(t1);
y = L2*sin(t1+t2)+L1*sin(t1);
simplify(x^2+y^2, 'Steps', 10)
%% Jacobian Calculation Explicit Method
% Forward Kinematics - Get T matrices from T structure
T01 = T(1).matrix;
T12 = T(2).matrix;
T23 = T(3).matrix;
T02 = simplify(T01 * T12);
T03 = simplify(T01 * T12 * T23);
P01 = T01(1:3, 4);
P02 = T02(1:3, 4);
% P03 = T03(1:3, 4);
Pe = T03(1:3,4); %BUG: This should not be Tfinal but T02 cause we don't care about that
% Direct Differentiation
% Follow the similar precedure with previous example [3 col of the transformation matrix]
Z01 = T01(1:3, 3); % z-axis of frame 1 (Joint 1 axis)
Z02 = T02(1:3, 3); % z-axis of frame 2 (Joint 1 axis)
% Z03 = T03(1:3, 3); % z-axis of frame 3 (Joint 1 axis)

J1 = [cross(Z01,(Pe-P01)); Z01]; % Revolute Joint Equation
J2 = [cross(Z02,(Pe-P02)); Z02]; 
% J3 = [cross(Z03,(Pe-P03)); Z03]; % Take this out because we don't need the last term
% J4 = [Z04; [0; 0; 0;]]; % Prismatic Joints
% J5 = [cross(Z05,(Pe-P05)); Z05]; 
% J6 = [cross(Z06,(Pe-P06)); Z06]; 
J_DD = simplify([J1, J2])

% BUG: We only care about the joint velocities and not the end effector value.


%% Double for loop
X = 0:0.1:1; % X-axis positions from 0 to 1 meter in 0.1 meter increments
Y = 0;

close all;
for c = 1:3 % Loops through each configuration
    Config1 = [0.25 0.75]; % L1 and L2 parameters
    Config2 = [0.5 0.5];
    Config3 = [0.75 0.25];
    ConfigAll = [Config1; Config2; Config3];
    L1_num = ConfigAll(c,1);
    L2_num = ConfigAll(c,2);

    % Recreates robot based on new Config
    L(1) = Link('revolute', 'd', 0, 'a', 0, 'alpha', 0, 'modified');    % base frame
    L(2) = Link('revolute', 'd', 0, 'a', L1_num, 'alpha', 0, 'modified');   % frame 1
    L(3) = Link('revolute', 'd', 0, 'a', L2_num, 'alpha', 0, 'modified');   % frame 2
    Robot = SerialLink(L, 'name', '2R SCARA Arm'); % Combine Link objects together to form a Robot Object

    figure;
    hold on;

    for i = 1:length(X) % Loops through every X coordinate
        % Calculate Ik for each X,Y combo
        disp("IK for the value X = " + X(i))

        [t1, t2] = IK(X(i), Y, L1_num, L2_num);

        % Store t1 and t2 values in cell arrays
        t1_values{c, i} = t1;
        t2_values{c, i} = t2;

        % Check if t1 or t2 are NaN
        if isnan(t1) || isnan(t2)
            continue; % Skip plotting if t1 or t2 are NaN
        end

        % Graph that robot position
            % figure()
            % disp("plot")
            % Robot.plot([t1 t2 0], 'workspace', [-1.5 1.5 -1.5 1.5 -.5 1]) % radians express
        % End Graph robot position
        
        % Calculates the Jacobian for that robot position
        J = calcJ(L1_num, L2_num, t1, t2);
        % Store J values in cell arrays
        J_values{c, i} = J;

        % Calculate the Eval and Evec
        [eVec, eVal] = eig(J*J.');
        eVec_values{c, i} = eVec; % represents the principal directions
        eVal_values{c, i} = diag(eVal); % Serve as the scaling factors

        %% Graph the Ellipses
        % Plot robot arms
        % figure;
        % hold on;
        plot([0, L1_num * cos(t1)], [0, L1_num * sin(t1)], 'b-', 'LineWidth', 2, 'DisplayName', 'Arm 1');
        plot([L1_num * cos(t1), X(i)], [L1_num * sin(t1), Y], 'r-', 'LineWidth', 2, 'DisplayName', 'Arm 2');
        
        % % Plot end effector position
        % plot(X(i), Y, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k', 'DisplayName', 'End Effector');

        % Ellipsoid Tests
            % % Velocity Ellipse (Make sure to use the diagonal values)
            % vScale1 = sqrt(eVal(1, 1)) * 1; % Scale factor for first velocity ellipse (adjust 0.2 for visualization)
            % quiver(X(i), Y, vScale1 * eVec(1, 1), vScale1 * eVec(2, 1), 'm', 'LineWidth', 2, 'DisplayName', 'Velocity Ellipse');
            % vScale2 = sqrt(eVal(2, 2)) * 1; % Scale factor for second velocity ellipse (adjust 0.2 for visualization)
            % quiver(X(i), Y, vScale2 * eVec(1, 2), vScale2 * eVec(2, 2), 'm', 'LineWidth', 2);
            
            % % TODO: Add in the full ellipse and should be done
            % % TODO: Add all the graphs into just one? 
            % % Force Ellipse
            % fScale1 = 1 / sqrt(eVal(1, 1)) * 1; % Scale factor for force ellipse (adjust 0.2 for visualization)
            % quiver(X(i), Y, fScale1 * eVec(1, 1), fScale1 * eVec(2, 1), 'g', 'LineWidth', 2, 'DisplayName', 'Force Ellipse');
            % fScale2 = 1 / sqrt(eVal(2, 2)) * 1; % Scale factor for force ellipse (adjust 0.2 for visualization)
            % quiver(X(i), Y, fScale2 * eVec(1, 2), fScale2 * eVec(2, 2), 'g', 'LineWidth', 2);
        % End Ellipsoid Tests

        %%%%%%%%%%%%%%%%%%%%%%%input the eval and evec

        % Define parameters
        x0 = X(i);  % Center x-coordinate
        y0 = Y;  % Center y-coordinate
        
        % Direction vectors (2x1)
        dir1 = eVec(:,1);
        mag1 = 1/sqrt(eVal(1,1));
        dir2 = eVec(:,2); % usually semi-major axis aka larger?
        mag2 = 1/sqrt(eVal(2,2)); % usually semi-major axis aka larger?

        % Parametric equations for ellipse using direction vectors
        x = x0 + mag2 * cos(theta) * dir2(1) + mag1 * sin(theta) * dir1(1);
        y = y0 + mag2 * cos(theta) * dir2(2) + mag1 * sin(theta) * dir1(2);
        
        % Angle parameter
        theta = linspace(0, 2*pi, 100);
        
        % % Parametric equations for ellipse using direction vectors
        % x = x0 + a * cos(theta) * dx(1) + b * sin(theta) * dy(1);
        % y = y0 + a * cos(theta) * dx(2) + b * sin(theta) * dy(2);
        
        % Plot the ellipse
        % figure;
        plot(x, y, 'b', 'LineWidth', 2);
        axis equal;  % Ensure equal scaling on both axes
        grid on;
        xlabel('X');
        ylabel('Y');
        title('Ellipse Plot');
%%%%%%%%%%%%%%%%%%%%%%%%

        xlim([-0.5,1.5]);
        ylim([-1.5,1.5]);
        grid on;

        % hold off;
        % drawnow
    end
    hold off;
end

% syms t1 t2;
% symbolic_expr = t1^2 + t2^2;
% t1_value = 1.5;  % Numeric value for t1
% t2_value = 0.7;  % Numeric value for t2
% numeric_expr = subs(symbolic_expr, {t1, t2}, {t1_value, t2_value});
% disp(['Result of substituting t1 = ', num2str(t1_value), ' and t2 = ', num2str(t2_value)]);
% disp(['Numeric Expression: ', char(numeric_expr)]);

% % Example parameters
% center = [0, 0];  % Center coordinates

% % Axes lengths and direction vectors
% axes_lengths = [2, 1];  % Semi-major and semi-minor axes lengths
% direction_vector_major = [1, 1];  % Direction vector for the semi-major axis (adjust as needed)
% direction_vector_minor = [-1, 1]; % Direction vector for the semi-minor axis (adjust as needed)

% % Calculate angles from direction vectors
% rotation_angle_major = atan2(direction_vector_major(2), direction_vector_major(1));
% rotation_angle_minor = atan2(direction_vector_minor(2), direction_vector_minor(1));

% % Plotting the ellipse
% hold on;
% ellipse(center, axes_lengths, rotation_angle_major, 'b');  % Semi-major axis
% ellipse(center, axes_lengths, rotation_angle_minor, 'r');  % Semi-minor axis
% hold off;

% axis equal;  % Ensure equal scaling on x and y axes
% grid on;

% input the eval and evec

% % Define parameters
% x0 = 3;  % Center x-coordinate
% y0 = 2;  % Center y-coordinate

% % Direction vectors (2x1)
% dx = [1; 1];  % Example direction vector along x-axis
% dy = [0; 1];  % Example direction vector along y-axis

% % Magnitudes (semi-major and semi-minor axes)
% a = 4;  % Example magnitude for semi-major axis
% b = 2;  % Example magnitude for semi-minor axis

% % Angle parameter
% theta = linspace(0, 2*pi, 100);

% % Parametric equations for ellipse using direction vectors
% x = x0 + a * cos(theta) * dx(1) + b * sin(theta) * dy(1);
% y = y0 + a * cos(theta) * dx(2) + b * sin(theta) * dy(2);

% % Plot the ellipse
% figure;
% plot(x, y, 'b', 'LineWidth', 2);
% axis equal;  % Ensure equal scaling on both axes
% grid on;
% xlabel('X');
% ylabel('Y');
% title('Ellipse Plot');
