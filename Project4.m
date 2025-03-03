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
simplify(sqrt(x^2+y^2), 'Steps', 10)
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

X = [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1];
Y = 0;

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

    for i = 1:3 % Loops through every X coordinate
        % Calculate Ik for each X,Y combo
        [t1, t2] = IK(X(i), Y, L1_num, L2_num);

        % Store t1 and t2 values in cell arrays
        t1_values{c, i} = t1;
        t2_values{c, i} = t2;

        % Check if t1 or t2 are NaN
        if isnan(t1) || isnan(t2)
            continue; % Skip plotting if t1 or t2 are NaN
        end

        %Graph that robot position
        figure()
        disp("plot")
        Robot.plot([t1 t2 0], 'workspace', [-1.5 1.5 -1.5 1.5 -.5 1]) % radians express

        % Calculates the Jacobian for that robot position
        J = calcJ(L1_num, L2_num, t1, t2);
        % Store J values in cell arrays
        J_values{c, i} = J;

        % Calculate the Eval and Evec
        [eVec, eVal] = eig(J);
        eVec_values{c, i} = eVec;
        eVal_values{c, i} = eVal;


        % % Add onto the ellipse
        % Robot.vellipse([t1 t2 0], 'fillcolor','b', 'edgecolor','w',alpha',0.5);
        % Robot.fellipse([t1 t2 0], 'fillcolor','r', 'edgecolor','w',alpha',0.5);
        % % BUG: Not working here for the ellipse, why Jacobian bad?
        % TODO: get both ellipses on to the graph
        % TODO: Graph the matrix stuff for each
        % TODO: If not able to then skip and say its singular
        % TODO: Solve the evals for each
    end

end


    % syms t1 t2; %TODO: here is the stuff for 
    % symbolic_expr = t1^2 + t2^2;
    % t1_value = 1.5;  % Numeric value for t1
    % t2_value = 0.7;  % Numeric value for t2
    % numeric_expr = subs(symbolic_expr, {t1, t2}, {t1_value, t2_value});
    % disp(['Result of substituting t1 = ', num2str(t1_value), ' and t2 = ', num2str(t2_value)]);
    % disp(['Numeric Expression: ', char(numeric_expr)]);

    % % redefine robot parameters to reflect the symbolic
    % for i = 1:5 %1:length(X)
    %     % Calculate Ik for each X,Y combo
    %     [t1, t2] = IK(X(i), Y, L1, L2);

    %     % Graph that robot position
    %     figure()
    %     Robot_num.plot([t1 t2 0], 'workspace', [-1.5 1.5 -1.5 1.5 -.5 1]) % radians express
    %     % FIXME: SO t1 is actually controlling t2 and t2 just changes the EE orientation
    %     % Bug: So the first robot link is not appearing for some reason here
        
    %     % Add onto the ellipse
    %     % Robot.vellipse([t1 t2], 'fillcolor','b', 'edgecolor','w',alpha',0.5);
    %     % Robot.fellipse([t1 t2], 'fillcolor','r', 'edgecolor','w',alpha',0.5);
    %     % BUG: Not working here for the ellipse, why Jacobian bad?
    %     disp("hello")
    %     % TODO: get both ellipses on to the graph
    %     % TODO: Graph the matrix stuff for each
    %     % TODO: If not able to then skip and say its singular
    % end
% % TODO: Graph the ellipses, first calculate the IK in btween though


% syms t1 t2; %TODO: here is the stuff for 
% symbolic_expr = t1^2 + t2^2;
% t1_value = 1.5;  % Numeric value for t1
% t2_value = 0.7;  % Numeric value for t2
% numeric_expr = subs(symbolic_expr, {t1, t2}, {t1_value, t2_value});
% disp(['Result of substituting t1 = ', num2str(t1_value), ' and t2 = ', num2str(t2_value)]);
% disp(['Numeric Expression: ', char(numeric_expr)]);


% % TODO: For loop for each configuration, for each x value



% for i = 1:5 %1:length(X)

%     % FIXME: SO t1 is actually controlling t2 and t2 just changes the EE orientation
%     % Bug: So the first robot link is not appearing for some reason here
    
%     % Add onto the ellipse
%     % Robot.vellipse([t1 t2], 'fillcolor','b', 'edgecolor','w',alpha',0.5);
%     % Robot.fellipse([t1 t2], 'fillcolor','r', 'edgecolor','w',alpha',0.5);
%     % BUG: Not working here for the ellipse, why Jacobian bad?

%     % TODO: get both ellipses on to the graph
%     % TODO: Graph the matrix stuff for each
%     % TODO: If not able to then skip and say its singular
% end