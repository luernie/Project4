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

% Robot Configurations
%L1=0.25; L2=0.75; % Configuration 1
%L1=0.5; L2=0.5; % Configuration 2
%L1=0.75; L2=0.25; % Configuration 3

% DH parameters for robot
alpha = [ 0 0   0]; % Create a symbolic representation of a constant
a   =   [0 L1  L2];
d   =   [0   0 0];
theta = [t1  t2 0];

% Define the robot with Robot Toolbox
L(1) = Link('revolute','d',0,'a',L1,'alpha',0,'modified');
L(2) = Link('revolute','d',0,'a',L2,'alpha',0,'modified');
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
%T23 = T(3).matrix;
T02 = simplify(T01 * T12);
%T03 = simplify(T01 * T12 * T23);
P01 = T01(1:3, 4);
P02 = T02(1:3, 4);
%P03 = T03(1:3, 4);
Pe = T02(1:3,4); %BUG: This should not be Tfinal but T02 cause we don't care about that
% Direct Differentiation
% Follow the similar precedure with previous example [3 col of the transformation matrix]
Z01 = T01(1:3, 3); % z-axis of frame 1 (Joint 1 axis)
Z02 = T02(1:3, 3); % z-axis of frame 2 (Joint 1 axis)
%Z03 = T03(1:3, 3); % z-axis of frame 3 (Joint 1 axis)
%%
J1 = [cross(Z01,(Pe-P01)); Z01]; % Revolute Joint Equation
J2 = [cross(Z02,(Pe-P02)); Z02]; 
% J3 = [cross(Z03,(Pe-P03)); Z03]; % Take this out because we don't need the last term
% J4 = [Z04; [0; 0; 0;]]; % Prismatic Joints
% J5 = [cross(Z05,(Pe-P05)); Z05]; 
% J6 = [cross(Z06,(Pe-P06)); Z06]; 
J_DD = simplify([J1, J2])
% det_J = (det(J_DD));

% BUG: We only care about the joint velocities and not the end effector value.

%% Eigenvalues and eigenvectors of the JJt
J_Jtranspose = simplify(J_DD * J_DD.');

% Calculate eigenvalues and eigenvectors
[eVec, eVal] = eig(J_Jtranspose);

% TODO: Graph the ellipses, first calculate the IK in btween though
% get the joint angles for different 
% check if the jacobian is singular
X = [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1];
X = [0];
Y = 0;

L1=0.5; L2=0.5; % Configuration 2

for i = 1:length(X)
    [t1, t2] = IK(Px, Py, Pz, L1, L2);
    figure()
    Robot.plot([t1 t2], 'workspace', [-1 1 -1 1 0 2]) % radians express
    disp("hello")
    % TODO: Plot the robot in that config
    % TODO: get both ellipses on to the graph
    % TODO: Graph the matrix stuff for each
    % TODO: If not able to then skip and say its singular
    figure()
    Robot.plot([t1 t2], 'workspace', [-1 1 -1 1 0 2]) % radians express
end


%% Plot robots: Robotname.plot()

%L1=0.5; L2=0.5; % Configuration 2

%testX = .350;
%testY = 0;
%estZ = 0;
%[testt1, testt2, testt3, testd4] = ScaraIK(testX, testY, testZ, 0, d1, a2, a3);

%figure()
%Robot.plot([t1 t2], 'workspace', [-1 1 -1 1 0 2]) % radians express

% Example usage of IK function
Px = 1;   % Replace with actual x-coordinate
Py = 2;   % Replace with actual y-coordinate
Pz = 0;   % Replace with actual z-coordinate
L1 = 3;   % Replace with actual link length L1
L2 = 2;   % Replace with actual link length L2

[t1, t2] = IK(Px, Py, L1, L2);
% Display results
disp(['t1 = ', num2str(t1)]);
disp(['t2 = ', num2str(t2)]);