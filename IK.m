%% IK Trying
function [t1, t2] = IK(Px, Py, L1, L2)
    % Calculate theta2
    ct2 = ((Px^2 + Py^2)^2 - L1^2 - L2^2) / (2 * L1 * L2); 
    st2 = sqrt(1 - (ct2)^2);
    t2 = atan2(st2, ct2);

    % Calculate sin(t2) and cos(t2)
    s2 = sin(t2);
    c2 = cos(t2);

    % Calculate theta1
    st1 = -L2 * s2 * Px + (L1 + L2 * c2) * Py; %denominator cancels out
    ct1 = (L1 + L2 * c2) * Px + L2 * s2 * Py;
    t1 = atan2(st1, ct1);
    disp("I AM INSIDE THIS FUNCTION")
end

%% FK Function
function [T, Tfinal] = FK(alpha, a, d, theta) % Calculated FK (263A Final Exam code)
    %syms d1 a2 a3
    % Initialization of the transformation from the base to the end effector
    T0_ee = eye(4);
    T = struct(); % Initialize structure array for T
    
    for i = 1:length(alpha)
        fprintf('T (%d) to (%d)\n', i-1, i)
        % Calculate each transformation matrix and store in structure array
        T(i).matrix = TF(a(i), alpha(i), d(i), theta(i)); % Assuming TF is defined elsewhere
        pretty(simplify(T(i).matrix))
        T0_ee = T0_ee * T(i).matrix; % Sequential multiplications
    end
    
    % Simplify the final transformation
    Tfinal = simplify(T0_ee, 'Steps', 10);
    
    % Display and return Tfinal
    pretty(Tfinal);
end
%% Subset FK Function
function T = TF(a,alpha,d,theta) % This function is used in FK
    T = [cos(theta) -sin(theta) 0 a
    sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d
    sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d
    0 0 0 1];
end