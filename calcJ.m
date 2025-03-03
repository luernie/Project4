function J = calcJ(L1,L2,t1,t2)
    % This function will be used to calculate the
    % Jacobian Matrix using given input
    % Fill this function
    J = [   - L2*sin(t1 + t2) - L1*sin(t1),     -L2*sin(t1 + t2);
            L2*cos(t1 + t2) + L1*cos(t1),       L2*cos(t1 + t2); ];
    end