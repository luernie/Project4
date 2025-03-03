%% IK Trying
function [t1, t2] = IK(Px, Py, L1, L2)
    % Calculate theta2
    ct2 = ((Px^2 + Py^2) - L1^2 - L2^2) / (2 * L1 * L2) % messed up on the square here
    st2 = sqrt(1 - (ct2)^2)
    
    % Check for valid values of ct2 and st2
    if ~isreal(ct2) || ~isreal(st2)
        t1 = NaN; t2 = NaN;
        return;
    end

    t2 = atan2(st2, ct2);

    % Calculate sin(t2) and cos(t2)
    s2 = sin(t2);
    c2 = cos(t2);

    % Calculate theta1
    st1 = -L2 * s2 * Px + (L1 + L2 * c2) * Py; %denominator cancels out
    ct1 = (L1 + L2 * c2) * Px + L2 * s2 * Py;
    t1 = atan2(st1, ct1);
end