function plotVellipse(X, Y, eVal, eVec, scale_factor)
    % Calculate semi-axes lengths
    semi_major = sqrt(eVal(1,1)) * scale_factor;
    semi_minor = sqrt(eVal(2,2)) * scale_factor;

    % Calculate ellipse orientation (angle in radians)
    angle_rad = atan2(eVec(2, 1), eVec(1, 1));

    % Parametric equation of the ellipse
    theta = linspace(0, 2*pi, 100);
    ellipse_x = X + semi_major * cos(theta) * cos(angle_rad) - semi_minor * sin(theta) * sin(angle_rad);
    ellipse_y = Y + semi_major * cos(theta) * sin(angle_rad) + semi_minor * sin(theta) * cos(angle_rad);

    % Plot the ellipse
    hold on;
    plot(ellipse_x, ellipse_y, 'm-', 'LineWidth', 2, 'DisplayName', 'Velocity Ellipse');
    hold off;
    
    % Adjust plot limits if needed
    xlim([-1.5, 1.5]); % Adjust as per your workspace limits
    ylim([-1.5, 1.5]); % Adjust as per your workspace limits
    grid on;
end
