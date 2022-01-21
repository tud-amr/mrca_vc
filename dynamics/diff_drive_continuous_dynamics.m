function dx = diff_drive_continuous_dynamics(x, u)

    % control input
    v = u(1);
    omega = u(2);
    
    % state
    theta = x(3);
    
    % differential
    x_dot = v*cos(theta);
    y_dot = v*sin(theta);
    theta_dot = omega;
    
    dx = [x_dot; y_dot; theta_dot];

end
