function robot = update_physics(robot, left_speed, right_speed)
    % UPDATE_PHYSICS Update robot position using differential drive kinematics
    %
    % Inputs:
    %   robot - robot struct with current state
    %   left_speed - left wheel speed (m/s)
    %   right_speed - right wheel speed (m/s)
    %
    % Output:
    %   robot - updated robot struct with new state
    
    % Current state
    x = robot.state(1);
    y = robot.state(2);
    theta = robot.state(3);
    
    % Add motor noise (real motors have variations)
    v_left = left_speed + robot.motor_noise * randn();
    v_right = right_speed + robot.motor_noise * randn();
    
    % Clamp to realistic motor limits
    v_left = max(-robot.max_speed, min(robot.max_speed, v_left));
    v_right = max(-robot.max_speed, min(robot.max_speed, v_right));
    
    % Differential drive kinematics
    v_linear = (v_left + v_right) / 2;
    v_angular = (v_right - v_left) / robot.wheelbase;
    
    % Update position (simple Euler integration)
    x_new = x + v_linear * cos(theta) * robot.dt;
    y_new = y + v_linear * sin(theta) * robot.dt;
    theta_new = theta + v_angular * robot.dt;
    
    % Keep angle in [-pi, pi]
    theta_new = atan2(sin(theta_new), cos(theta_new));
    
    % Update state [x, y, theta, v_left, v_right]
    robot.state = [x_new, y_new, theta_new, v_left, v_right];
end