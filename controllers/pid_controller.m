function [left_speed, right_speed, controller_state] = pid_controller(error, dt, controller_state)
    % Get previous state values
    integral_sum = controller_state.integral;
    last_error = controller_state.last_error;
    
    % Calculate new integral and derivative
    integral_sum = integral_sum + error * dt;
    derivative = (error - last_error) / dt;
    
    % PID parameters
    Kp = 2.0;
    Ki = 0.1;
    Kd = 0.5;
    max_output = 50;
    
    %PID output calculation
    pid_output = Kp*error + Ki*integral_sum + Kd*derivative;
    
    % Apply saturation (Clamping)
    if pid_output > max_output
        pid_output = max_output;
    elseif pid_output < -max_output
        pid_output = -max_output;
    end
    
    % Convert PID output to differential wheel speeds
    base_speed = 0.2;  % Forward speed (m/s)
    speed_diff = pid_output * 0.01;  % Scale factor

    left_speed = base_speed - speed_diff;
    right_speed = base_speed + speed_diff;
    
    % Update state for next iteration
    controller_state.integral = integral_sum;
    controller_state.last_error = error;
end



