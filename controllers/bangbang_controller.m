function [left_speed, right_speed, controller_state] = bangbang_controller(error, dt, controller_state)
    base_speed = 0.2;        % Forward speed
    turn_speed = 0.1;        % How much to turn
    error_threshold = 5;     % Dead zone
    
    if error > error_threshold
        % Line is to the right, turn right
        left_speed = base_speed + turn_speed;
        right_speed = base_speed - turn_speed;
    elseif error < -error_threshold  
        % Line is to the left, turn left
        left_speed = base_speed - turn_speed;
        right_speed = base_speed + turn_speed;
    else
        % Go straight (error is small)
        left_speed = base_speed;
        right_speed = base_speed;
    end
    
end