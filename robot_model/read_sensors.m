function sensor_readings = read_sensors(robot, line_position)
    % READ_SENSORS Simulate photoresistor array reading the line
    % 
    % Inputs:
    %   robot - robot struct from setup_robot()
    %   line_position - where line center is relative to robot center (m)
    % 
    % Output:
    %   sensor_readings - array of 5 sensor values [0-1]
    %                     0 = black line (low resistance)
    %                     1 = white surface (high resistance)
    
    sensor_readings = zeros(1, robot.num_sensors);
    
    for i = 1:robot.num_sensors
        % Distance from this sensor to the line center
        sensor_to_line = abs(robot.sensor_positions(i) - line_position);
        
        % Photoresistor logic: 
        % LOW value (0-0.3) when on black line
        % HIGH value (0.7-1.0) when on white surface
        if sensor_to_line <= robot.line_width/2
            % On the line - dark reading (low resistance, low voltage)
            base_reading = 0.15;
        else
            % Off the line - bright reading (high resistance, high voltage)
            base_reading = 0.85;
        end
        
        % Add realistic noise (Gaussian)
        noise = robot.sensor_noise * randn();
        sensor_readings(i) = max(0, min(1, base_reading + noise));
    end
end