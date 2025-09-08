function error = calculate_error(sensor_readings)
    % CALCULATE_ERROR Convert sensor array readings to line position error
    %
    % Input:
    %   sensor_readings - array of 5 sensor values [0-1]
    %
    % Output:
    %   error - line position error
    %           0 = line centered
    %           negative = line to the left (robot should turn left)
    %           positive = line to the right (robot should turn right)
    %           ±100 = line lost (emergency case)
    
    % Weighted position method (standard for line following)
    weights = [-2, -1, 0, 1, 2];  % Left sensors negative, right positive
    
    % Threshold to determine which sensors see the line
    line_threshold = 0.5;  % Below this = on line (dark), above = off line (bright)
    
    % Find active sensors (those seeing the line)
    active_sensors = sensor_readings < line_threshold;
    
    if sum(active_sensors) == 0
        % No line detected - critical error!
        error = 100;  % Large positive error (could also use sign of last known error)
        return;
    end
    
    % Weighted average of line position
    % Inverted sensor values so darker readings have more weight
    line_strength = 1 - sensor_readings;  % Convert so 1=line, 0=background
    
    numerator = sum(weights .* active_sensors .* line_strength);
    denominator = sum(active_sensors .* line_strength);
    
    % Normalize and scale to reasonable range
    raw_position = numerator / denominator;
    error = raw_position * 25;  % Scale factor (adjust based on testing)
    
    % Clamp to prevent extreme values
    error = max(-100, min(100, error));
end