function robot = setup_robot()
    % SETUP_ROBOT Initialize robot parameters for line follower simulation
    % Returns robot struct with all necessary parameters
    
    % Robot physical parameters (adjust to match your friend's robot)
    robot.wheelbase = 0.15;        % Distance between wheels (m)
    robot.wheel_radius = 0.03;     % Wheel radius (m)
    robot.max_speed = 0.5;         % Max wheel speed (m/s)
    
    % Sensor array setup (5 photoresistors)
    robot.num_sensors = 5;
    robot.sensor_spacing = 0.02;   % 2cm between sensors
    robot.sensor_positions = linspace(-0.04, 0.04, 5); % [-4cm to +4cm]
    
    % Environment parameters
    robot.line_width = 0.03;       % Black line width (3cm)
    robot.dt = 0.01;               % Simulation time step (10ms - like Arduino)
    
    % Initial robot state [x, y, theta, v_left, v_right]
    robot.state = [0, 0, 0, 0, 0]; % Start at origin, facing forward
    
    % Noise parameters (real world isn't perfect)
    robot.sensor_noise = 0.1;     % Sensor reading noise std
    robot.motor_noise = 0.05;     % Motor speed variation std
    
    % Control parameters
    robot.base_speed = 0.2;        % Default forward speed (m/s)
    
    fprintf('Robot initialized: %d sensors, %.1fcm wheelbase, %.1fcm line width\n', ...
            robot.num_sensors, robot.wheelbase*100, robot.line_width*100);
end