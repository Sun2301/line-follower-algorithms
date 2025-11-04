%% MAIN SIMULATION SCRIPT
clear; clc; close all;

%% ADD PATHS TO SUBDIRECTORIES
addpath('robot_model');
addpath('controllers'); 
addpath('utils');

%% 1. SETUP ROBOT
fprintf('=== LINE FOLLOWER SIMULATION ===\n');
robot = setup_robot();  % calls robot_model/setup_robot.m

%% 2. SIMULATION PARAMETERS
sim_time = 90;                          % 5 second test run
time_steps = round(sim_time / robot.dt);
time_vector = 0:robot.dt:sim_time;

% Define line path in WORLD coordinates
world_line_x =time_vector *robot.base_speed; %Line extends forward
world_line_y =0.02*sin(0.5*world_line_x); % sin shape in y
track_curve = world_line_y;
%% 3. INITIALIZE RESULTS STORAGE
results = initialize_results(time_vector, time_steps);

%% 4. TEST ALL THREE CONTROLLERS
controllers = {'PID', 'Sliding Mode', 'Bang-Bang'};

for ctrl_idx = 1:length(controllers)
    controller_name = controllers{ctrl_idx};
    fprintf('\n--- Testing %s Controller ---\n', controller_name);
    
    % Reset robot to start position for fair comparison
    robot = reset_robot_state(robot);
    
    % Controller-specific initialization
    switch controller_name
        case 'PID'
            controller_state = init_pid_controller();
        case 'Sliding Mode'
            controller_state = init_sliding_mode_controller();
        case 'Bang-Bang'
            controller_state = init_bangbang_controller();
    end
    
    % Run simulation loop
    for i = 1:time_steps+1
        current_time = (i-1) * robot.dt;
        
        
        % Lets calculate where robot sees the line (relative position)
        robot_x=robot.state(1);
        robot_y=robot.state(2);
        %find the target line positionat robot's X location
        target_y=0.02*sin(0.5*robot_x);
        % line position relative to robot (positive = line is to robot's right)
        line_position = target_y-robot_y;
        % 1. Simulate sensor readings
        sensor_readings = read_sensors(robot, line_position);  % robot_model/read_sensors.m
        
        % 2. Calculate error
        error = calculate_error(sensor_readings);  % utils/calculate_error.m
        
        % 3. Controller logic (separate files)
        switch controller_name
            case 'PID'
                [left_speed, right_speed, controller_state] = pid_controller(error, robot.dt, controller_state);
            case 'Sliding Mode'
                [left_speed, right_speed, controller_state] = sliding_mode_controller(error, robot.dt, controller_state);
            case 'Bang-Bang'
                [left_speed, right_speed, controller_state] = bangbang_controller(error, robot.dt, controller_state);
        end
        
        % 4. Update robot physics
        robot = update_physics(robot, left_speed, right_speed);  % robot_model/update_physics.m
        
        % 5. Store results
        results = store_results(results, ctrl_idx, i, error, left_speed, right_speed, robot.state);
        
        % 6. Debug output every second
        if mod(i-1, round(1/robot.dt)) == 0
            fprintf('t=%.1fs: Error=%+.1f, Speeds=[%.2f, %.2f], Pos=[%.3f, %.3f]\n', ...
                    current_time, error, left_speed, right_speed, robot.state(1), robot.state(2));
        end
    end
    
    fprintf('%s Controller completed!\n', controller_name);
end

%% 5. COMPARE RESULTS
fprintf('\n=== CONTROLLER COMPARISON ===\n');
compare_controllers(results);  % utils/visualize_results.m

%% 6. GENERATE PLOTS
visualize_results(results, time_vector, track_curve);  % utils/visualize_results.m

fprintf('\nSimulation complete! Check the plots for performance comparison.\n');

%% HELPER FUNCTIONS (inline for main script organization)
function results = initialize_results(time_vector, time_steps)
    results.time = time_vector;
    results.controllers = {'PID', 'Sliding Mode', 'Bang-Bang'};
    
    for i = 1:3  % Three controllers
        results.error{i} = zeros(1, time_steps+1);
        results.left_speed{i} = zeros(1, time_steps+1);
        results.right_speed{i} = zeros(1, time_steps+1);
        results.position{i} = zeros(2, time_steps+1);  % [x; y]
    end
end

function robot = reset_robot_state(robot)
    % Reset robot to starting position and state
    robot.state = [0, 0, 0, 0, 0];  % [x, y, theta, v_left, v_right]
end

function results = store_results(results, ctrl_idx, time_idx, error, left_speed, right_speed, robot_state)
    results.error{ctrl_idx}(time_idx) = error;
    results.left_speed{ctrl_idx}(time_idx) = left_speed;
    results.right_speed{ctrl_idx}(time_idx) = right_speed;
    results.position{ctrl_idx}(:, time_idx) = robot_state(1:2)';  % [x, y]
end

function controller_state = init_pid_controller()
    controller_state.integral = 0;
    controller_state.last_error = 0;
end

function controller_state = init_sliding_mode_controller()
    controller_state.last_error = 0;
end

function controller_state = init_bangbang_controller()
    controller_state = struct();  % Bang-bang doesn't need state
end

function compare_controllers(results)
    fprintf('Performance Summary:\n');
    for i = 1:length(results.controllers)
        max_error = max(abs(results.error{i}));
        final_pos = results.position{i}(:, end);
        fprintf('%s: Max Error=%.2f, Final Pos=[%.3f, %.3f]\n', ...
                results.controllers{i}, max_error, final_pos(1), final_pos(2));
    end
end