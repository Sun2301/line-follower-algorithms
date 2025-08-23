function visualize_results(results, time_vector, track_curve)
    % VISUALIZE_RESULTS Generate comparison plots for all three controllers
    %
    % Inputs:
    %   results - results struct from main simulation
    %   time_vector - time array for plotting
    %   track_curve - reference line path
    
    controller_names = results.controllers;
    colors = {'b', 'r', 'g'};  % Blue, Red, Green
    line_styles = {'-', '--', ':'};
    
    % Create main comparison figure
    figure('Name', 'Controller Comparison', 'Position', [100, 100, 1200, 800]);
    
    %% Plot 1: Line Following Error
    subplot(2,3,1);
    hold on;
    for i = 1:length(controller_names)
        plot(time_vector, results.error{i}, 'Color', colors{i}, ...
             'LineStyle', line_styles{i}, 'LineWidth', 2, ...
             'DisplayName', controller_names{i});
    end
    hold off;
    grid on;
    title('Line Following Error');
    xlabel('Time (s)');
    ylabel('Error');
    legend('show', 'Location', 'best');
    
    %% Plot 2: Wheel Speeds - Left
    subplot(2,3,2);
    hold on;
    for i = 1:length(controller_names)
        plot(time_vector, results.left_speed{i}, 'Color', colors{i}, ...
             'LineStyle', line_styles{i}, 'LineWidth', 2, ...
             'DisplayName', [controller_names{i} ' Left']);
    end
    hold off;
    grid on;
    title('Left Wheel Speed');
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    legend('show', 'Location', 'best');
    
    %% Plot 3: Wheel Speeds - Right
    subplot(2,3,3);
    hold on;
    for i = 1:length(controller_names)
        plot(time_vector, results.right_speed{i}, 'Color', colors{i}, ...
             'LineStyle', line_styles{i}, 'LineWidth', 2, ...
             'DisplayName', [controller_names{i} ' Right']);
    end
    hold off;
    grid on;
    title('Right Wheel Speed');
    xlabel('Time (s)');
    ylabel('Speed (m/s)');
    legend('show', 'Location', 'best');
    
    %% Plot 4: Robot Paths
    subplot(2,3,4);
    hold on;
    % Plot reference line
    reference_x = time_vector * 0.2;  % Assuming forward motion
    plot(reference_x, track_curve, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference Line');
    
    % Plot robot paths
    for i = 1:length(controller_names)
        plot(results.position{i}(1,:), results.position{i}(2,:), ...
             'Color', colors{i}, 'LineStyle', line_styles{i}, 'LineWidth', 2, ...
             'DisplayName', controller_names{i});
    end
    hold off;
    grid on; axis equal;
    title('Robot Paths');
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    legend('show', 'Location', 'best');
    
    %% Plot 5: Speed Difference (Steering Effort)
    subplot(2,3,5);
    hold on;
    for i = 1:length(controller_names)
        speed_diff = results.right_speed{i} - results.left_speed{i};
        plot(time_vector, speed_diff, 'Color', colors{i}, ...
             'LineStyle', line_styles{i}, 'LineWidth', 2, ...
             'DisplayName', controller_names{i});
    end
    hold off;
    grid on;
    title('Steering Effort (Right - Left Speed)');
    xlabel('Time (s)');
    ylabel('Speed Difference (m/s)');
    legend('show', 'Location', 'best');
    
    %% Plot 6: Performance Metrics
    subplot(2,3,6);
    
    % Calculate metrics
    metrics = calculate_performance_metrics(results);
    
    % Bar plot of key metrics
    metric_names = {'Max Error', 'RMS Error', 'Final Deviation'};
    bar_data = [metrics.max_error; metrics.rms_error; metrics.final_deviation]';
    
    bar(bar_data);
    set(gca, 'XTickLabel', controller_names);
    ylabel('Error Magnitude');
    title('Performance Metrics');
    legend(metric_names, 'Location', 'best');
    grid on;
    
    % Print summary to console
    print_performance_summary(metrics, controller_names);
end

function metrics = calculate_performance_metrics(results)
    % Calculate performance metrics for each controller
    n_controllers = length(results.controllers);
    
    metrics.max_error = zeros(1, n_controllers);
    metrics.rms_error = zeros(1, n_controllers);
    metrics.final_deviation = zeros(1, n_controllers);
    
    for i = 1:n_controllers
        errors = results.error{i};
        final_pos = results.position{i}(:, end);
        
        metrics.max_error(i) = max(abs(errors));
        metrics.rms_error(i) = sqrt(mean(errors.^2));
        metrics.final_deviation(i) = abs(final_pos(2)); % Y deviation from straight line
    end
end

function print_performance_summary(metrics, controller_names)
    fprintf('\n=== PERFORMANCE SUMMARY ===\n');
    fprintf('Controller      | Max Error | RMS Error | Final Y Dev\n');
    fprintf('----------------|-----------|-----------|------------\n');
    
    for i = 1:length(controller_names)
        fprintf('%-15s | %8.2f  | %8.2f  | %8.3f\n', ...
                controller_names{i}, ...
                metrics.max_error(i), ...
                metrics.rms_error(i), ...
                metrics.final_deviation(i));
    end
    fprintf('\n');
end