function settling_time = calculate_settling_time(time, data, threshold_percentage)
    % Calculate the final value (assumed as the mean of the last 10% of the data)
    final_value = mean(data(end - floor(0.1 * length(data)) : end));
    
    % Calculate the threshold band
    threshold = threshold_percentage / 100 * abs(final_value);
    
    % Find the settling time
    settling_time = NaN; % Initialize as NaN
    for i = 1:length(data)
        if all(abs(data(i:end) - final_value) < threshold)
            settling_time = time(i);
            break;
        end
    end
end