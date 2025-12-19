clear; clc;
%% ========================================================================
%  NARX FOR SEM LAP STRATEGY OPTIMIZATION - PER LAP PROCESSING
%  COMPLETE VERSION: With visualization and NARX training
%% ========================================================================
%% INITIALIZATION
basePath = 'D:\.kuliah\S1\rakata\2025';
telemetryFile = fullfile(basePath, '26novfiltered.csv');
logbookFile = fullfile(basePath, 'Logbook_TD_26_10.csv');
assert(isfile(telemetryFile), 'Telemetry file not found!');
assert(isfile(logbookFile), 'Logbook file not found!');

%% LOAD TELEMETRY DATA - PRESERVE ALL DATA
fprintf('=== LOADING TELEMETRY DATA ===\n');
T = readtable(telemetryFile);
lat_all = double(T.lat);
lng_all = double(T.lng);
speed_all = double(T.kecepatan);
throttle_all = double(T.throttle);
current_all = double(T.arus);
millis_all = double(T.millis);

% Tampilkan info awal
total_data_points = length(lat_all);
fprintf('Original data points: %d\n', total_data_points);

% Identifikasi data valid
valid_gps = ~(lat_all == 0 & lng_all == 0);
fprintf('Valid GPS points (non 0,0): %d (%.1f%%)\n', sum(valid_gps), sum(valid_gps)/total_data_points*100);

%% LOAD LOGBOOK
fprintf('\n=== LOADING LOGBOOK ===\n');
LB = readtable(logbookFile);

% Get column names
throttleCols = contains(lower(LB.Properties.VariableNames), 'throtle') & ...
               contains(lower(LB.Properties.VariableNames), '_s');
glidingCols = contains(lower(LB.Properties.VariableNames), 'gliding') & ...
              contains(lower(LB.Properties.VariableNames), '_s');
throttleVars = LB.Properties.VariableNames(throttleCols);
glidingVars = LB.Properties.VariableNames(glidingCols);

% Get column names for lap boundaries
dataAwalCol = LB.Properties.VariableNames(contains(LB.Properties.VariableNames, 'Data_awal', 'IgnoreCase', true));
dataAkhirCol = LB.Properties.VariableNames(contains(LB.Properties.VariableNames, 'Data_akhir', 'IgnoreCase', true));

if isempty(dataAwalCol)
    error('Column "Data_awal" not found in logbook!');
end
if isempty(dataAkhirCol)
    error('Column "Data_akhir" not found in logbook!');
end

dataAwalCol = dataAwalCol{1};
dataAkhirCol = dataAkhirCol{1};

num_laps = height(LB);
fprintf('Number of laps in logbook: %d\n', num_laps);

%% FIX LAP BOUNDARIES
fprintf('\n=== FIXING LAP BOUNDARIES ===\n');

% Get raw boundaries
raw_start = LB.(dataAwalCol);
raw_end = LB.(dataAkhirCol);

% Initialize fixed boundaries
fixed_start = raw_start;
fixed_end = raw_end;

% Fix missing boundaries
for lap_idx = 1:num_laps
    % Fix start if NaN
    if isnan(fixed_start(lap_idx))
        if lap_idx == 1
            fixed_start(lap_idx) = 1;
        elseif ~isnan(fixed_end(lap_idx-1))
            fixed_start(lap_idx) = fixed_end(lap_idx-1) + 1;
        else
            fixed_start(lap_idx) = 1;
        end
    end
    
    % Fix end if NaN
    if isnan(fixed_end(lap_idx))
        if lap_idx < num_laps && ~isnan(fixed_start(lap_idx+1))
            fixed_end(lap_idx) = fixed_start(lap_idx+1) - 1;
        else
            fixed_end(lap_idx) = total_data_points;
        end
    end
end

% Ensure start < end
for lap_idx = 1:num_laps
    if fixed_start(lap_idx) >= fixed_end(lap_idx)
        if lap_idx < num_laps && ~isnan(fixed_start(lap_idx+1))
            fixed_end(lap_idx) = fixed_start(lap_idx+1) - 1;
        else
            fixed_end(lap_idx) = min(total_data_points, fixed_start(lap_idx) + 1000);
        end
    end
end

% SPECIAL: Make laps 7-9 continuous
if num_laps >= 9
    fprintf('\nMaking Lap 7-9 continuous...\n');
    
    % Use Lap 7 start as reference
    start_ref = fixed_start(7);
    
    % Find end for the package
    if ~isnan(fixed_end(9)) && fixed_end(9) > 0
        end_ref = fixed_end(9);
    elseif num_laps > 9 && ~isnan(fixed_start(10))
        end_ref = fixed_start(10) - 1;
    else
        end_ref = min(total_data_points, start_ref + 3000);
    end
    
    % Divide into 3 equal parts
    total_points = end_ref - start_ref + 1;
    points_per_lap = floor(total_points / 3);
    
    fixed_start(7) = start_ref;
    fixed_end(7) = start_ref + points_per_lap - 1;
    
    fixed_start(8) = fixed_end(7) + 1;
    fixed_end(8) = fixed_start(8) + points_per_lap - 1;
    
    fixed_start(9) = fixed_end(8) + 1;
    fixed_end(9) = end_ref;
end

% Final validation
for lap_idx = 1:num_laps
    fixed_start(lap_idx) = max(1, min(fixed_start(lap_idx), total_data_points));
    fixed_end(lap_idx) = max(1, min(fixed_end(lap_idx), total_data_points));
    
    % Ensure minimum points
    if fixed_end(lap_idx) - fixed_start(lap_idx) < 50
        fixed_end(lap_idx) = min(total_data_points, fixed_start(lap_idx) + 100);
    end
end

% Update logbook
LB.(dataAwalCol) = fixed_start;
LB.(dataAkhirCol) = fixed_end;

%% VISUALIZE ORIGINAL DATA WITH BOUNDARIES
fprintf('\n=== VISUALIZING LAP BOUNDARIES ===\n');
figure('Position', [100, 100, 1400, 600], 'Name', 'Lap Boundaries');

% Full view
subplot(2,1,1);
plot(1:total_data_points, speed_all, 'b-', 'LineWidth', 0.5);
hold on;

colors = lines(num_laps);
for i = 1:num_laps
    start_idx = fixed_start(i);
    end_idx = fixed_end(i);
    
    if start_idx < end_idx
        plot([start_idx start_idx], [0 max(speed_all)], '--g', 'LineWidth', 1);
        plot([end_idx end_idx], [0 max(speed_all)], '--r', 'LineWidth', 1);
        
        % Label
        text(mean([start_idx, end_idx]), max(speed_all)*0.95, sprintf('L%d', i), ...
             'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', colors(i,:));
    end
end

xlabel('Data Point Index');
ylabel('Speed (km/h)');
title('All Lap Boundaries');
grid on;

% Zoom view for first 3 laps
subplot(2,1,2);
zoom_start = max(1, fixed_start(1) - 100);
zoom_end = min(total_data_points, fixed_end(min(3, num_laps)) + 100);
zoom_range = zoom_start:zoom_end;

plot(zoom_range, speed_all(zoom_range), 'b-', 'LineWidth', 1);
hold on;

for i = 1:min(3, num_laps)
    start_idx = fixed_start(i);
    end_idx = fixed_end(i);
    
    if start_idx >= zoom_start && end_idx <= zoom_end
        plot([start_idx start_idx], [0 max(speed_all(zoom_range))], '--g', 'LineWidth', 2);
        plot([end_idx end_idx], [0 max(speed_all(zoom_range))], '--r', 'LineWidth', 2);
        
        % Label with indices
        text(mean([start_idx, end_idx]), max(speed_all(zoom_range))*0.8, ...
             sprintf('Lap %d\n%d-%d', i, start_idx, end_idx), ...
             'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'Color', colors(i,:));
    end
end

xlabel('Data Point Index');
ylabel('Speed (km/h)');
title('Zoom: First 3 Laps');
grid on;
xlim([zoom_start zoom_end]);

%% PROCESS EACH LAP
fprintf('\n=== PROCESSING LAPS ===\n');

all_laps_U = [];
all_laps_Y = [];
lap_info = [];
valid_lap_count = 0;

% Haversine function
haversine = @(lat1, lon1, lat2, lon2) ...
    6371000 * 2 * asin(sqrt(sind((lat2-lat1)/2).^2 + ...
    cosd(lat1) .* cosd(lat2) .* sind((lon2-lon1)/2).^2));

for lap_idx = 1:num_laps
    fprintf('\n--- Processing Lap %d/%d ---\n', lap_idx, num_laps);
    
    % Get boundaries
    data_awal = LB.(dataAwalCol)(lap_idx);
    data_akhir = LB.(dataAkhirCol)(lap_idx);
    
    % Validate
    if isnan(data_awal) || isnan(data_akhir) || data_awal >= data_akhir
        fprintf('Invalid boundaries. Skipping.\n');
        continue;
    end
    
    lap_start_idx = max(1, floor(data_awal));
    lap_end_idx = min(total_data_points, ceil(data_akhir));
    
    if lap_start_idx >= lap_end_idx
        fprintf('Invalid: start >= end. Skipping.\n');
        continue;
    end
    
    fprintf('Boundaries: %d to %d\n', lap_start_idx, lap_end_idx);
    
    % Extract data
    lat = lat_all(lap_start_idx:lap_end_idx);
    lng = lng_all(lap_start_idx:lap_end_idx);
    speed = speed_all(lap_start_idx:lap_end_idx);
    throttle = throttle_all(lap_start_idx:lap_end_idx);
    current = current_all(lap_start_idx:lap_end_idx);
    millis = millis_all(lap_start_idx:lap_end_idx);
    
    N = length(lat);
    
    % Identify invalid GPS
    invalid_gps = (lat == 0 & lng == 0);
    valid_idx = ~invalid_gps;
    
    if sum(valid_idx) < 10
        fprintf('Not enough valid GPS data. Skipping.\n');
        continue;
    end
    
    % Use only valid data for calculations
    lat_valid = lat(valid_idx);
    lng_valid = lng(valid_idx);
    speed_valid = speed(valid_idx);
    throttle_valid = throttle(valid_idx);
    current_valid = current(valid_idx);
    millis_valid = millis(valid_idx);
    
    N_valid = length(lat_valid);
    
    %% CALCULATE DISTANCE
    distances = zeros(N_valid-1, 1);
    for i = 1:N_valid-1
        distances(i) = haversine(lat_valid(i), lng_valid(i), lat_valid(i+1), lng_valid(i+1));
    end
    
    % Clean distances
    median_dist = median(distances);
    std_dist = std(distances);
    max_allowed = median_dist + 3*std_dist;
    distances(distances > max_allowed) = median_dist;
    distances(isnan(distances)) = 0.1;
    distances(distances < 0.1) = 0.1;
    
    distance = [0; cumsum(distances)];
    total_distance = distance(end);
    
    fprintf('Distance: %.2f m (%.3f km), Valid points: %d\n', total_distance, total_distance/1000, N_valid);
    
    %% CALCULATE ROAD SLOPE
    roadSlope = [0; diff(speed_valid) ./ diff(distance)];
    roadSlope(~isfinite(roadSlope)) = 0;
    roadSlope_pct = roadSlope * 100;
    
    %% CALCULATE CURVATURE
    curvature = zeros(N_valid, 1);
    for i = 2:N_valid-1
        y1 = sind(lng_valid(i)-lng_valid(i-1)) * cosd(lat_valid(i));
        x1 = cosd(lat_valid(i-1)) * sind(lat_valid(i)) - sind(lat_valid(i-1)) * cosd(lat_valid(i)) * cosd(lng_valid(i)-lng_valid(i-1));
        bearing1 = atan2d(y1, x1);
        
        y2 = sind(lng_valid(i+1)-lng_valid(i)) * cosd(lat_valid(i+1));
        x2 = cosd(lat_valid(i)) * sind(lat_valid(i+1)) - sind(lat_valid(i)) * cosd(lat_valid(i+1)) * cosd(lng_valid(i+1)-lng_valid(i));
        bearing2 = atan2d(y2, x2);
        
        bearing_change = abs(bearing2 - bearing1);
        if bearing_change > 180
            bearing_change = 360 - bearing_change;
        end
        curvature(i) = bearing_change;
    end
    curvature = smoothdata(curvature, 'gaussian', min(10, ceil(N_valid/10)));
    
    %% GET THROTTLE/GLIDING EVENTS
    time_s = (millis_valid - millis_valid(1)) / 1000;
    
    eventThrottle = [];
    eventGliding = [];
    
    for col_idx = 1:length(throttleVars)
        val = LB.(throttleVars{col_idx})(lap_idx);
        if ~isnan(val) && val > 0
            eventThrottle = [eventThrottle; val];
        end
    end
    
    for col_idx = 1:length(glidingVars)
        val = LB.(glidingVars{col_idx})(lap_idx);
        if ~isnan(val) && val > 0
            eventGliding = [eventGliding; val];
        end
    end
    
    %% MAP EVENTS TO DISTANCE
    throttle_distances = zeros(length(eventThrottle), 1);
    gliding_distances = zeros(length(eventGliding), 1);
    
    for i = 1:length(eventThrottle)
        [~, idx] = min(abs(time_s - eventThrottle(i)));
        throttle_distances(i) = distance(idx);
    end
    
    for i = 1:length(eventGliding)
        [~, idx] = min(abs(time_s - eventGliding(i)));
        gliding_distances(i) = distance(idx);
    end
    
    %% CREATE THROTTLE STATE
    throttle_state = zeros(N_valid, 1);
    
    for i = 1:length(throttle_distances)
        t_start = throttle_distances(i);
        
        if i <= length(gliding_distances)
            t_end = gliding_distances(i);
        else
            t_end = distance(end);
        end
        
        idx_on = find(distance >= t_start & distance < t_end);
        throttle_state(idx_on) = 1;
    end
    
    fprintf('Throttle ON: %.1f%%, Events: %d throttle, %d gliding\n', ...
            sum(throttle_state)/N_valid*100, length(eventThrottle), length(eventGliding));
    
    %% CALCULATE SPEED WINDOWS
    segment_length = 50;
    num_segments = max(1, ceil(total_distance / segment_length));
    
    speed_upper = zeros(N_valid, 1);
    speed_lower = zeros(N_valid, 1);
    
    window_size = 150;
    
    for i = 1:N_valid
        in_window = abs(distance - distance(i)) <= window_size/2;
        
        if sum(in_window) < 3
            in_window = true(size(distance));
        end
        
        window_speeds = speed_valid(in_window);
        
        speed_p75 = prctile(window_speeds, 75);
        speed_p25 = prctile(window_speeds, 25);
        speed_median = median(window_speeds);
        
        iqr_margin = (speed_p75 - speed_p25) * 1.5;
        iqr_margin = max(iqr_margin, 5);
        
        speed_upper(i) = min(50, speed_median + iqr_margin);
        speed_lower(i) = max(3, speed_median - iqr_margin);
    end
    
    speed_upper = smoothdata(speed_upper, 'gaussian', min(50, ceil(N_valid/5)));
    speed_lower = smoothdata(speed_lower, 'gaussian', min(50, ceil(N_valid/5)));
    
    %% BUILD SEGMENT FEATURES
    segment_distance_arr = zeros(num_segments, 1);
    segment_slope_avg = zeros(num_segments, 1);
    segment_slope_max = zeros(num_segments, 1);
    segment_curve_avg = zeros(num_segments, 1);
    segment_curve_max = zeros(num_segments, 1);
    segment_throttle_ratio = zeros(num_segments, 1);
    segment_speed_upper = zeros(num_segments, 1);
    segment_speed_lower = zeros(num_segments, 1);
    
    for seg = 1:num_segments
        seg_start = (seg-1) * segment_length;
        seg_end = seg * segment_length;
        seg_idx = find(distance >= seg_start & distance < seg_end);
        
        if isempty(seg_idx)
            segment_distance_arr(seg) = seg_start + segment_length/2;
            continue;
        end
        
        segment_distance_arr(seg) = mean(distance(seg_idx));
        segment_slope_avg(seg) = mean(roadSlope_pct(seg_idx));
        segment_slope_max(seg) = max(abs(roadSlope_pct(seg_idx)));
        segment_curve_avg(seg) = mean(curvature(seg_idx));
        segment_curve_max(seg) = max(curvature(seg_idx));
        segment_throttle_ratio(seg) = mean(throttle_state(seg_idx));
        segment_speed_upper(seg) = mean(speed_upper(seg_idx));
        segment_speed_lower(seg) = mean(speed_lower(seg_idx));
    end
    
    %% CALCULATE LAP PARAMETERS
    lap_time = time_s(end);
    lap_energy = sum(abs(current_valid) .* diff([time_s; time_s(end)])) * 12;
    lap_max_speed = max(speed_valid);
    lap_aggressiveness = sum(throttle_state) / N_valid;
    
    fprintf('Time: %.1fs, Energy: %.1fkJ, Max speed: %.1fkm/h\n', ...
            lap_time, lap_energy/1000, lap_max_speed);
    
    %% BUILD NARX INPUT/OUTPUT
    U_lap = zeros(9, num_segments);
    
    for seg = 1:num_segments
        U_lap(1, seg) = segment_distance_arr(seg) / max(total_distance, 1);
        U_lap(2, seg) = segment_slope_avg(seg) / 20;
        U_lap(3, seg) = segment_slope_max(seg) / 20;
        U_lap(4, seg) = segment_curve_avg(seg) / 90;
        U_lap(5, seg) = segment_curve_max(seg) / 90;
        U_lap(6, seg) = lap_time / 300;
        U_lap(7, seg) = lap_energy / 500000;
        U_lap(8, seg) = lap_max_speed / 50;
        U_lap(9, seg) = lap_aggressiveness;
    end
    
    Y_lap = zeros(3, num_segments);
    
    for seg = 1:num_segments
        Y_lap(1, seg) = segment_speed_upper(seg) / 50;
        Y_lap(2, seg) = segment_speed_lower(seg) / 50;
        Y_lap(3, seg) = segment_throttle_ratio(seg);
    end
    
    %% ACCUMULATE DATA
    all_laps_U = [all_laps_U, U_lap];
    all_laps_Y = [all_laps_Y, Y_lap];
    
    % Store lap info
    valid_lap_count = valid_lap_count + 1;
    lap_info(valid_lap_count).lap_number = lap_idx;
    lap_info(valid_lap_count).num_segments = num_segments;
    lap_info(valid_lap_count).total_distance = total_distance;
    lap_info(valid_lap_count).lap_time = lap_time;
    lap_info(valid_lap_count).lap_energy = lap_energy;
    lap_info(valid_lap_count).throttle_distances = throttle_distances;
    lap_info(valid_lap_count).gliding_distances = gliding_distances;
    lap_info(valid_lap_count).distance = distance;
    lap_info(valid_lap_count).speed = speed_valid;
    lap_info(valid_lap_count).speed_upper = speed_upper;
    lap_info(valid_lap_count).speed_lower = speed_lower;
    lap_info(valid_lap_count).throttle_state = throttle_state;
    lap_info(valid_lap_count).lat = lat_valid;
    lap_info(valid_lap_count).lng = lng_valid;
    lap_info(valid_lap_count).data_start_idx = lap_start_idx;
    lap_info(valid_lap_count).data_end_idx = lap_end_idx;
    
    fprintf('Lap %d processed: %d segments\n', lap_idx, num_segments);
end

%% CHECK DATA
if isempty(all_laps_U)
    error('No valid lap data found!');
end

fprintf('\n=== PROCESSING COMPLETE ===\n');
fprintf('Processed %d laps, %d total segments\n', valid_lap_count, size(all_laps_U, 2));

%% CREATE AND TRAIN NARX NETWORK
fprintf('\n=== TRAINING NARX NETWORK ===\n');

% Prepare data for NARX
Ucell = con2seq(all_laps_U);
Ycell = con2seq(all_laps_Y);

% NARX network parameters
inputDelays = 1:3;
feedbackDelays = 1:3;
hiddenSize = 15;

% Create NARX network
net = narxnet(inputDelays, feedbackDelays, hiddenSize);
net.trainFcn = 'trainlm';
net.trainParam.epochs = 100;
net.trainParam.goal = 1e-4;
net.trainParam.max_fail = 15;
net.trainParam.showWindow = true;

% Prepare training data
[X, Xi, Ai, T] = preparets(net, Ucell, {}, Ycell);

% Train network
fprintf('Training with %d samples...\n', length(X));
[net, tr] = train(net, X, T, Xi, Ai);

fprintf('\nTraining complete!\n');
fprintf('Epochs: %d\n', tr.num_epochs);
fprintf('Best performance: %.6f\n', tr.best_perf);

% Close loop for prediction
netc = closeloop(net);

%% SAVE MODEL AND DATA
save(fullfile(basePath, 'NARX_SEM_Model_Final.mat'), 'netc', 'net', 'tr', ...
     'lap_info', 'all_laps_U', 'all_laps_Y', 'LB', 'fixed_start', 'fixed_end');

fprintf('\nModel saved: NARX_SEM_Model_Final.mat\n');

%% VISUALIZATION - SAME AS YOUR ORIGINAL CODE
fprintf('\n=== CREATING DETAILED VISUALIZATION ===\n');

if ~isempty(lap_info) && length(lap_info) >= 1
    lap1 = lap_info(1);
    
    figure('Position', [50, 50, 1800, 1000], 'Name', sprintf('Lap %d Strategy Analysis', lap1.lap_number));
    
    % Plot 1: GPS Track dengan Throttle Strategy Overlay
    subplot(2, 3, 1);
    colors = zeros(length(lap1.speed), 3);
    throttle_on = lap1.throttle_state > 0.5;
    colors(throttle_on, 1) = 1; % Red for throttle
    colors(~throttle_on, 3) = 0.7; % Blue for gliding
    
    scatter(lap1.lng, lap1.lat, 30, colors, 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
    
    hold on;
    for i = 1:length(lap1.throttle_distances)
        [~, idx] = min(abs(lap1.distance - lap1.throttle_distances(i)));
        plot(lap1.lng(idx), lap1.lat(idx), 'go', 'MarkerSize', 12, 'LineWidth', 3);
    end
    for i = 1:length(lap1.gliding_distances)
        [~, idx] = min(abs(lap1.distance - lap1.gliding_distances(i)));
        plot(lap1.lng(idx), lap1.lat(idx), 'ms', 'MarkerSize', 12, 'LineWidth', 3);
    end
    
    title(sprintf('Lap %d - GPS Track + Throttle Strategy', lap1.lap_number));
    xlabel('Longitude');
    ylabel('Latitude');
    legend('Track', 'Throttle Start', 'Gliding Start', 'Location', 'best');
    grid on;
    axis equal tight;
    
    % Plot 2: Speed Window (SAMA SEPERTI KODE AWAL)
    subplot(2, 3, 2);
    h1 = plot(lap1.distance/1000, lap1.speed_upper, 'g-', 'LineWidth', 2.5);
    hold on;
    h2 = plot(lap1.distance/1000, lap1.speed_lower, 'b-', 'LineWidth', 2.5);
    h3 = plot(lap1.distance/1000, lap1.speed, 'k-', 'LineWidth', 1.5);
    
    % Fill area
    fill([lap1.distance/1000; flipud(lap1.distance/1000)], ...
         [lap1.speed_upper; flipud(lap1.speed_lower)], ...
         'g', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    
    xlabel('Distance (km)');
    ylabel('Speed (km/h)');
    title('Speed Strategy Window (Optimized)');
    legend([h1 h2 h3], {'Upper Bound', 'Lower Bound', 'Actual Speed'}, 'Location', 'best');
    grid on;
    xlim([0 max(lap1.distance/1000)]);
    ylim([0 max(lap1.speed_upper)*1.1]);
    
    % Plot 3: Speed colored by Throttle State
    subplot(2, 3, 3);
    scatter(lap1.distance/1000, lap1.speed, 30, lap1.throttle_state, 'filled');
    colormap(gca, [0 0.4 0.8; 1 0.2 0.2]);
    colorbar('Ticks', [0, 1], 'TickLabels', {'Gliding', 'Throttle'});
    xlabel('Distance (km)');
    ylabel('Speed (km/h)');
    title('Speed vs Distance (colored by Throttle State)');
    grid on;
    xlim([0 max(lap1.distance/1000)]);
    
    % Plot 4-6: Statistics for all laps
    subplot(2, 3, 4);
    if ~isempty(lap_info)
        lap_numbers = arrayfun(@(x) x.lap_number, lap_info);
        lap_times = arrayfun(@(x) x.lap_time, lap_info);
        bar(lap_numbers, lap_times, 'FaceColor', [0.2 0.6 0.8]);
        xlabel('Lap Number');
        ylabel('Time (s)');
        title('Lap Times');
        grid on;
        
        for i = 1:length(lap_times)
            text(lap_numbers(i), lap_times(i), sprintf('%.0fs', lap_times(i)), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
        end
    end
    
    subplot(2, 3, 5);
    if ~isempty(lap_info)
        lap_energies = arrayfun(@(x) x.lap_energy/1000, lap_info);
        bar(lap_numbers, lap_energies, 'FaceColor', [0.8 0.4 0.2]);
        xlabel('Lap Number');
        ylabel('Energy (kJ)');
        title('Energy Consumption per Lap');
        grid on;
        
        for i = 1:length(lap_energies)
            text(lap_numbers(i), lap_energies(i), sprintf('%.1fkJ', lap_energies(i)), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
        end
    end
    
    subplot(2, 3, 6);
    if ~isempty(lap_info)
        lap_distances = arrayfun(@(x) x.total_distance/1000, lap_info);
        bar(lap_numbers, lap_distances, 'FaceColor', [0.4 0.8 0.4]);
        xlabel('Lap Number');
        ylabel('Distance (km)');
        title('Lap Distances');
        grid on;
        
        for i = 1:length(lap_distances)
            text(lap_numbers(i), lap_distances(i), sprintf('%.2fkm', lap_distances(i)), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
        end
    end
    
    % Overall title
    sgtitle(sprintf('Lap %d Analysis | Distance: %.2fkm | Time: %.1fs | Energy: %.1fkJ', ...
            lap1.lap_number, lap1.total_distance/1000, lap1.lap_time, lap1.lap_energy/1000), ...
            'FontSize', 14, 'FontWeight', 'bold');
end

%% TRAINING PERFORMANCE VISUALIZATION
figure('Position', [100, 100, 800, 600], 'Name', 'NARX Training Performance');
subplot(2,2,1);
plot(tr.perf);
title('Training Performance');
xlabel('Epoch');
ylabel('Mean Squared Error');
grid on;

subplot(2,2,2);
plot(tr.gradient);
title('Training Gradient');
xlabel('Epoch');
ylabel('Gradient');
grid on;

subplot(2,2,3);
bar(tr.num_epochs);
title('Training Epochs');
xlabel('Training');
ylabel('Epochs');

subplot(2,2,4);
plotperform(tr);

%% FINAL SUMMARY
fprintf('\n=== FINAL SUMMARY ===\n');
fprintf('Processed %d laps successfully\n', length(lap_info));
fprintf('Model trained with %d total segments\n', size(all_laps_U, 2));
fprintf('Total training time: %.1f seconds\n', tr.time(end));

% Display lap statistics
fprintf('\nLAP STATISTICS:\n');
fprintf('%-6s %-10s %-10s %-10s %-10s %-12s\n', ...
        'Lap', 'Distance(km)', 'Time(s)', 'Points', 'Segments', 'Throttle(%)');
for i = 1:length(lap_info)
    throttle_pct = sum(lap_info(i).throttle_state) / length(lap_info(i).speed) * 100;
    fprintf('%-6d %-10.2f %-10.1f %-10d %-10d %-12.1f\n', ...
            lap_info(i).lap_number, ...
            lap_info(i).total_distance/1000, ...
            lap_info(i).lap_time, ...
            length(lap_info(i).speed), ...
            lap_info(i).num_segments, ...
            throttle_pct);
end

fprintf('\n=== ALL PROCESSES COMPLETED ===\n');
