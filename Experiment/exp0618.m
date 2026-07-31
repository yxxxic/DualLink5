clc;
clear;
close all;

experimentDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(experimentDir);
run(fullfile(projectRoot, 'startup.m'));
addpath(experimentDir);
geometry = duallink5.model.defaultGeometry();

data_dir = fullfile(experimentDir, '实验数据', '0618');
output_dir = fullfile(data_dir, 'figures');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

all_csv = dir(fullfile(data_dir, '*.csv'));
all_names = string({all_csv.name});

record_mask = startsWith(all_names, "record_");
angle_files = sort(all_names(~record_mask));
record_files = sort(all_names(record_mask));

if numel(angle_files) ~= numel(record_files)
    error('duallink5exp:ExperimentInventoryMismatch', ...
        'Angle file count (%d) does not match record file count (%d).', ...
        numel(angle_files), numel(record_files));
end

experiment_labels = ["D40_ccw", "D40_cw", "D30_ccw"];
experiment_titles = ["diameter 40 mm, CCW", "diameter 40 mm, CW", ...
    "diameter 30 mm, CCW"];
expected_pairs = numel(experiment_labels);

if numel(angle_files) ~= expected_pairs
    error('duallink5exp:ExperimentInventoryMismatch', ...
        'Expected exactly %d paired experiments, found %d pairs.', ...
        expected_pairs, numel(angle_files));
end

fprintf('Found %d paired 0618 experiment files.\n', numel(angle_files));

for i = 1:expected_pairs
    angle_file = fullfile(data_dir, angle_files(i));
    record_file = fullfile(data_dir, record_files(i));

    experiment_label = experiment_labels(i);
    experiment_title = experiment_titles(i);

    optical = readOpticalRecord(record_file);
    angle_data = readAngleRecord(angle_file);
    [mt_angle1_f, mt_angle2_f, mt_filter_info] = filterMtAngles( ...
        angle_data.mt_angle1_deg, angle_data.mt_angle2_deg);

    % Historical 2026-06 logs are preserved with their existing assumption:
    % channel 1 is logical theta, channel 2 is logical phi, both already in deg.
    % Future logs must replace these aliases with a versioned sign/zero/gear
    % calibration before calling computeGTrajectory.
    thetaMtLogicalDeg = mt_angle1_f;
    phiMtLogicalDeg = mt_angle2_f;
    thetaRsLogicalDeg = angle_data.rs_id1_deg;
    phiRsLogicalDeg = angle_data.rs_id2_deg;

    trajectoryMt = duallink5exp.computeGTrajectory( ...
        thetaMtLogicalDeg, phiMtLogicalDeg, geometry);
    x_g_mt = trajectoryMt.x;
    y_g_mt = trajectoryMt.y;

    trajectoryRs = duallink5exp.computeGTrajectory( ...
        thetaRsLogicalDeg, phiRsLogicalDeg, geometry);
    x_g_rs = trajectoryRs.x;
    y_g_rs = trajectoryRs.y;
    optical_at_angle_time = interpolateOpticalToTimes(optical, angle_data.t_abs);

    optical_error = calcTrajectoryError( ...
        x_g_rs, y_g_rs, optical_at_angle_time.x, optical_at_angle_time.y);
    mt_error = calcTrajectoryError(x_g_rs, y_g_rs, x_g_mt, y_g_mt);

    fig = plotComparison( ...
        i, experiment_title, angle_files(i), record_files(i), ...
        optical, x_g_mt, y_g_mt, x_g_rs, y_g_rs);

    output_name = sprintf('exp0618_%02d_%s.png', i, experiment_label);
    duallink5.viz.exportFigure(fig, fullfile(output_dir, output_name), ...
        struct('resolution', 300));

    fprintf('%02d %-8s: %s  <->  %s  (mt spikes: angle1=%d, angle2=%d, invalid=%d, jumps=%d, discarded=%d)\n', ...
        i, experiment_label, angle_files(i), record_files(i), ...
        mt_filter_info.angle1_spikes, mt_filter_info.angle2_spikes, ...
        mt_filter_info.invalid_angle_rows, mt_filter_info.large_jump_rows, ...
        mt_filter_info.discarded_rows);
    printErrorStats('  optical-vs-rs', optical_error);
    printErrorStats('  mt-vs-rs     ', mt_error);
end

first_record = fullfile(data_dir, record_files(1));
first_optical = readOpticalRecord(first_record);
fprintf('First optical point after offset: x = %.4f mm, y = %.4f mm\n', ...
    1e3 * first_optical.x(1), 1e3 * first_optical.y(1));

function optical = readOpticalRecord(file)
    opts = detectImportOptions(file, 'VariableNamingRule', 'preserve');
    T = readtable(file, opts);

    optical = struct();
    optical.t_abs = parseTimeColumn(T.("timestamp"), 'yyyy-MM-dd HH:mm:ss.SSSSSS');
    optical.x = (T.("aimooe_coord_position.x") - 36) * 1e-3;
    optical.y = (T.("aimooe_coord_position.y") - 30) * 1e-3;
    optical.z = T.("aimooe_coord_position.z") * 1e-3;
    optical.units.position = "m";
end

function angle_data = readAngleRecord(file)
    opts = detectImportOptions(file, 'VariableNamingRule', 'preserve');
    T = readtable(file, opts);

    required_names = [ ...
        "mt_angle1_deg", "mt_angle2_deg", ...
        "rs_id1_deg", "rs_id2_deg"];
    actual_names = string(T.Properties.VariableNames);
    missing_names = required_names(~ismember(required_names, actual_names));
    if ~isempty(missing_names)
        error('Missing required columns in %s: %s', ...
            file, strjoin(missing_names, ', '));
    end

    angle_data = struct();
    angle_data.t_abs = parseTimeColumn(T.("system_time"), 'yyyy-MM-dd''T''HH:mm:ss.SSS');
    angle_data.mt_angle1_deg = T.("mt_angle1_deg");
    angle_data.mt_angle2_deg = T.("mt_angle2_deg");
    angle_data.rs_id1_deg = T.("rs_id1_deg");
    angle_data.rs_id2_deg = T.("rs_id2_deg");
end

function t_abs = parseTimeColumn(time_raw, input_format)
    if isdatetime(time_raw)
        t_abs = time_raw;
    else
        t_abs = datetime(string(time_raw), 'InputFormat', input_format);
    end
    t_abs = t_abs(:);
end

function optical_interp = interpolateOpticalToTimes(optical, target_t_abs)
    t0 = min([optical.t_abs; target_t_abs(:)]);
    optical_t = seconds(optical.t_abs - t0);
    target_t = seconds(target_t_abs(:) - t0);

    optical_interp = struct();
    optical_interp.x = duallink5exp.interpolateTimeSeries( ...
        optical_t, optical.x, target_t);
    optical_interp.y = duallink5exp.interpolateTimeSeries( ...
        optical_t, optical.y, target_t);
end

function stats = calcTrajectoryError(ref_x, ref_y, test_x, test_y)
    ref_x = ref_x(:);
    ref_y = ref_y(:);
    test_x = test_x(:);
    test_y = test_y(:);

    if numel(ref_x) ~= numel(test_x) || numel(ref_y) ~= numel(test_y)
        error('Trajectory vectors must have matching lengths for error calculation.');
    end

    valid = isfinite(ref_x) & isfinite(ref_y) & isfinite(test_x) & isfinite(test_y);
    err = hypot(test_x(valid) - ref_x(valid), test_y(valid) - ref_y(valid));

    stats = struct();
    stats.n = numel(err);
    if isempty(err)
        stats.mean = NaN;
        stats.rmse = NaN;
        stats.max = NaN;
    else
        stats.mean = mean(err);
        stats.rmse = sqrt(mean(err .^ 2));
        stats.max = max(err);
    end
end

function printErrorStats(label, stats)
    fprintf('%s error: mean=%.3f mm, rmse=%.3f mm, max=%.3f mm, n=%d\n', ...
        label, 1e3 * stats.mean, 1e3 * stats.rmse, ...
        1e3 * stats.max, stats.n);
end

function [angle1_f, angle2_f, info] = filterMtAngles(angle1, angle2)
    angle1_u = rad2deg(unwrap(deg2rad(angle1)));
    angle2_u = rad2deg(unwrap(deg2rad(angle2)));

    k = 25;
    nsigma = 3;
    severe_jump_deg = 2;

    [angle1_f, idx1] = hampel(angle1_u, k, nsigma);
    [angle2_f, idx2] = hampel(angle2_u, k, nsigma);

    severe_idx1 = idx1 & abs(angle1_u - angle1_f) > severe_jump_deg;
    severe_idx2 = idx2 & abs(angle2_u - angle2_f) > severe_jump_deg;
    invalid_angle_rows = angle1_u < 0 | angle2_u < 0;
    jump_rows = findLargeJumpRows(angle1_u, severe_jump_deg) | ...
        findLargeJumpRows(angle2_u, severe_jump_deg);
    discarded_rows = severe_idx1 | severe_idx2 | invalid_angle_rows | jump_rows;

    angle1_f(idx1 | discarded_rows) = NaN;
    angle2_f(idx2 | discarded_rows) = NaN;

    info = struct();
    info.angle1_spikes = nnz(idx1);
    info.angle2_spikes = nnz(idx2);
    info.invalid_angle_rows = nnz(invalid_angle_rows);
    info.large_jump_rows = nnz(jump_rows);
    info.discarded_rows = nnz(discarded_rows);
end

function rows = findLargeJumpRows(angle_vec, max_step_deg)
    angle_vec = angle_vec(:);
    rows = false(size(angle_vec));

    if numel(angle_vec) < 2
        return;
    end

    jump_pair = abs(diff(angle_vec)) > max_step_deg;
    rows(1:end-1) = rows(1:end-1) | jump_pair;
    rows(2:end) = rows(2:end) | jump_pair;
end

function fig = plotComparison( ...
    group_id, experiment_title, angle_name, record_name, ...
    optical, x_g_mt, y_g_mt, x_g_rs, y_g_rs)

    fig = figure('Color', 'w', ...
        'Name', sprintf('exp0618 %02d %s', group_id, experiment_title));
    ax = axes(fig);
    ax.Toolbar.Visible = 'off';
    hold on;
    grid on;
    box on;
    axis equal;

    plotValid(1e3 * optical.x, 1e3 * optical.y, ...
        '-', [0.05 0.05 0.05], 1.8, 'none', 4, 'optical x-36, y-30');
    plotValid(1e3 * x_g_mt, 1e3 * y_g_mt, ...
        '-o', [0.00 0.25 0.85], 0.6, 'o', 3.2, 'G from mt\_angle');
    plotValid(1e3 * x_g_rs, 1e3 * y_g_rs, ...
        '-s', [0.85 0.10 0.10], 0.6, 's', 3.2, 'G from rs\_id');

    xlabel('x / mm');
    ylabel('y / mm');
    title(sprintf('exp0618 %02d %s: %s <-> %s', ...
        group_id, experiment_title, angle_name, record_name), ...
        'Interpreter', 'none');
    legend('Location', 'best', 'Interpreter', 'none');
    hold off;
end

function h = plotValid(x, y, line_style, color, line_width, marker, marker_size, display_name)
    x = x(:);
    y = y(:);
    valid = isfinite(x) & isfinite(y);

    if ~any(valid)
        warning('No valid points to plot for %s.', display_name);
        h = plot(NaN, NaN, line_style, ...
            'Color', color, ...
            'LineWidth', line_width, ...
            'Marker', marker, ...
            'MarkerSize', marker_size, ...
            'MarkerFaceColor', 'w', ...
            'DisplayName', display_name);
        return;
    end

    h = plot(x, y, line_style, ...
        'Color', color, ...
        'LineWidth', line_width, ...
        'Marker', marker, ...
        'MarkerSize', marker_size, ...
        'MarkerFaceColor', 'w', ...
        'DisplayName', display_name);
end
