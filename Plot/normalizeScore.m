clc;

%% 综合归一化评分
addpath('../Definition');
addpath('../Kinematics');

% 如果工作区中没有优化结果，则自动加载结果文件。
% 留空表示自动加载当前目录下最新的 two_link_optimization_*.mat。
% 也可以手动指定，例如：
% result_file = 'two_link_optimization_l_drivel_l_trans.mat';
result_file = 'two_link_optimization_l_base_l_drives.mat';

if ~exist('workspace_area_map', 'var') || ...
   ~exist('rect_area_map', 'var') || ...
   ~exist('area_ratio_map', 'var') || ...
   ~exist('param1_vec', 'var') || ...
   ~exist('param2_vec', 'var') || ...
   ~exist('param1_name', 'var') || ...
   ~exist('param2_name', 'var')

    if isempty(result_file)
        files = dir('two_link_optimization_*.mat');

        if isempty(files)
            error(['未找到优化结果文件。请先运行 optimize_two_links.m，', ...
                   '或手动设置 result_file。']);
        end

        [~, idx] = max([files.datenum]);
        result_file = files(idx).name;
    end

    fprintf('加载优化结果文件：%s\n', result_file);
    S = load(result_file);

    required_vars = { ...
        'workspace_area_map', 'rect_area_map', 'area_ratio_map', ...
        'param1_vec', 'param2_vec', 'param1_name', 'param2_name'};

    for k = 1:numel(required_vars)
        if ~isfield(S, required_vars{k})
            error('结果文件 %s 中缺少变量：%s', result_file, required_vars{k});
        end
    end

    workspace_area_map = S.workspace_area_map;
    rect_area_map      = S.rect_area_map;
    area_ratio_map     = S.area_ratio_map;
    param1_vec         = S.param1_vec;
    param2_vec         = S.param2_vec;
    param1_name        = S.param1_name;
    param2_name        = S.param2_name;
end

%% 1. 三个指标分别归一化
W1 = normalizeMap(workspace_area_map);
W2 = normalizeMap(rect_area_map);
W3 = normalizeMap(area_ratio_map);

%% 2. 权重设置
% score = w_workspace * 工作空间面积 + w_rect * 最大矩形面积 + w_ratio * 面积比
w_workspace = 0.5;
w_rect      = 0.5;
w_ratio     = 0.0;

weight_sum = w_workspace + w_rect + w_ratio;
if abs(weight_sum) < 1e-12
    error('权重之和不能为 0。');
end

% 如果权重和不是 1，则自动归一化，避免分数尺度混乱。
w_workspace = w_workspace / weight_sum;
w_rect      = w_rect / weight_sum;
w_ratio     = w_ratio / weight_sum;

score_map = w_workspace * W1 + w_rect * W2 + w_ratio * W3;

%% 3. 查找综合最优结果
[best_score, best_score_param1, best_score_param2] = ...
    findBestMetric(score_map, param1_vec, param2_vec);

fprintf('\n========== 综合优化结果 ==========\n');
fprintf('权重：workspace = %.3f, rect = %.3f, ratio = %.3f\n', ...
    w_workspace, w_rect, w_ratio);
fprintf('%s = %.2f mm, %s = %.2f mm, score = %.4f\n', ...
    param1_name, best_score_param1, param2_name, best_score_param2, best_score);

%% 4. 绘制综合评分图
plotScoreMap(param1_vec, param2_vec, score_map, ...
    param1_name, param2_name, '综合优化指标', 'Normalized Score');

%% 5. 保存综合评分结果
score_result_file = sprintf('normalized_score_%s_%s.mat', param1_name, param2_name);

save(score_result_file, ...
    'param1_name', 'param2_name', 'param1_vec', 'param2_vec', ...
    'w_workspace', 'w_rect', 'w_ratio', ...
    'W1', 'W2', 'W3', 'score_map', ...
    'best_score', 'best_score_param1', 'best_score_param2');

fprintf('综合评分结果已保存到：%s\n', score_result_file);

%% 局部函数
function [best_value, best_param1, best_param2] = findBestMetric(metric_map, param1_vec, param2_vec)
    valid = isfinite(metric_map);

    if ~any(valid(:))
        best_value = NaN;
        best_param1 = NaN;
        best_param2 = NaN;
        return;
    end

    temp = metric_map;
    temp(~valid) = -inf;

    [best_value, idx] = max(temp(:));
    [row, col] = ind2sub(size(metric_map), idx);

    best_param1 = param1_vec(col);
    best_param2 = param2_vec(row);
end

function plotScoreMap(param1_vec, param2_vec, metric_map, param1_name, param2_name, fig_title, colorbar_label)
    [P1_grid, P2_grid] = meshgrid(param1_vec, param2_vec);

    [best_value, best_param1, best_param2] = ...
        findBestMetric(metric_map, param1_vec, param2_vec);

    figure;
    hold on;

    surf(P1_grid, P2_grid, metric_map, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.95);

    plot3(best_param1, best_param2, best_value, 'rp', ...
        'MarkerSize', 16, ...
        'MarkerFaceColor', 'r', ...
        'LineWidth', 1.5);

    xlabel(sprintf('%s / mm', param1_name), 'Interpreter', 'none');
    ylabel(sprintf('%s / mm', param2_name), 'Interpreter', 'none');
    zlabel(colorbar_label, 'Interpreter', 'none');

    title(sprintf('%s\n最优：%s = %.2f mm, %s = %.2f mm, score = %.4f', ...
        fig_title, param1_name, best_param1, param2_name, best_param2, best_value), ...
        'Interpreter', 'none');

    cb = colorbar;
    ylabel(cb, colorbar_label, 'Interpreter', 'none');

    colormap jet;
    grid on;
    box on;
    view(45, 30);
end
