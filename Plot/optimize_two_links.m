clc;
clear;
close all;

addpath('../Definition');
addpath('../Kinematics');

%% 1. 原始参数
run('../Definition/set_parameter.m');
params0 = params;

%% 2. 只需要修改这里：选择任意两根待优化连杆和搜索范围
% 可选字段名通常来自 set_parameter.m，例如：
% 'l_base', 'l_drivel', 'l_drives', 'l_driven', 'l_trans'
param1_name = 'l_base';   % 横轴，对应矩阵列
param2_name = 'l_drives';   % 纵轴，对应矩阵行

param1_vec = 60:1:90;
param2_vec = 60:1:90;

if ~isfield(params0, param1_name)
    error('params 中不存在字段：%s', param1_name);
end

if ~isfield(params0, param2_name)
    error('params 中不存在字段：%s', param2_name);
end

if strcmp(param1_name, param2_name)
    error('param1_name 和 param2_name 不能相同。');
end

%% 3. 关节角范围
theta_lim = [0, 180];
phi_lim   = [0, 90];

%% 4. 采样精度设置
% 优化时计算量很大，可以先用较低精度粗扫，再提高精度细扫。
nTheta = 100;
nPhi   = 100;

% 最大矩形搜索网格
Nx = 500;
Ny = 500;

%% 5. 初始化结果矩阵
n1 = length(param1_vec);
n2 = length(param2_vec);

% 行对应 param2，列对应 param1
workspace_area_map = nan(n2, n1);
rect_area_map      = nan(n2, n1);
area_ratio_map     = nan(n2, n1);

%% 6. 遍历计算
tic;

total_num = n1 * n2;
count = 0;

for i = 1:n2
    for j = 1:n1
        count = count + 1;

        params.(param1_name) = param1_vec(j);
        params.(param2_name) = param2_vec(i);
        params.l_drivel = params.l_base;
        params.l_driven = params.l_drives;
        params.l_trans = params.l_base;
        params.l_parallel_b = params.l_base*0.75;
        params.l_parallel_a = params.l_base*0.75;
        
        params.l_D_Pbeta2 = params.l_parallel_b / 2;
        params.l_Pbeta2_Pbeta3 = params.l_parallel_b;
        params.l_E_Palpha1 = params.l_parallel_a / 2;
        params.l_Palpha1_Palpha4 = params.l_parallel_a;

        fprintf('计算进度：%4d / %4d, %s = %.2f, %s = %.2f\n', ...
            count, total_num, ...
            param1_name, params.(param1_name), ...
            param2_name, params.(param2_name));

        try
            [workspace_area, rect_area, area_ratio] = evaluateWorkspaceMetrics( ...
                params, theta_lim, phi_lim, nTheta, nPhi, Nx, Ny);

            workspace_area_map(i, j) = workspace_area;
            rect_area_map(i, j)      = rect_area;
            area_ratio_map(i, j)     = area_ratio;

        catch ME
            warning('该组参数计算失败：%s = %.2f, %s = %.2f', ...
                param1_name, params.(param1_name), ...
                param2_name, params.(param2_name));
            warning(ME.message);

            workspace_area_map(i, j) = NaN;
            rect_area_map(i, j)      = NaN;
            area_ratio_map(i, j)     = NaN;
        end
    end
end

toc;

%% 7. 找到三个指标各自的最优解
[best_ws_area, best_ws_param1, best_ws_param2] = ...
    findBestMetric(workspace_area_map, param1_vec, param2_vec);

[best_rect_area, best_rect_param1, best_rect_param2] = ...
    findBestMetric(rect_area_map, param1_vec, param2_vec);

[best_ratio, best_ratio_param1, best_ratio_param2] = ...
    findBestMetric(area_ratio_map, param1_vec, param2_vec);

fprintf('\n========== 优化结果 ==========\n');

fprintf('工作空间面积最大：\n');
fprintf('%s = %.2f mm, %s = %.2f mm, workspace area = %.4f mm^2\n\n', ...
    param1_name, best_ws_param1, param2_name, best_ws_param2, best_ws_area);

fprintf('最大矩形面积最大：\n');
fprintf('%s = %.2f mm, %s = %.2f mm, max rect area = %.4f mm^2\n\n', ...
    param1_name, best_rect_param1, param2_name, best_rect_param2, best_rect_area);

fprintf('面积比最大：\n');
fprintf('%s = %.2f mm, %s = %.2f mm, area ratio = %.4f\n\n', ...
    param1_name, best_ratio_param1, param2_name, best_ratio_param2, best_ratio);

%% 8. 绘制三张指标图
plotOptimizationMap(param1_vec, param2_vec, workspace_area_map, ...
    param1_name, param2_name, '工作空间面积', 'Workspace Area / mm^2');

plotOptimizationMap(param1_vec, param2_vec, rect_area_map, ...
    param1_name, param2_name, '最大矩形面积', 'Max Rectangle Area / mm^2');

plotOptimizationMap(param1_vec, param2_vec, area_ratio_map, ...
    param1_name, param2_name, '面积比', 'Area Ratio');

%% 9. 保存结果
result_file = sprintf('two_link_optimization_%s_%s.mat', param1_name, param2_name);

save(result_file, ...
    'param1_name', 'param2_name', 'param1_vec', 'param2_vec', ...
    'theta_lim', 'phi_lim', 'nTheta', 'nPhi', 'Nx', 'Ny', ...
    'workspace_area_map', 'rect_area_map', 'area_ratio_map', ...
    'best_ws_area', 'best_ws_param1', 'best_ws_param2', ...
    'best_rect_area', 'best_rect_param1', 'best_rect_param2', ...
    'best_ratio', 'best_ratio_param1', 'best_ratio_param2');

fprintf('结果已保存到：%s\n', result_file);

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

function plotOptimizationMap(param1_vec, param2_vec, metric_map, param1_name, param2_name, fig_title, colorbar_label)
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

    title(sprintf('%s\n最优：%s = %.2f mm, %s = %.2f mm, value = %.4f', ...
        fig_title, param1_name, best_param1, param2_name, best_param2, best_value), ...
        'Interpreter', 'none');

    cb = colorbar;
    ylabel(cb, colorbar_label, 'Interpreter', 'none');

    colormap jet;
    grid on;
    box on;
    view(45, 30);
end
