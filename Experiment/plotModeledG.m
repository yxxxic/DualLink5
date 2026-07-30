clc;
clear;
close all;

addpath('../Definition');
addpath('../Kinematics');
addpath('../Plot'); 
%% 输入 CSV 文件
% 水平放置
% file = 'record_20260520_164213.csv';
% file = 'record_20260520_164514.csv';
% file = 'record_20260527_143842.csv';
% file = 'record_20260527_144032.csv';
% file = 'record_20260527_144413.csv';
% file = 'record_20260527_150518.csv';

% file = 'record_20260528_095450.csv';
% file = 'record_20260528_095557.csv';
% file = 'record_20260528_095706.csv';
% file = 'record_20260528_095824.csv';
% file = 'record_20260528_100530.csv';
% file = 'record_20260528_100658.csv';

% 竖直放置
% file = 'record_20260529_142149.csv';
% file = 'record_20260529_142237.csv';
% file = 'record_20260529_142421.csv';  % 来回无负载  有效
% file = 'record_20260529_142454.csv';  % 150g去
file = 'record_20260529_142542.csv';  % 150g回
% file = 'record_20260529_142637.csv';  % 50g来回
% file = 'record_20260529_150350.csv';  % 来回无负载  有效
% file = 'record_20260529_152154.csv';  % 升高并依次放置50g 100g
%% 读取实验角度数据
run('../Definition/set_parameter.m');
data = extractExperimentAngles(file);

t_sec = data.t_sec;

angle1 = data.angle1;
angle2 = data.angle2;

ft_pos1 = data.ft_pos1;
ft_pos2 = data.ft_pos2;

%% ft_pos 不是一一对应采样，插值到完整时间轴
ft_pos1_interp = interp1( ...
    t_sec(data.idx_id1), ft_pos1(data.idx_id1), ...
    t_sec, 'linear', 'extrap');

ft_pos2_interp = interp1( ...
    t_sec(data.idx_id2), ft_pos2(data.idx_id2), ...
    t_sec, 'linear', 'extrap');

%% 分别用两组角度建模得到 G 点
[x_G_angle, y_G_angle] = computeGTrajectory(angle1, angle2, params);
[x_G_ft, y_G_ft] = computeGTrajectory(ft_pos1_interp, ft_pos2_interp, params);

%% 绘制两组 G 点（颜色渐变表示时间顺序）
figure('Color', 'w');
hold on;
grid on;
box on;
axis equal;

valid_angle = isfinite(x_G_angle) & isfinite(y_G_angle) & isfinite(t_sec);
valid_ft = isfinite(x_G_ft) & isfinite(y_G_ft) & isfinite(t_sec);

if ~any(valid_angle)
    error('angle1/angle2 建模结果中没有有效 G 点。');
end

if ~any(valid_ft)
    error('ft_pos1/ft_pos2 建模结果中没有有效 G 点。');
end

plotTimeGradientLine( ...
    x_G_angle(valid_angle), y_G_angle(valid_angle), t_sec(valid_angle), ...
    2.2, 'G from mt6835 angle1/angle2');

plotTimeGradientLine( ...
    x_G_ft(valid_ft), y_G_ft(valid_ft), t_sec(valid_ft), ...
    4.0, 'G from interpolated ft\_pos1/ft\_pos2');

colormap turbo;
cb = colorbar;
ylabel(cb, 'Time / s');

xlabel('x_G / mm');
ylabel('y_G / mm');
% title('Modeled G Trajectories from Two Angle Sources (Color = Time)');
% legend('Location', 'best');
hold off;
saveFigIEEE('6');
%% 如需同时检查插值后的角度，可打开下面这段
% figure('Color', 'w');
% hold on;
% grid on;
% box on;
% plot(t_sec, angle1, 'LineWidth', 1.1, 'DisplayName', 'angle1');
% plot(t_sec, angle2, 'LineWidth', 1.1, 'DisplayName', 'angle2');
% plot(t_sec, ft_pos1_interp, '--', 'LineWidth', 1.1, 'DisplayName', 'ft\_pos1 interp');
% plot(t_sec, ft_pos2_interp, '--', 'LineWidth', 1.1, 'DisplayName', 'ft\_pos2 interp');
% xlabel('Time / s');
% ylabel('Angle / deg');
% legend('Location', 'best');
% hold off;

%% 局部函数
function [x_G, y_G] = computeGTrajectory(theta_vec, phi_vec, params)
    n = numel(theta_vec);
    x_G = nan(size(theta_vec));
    y_G = nan(size(phi_vec));

    for k = 1:n
        theta = theta_vec(k);
        phi = phi_vec(k);

        if isnan(theta) || isnan(phi)
            continue;
        end

        try
            [~, ~, ~, ~, ~, ~, ~, ~, ~, ~, x_G(k), y_G(k)] = ...
                modelingfx(theta, phi, params);
        catch
            x_G(k) = NaN;
            y_G(k) = NaN;
        end
    end
end

function h = plotTimeGradientLine(x, y, t, line_width, display_name)
    x = x(:).';
    y = y(:).';
    t = t(:).';

    h = surface( ...
        [x; x], ...
        [y; y], ...
        zeros(2, numel(x)), ...
        [t; t], ...
        'FaceColor', 'none', ...
        'EdgeColor', 'interp', ...
        'LineWidth', line_width, ...
        'DisplayName', display_name);
end
