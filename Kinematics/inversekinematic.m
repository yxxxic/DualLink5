% 已知点G坐标 G = [x_G, y_G]
% 求theta和phi

clc;
clear;
close all;

addpath('../Definition'); 
addpath('../Kinematics');

run('../Definition/set_parameter.m');

theta_lim = [0, 180];
phi_lim   = [0, 90];

nTheta = 100;
nPhi   = 100;

Nx = 100;
Ny = 100;

doPlot = true;

%% 1. 计算工作空间内的最大矩形
[rect_area, rect_length, rect_width, rect_corners, info] = ...
    calcMaxWorkspaceRectangle(params, theta_lim, phi_lim, ...
    nTheta, nPhi, Nx, Ny, doPlot);

%% 2. 已知目标点 G
x_G = -20;
y_G = 140;

G = [x_G, y_G];

%% 3. 判断 G 是否在最大矩形内部

% 闭合矩形角点，用于 inpolygon
rect_x = [
    rect_corners(:,1);
    rect_corners(1,1)
];

rect_y = [
    rect_corners(:,2);
    rect_corners(1,2)
];

% inRect 表示严格在内部
% onRect 表示在边界上
[inRect, onRect] = inpolygon(x_G, y_G, rect_x, rect_y);

% 如果允许点在边界上，则使用：
isAllowed = inRect || onRect;

% 如果你要求必须严格在矩形内部，不允许在边界上，则改成：
% isAllowed = inRect && ~onRect;

%% 4. 绘制目标点 G
figure(gcf);
hold on;

if isAllowed
    plot(x_G, y_G, 'rp', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', 'r');

    text(x_G, y_G, '  G', ...
        'Color', 'r', ...
        'FontSize', 12, ...
        'FontWeight', 'bold');
else
    plot(x_G, y_G, 'kp', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', 'k');

    text(x_G, y_G, '  G outside', ...
        'Color', 'k', ...
        'FontSize', 12, ...
        'FontWeight', 'bold');
end

%% 5. 如果 G 不在矩形内，则不进行逆运动学
if ~isAllowed
    fprintf('点 G = [%.4f, %.4f] 不在最大矩形内部，禁止进行逆运动学。\n', ...
        x_G, y_G);

    fprintf('最大矩形四个角点为：\n');
    fprintf('左下角: [%.4f, %.4f]\n', rect_corners(1,1), rect_corners(1,2));
    fprintf('右下角: [%.4f, %.4f]\n', rect_corners(2,1), rect_corners(2,2));
    fprintf('右上角: [%.4f, %.4f]\n', rect_corners(3,1), rect_corners(3,2));
    fprintf('左上角: [%.4f, %.4f]\n', rect_corners(4,1), rect_corners(4,2));

    return;
end

fprintf('点 G = [%.4f, %.4f] 在最大矩形内部，允许进行逆运动学。\n', ...
    x_G, y_G);

%% 6. 由 G 计算 P
x_B = params.l_base;
y_B = 0;

x_P = (x_G + x_B) / 2;
y_P = (y_G + y_B) / 2;

P = [x_P, y_P];

%% 7. 逆运动学求解
sol = inverse_kinematics_from_P(P, params);

%% 8. 输出所有解
if isempty(sol)
    fprintf('逆运动学无解。\n');
else
    for k = 1:length(sol)
        fprintf('解 %d: theta = %.4f deg, phi = %.4f deg\n', ...
            k, sol(k).theta, sol(k).phi);
    end
end