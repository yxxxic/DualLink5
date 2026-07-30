function [workspace_area, rect_area, area_ratio] = evaluateWorkspaceMetrics( ...
    params, theta_lim, phi_lim, nTheta, nPhi, Nx, Ny)
% 计算指标的函数，把三大指标放进来


%% 1. 采样网格
theta_vec = linspace(theta_lim(1), theta_lim(2), nTheta);
phi_vec   = linspace(phi_lim(1),   phi_lim(2),   nPhi);

[Theta, Phi] = meshgrid(theta_vec, phi_vec);

x_G = nan(size(Theta));
y_G = nan(size(Theta));

%% 2. 遍历所有采样点，计算末端点 G
for k = 1:numel(Theta)

    th = Theta(k);
    ph = Phi(k);

    try
        if check_interference(th, ph, params)

            [~, ~, ~, ~, ~, ~, ~, ~, ~, ~, x_G(k), y_G(k)] = ...
                modelingfx(th, ph, params);

        end
    catch
        x_G(k) = NaN;
        y_G(k) = NaN;
    end
end

%% 3. 提取有效工作空间点
id_ok = ~isnan(x_G) & ~isnan(y_G);

x_ws = x_G(id_ok);
y_ws = y_G(id_ok);

if length(x_ws) < 3
    workspace_area = NaN;
    rect_area      = NaN;
    area_ratio     = NaN;
    return;
end

% 去掉重复点
P = unique([x_ws(:), y_ws(:)], 'rows');

if size(P, 1) < 3
    workspace_area = NaN;
    rect_area      = NaN;
    area_ratio     = NaN;
    return;
end

x_ws = P(:, 1);
y_ws = P(:, 2);

%% 4. alphaShape 计算工作空间面积
try
    shp0 = alphaShape(x_ws, y_ws);

    alpha_ref = criticalAlpha(shp0, 'one-region');

    alpha = alpha_ref;

    shp = alphaShape(x_ws, y_ws, alpha);

    workspace_area = area(shp);

catch
    workspace_area = NaN;
    rect_area      = NaN;
    area_ratio     = NaN;
    return;
end

if isnan(workspace_area) || workspace_area <= 0
    rect_area  = NaN;
    area_ratio = NaN;
    return;
end

%% 5. 搜索工作空间内部最大轴对齐矩形
x_min = min(x_ws);
x_max = max(x_ws);
y_min = min(y_ws);
y_max = max(y_ws);

if x_max <= x_min || y_max <= y_min
    rect_area  = NaN;
    area_ratio = NaN;
    return;
end

x_edges = linspace(x_min, x_max, Nx + 1);
y_edges = linspace(y_min, y_max, Ny + 1);

dx = x_edges(2) - x_edges(1);
dy = y_edges(2) - y_edges(1);

x_centers = 0.5 * (x_edges(1:end-1) + x_edges(2:end));
y_centers = 0.5 * (y_edges(1:end-1) + y_edges(2:end));

[Xc, Yc] = meshgrid(x_centers, y_centers);

[XL, YB] = meshgrid(x_edges(1:end-1), y_edges(1:end-1));
[XR, YT] = meshgrid(x_edges(2:end),   y_edges(2:end));

inside_center = inShape(shp, Xc, Yc);

inside_lb = inShape(shp, XL, YB);
inside_rb = inShape(shp, XR, YB);
inside_lt = inShape(shp, XL, YT);
inside_rt = inShape(shp, XR, YT);

inside_mask = inside_center & inside_lb & inside_rb & inside_lt & inside_rt;

if ~any(inside_mask(:))
    rect_area  = 0;
    area_ratio = 0;
    return;
end

[rect_area, ~, ~, ~, ~] = largestRectangleInMask(inside_mask, dx, dy);

if isnan(rect_area) || rect_area < 0
    rect_area = NaN;
    area_ratio = NaN;
    return;
end

%% 6. 面积比
area_ratio = rect_area / workspace_area;

end

