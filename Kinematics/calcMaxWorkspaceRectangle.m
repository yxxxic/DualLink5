function [rect_area, rect_length, rect_width, rect_corners, info] = ...
    calcMaxWorkspaceRectangle(params, theta_lim, phi_lim, nTheta, nPhi, Nx, Ny, doPlot, alphaScale)
% calcMaxWorkspaceRectangle
% 
% 在机构工作空间内搜索最大轴对齐矩形，并输出矩形面积、长宽和四个角点。
%
% 输入：
%   params      : 机构参数结构体
%   theta_lim   : theta 角范围，例如 [0, 180]
%   phi_lim     : phi 角范围，例如 [0, 90]
%   nTheta      : theta 采样数，例如 100
%   nPhi        : phi 采样数，例如 100
%   Nx          : 工作空间 x 方向搜索网格数，例如 500
%   Ny          : 工作空间 y 方向搜索网格数，例如 500
%   doPlot      : 是否绘图，true 或 false
%   alphaScale  : alphaShape 的 alpha 缩放系数，默认 1
%
% 输出：
%   rect_area    : 最大矩形面积
%   rect_length  : 矩形 x 方向长度
%   rect_width   : 矩形 y 方向宽度
%   rect_corners : 矩形四个角点坐标，顺序为：
%                  左下、右下、右上、左上
%   info         : 额外信息结构体

    %% 默认参数
    if nargin < 2 || isempty(theta_lim)
        theta_lim = [0, 180];
    end

    if nargin < 3 || isempty(phi_lim)
        phi_lim = [0, 90];
    end

    if nargin < 4 || isempty(nTheta)
        nTheta = 100;
    end

    if nargin < 5 || isempty(nPhi)
        nPhi = 100;
    end

    if nargin < 6 || isempty(Nx)
        Nx = 500;
    end

    if nargin < 7 || isempty(Ny)
        Ny = 500;
    end

    if nargin < 8 || isempty(doPlot)
        doPlot = true;
    end

    if nargin < 9 || isempty(alphaScale)
        alphaScale = 1;
    end

    %% 1. 关节角采样网格
    theta_vec = linspace(theta_lim(1), theta_lim(2), nTheta);
    phi_vec   = linspace(phi_lim(1),   phi_lim(2),   nPhi);

    [Theta, Phi] = meshgrid(theta_vec, phi_vec);

    x_G = nan(size(Theta));
    y_G = nan(size(Theta));

    %% 2. 遍历所有采样点，获得有效工作空间点
    for i = 1:numel(Theta)

        th = Theta(i); 
        ph = Phi(i);

        if check_interference(th, ph, params)

            [~, ~, ~, ~, ~, ~, ~, ~, ~, ~, x_G(i), y_G(i)] = ...
                modelingfx(th, ph, params);

        end
    end

    %% 3. 提取有效工作空间点
    id_ok = ~isnan(x_G) & ~isnan(y_G);

    x_ws = x_G(id_ok);
    y_ws = y_G(id_ok);

    if isempty(x_ws)
        error('没有得到有效工作空间点，请检查角度范围、干涉条件或机构参数。');
    end

    % 去掉重复点
    P = unique([x_ws(:), y_ws(:)], 'rows');

    x_ws = P(:, 1);
    y_ws = P(:, 2);

    if numel(x_ws) < 3
        error('有效工作空间点数量太少，无法构造 alphaShape。');
    end

    %% 4. 用 alphaShape 表示工作空间边界
    shp0 = alphaShape(x_ws, y_ws);

    alpha_ref = criticalAlpha(shp0, 'one-region');

    alpha = alphaScale * alpha_ref;

    shp = alphaShape(x_ws, y_ws, alpha);

    workspace_area = area(shp);

    %% 5. 在工作空间内搜索最大轴对齐矩形
    x_min = min(x_ws);
    x_max = max(x_ws);
    y_min = min(y_ws);
    y_max = max(y_ws);

    x_edges = linspace(x_min, x_max, Nx + 1);
    y_edges = linspace(y_min, y_max, Ny + 1);

    dx = x_edges(2) - x_edges(1);
    dy = y_edges(2) - y_edges(1);

    % 网格单元中心
    x_centers = 0.5 * (x_edges(1:end-1) + x_edges(2:end));
    y_centers = 0.5 * (y_edges(1:end-1) + y_edges(2:end));

    [Xc, Yc] = meshgrid(x_centers, y_centers);

    % 网格单元四个角点
    [XL, YB] = meshgrid(x_edges(1:end-1), y_edges(1:end-1));
    [XR, YT] = meshgrid(x_edges(2:end),   y_edges(2:end));

    % 为了保守，要求中心点和四个角点都在工作空间内
    inside_center = inShape(shp, Xc, Yc);

    inside_lb = inShape(shp, XL, YB);
    inside_rb = inShape(shp, XR, YB);
    inside_lt = inShape(shp, XL, YT);
    inside_rt = inShape(shp, XR, YT);

    inside_mask = inside_center & inside_lb & inside_rb & inside_lt & inside_rt;

    %% 6. 在二值矩阵中寻找最大矩形
    [rect_area, r1, r2, c1, c2] = largestRectangleInMask(inside_mask, dx, dy);

    if rect_area <= 0 || isnan(r1) || isnan(c1)
        error('未能在工作空间内找到有效矩形。');
    end

    rect_x1 = x_edges(c1);
    rect_x2 = x_edges(c2 + 1);
    rect_y1 = y_edges(r1);
    rect_y2 = y_edges(r2 + 1);

    rect_length = rect_x2 - rect_x1;
    rect_width  = rect_y2 - rect_y1;

    % 四个角点：左下、右下、右上、左上
    rect_corners = [
        rect_x1, rect_y1;
        rect_x2, rect_y1;
        rect_x2, rect_y2;
        rect_x1, rect_y2
    ];

    %% 7. 额外输出信息
    info = struct();

    info.workspace_area = workspace_area;
    info.alpha = alpha;
    info.alpha_ref = alpha_ref;
    info.alphaScale = alphaScale;

    info.x_ws = x_ws;
    info.y_ws = y_ws;
    info.shp = shp;

    info.rect_x1 = rect_x1;
    info.rect_x2 = rect_x2;
    info.rect_y1 = rect_y1;
    info.rect_y2 = rect_y2;

    info.rect_length_x = rect_length;
    info.rect_width_y = rect_width;
    info.rect_long_side = max(rect_length, rect_width);
    info.rect_short_side = min(rect_length, rect_width);

    info.Nx = Nx;
    info.Ny = Ny;
    info.nTheta = nTheta;
    info.nPhi = nPhi;

    %% 8. 打印结果
    fprintf('工作空间面积约为：%.4f mm^2\n', workspace_area);
    fprintf('最大矩形面积约为：%.4f mm^2\n', rect_area);
    fprintf('矩形 x 方向长度：%.4f mm\n', rect_length);
    fprintf('矩形 y 方向宽度：%.4f mm\n', rect_width);

    fprintf('矩形四个角点坐标：\n');
    fprintf('左下角：x = %.4f, y = %.4f\n', rect_corners(1,1), rect_corners(1,2));
    fprintf('右下角：x = %.4f, y = %.4f\n', rect_corners(2,1), rect_corners(2,2));
    fprintf('右上角：x = %.4f, y = %.4f\n', rect_corners(3,1), rect_corners(3,2));
    fprintf('左上角：x = %.4f, y = %.4f\n', rect_corners(4,1), rect_corners(4,2));

    %% 9. 绘图
    if doPlot

        figure;
        hold on;

        % 工作空间散点
        plot(x_ws, y_ws, '.', ...
            'MarkerSize', 2);

        % 工作空间边界
        % plot(shp, ...
        %     'FaceColor', [0.3 0.7 1.0], ...
        %     'FaceAlpha', 0.20, ...
        %     'EdgeColor', 'r', ...
        %     'LineWidth', 1.2);

        % 最大矩形
        rect_x = [
            rect_corners(1,1), ...
            rect_corners(2,1), ...
            rect_corners(3,1), ...
            rect_corners(4,1), ...
            rect_corners(1,1)
        ];

        rect_y = [
            rect_corners(1,2), ...
            rect_corners(2,2), ...
            rect_corners(3,2), ...
            rect_corners(4,2), ...
            rect_corners(1,2)
        ];

        patch(rect_x, rect_y, 'g', ...
            'FaceAlpha', 0.30, ...
            'EdgeColor', 'k', ...
            'LineWidth', 2);

        axis equal;
        grid on;
        xlabel('x_G');
        ylabel('y_G');

        title(sprintf('Workspace Area = %.2f mm^2, Max Rect Area = %.2f mm^2', ...
            workspace_area, rect_area));

    end

end


function [bestArea, bestR1, bestR2, bestC1, bestC2] = largestRectangleInMask(M, dx, dy)
% 在二值矩阵 M 中寻找面积最大的全 true 矩形
%
% M 的行对应 y 方向
% M 的列对应 x 方向
% dx, dy 是每个网格单元的实际尺寸

    [nRow, nCol] = size(M);

    heights = zeros(1, nCol);

    bestArea = 0;
    bestR1 = NaN;
    bestR2 = NaN;
    bestC1 = NaN;
    bestC2 = NaN;

    for r = 1:nRow

        % 更新当前行对应的直方图高度
        for c = 1:nCol
            if M(r, c)
                heights(c) = heights(c) + 1;
            else
                heights(c) = 0;
            end
        end

        % 当前行为底边，求直方图中的最大矩形
        [areaCells, h, ~, c1, c2] = largestRectangleInHistogram(heights);

        areaReal = areaCells * dx * dy;

        if areaReal > bestArea
            bestArea = areaReal;

            bestR2 = r;
            bestR1 = r - h + 1;

            bestC1 = c1;
            bestC2 = c2;
        end
    end
end


function [bestAreaCells, bestH, bestW, bestL, bestR] = largestRectangleInHistogram(h)
% 求直方图中的最大矩形
%
% 输入：
%   h : 每一列的高度
%
% 输出：
%   bestAreaCells : 最大矩形包含的网格数量
%   bestH         : 矩形高度，单位为网格数
%   bestW         : 矩形宽度，单位为网格数
%   bestL         : 左列编号
%   bestR         : 右列编号

    h = [h, 0];   % 添加哨兵
    stack = [];

    bestAreaCells = 0;
    bestH = 0;
    bestW = 0;
    bestL = NaN;
    bestR = NaN;

    for i = 1:length(h)

        while ~isempty(stack) && h(i) < h(stack(end))

            top = stack(end);
            stack(end) = [];

            height = h(top);

            if isempty(stack)
                left = 1;
            else
                left = stack(end) + 1;
            end

            right = i - 1;
            width = right - left + 1;

            areaCells = height * width;

            if areaCells > bestAreaCells
                bestAreaCells = areaCells;
                bestH = height;
                bestW = width;
                bestL = left;
                bestR = right;
            end
        end

        stack(end + 1) = i;
    end
end