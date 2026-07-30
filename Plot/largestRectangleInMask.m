function [bestArea, bestR1, bestR2, bestC1, bestC2] = largestRectangleInMask(M, dx, dy)
% 在二值矩阵 M 中寻找面积最大的全 true 矩形
%
% M 的行对应 y 方向，列对应 x 方向
% dx, dy 是每个网格单元的实际尺寸

    [nRow, nCol] = size(M);

    heights = zeros(1, nCol);

    bestArea = 0;
    bestR1 = NaN;
    bestR2 = NaN;
    bestC1 = NaN;
    bestC2 = NaN;

    for r = 1:nRow

        % 更新直方图高度
        for c = 1:nCol
            if M(r, c)
                heights(c) = heights(c) + 1;
            else
                heights(c) = 0;
            end
        end

        % 当前行为底边，求直方图中的最大矩形
        [areaCells, h, w, c1, c2] = largestRectangleInHistogram(heights);

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
% h 是每一列的高度
% 返回：
% bestAreaCells : 最大矩形包含的网格数
% bestH         : 矩形高度，单位为网格数
% bestW         : 矩形宽度，单位为网格数
% bestL, bestR  : 左右列索引

    h = [h, 0];   % 哨兵
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