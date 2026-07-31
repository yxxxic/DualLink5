function result = largestRectangleInMask(mask, dx, dy)
if ~isValidMask(mask) || ~isPositiveFiniteScalar(dx) || ...
        ~isPositiveFiniteScalar(dy)
    error('duallink5:workspace:InvalidRectangleInput', ...
        'mask must be 2-D and dx/dy must be positive finite scalars.');
end
mask = logical(mask);
dx = double(dx);
dy = double(dy);
heights = zeros(1, size(mask, 2));
best = emptyRectangle();

for row = 1:size(mask, 1)
    heights(mask(row, :)) = heights(mask(row, :)) + 1;
    heights(~mask(row, :)) = 0;
    candidate = histogramRectangle(heights, row);
    if candidate.areaCells > best.areaCells
        best = candidate;
    end
end

result = best;
result.area = best.areaCells * dx * dy;
if best.areaCells == 0
    result.width = 0;
    result.height = 0;
else
    result.width = (best.right - best.left + 1) * dx;
    result.height = (best.bottom - best.top + 1) * dy;
end
end

function best = histogramRectangle(heights, row)
extended = [heights, 0];
stack = zeros(1, numel(extended));
stackSize = 0;
best = emptyRectangle();

for index = 1:numel(extended)
    while stackSize > 0 && ...
            extended(stack(stackSize)) > extended(index)
        height = extended(stack(stackSize));
        stackSize = stackSize - 1;
        if stackSize == 0
            left = 1;
        else
            left = stack(stackSize) + 1;
        end
        right = index - 1;
        area = height * (right - left + 1);
        if area > best.areaCells
            best.areaCells = area;
            best.bottom = row;
            best.top = row - height + 1;
            best.left = left;
            best.right = right;
        end
    end
    stackSize = stackSize + 1;
    stack(stackSize) = index;
end
end

function result = emptyRectangle()
result = struct( ...
    'areaCells', 0, ...
    'top', 0, ...
    'bottom', 0, ...
    'left', 0, ...
    'right', 0);
end

function valid = isValidMask(mask)
valid = islogical(mask) && ismatrix(mask);
if isnumeric(mask) && isreal(mask) && ismatrix(mask)
    valid = all(isfinite(mask), 'all') && ...
        all(mask == 0 | mask == 1, 'all');
end
end

function valid = isPositiveFiniteScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value > 0;
end
