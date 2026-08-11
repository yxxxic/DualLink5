function curves = extractZeroContours(theta, phi, values)
curves = repmat(struct('theta', [], 'phi', []), 0, 1);
if numel(theta) < 2 || numel(phi) < 2 || ...
        ~any(isfinite(values), 'all')
    return
end
matrix = contourc(theta, phi, values.', [0, 0]);
column = 1;
while column <= size(matrix, 2)
    pointCount = matrix(2, column);
    first = column + 1;
    last = column + pointCount;
    if pointCount >= 2 && last <= size(matrix, 2)
        curve.theta = matrix(1, first:last);
        curve.phi = matrix(2, first:last);
        curves(end + 1, 1) = curve; %#ok<AGROW>
    end
    column = last + 1;
end
end
