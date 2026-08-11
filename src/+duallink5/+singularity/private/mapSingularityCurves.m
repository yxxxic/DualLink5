function mapped = mapSingularityCurves( ...
        curves, geometry, options, thetaGrid, phiGrid, mechanicalMask)
mapped = repmat(struct('theta', [], 'phi', [], 'pointG', [], ...
    'adjacentMechanical', []), 0, 1);
for curveIndex = 1:numel(curves)
    curve = curves(curveIndex);
    pointCount = numel(curve.theta);
    pointG = nan(2, pointCount);
    adjacent = false(1, pointCount);
    for pointIndex = 1:pointCount
        q = [curve.theta(pointIndex), curve.phi(pointIndex)];
        side = evaluateSide(q, geometry, options);
        if side.reachable
            pointG(:, pointIndex) = side.pointG;
        end
        [~, row] = min(abs(thetaGrid(:, 1) - q(1)));
        [~, column] = min(abs(phiGrid(1, :) - q(2)));
        rowRange = max(1, row - 1):min(size(mechanicalMask, 1), row + 1);
        columnRange = max(1, column - 1): ...
            min(size(mechanicalMask, 2), column + 1);
        adjacent(pointIndex) = any( ...
            mechanicalMask(rowRange, columnRange), 'all');
    end
    mapped(curveIndex).theta = curve.theta;
    mapped(curveIndex).phi = curve.phi;
    mapped(curveIndex).pointG = pointG;
    mapped(curveIndex).adjacentMechanical = adjacent;
end
end
