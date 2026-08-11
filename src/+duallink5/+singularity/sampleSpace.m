function samples = sampleSpace(grid, geometry, options)
if nargin < 3
    options = struct();
end
geometry = duallink5.model.validateGeometry(geometry);
options = normalizeOptions(options, geometry);
[theta, phi] = validateGrid(grid);
[thetaGrid, phiGrid] = ndgrid(theta, phi);
gridShape = size(thetaGrid);
samples = allocateSamples(thetaGrid, phiGrid);
topologyReference = prepareTopologyReference(geometry, options);

for linearIndex = 1:numel(thetaGrid)
    [row, column] = ind2sub(gridShape, linearIndex);
    q = [thetaGrid(row, column), phiGrid(row, column)];
    side = evaluateSide(q, geometry, options);
    samples = copySide(samples, side, row, column);
    assessment = assessIdealAssembly( ...
        q, geometry, options, topologyReference);
    samples.reasonMap(row, column) = assessment.statusCode;
    samples.mechanicallyValidMask(row, column) = assessment.valid;
    samples.safeUsableMask(row, column) = ...
        assessment.valid && ~side.near.any;
    if assessment.valid && side.near.any
        samples.reasonMap(row, column) = "NEAR_SINGULAR";
    end
end

samples.curves = buildCurves( ...
    samples, theta, phi, geometry, options);
samples.metadata.units = geometry.units;
samples.metadata.displayLengthUnit = "mm";
samples.metadata.displayAngleUnit = "deg";
samples.metadata.referenceQ = geometry.analysis.referenceQ;
samples.metadata.geometryVersion = geometry.version;
samples.metadata.branchId = options.branchId;
samples.metadata.collisionProfile = options.collisionProfile;
samples.metadata.topologyProfile = options.topologyProfile;
samples.metadata.exactThreshold = options.exactThreshold;
samples.metadata.nearMetricThreshold = options.nearMetricThreshold;
samples.metadata.nearConditionThreshold = ...
    options.nearConditionThreshold;
end

function [theta, phi] = validateGrid(grid)
if ~isstruct(grid) || ~isscalar(grid) || ...
        ~isfield(grid, 'theta') || ~isfield(grid, 'phi') || ...
        ~isIncreasingFiniteVector(grid.theta) || ...
        ~isIncreasingFiniteVector(grid.phi)
    error('duallink5:singularity:InvalidGrid', ...
        ['grid.theta and grid.phi must be nonempty, finite, real, ', ...
         'strictly increasing vectors.']);
end
theta = full(double(grid.theta(:).'));
phi = full(double(grid.phi(:).'));
end

function valid = isIncreasingFiniteVector(value)
valid = isnumeric(value) && isreal(value) && isvector(value) && ...
    ~isempty(value) && all(isfinite(value), 'all') && ...
    all(diff(double(value(:))) > 0);
end

function samples = allocateSamples(thetaGrid, phiGrid)
shape = size(thetaGrid);
samples.thetaGrid = thetaGrid;
samples.phiGrid = phiGrid;
samples.x = nan(shape);
samples.y = nan(shape);
samples.orientationSensitivity = nan(shape);
samples.closureDistance = nan(shape);
samples.branchId = zeros(shape, 'int8');
samples.theoreticalReachableMask = false(shape);
samples.mechanicallyValidMask = false(shape);
samples.safeUsableMask = false(shape);
samples.reasonMap = strings(shape);
samples.classificationMap = strings(shape);
samples.constraintMatrix = nan([2, 2, shape]);
samples.taskActuationMatrix = nan([2, 2, shape]);
samples.taskJacobian = nan([2, 2, shape]);
samples.singularValues.constraint = nan([2, shape]);
samples.singularValues.task = nan([2, shape]);
names = {'typeITheta', 'typeIPhi', 'typeII'};
for index = 1:numel(names)
    name = names{index};
    samples.signedIndicator.(name) = nan(shape);
    samples.margins.(name) = nan(shape);
end
samples.conditionNumber.constraint = inf(shape);
samples.conditionNumber.task = inf(shape);
flagNames = {'typeITheta', 'typeIPhi', 'typeI', 'typeII', ...
    'typeIII', 'any'};
for index = 1:numel(flagNames)
    name = flagNames{index};
    samples.exact.(name) = false(shape);
    samples.near.(name) = false(shape);
end
samples.near.taskCondition = false(shape);
end

function samples = copySide(samples, side, row, column)
samples.theoreticalReachableMask(row, column) = side.reachable;
samples.classificationMap(row, column) = side.classification;
samples.branchId(row, column) = side.branchId;
samples.closureDistance(row, column) = side.closureDistance;
samples.x(row, column) = side.pointG(1);
samples.y(row, column) = side.pointG(2);
samples.orientationSensitivity(row, column) = ...
    side.orientationSensitivity;
samples.constraintMatrix(:, :, row, column) = ...
    side.matrices.constraint;
samples.taskActuationMatrix(:, :, row, column) = ...
    side.matrices.taskActuation;
samples.taskJacobian(:, :, row, column) = ...
    side.matrices.taskJacobian;
samples.singularValues.constraint(:, row, column) = ...
    side.singularValues.constraint;
samples.singularValues.task(:, row, column) = ...
    side.singularValues.task;
metricNames = {'typeITheta', 'typeIPhi', 'typeII'};
for index = 1:numel(metricNames)
    name = metricNames{index};
    samples.signedIndicator.(name)(row, column) = ...
        side.signedIndicator.(name);
    samples.margins.(name)(row, column) = side.margins.(name);
end
samples.conditionNumber.constraint(row, column) = ...
    side.conditionNumber.constraint;
samples.conditionNumber.task(row, column) = ...
    side.conditionNumber.task;
flagNames = {'typeITheta', 'typeIPhi', 'typeI', 'typeII', ...
    'typeIII', 'any'};
for index = 1:numel(flagNames)
    name = flagNames{index};
    samples.exact.(name)(row, column) = side.exact.(name);
    samples.near.(name)(row, column) = side.near.(name);
end
samples.near.taskCondition(row, column) = ...
    side.near.taskCondition;
end

function curves = buildCurves(samples, theta, phi, geometry, options)
typeIThetaValues = samples.signedIndicator.typeITheta;
typeIPhiValues = samples.signedIndicator.typeIPhi;
typeIThetaValues(~samples.theoreticalReachableMask) = NaN;
typeIPhiValues(~samples.theoreticalReachableMask) = NaN;
typeITheta = extractZeroContours(theta, phi, typeIThetaValues);
typeIPhi = extractZeroContours(theta, phi, typeIPhiValues);
bounds = [theta([1, end]); phi([1, end])];
typeITheta = refineTypeICurves( ...
    typeITheta, 'typeITheta', geometry, options, bounds);
typeIPhi = refineTypeICurves( ...
    typeIPhi, 'typeIPhi', geometry, options, bounds);

L = geometry.links;
analyticDistance = hypot( ...
    L.link1 * cos(samples.thetaGrid) - L.link5 + ...
    L.link2 * cos(samples.phiGrid), ...
    L.link1 * sin(samples.thetaGrid) - ...
    L.link2 * sin(samples.phiGrid));
outerTarget = L.link3 + L.link4;
innerTarget = abs(L.link4 - L.link3);
typeIIOuter = extractZeroContours( ...
    theta, phi, analyticDistance - outerTarget);
typeIIInner = extractZeroContours( ...
    theta, phi, analyticDistance - innerTarget);

typeIIOuter = refineTypeIICurves( ...
    typeIIOuter, outerTarget, geometry, bounds);
typeIIInner = refineTypeIICurves( ...
    typeIIInner, innerTarget, geometry, bounds);

exactIndices = find(samples.exact.typeIII & ...
    isfinite(samples.closureDistance));
sampledTypeIII = [reshape(samples.thetaGrid(exactIndices), 1, []); ...
    reshape(samples.phiGrid(exactIndices), 1, [])];
sampledDistance = reshape(samples.closureDistance(exactIndices), 1, []);
outerSeed = abs(sampledDistance - outerTarget) <= ...
    abs(sampledDistance - innerTarget);
typeIIIOuter = findTypeIIILoci(typeIIOuter, outerTarget, ...
    geometry, options, bounds, sampledTypeIII(:, outerSeed));
typeIIIInner = findTypeIIILoci(typeIIInner, innerTarget, ...
    geometry, options, bounds, sampledTypeIII(:, ~outerSeed));

curves.typeITheta = mapSingularityCurves( ...
    typeITheta, geometry, options, samples.thetaGrid, ...
    samples.phiGrid, samples.mechanicallyValidMask);
curves.typeIPhi = mapSingularityCurves( ...
    typeIPhi, geometry, options, samples.thetaGrid, ...
    samples.phiGrid, samples.mechanicallyValidMask);
curves.typeIIOuter = mapTangencyContours( ...
    typeIIOuter, outerTarget, geometry, samples);
curves.typeIIInner = mapTangencyContours( ...
    typeIIInner, innerTarget, geometry, samples);
curves.typeIII = [mapTangencyContours( ...
    typeIIIOuter, outerTarget, geometry, samples); ...
    mapTangencyContours(typeIIIInner, innerTarget, geometry, samples)];
end

function refined = refineTypeICurves( ...
        curves, indicatorName, geometry, options, bounds)
template = struct('theta', [], 'phi', []);
refined = repmat(template, 0, 1);
for curveIndex = 1:numel(curves)
    source = [curves(curveIndex).theta; curves(curveIndex).phi];
    projected = nan(size(source));
    keep = false(1, size(source, 2));
    for pointIndex = 1:size(source, 2)
        [q, converged] = refineTypeIRoot( ...
            source(:, pointIndex), indicatorName, ...
            geometry, options, bounds);
        [~, valid, publicExact] = typeIIndicator( ...
            q, indicatorName, geometry, options);
        if converged && valid && publicExact
            projected(:, pointIndex) = q;
            keep(pointIndex) = true;
        end
    end
    projected = projected(:, keep);
    if size(projected, 2) >= 2
        refined(end + 1, 1) = struct( ...
            'theta', projected(1, :), ...
            'phi', projected(2, :)); %#ok<AGROW>
    end
end
end

function [q, converged] = refineTypeIRoot( ...
        q, indicatorName, geometry, options, bounds)
converged = false;
metricTolerance = max(100 * eps, ...
    min(options.exactThreshold, 1e-10));
acceptanceTolerance = min(metricTolerance, options.exactThreshold);
finiteDifferenceStep = 1e-7;
iteration = 0;
while iteration < 15
    iteration = iteration + 1;
    [metric, valid] = typeIIndicator( ...
        q, indicatorName, geometry, options);
    if ~valid
        return
    end
    if abs(metric) <= acceptanceTolerance
        converged = true;
        return
    end
    gradient = zeros(1, 2);
    for column = 1:2
        offset = zeros(2, 1);
        offset(column) = finiteDifferenceStep;
        [plusMetric, plusValid] = typeIIndicator( ...
            q + offset, indicatorName, geometry, options);
        [minusMetric, minusValid] = typeIIndicator( ...
            q - offset, indicatorName, geometry, options);
        if ~plusValid || ~minusValid
            return
        end
        gradient(column) = (plusMetric - minusMetric) / ...
            (2 * finiteDifferenceStep);
    end
    denominator = dot(gradient, gradient);
    if ~isfinite(denominator) || denominator <= eps
        return
    end
    correction = metric * gradient.' / denominator;
    if norm(correction) > 0.25
        correction = 0.25 * correction / norm(correction);
    end
    accepted = false;
    for backtrack = 0:8
        proposal = min(max(q - 0.5^backtrack * correction, ...
            bounds(:, 1)), bounds(:, 2));
        [proposalMetric, proposalValid] = typeIIndicator( ...
            proposal, indicatorName, geometry, options);
        if proposalValid && abs(proposalMetric) < abs(metric)
            q = proposal;
            accepted = true;
            break
        end
    end
    if ~accepted
        return
    end
end
[metric, valid] = typeIIndicator(q, indicatorName, geometry, options);
converged = valid && abs(metric) <= acceptanceTolerance;
end

function [value, valid, exact] = typeIIndicator( ...
        q, indicatorName, geometry, options)
side = evaluateSide(q.', geometry, options);
value = side.signedIndicator.(indicatorName);
valid = side.reachable && isfinite(value);
exact = side.exact.(indicatorName);
end

function mapped = mapTangencyContours( ...
        curves, targetDistance, geometry, samples)
template = struct('theta', [], 'phi', [], 'pointG', [], ...
    'adjacentMechanical', []);
if targetDistance <= geometry.tolerance.length
    mapped = repmat(template, 0, 1);
    return
end
mapped = repmat(template, numel(curves), 1);
L = geometry.links;
B = [L.link5; 0];
for curveIndex = 1:numel(curves)
    curve = curves(curveIndex);
    pointCount = numel(curve.theta);
    pointG = nan(2, pointCount);
    adjacent = false(1, pointCount);
    for pointIndex = 1:pointCount
        thetaValue = curve.theta(pointIndex);
        phiValue = curve.phi(pointIndex);
        E = L.link1 * [cos(thetaValue); sin(thetaValue)];
        C = [L.link5 - L.link2 * cos(phiValue); ...
            L.link2 * sin(phiValue)];
        delta = E - C;
        actualDistance = norm(delta);
        if actualDistance > geometry.tolerance.length
            unitCE = delta / actualDistance;
            along = (L.link3^2 - L.link4^2 + ...
                targetDistance^2) / (2 * targetDistance);
            D = C + along * unitCE;
            pointG(:, pointIndex) = D + E - B;
        end
        [~, row] = min(abs( ...
            samples.thetaGrid(:, 1) - thetaValue));
        [~, column] = min(abs( ...
            samples.phiGrid(1, :) - phiValue));
        rowRange = max(1, row - 1): ...
            min(size(samples.mechanicallyValidMask, 1), row + 1);
        columnRange = max(1, column - 1): ...
            min(size(samples.mechanicallyValidMask, 2), column + 1);
        adjacent(pointIndex) = any(samples.mechanicallyValidMask( ...
            rowRange, columnRange), 'all');
    end
    mapped(curveIndex).theta = curve.theta;
    mapped(curveIndex).phi = curve.phi;
    mapped(curveIndex).pointG = pointG;
    mapped(curveIndex).adjacentMechanical = adjacent;
end
end
