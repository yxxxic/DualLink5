function loci = findTypeIIILoci( ...
        typeIICurves, targetDistance, geometry, options, bounds, seedPoints)
candidatePoints = reshape(seedPoints, 2, []);
candidateTypes = strings(1, size(candidatePoints, 2));
for index = 1:size(candidatePoints, 2)
    [thetaMetric, phiMetric] = tangentIndicators( ...
        candidatePoints(:, index), targetDistance, geometry);
    if min(abs([thetaMetric, phiMetric])) <= options.exactThreshold
        if abs(thetaMetric) <= abs(phiMetric)
            candidateTypes(index) = "theta";
        else
            candidateTypes(index) = "phi";
        end
    end
end

detectionTolerance = max(options.exactThreshold, 1e-10);
for curveIndex = 1:numel(typeIICurves)
    points = [typeIICurves(curveIndex).theta; ...
        typeIICurves(curveIndex).phi];
    thetaMetric = nan(1, size(points, 2));
    phiMetric = nan(1, size(points, 2));
    for pointIndex = 1:size(points, 2)
        [thetaMetric(pointIndex), phiMetric(pointIndex)] = ...
            tangentIndicators(points(:, pointIndex), ...
            targetDistance, geometry);
    end
    metricValues = [thetaMetric; phiMetric];
    metricTypes = ["theta", "phi"];
    for metricIndex = 1:2
        values = metricValues(metricIndex, :);
        for segment = 1:(numel(values) - 1)
            firstValue = values(segment);
            secondValue = values(segment + 1);
            if abs(firstValue) <= detectionTolerance
                candidatePoints(:, end + 1) = points(:, segment); %#ok<AGROW>
                candidateTypes(end + 1) = metricTypes(metricIndex); %#ok<AGROW>
            elseif firstValue * secondValue < 0
                fraction = firstValue / (firstValue - secondValue);
                candidatePoints(:, end + 1) = ...
                    points(:, segment) + fraction * ...
                    (points(:, segment + 1) - points(:, segment)); %#ok<AGROW>
                candidateTypes(end + 1) = metricTypes(metricIndex); %#ok<AGROW>
            end
            if isfinite(firstValue) && isfinite(secondValue)
                candidatePoints(:, end + 1) = 0.5 * ...
                    (points(:, segment) + ...
                    points(:, segment + 1)); %#ok<AGROW>
                candidateTypes(end + 1) = metricTypes(metricIndex); %#ok<AGROW>
            end
            if segment == numel(values) - 1 && ...
                    abs(secondValue) <= detectionTolerance
                candidatePoints(:, end + 1) = points(:, segment + 1); %#ok<AGROW>
                candidateTypes(end + 1) = metricTypes(metricIndex); %#ok<AGROW>
            end
        end
        magnitudes = abs(values);
        seedMask = false(size(values));
        if ~isempty(values)
            seedMask(1) = isfinite(values(1));
            seedMask(end) = isfinite(values(end));
        end
        for pointIndex = 2:(numel(values) - 1)
            seedMask(pointIndex) = isfinite(magnitudes(pointIndex)) && ...
                isfinite(magnitudes(pointIndex - 1)) && ...
                isfinite(magnitudes(pointIndex + 1)) && ...
                magnitudes(pointIndex) <= magnitudes(pointIndex - 1) && ...
                magnitudes(pointIndex) <= magnitudes(pointIndex + 1);
        end
        seedIndices = find(seedMask);
        for seedIndex = reshape(seedIndices, 1, [])
            candidatePoints(:, end + 1) = points(:, seedIndex); %#ok<AGROW>
            candidateTypes(end + 1) = metricTypes(metricIndex); %#ok<AGROW>
        end
    end
end

refinedPoints = zeros(2, 0);
for index = 1:size(candidatePoints, 2)
    if candidateTypes(index) == ""
        continue
    end
    [q, converged] = refineRoot(candidatePoints(:, index), ...
        targetDistance, candidateTypes(index), geometry, options, bounds);
    publicExact = isPublicExactTypeIII(q, targetDistance, ...
        candidateTypes(index), geometry, options);
    if converged && publicExact && (isempty(refinedPoints) || ...
            all(vecnorm(refinedPoints - q, 2, 1) > 1e-8))
        refinedPoints(:, end + 1) = q; %#ok<AGROW>
    end
end
if isempty(refinedPoints)
    loci = repmat(struct('theta', [], 'phi', []), 0, 1);
else
    loci = struct('theta', refinedPoints(1, :), ...
        'phi', refinedPoints(2, :));
end
end

function [q, converged] = refineRoot( ...
        q, targetDistance, type, geometry, options, bounds)
converged = false;
lengthScale = max([geometry.links.link3, ...
    geometry.links.link4, targetDistance]);
closureTolerance = 32 * eps(max([targetDistance, ...
    geometry.links.link3, geometry.links.link4]));
metricTolerance = max(100 * eps, ...
    min(options.exactThreshold, 1e-10));
acceptanceTolerance = min(metricTolerance, options.exactThreshold);
finiteDifferenceStep = 1e-7;
iteration = 0;
while iteration < 20
    iteration = iteration + 1;
    residual = combinedResidual( ...
        q, targetDistance, type, geometry, lengthScale);
    if abs(residual(1)) <= closureTolerance / lengthScale && ...
            abs(residual(2)) <= acceptanceTolerance
        converged = true;
        return
    end
    jacobian = zeros(2);
    for column = 1:2
        offset = zeros(2, 1);
        offset(column) = finiteDifferenceStep;
        jacobian(:, column) = (combinedResidual( ...
            q + offset, targetDistance, type, geometry, lengthScale) - ...
            combinedResidual(q - offset, targetDistance, type, ...
            geometry, lengthScale)) / (2 * finiteDifferenceStep);
    end
    if any(~isfinite(jacobian), 'all') || rcond(jacobian) <= 1e-12
        return
    end
    direction = jacobian \ residual;
    if norm(direction) > 0.25
        direction = 0.25 * direction / norm(direction);
    end
    accepted = false;
    for backtrack = 0:8
        proposal = min(max(q - 0.5^backtrack * direction, ...
            bounds(:, 1)), bounds(:, 2));
        proposalResidual = combinedResidual( ...
            proposal, targetDistance, type, geometry, lengthScale);
        if norm(proposalResidual) < norm(residual)
            q = proposal;
            accepted = true;
            break
        end
    end
    if ~accepted
        return
    end
end
residual = combinedResidual( ...
    q, targetDistance, type, geometry, lengthScale);
converged = abs(residual(1)) <= closureTolerance / lengthScale && ...
    abs(residual(2)) <= acceptanceTolerance;
end

function valid = isPublicExactTypeIII( ...
        q, targetDistance, type, geometry, options)
valid = false;
L = geometry.links;
E = L.link1 * [cos(q(1)); sin(q(1))];
C = [L.link5 - L.link2 * cos(q(2)); L.link2 * sin(q(2))];
actualDistance = norm(E - C);
closureTolerance = 32 * eps(max([targetDistance, ...
    geometry.links.link3, geometry.links.link4]));
if abs(actualDistance - targetDistance) > closureTolerance
    return
end
[thetaMetric, phiMetric, typeIIMetric] = ...
    tangentIndicators(q, targetDistance, geometry);
if type == "theta"
    selectedMetric = thetaMetric;
else
    selectedMetric = phiMetric;
end
if ~isfinite(selectedMetric) || ~isfinite(typeIIMetric) || ...
        abs(selectedMetric) > options.exactThreshold || ...
        abs(typeIIMetric) > options.exactThreshold
    return
end
side = evaluateSide(q.', geometry, options);
if type == "theta"
    selectedTypeI = side.exact.typeITheta;
else
    selectedTypeI = side.exact.typeIPhi;
end
valid = side.reachable && isfinite(side.closureDistance) && ...
    abs(side.closureDistance - targetDistance) <= closureTolerance && ...
    selectedTypeI && side.exact.typeII && side.exact.typeIII;
end

function residual = combinedResidual( ...
        q, targetDistance, type, geometry, lengthScale)
L = geometry.links;
E = L.link1 * [cos(q(1)); sin(q(1))];
C = [L.link5 - L.link2 * cos(q(2)); L.link2 * sin(q(2))];
[thetaMetric, phiMetric] = tangentIndicators( ...
    q, targetDistance, geometry);
if type == "theta"
    typeIMetric = thetaMetric;
else
    typeIMetric = phiMetric;
end
residual = [(norm(E - C) - targetDistance) / lengthScale; typeIMetric];
end

function [thetaMetric, phiMetric, typeIIMetric] = ...
        tangentIndicators(q, targetDistance, geometry)
L = geometry.links;
theta = q(1);
phi = q(2);
E = L.link1 * [cos(theta); sin(theta)];
C = [L.link5 - L.link2 * cos(phi); L.link2 * sin(phi)];
delta = E - C;
distance = norm(delta);
if distance <= geometry.tolerance.length || ...
        targetDistance <= geometry.tolerance.length
    thetaMetric = NaN;
    phiMetric = NaN;
    typeIIMetric = NaN;
    return
end
along = (L.link3^2 - L.link4^2 + targetDistance^2) / ...
    (2 * targetDistance);
D = C + along * delta / distance;
u = D - C;
v = D - E;
eTheta = L.link1 * [-sin(theta); cos(theta)];
cPhi = L.link2 * [sin(phi); cos(phi)];
thetaMetric = dot(v, eTheta) / (L.link4 * L.link1);
phiMetric = dot(u, cPhi) / (L.link3 * L.link2);
typeIIMetric = (u(1) * v(2) - u(2) * v(1)) / ...
    (L.link3 * L.link4);
end
