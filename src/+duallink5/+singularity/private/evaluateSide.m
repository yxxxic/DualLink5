function result = evaluateSide(q, geometry, options)
L = geometry.links;
B = [L.link5; 0];
theta = q(1);
phi = q(2);
E = L.link1 * [cos(theta); sin(theta)];
C = [L.link5 - L.link2 * cos(phi); L.link2 * sin(phi)];

result = emptyResult(q);
[candidate, closureStatus] = selectClosureCandidate( ...
    C, E, L, geometry, options);
result.closureStatusCode = closureStatus;
if isempty(candidate)
    return
end
result.closureDistance = norm(E - C);

D = candidate.D;
G = D + E - B;
u = D - C;
v = D - E;
eTheta = L.link1 * [-sin(theta); cos(theta)];
cPhi = L.link2 * [sin(phi); cos(phi)];
crossUV = cross2(u, v);

result.reachable = true;
result.branchId = candidate.branchId;
result.pointG = G;
result.signedIndicator.typeITheta = ...
    dot(v, eTheta) / (L.link4 * L.link1);
result.signedIndicator.typeIPhi = ...
    dot(u, cPhi) / (L.link3 * L.link2);
result.signedIndicator.typeII = ...
    crossUV / (L.link3 * L.link4);
result.margins.typeITheta = ...
    abs(result.signedIndicator.typeITheta);
result.margins.typeIPhi = abs(result.signedIndicator.typeIPhi);
result.margins.typeII = abs(result.signedIndicator.typeII);

constraint = [u.'; v.'];
taskActuation = [dot(u, eTheta), dot(u, cPhi); ...
    2 * dot(v, eTheta), 0];
normalizedConstraint = [u.' / L.link3; v.' / L.link4];
constraintSingularValues = svd(normalizedConstraint);
result.matrices.constraint = constraint;
result.matrices.taskActuation = taskActuation;
result.singularValues.constraint = constraintSingularValues;
result.conditionNumber.constraint = ...
    safeConditionNumber(constraintSingularValues);

exactITheta = result.margins.typeITheta <= ...
    options.exactThreshold;
exactIPhi = result.margins.typeIPhi <= options.exactThreshold;
exactI = exactITheta || exactIPhi;
exactII = result.margins.typeII <= options.exactThreshold;
result.exact = exactFlags(exactITheta, exactIPhi, exactI, exactII);

if exactII
    result.matrices.taskJacobian = nan(2);
    result.singularValues.task = [NaN; NaN];
    result.conditionNumber.task = Inf;
    result.dpsi_dq = [NaN, NaN];
    result.orientationSensitivity = NaN;
else
    taskJacobian = constraint \ taskActuation;
    normalizedTask = taskJacobian / L.link4;
    taskSingularValues = svd(normalizedTask);
    result.matrices.taskJacobian = taskJacobian;
    result.singularValues.task = taskSingularValues;
    result.conditionNumber.task = ...
        safeConditionNumber(taskSingularValues);
    result.dpsi_dq = orientationDerivative( ...
        constraint, u, v, eTheta, cPhi);
    result.orientationSensitivity = norm(result.dpsi_dq, 2);
end

nearITheta = result.margins.typeITheta < ...
    options.nearMetricThreshold;
nearIPhi = result.margins.typeIPhi < ...
    options.nearMetricThreshold;
nearI = nearITheta || nearIPhi;
nearII = result.margins.typeII < options.nearMetricThreshold || ...
    result.conditionNumber.constraint > ...
    options.nearConditionThreshold;
nearTask = result.conditionNumber.task > ...
    options.nearConditionThreshold;
result.near = nearFlags( ...
    result.exact, nearITheta, nearIPhi, nearI, nearII, nearTask);
result.classification = classify(result.exact, result.near);
end

function [candidate, closureStatus] = selectClosureCandidate( ...
        C, E, L, geometry, options)
candidate = [];
distance = norm(E - C);
targets = [L.link3 + L.link4, abs(L.link4 - L.link3)];
[targetResidual, targetIndex] = min(abs(distance - targets));
tangencyTolerance = 32 * eps(max([L.link3, L.link4, distance]));
targetDistance = targets(targetIndex);
if targetDistance > geometry.tolerance.length && ...
        targetResidual <= tangencyTolerance
    direction = (E - C) / distance;
    along = (L.link3^2 - L.link4^2 + targetDistance^2) / ...
        (2 * targetDistance);
    candidate = struct('D', C + along * direction, ...
        'branchId', int8(0));
    closureStatus = "NEAR_SINGULAR";
    return
end

[candidates, closureStatus] = duallink5.kinematics.solveClosure( ...
    C, E, L.link3, L.link4, tangencyTolerance);
if isempty(candidates)
    return
end
if closureStatus == "NEAR_SINGULAR"
    candidate = candidates(1);
    return
end
branchIds = [candidates.branchId];
selected = find(branchIds == options.branchId, 1);
if isempty(selected)
    closureStatus = "NO_VALID_BRANCH";
else
    candidate = candidates(selected);
end
end

function result = emptyResult(q)
result.q = q;
result.reachable = false;
result.closureStatusCode = "UNREACHABLE_CLOSURE";
result.closureDistance = NaN;
result.branchId = int8(0);
result.pointG = [NaN; NaN];
result.matrices.constraint = nan(2);
result.matrices.taskActuation = nan(2);
result.matrices.taskJacobian = nan(2);
result.signedIndicator = scalarFields(NaN, NaN, NaN);
result.margins = scalarFields(NaN, NaN, NaN);
result.singularValues.constraint = [NaN; NaN];
result.singularValues.task = [NaN; NaN];
result.conditionNumber.constraint = Inf;
result.conditionNumber.task = Inf;
result.exact = exactFlags(false, false, false, false);
result.near = nearFlags( ...
    result.exact, false, false, false, false, false);
result.dpsi_dq = [NaN, NaN];
result.orientationSensitivity = NaN;
result.classification = "UNREACHABLE_CLOSURE";
end

function fields = scalarFields(thetaValue, phiValue, typeIIValue)
fields.typeITheta = thetaValue;
fields.typeIPhi = phiValue;
fields.typeII = typeIIValue;
end

function flags = exactFlags(typeITheta, typeIPhi, typeI, typeII)
flags.typeITheta = logical(typeITheta);
flags.typeIPhi = logical(typeIPhi);
flags.typeI = logical(typeI);
flags.typeII = logical(typeII);
flags.typeIII = logical(typeI && typeII);
flags.any = logical(typeI || typeII);
end

function flags = nearFlags( ...
        exact, typeITheta, typeIPhi, typeI, typeII, taskCondition)
flags.typeITheta = logical(typeITheta || exact.typeITheta);
flags.typeIPhi = logical(typeIPhi || exact.typeIPhi);
flags.typeI = logical(typeI || exact.typeI);
flags.typeII = logical(typeII || exact.typeII);
flags.typeIII = logical(flags.typeI && flags.typeII);
flags.taskCondition = logical(taskCondition);
flags.any = logical(flags.typeI || flags.typeII || taskCondition);
end

function classification = classify(exact, near)
if exact.typeIII
    classification = "TYPE_III";
elseif exact.typeII
    classification = "TYPE_II";
elseif exact.typeI
    classification = "TYPE_I";
elseif near.typeI && near.typeII
    classification = "NEAR_MULTIPLE";
elseif near.typeII
    classification = "NEAR_TYPE_II";
elseif near.typeI
    classification = "NEAR_TYPE_I";
elseif near.taskCondition
    classification = "NEAR_TASK";
else
    classification = "REGULAR";
end
end

function derivative = orientationDerivative( ...
        constraint, u, v, eTheta, cPhi)
dE = [eTheta, zeros(2, 1)];
dC = [zeros(2, 1), cPhi];
dD = zeros(2, 2);
for column = 1:2
    rightHandSide = [dot(u, dC(:, column)); ...
        dot(v, dE(:, column))];
    dD(:, column) = constraint \ rightHandSide;
end
dLink = dD - dE;
derivative = zeros(1, 2);
for column = 1:2
    derivative(column) = cross2(v, dLink(:, column)) / dot(v, v);
end
end

function value = safeConditionNumber(singularValues)
if numel(singularValues) ~= 2 || ...
        any(~isfinite(singularValues)) || singularValues(2) <= eps
    value = Inf;
else
    value = singularValues(1) / singularValues(2);
end
end

function value = cross2(first, second)
value = first(1) * second(2) - first(2) * second(1);
end
