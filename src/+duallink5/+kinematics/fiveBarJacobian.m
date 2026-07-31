function result = fiveBarJacobian(q, pose, geometry, taskSpec)
q = q(:).';
theta = q(1);
phi = q(2);
L = geometry.links;
C = pose.points.C;
D = pose.points.D;
E = pose.points.E;

dE = [-L.link1 * sin(theta), 0; ...
    L.link1 * cos(theta), 0];
dC = [0, L.link2 * sin(phi); ...
    0, L.link2 * cos(phi)];
constraint = [(D - C).'; (D - E).'];
closureConditionNumber = cond(constraint);
includeOrientation = isfield(taskSpec, 'includeOrientation') && ...
    taskSpec.includeOrientation;
taskDimension = 2 + double(includeOrientation);

if ~isfinite(closureConditionNumber) || ...
        closureConditionNumber >= ...
        geometry.tolerance.singularityCondition
    result.matrix = nan(taskDimension, 2);
    result.closureConditionNumber = closureConditionNumber;
    result.taskConditionNumber = Inf;
    result.conditionNumber = Inf;
    result.taskRowScale = [ones(1, ...
        taskDimension - includeOrientation), ...
        repmat(L.link4, 1, double(includeOrientation))];
    result.nearSingular = true;
    result.statusCode = "NEAR_SINGULAR";
    return
end

dD = zeros(2, 2);
for column = 1:2
    rightHandSide = [(D - C).' * dC(:, column); ...
        (D - E).' * dE(:, column)];
    dD(:, column) = constraint \ rightHandSide;
end

dP = (dD + dE) / 2;
dG = dD + dE;
linkVector = D - E;
dLink = dD - dE;
dPsi = zeros(1, 2);
for column = 1:2
    dPsi(column) = cross2(linkVector, dLink(:, column)) / ...
        dot(linkVector, linkVector);
end

kind = string(taskSpec.kind);
if kind == "sharedCenter"
    positionJacobian = dP;
elseif kind == "pointG"
    positionJacobian = dG;
elseif ismember(kind, ["sharedOffset", "marker"])
    offset = taskSpec.offset(:);
    psi = pose.sharedLink.orientation;
    derivativeRotation = [-sin(psi), -cos(psi); ...
        cos(psi), -sin(psi)];
    positionJacobian = dP + ...
        derivativeRotation * offset * dPsi;
else
    error('duallink5:kinematics:InvalidTaskSpec', ...
        'Unsupported task kind: %s.', kind);
end

if includeOrientation
    result.matrix = [positionJacobian; dPsi];
else
    result.matrix = positionJacobian;
end
scaledTaskMatrix = result.matrix;
taskRowScale = ones(1, taskDimension);
if includeOrientation
    taskRowScale(end) = L.link4;
    scaledTaskMatrix(end, :) = ...
        L.link4 * scaledTaskMatrix(end, :);
end
taskConditionNumber = cond(scaledTaskMatrix);
result.closureConditionNumber = closureConditionNumber;
result.taskConditionNumber = taskConditionNumber;
result.taskRowScale = taskRowScale;
result.conditionNumber = max( ...
    closureConditionNumber, taskConditionNumber);
result.nearSingular = ~isfinite(taskConditionNumber) || ...
    taskConditionNumber >= geometry.tolerance.singularityCondition;
if result.nearSingular
    result.statusCode = "NEAR_SINGULAR";
else
    result.statusCode = "OK";
end
end

function value = cross2(a, b)
value = a(1) * b(2) - a(2) * b(1);
end
