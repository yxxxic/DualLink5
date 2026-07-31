function result = fiveBarJacobian(q, pose, geometry, taskSpec)
q = normalizeJointVector(q);
geometry = duallink5.model.validateGeometry(geometry);
pose = normalizePose(pose);
taskSpec = normalizeTaskSpec(taskSpec);
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
includeOrientation = taskSpec.includeOrientation;
taskDimension = 2 + double(includeOrientation);
constraint = [(D - C).'; (D - E).'];
linkVector = D - E;
sharedLinkLength = norm(linkVector);
if ~isfinite(sharedLinkLength)
    result = singularResult(taskDimension, includeOrientation, ...
        L.link4, Inf);
    return
end
closureConditionNumber = cond(constraint);

if ~isfinite(closureConditionNumber) || ...
        closureConditionNumber >= ...
        geometry.tolerance.singularityCondition
    result = singularResult(taskDimension, includeOrientation, ...
        L.link4, closureConditionNumber);
    return
end

if sharedLinkLength <= geometry.tolerance.length
    result = singularResult(taskDimension, includeOrientation, ...
        L.link4, closureConditionNumber);
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
dLink = dD - dE;
dPsi = zeros(1, 2);
for column = 1:2
    dPsi(column) = cross2(linkVector, dLink(:, column)) / ...
        dot(linkVector, linkVector);
end

kind = taskSpec.kind;
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

function result = singularResult( ...
        taskDimension, includeOrientation, orientationScale, ...
        closureConditionNumber)
taskRowScale = ones(1, taskDimension);
if includeOrientation
    taskRowScale(end) = orientationScale;
end
result.matrix = nan(taskDimension, 2);
result.closureConditionNumber = closureConditionNumber;
result.taskConditionNumber = Inf;
result.conditionNumber = Inf;
result.taskRowScale = taskRowScale;
result.nearSingular = true;
result.statusCode = "NEAR_SINGULAR";
end

function q = normalizeJointVector(q)
if ~(isnumeric(q) && isreal(q) && numel(q) == 2 && ...
        all(isfinite(q), 'all'))
    error('duallink5:kinematics:InvalidJointVector', ...
        'q must contain finite numeric real [theta, phi].');
end
q = double(q(:).');
end

function pose = normalizePose(pose)
if ~isstruct(pose) || ~isscalar(pose) || ...
        ~isfield(pose, 'points') || ...
        ~isstruct(pose.points) || ~isscalar(pose.points) || ...
        ~isfield(pose, 'sharedLink') || ...
        ~isstruct(pose.sharedLink) || ~isscalar(pose.sharedLink) || ...
        ~isfield(pose.sharedLink, 'orientation')
    invalidPose();
end

names = {'C', 'D', 'E'};
for index = 1:numel(names)
    name = names{index};
    if ~isfield(pose.points, name)
        invalidPose();
    end
    point = pose.points.(name);
    if ~isnumeric(point) || ~isreal(point) || ...
            ~isvector(point) || numel(point) ~= 2 || ...
            any(~isfinite(point), 'all')
        invalidPose();
    end
    pose.points.(name) = double(point(:));
end

orientation = pose.sharedLink.orientation;
if ~(isnumeric(orientation) && isreal(orientation) && ...
        isscalar(orientation) && isfinite(orientation))
    invalidPose();
end
pose.sharedLink.orientation = double(orientation);
end

function taskSpec = normalizeTaskSpec(taskSpec)
if ~isstruct(taskSpec) || ~isscalar(taskSpec) || ...
        ~isfield(taskSpec, 'kind')
    invalidTaskSpec('taskSpec.kind is required.');
end
try
    kind = string(taskSpec.kind);
catch
    invalidTaskSpec('kind must be scalar string-convertible text.');
end
if ~isscalar(kind) || ismissing(kind) || ...
        ~ismember(kind, ["sharedCenter", "pointG", ...
        "sharedOffset", "marker"])
    invalidTaskSpec('Unsupported task kind.');
end

includeOrientation = false;
if isfield(taskSpec, 'includeOrientation')
    includeOrientation = taskSpec.includeOrientation;
end
validInclude = isscalar(includeOrientation) && ...
    (islogical(includeOrientation) || ...
    (isnumeric(includeOrientation) && isreal(includeOrientation) && ...
    isfinite(includeOrientation) && ...
    ismember(double(includeOrientation), [0, 1])));
if ~validInclude
    invalidTaskSpec('includeOrientation must be logical.');
end

taskSpec.kind = kind;
taskSpec.includeOrientation = logical(includeOrientation);
if ismember(kind, ["sharedOffset", "marker"])
    if ~isfield(taskSpec, 'offset')
        invalidTaskSpec('%s requires taskSpec.offset.', kind);
    end
    offset = taskSpec.offset;
    if ~isnumeric(offset) || ~isreal(offset) || ...
            ~isvector(offset) || numel(offset) ~= 2 || ...
            any(~isfinite(offset), 'all')
        invalidTaskSpec( ...
            '%s requires a finite real numeric 2-vector offset.', kind);
    end
    taskSpec.offset = double(offset(:));
end

if isfield(taskSpec, 'orientationOffset')
    orientationOffset = taskSpec.orientationOffset;
    if ~(isnumeric(orientationOffset) && ...
            isreal(orientationOffset) && ...
            isscalar(orientationOffset) && ...
            isfinite(orientationOffset))
        invalidTaskSpec( ...
            'orientationOffset must be a finite numeric scalar.');
    end
    taskSpec.orientationOffset = double(orientationOffset);
end
end

function invalidPose()
error('duallink5:validation:InvalidPose', ...
    ['pose must contain finite real two-vector points C, D, E ' ...
    'and a finite scalar shared-link orientation.']);
end

function invalidTaskSpec(message, varargin)
error('duallink5:kinematics:InvalidTaskSpec', ...
    message, varargin{:});
end
