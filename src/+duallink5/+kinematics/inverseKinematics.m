function solutions = inverseKinematics(target, geometry, options)
if nargin < 3
    options = struct();
end
geometry = duallink5.model.validateGeometry(geometry);
if ~isstruct(options) || ~isscalar(options)
    invalidOptions();
end

[forwardTolerance, jointTolerance, collisionProfile] = ...
    validateOptions(options, geometry);
[kind, position] = validateTarget(target);

links = geometry.links;
A = [0; 0];
B = [links.link5; 0];
if kind == "sharedCenter"
    P = position;
elseif kind == "pointG"
    P = (position + B) / 2;
else
    error('duallink5:kinematics:UnsupportedIKTask', ...
        'Inverse kinematics supports sharedCenter and pointG targets.');
end

template = struct( ...
    'q', zeros(1, 2), ...
    'branchId', int8(0), ...
    'valid', false, ...
    'forwardResidual', Inf, ...
    'closureResidual', Inf, ...
    'collisionFree', NaN, ...
    'nearSingular', false, ...
    'statusCode', "UNINITIALIZED", ...
    'pose', struct());
solutions = repmat(template, 0, 1);
buffer = repmat(template, 4, 1);
count = 0;

eCandidates = circleIntersections( ...
    A, links.link1, P, links.link4 / 2, geometry.tolerance.length);
for eIndex = 1:size(eCandidates, 2)
    E = eCandidates(:, eIndex);
    D = 2 * P - E;
    cCandidates = circleIntersections( ...
        B, links.link2, D, links.link3, geometry.tolerance.length);
    for cIndex = 1:size(cCandidates, 2)
        C = cCandidates(:, cIndex);
        theta = atan2(E(2), E(1));
        phi = atan2(C(2), links.link5 - C(1));
        branchId = int8(sign(cross2(E - C, D - C)));
        if branchId == 0
            continue
        end

        forwardOptions = struct( ...
            'collisionProfile', collisionProfile, ...
            'branchId', branchId, ...
            'branchMode', "fixed");
        pose = duallink5.kinematics.forwardFiveBar( ...
            [theta, phi], geometry, forwardOptions);
        if ~pose.quality.valid
            continue
        end
        if kind == "sharedCenter"
            predicted = pose.sharedLink.center;
        else
            predicted = pose.points.G;
        end
        forwardResidual = norm(predicted - position);
        if forwardResidual > forwardTolerance
            continue
        end

        count = count + 1;
        buffer(count).q = [theta, phi];
        buffer(count).branchId = pose.quality.branchId;
        buffer(count).valid = pose.quality.valid;
        buffer(count).forwardResidual = forwardResidual;
        buffer(count).closureResidual = pose.quality.closureResidual;
        buffer(count).collisionFree = pose.quality.collisionFree;
        buffer(count).nearSingular = ...
            pose.quality.statusCode == "NEAR_SINGULAR";
        buffer(count).statusCode = pose.quality.statusCode;
        buffer(count).pose = pose;
    end
end

if count > 0
    solutions = deduplicate(buffer(1:count), jointTolerance, template);
end
end

function [forwardTolerance, jointTolerance, collisionProfile] = ...
        validateOptions(options, geometry)
allowedNames = {'collisionProfile', 'forwardTolerance', 'jointTolerance'};
if any(~ismember(fieldnames(options), allowedNames))
    invalidOptions();
end

forwardTolerance = getOption(options, 'forwardTolerance', 1e-8);
jointTolerance = getOption(options, 'jointTolerance', 1e-10);
if ~isPositiveFiniteScalar(forwardTolerance) || ...
        ~isPositiveFiniteScalar(jointTolerance)
    invalidOptions();
end
forwardTolerance = double(forwardTolerance);
jointTolerance = double(jointTolerance);

collisionProfile = ...
    duallink5.validation.validateCollisionConfiguration( ...
        getOption(options, 'collisionProfile', "centerline"), ...
        geometry.collision, true);
end

function [kind, position] = validateTarget(target)
if ~isstruct(target) || ~isscalar(target) || ...
        ~isfield(target, 'kind') || ~isfield(target, 'position')
    invalidTarget();
end
try
    kind = string(target.kind);
catch
    invalidTarget();
end
if ~isscalar(kind) || ismissing(kind) || strlength(kind) == 0 || ...
        ~(isnumeric(target.position) && isreal(target.position) && ...
          numel(target.position) == 2 && ...
          all(isfinite(target.position(:))))
    invalidTarget();
end
position = double(target.position(:));
end

function points = circleIntersections( ...
        firstCenter, firstRadius, secondCenter, secondRadius, tolerance)
delta = secondCenter - firstCenter;
distance = norm(delta);
if distance < tolerance || ...
        distance > firstRadius + secondRadius + tolerance || ...
        distance < abs(firstRadius - secondRadius) - tolerance
    points = zeros(2, 0);
    return
end

unitDirection = delta / distance;
along = (firstRadius^2 - secondRadius^2 + distance^2) / ...
    (2 * distance);
heightSquared = firstRadius^2 - along^2;
squaredTolerance = tolerance * max( ...
    [firstRadius, secondRadius, distance, tolerance]);
if heightSquared < -squaredTolerance
    points = zeros(2, 0);
    return
end

height = sqrt(max(0, heightSquared));
basePoint = firstCenter + along * unitDirection;
if height <= tolerance
    points = basePoint;
    return
end
normal = [-unitDirection(2); unitDirection(1)];
points = [basePoint + height * normal, basePoint - height * normal];
end

function uniqueSolutions = deduplicate(solutions, tolerance, template)
uniqueSolutions = repmat(template, 0, 1);
for candidateIndex = 1:numel(solutions)
    duplicate = false;
    for existingIndex = 1:numel(uniqueSolutions)
        difference = solutions(candidateIndex).q - ...
            uniqueSolutions(existingIndex).q;
        if norm(wrapVector(difference)) <= tolerance
            duplicate = true;
            break
        end
    end
    if ~duplicate
        uniqueSolutions(end + 1, 1) = solutions(candidateIndex); %#ok<AGROW>
    end
end
end

function wrapped = wrapVector(value)
wrapped = atan2(sin(value), cos(value));
end

function value = cross2(first, second)
value = first(1) * second(2) - first(2) * second(1);
end

function value = getOption(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end

function valid = isPositiveFiniteScalar(value)
valid = isnumeric(value) && isreal(value) && isscalar(value) && ...
    isfinite(value) && value > 0;
end

function invalidTarget()
error('duallink5:kinematics:InvalidIKTarget', ...
    'target must contain a scalar kind and a finite 2-vector position.');
end

function invalidOptions()
error('duallink5:kinematics:InvalidIKOptions', ...
    ['options may contain collisionProfile plus positive finite ', ...
     'forwardTolerance and jointTolerance scalars.']);
end
