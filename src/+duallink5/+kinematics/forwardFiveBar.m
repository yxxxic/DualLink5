function pose = forwardFiveBar(q, geometry, options)
if nargin < 3
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error('duallink5:kinematics:InvalidBranchConfiguration', ...
        'options must be a scalar struct.');
end
geometry = duallink5.model.validateGeometry(geometry);
collisionProfile = ...
    duallink5.validation.validateCollisionConfiguration( ...
        getOption(options, 'collisionProfile', "centerline"), ...
        geometry.collision, true);
if ~(isnumeric(q) && isreal(q) && numel(q) == 2)
    error('duallink5:kinematics:InvalidJointVector', ...
        'q must contain numeric real [theta, phi].');
end
q = q(:).';

pose = invalidPose("UNINITIALIZED");
pose.metadata.units = geometry.units;
pose.metadata.convention = geometry.convention;
pose.metadata.geometryVersion = geometry.version;
pose.metadata.q = q;
branchMode = string(getOption(options, 'branchMode', "fixed"));
branchValue = getOption( ...
    options, 'branchId', geometry.assembly.defaultBranch);
previousPose = getOption(options, 'previousPose', []);
maxContinuityCost = getOption(options, 'maxContinuityCost', Inf);
validFixedBranch = isscalar(branchValue) && isnumeric(branchValue) && ...
    isfinite(branchValue) && ismember(double(branchValue), [-1, 1]);
validPrevious = ~isempty(previousPose) && isstruct(previousPose) && ...
    isfield(previousPose, 'points') && ...
    isfield(previousPose.points, 'D') && ...
    isnumeric(previousPose.points.D) && ...
    numel(previousPose.points.D) == 2 && ...
    all(isfinite(previousPose.points.D));
if validPrevious
    previousPose.points.D = previousPose.points.D(:);
end
if ~isscalar(branchMode) || ...
        ~ismember(branchMode, ["fixed", "continuous"]) || ...
        (branchMode == "fixed" && ~validFixedBranch) || ...
        (branchMode == "continuous" && ~validPrevious)
    error('duallink5:kinematics:InvalidBranchConfiguration', ...
        'Use fixed with branchId +/-1, or continuous with previousPose.');
end
if ~(isscalar(maxContinuityCost) && ...
        isnumeric(maxContinuityCost) && ...
        ~isnan(maxContinuityCost) && maxContinuityCost > 0)
    error('duallink5:kinematics:InvalidBranchConfiguration', ...
        'maxContinuityCost must be positive or Inf.');
end
if validFixedBranch
    branchId = int8(branchValue);
else
    branchId = geometry.assembly.defaultBranch;
end
if any(~isfinite(q))
    pose.quality.statusCode = "NONFINITE_INPUT";
    return
end

theta = q(1);
phi = q(2);
L = geometry.links;
A = [0; 0];
B = [L.link5; 0];
E = L.link1 * [cos(theta); sin(theta)];
C = [L.link5 - L.link2 * cos(phi); L.link2 * sin(phi)];

[candidates, closureStatus] = duallink5.kinematics.solveClosure( ...
    C, E, L.link3, L.link4, geometry.tolerance.length);
if isempty(candidates)
    pose.quality.statusCode = closureStatus;
    return
end
if closureStatus == "NEAR_SINGULAR"
    pose.quality.branchId = candidates(1).branchId;
    pose.quality.closureResidual = candidates(1).closureResidual;
    pose.quality.statusCode = "NEAR_SINGULAR";
    return
end

[candidate, continuityCost, selectStatus] = selectCandidate( ...
    candidates, branchMode, branchId, previousPose, maxContinuityCost);
if isempty(candidate)
    pose.quality.continuityCost = continuityCost;
    pose.quality.statusCode = selectStatus;
    return
end

D = candidate.D;
G = D + E - B;
parallel = duallink5.kinematics.computeParallelPoints( ...
    A, C, D, E, geometry);

points = struct('A', A, 'B', B, 'C', C, 'D', D, 'E', E, 'G', G);
parallelNames = fieldnames(parallel);
for index = 1:numel(parallelNames)
    points.(parallelNames{index}) = parallel.(parallelNames{index});
end

pose.points = points;
pose.sharedLink.start = E;
pose.sharedLink.end = D;
pose.sharedLink.center = (D + E) / 2;
pose.sharedLink.orientation = atan2(D(2) - E(2), D(1) - E(1));
pose.quality.valid = true;
pose.quality.branchId = candidate.branchId;
pose.quality.closureResidual = candidate.closureResidual;
pose.quality.continuityCost = continuityCost;
pose.quality.collisionFree = NaN;
pose.quality.clearanceModelApplied = false;
pose.quality.statusCode = closureStatus;
pose = duallink5.validation.assessFiveBarPose( ...
    pose, geometry, collisionProfile);
end

function [candidate, cost, status] = selectCandidate( ...
        candidates, branchMode, branchId, previousPose, maxCost)
candidate = [];
cost = NaN;
status = "NO_VALID_BRANCH";

if branchMode == "continuous" && ~isempty(previousPose)
    costs = arrayfun( ...
        @(item) norm(item.D - previousPose.points.D), candidates);
    [cost, index] = min(costs);
    if cost > maxCost
        status = "BRANCH_DISCONTINUITY";
        candidate = [];
        return
    end
    candidate = candidates(index);
    status = "OK";
    return
end

index = find([candidates.branchId] == branchId, 1, 'first');
if ~isempty(index)
    candidate = candidates(index);
    cost = 0;
    status = "OK";
end
end

function value = getOption(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end

function pose = invalidPose(statusCode)
pose.points = struct();
pose.sharedLink = struct( ...
    'start', [], 'end', [], 'center', [], 'orientation', NaN);
pose.quality = struct( ...
    'valid', false, ...
    'branchId', int8(0), ...
    'closureResidual', Inf, ...
    'continuityCost', NaN, ...
    'collisionFree', NaN, ...
    'clearanceModelApplied', false, ...
    'statusCode', string(statusCode));
pose.metadata = struct();
end
