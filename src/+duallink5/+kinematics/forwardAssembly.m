function assembly = forwardAssembly(q, geometry, options)
if nargin < 3
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error('duallink5:kinematics:InvalidAssemblyMode', ...
        'options must be a scalar struct.');
end

try
    mode = string(getOption(options, 'mode', "ideal"));
catch
    invalidMode();
end
if ~isscalar(mode) || ismissing(mode) || ...
        ~ismember(mode, ["ideal", "diagnostic"])
    invalidMode();
end
if ~isstruct(q) || ~isscalar(q) || ...
        ~isfield(q, 'lower') || ~isfield(q, 'upper')
    error('duallink5:kinematics:InvalidAssemblyInput', ...
        'q.lower and q.upper are required.');
end

lowerOptions = options;
upperOptions = options;
if isfield(options, 'previousAssembly')
    previous = options.previousAssembly;
    if ~isstruct(previous) || ~isscalar(previous) || ...
            ~isfield(previous, 'lower') || ...
            ~isfield(previous, 'upperLocal')
        error('duallink5:kinematics:InvalidBranchConfiguration', ...
            ['previousAssembly must contain lower and upperLocal ' ...
             'poses.']);
    end
    lowerOptions.previousPose = previous.lower;
    upperOptions.previousPose = previous.upperLocal;
end

lower = duallink5.kinematics.forwardFiveBar( ...
    q.lower, geometry, lowerOptions);
upperLocal = duallink5.kinematics.forwardFiveBar( ...
    q.upper, geometry, upperOptions);

assembly = struct();
assembly.lower = lower;
assembly.upperLocal = upperLocal;
assembly.quality.valid = false;
assembly.quality.diagnosticAvailable = false;
assembly.quality.statusCode = "UNINITIALIZED";
assembly.quality.parallelSharedClearance = NaN;
assembly.symmetryResidual = [NaN; NaN; NaN];
assembly.metadata.mode = mode;
assembly.metadata.units = geometry.units;
assembly.metadata.symmetryResidualUnits = ["m"; "m"; "rad"];

if ~lower.quality.valid
    assembly.quality.statusCode = lower.quality.statusCode;
    assembly.metadata.failingSide = "lower";
    return
end
if ~upperLocal.quality.valid
    assembly.quality.statusCode = upperLocal.quality.statusCode;
    assembly.metadata.failingSide = "upper";
    return
end

rotation = -eye(2);
upperA = lower.points.G + [geometry.links.link5; 0];
upper = duallink5.kinematics.transformFiveBarPose( ...
    upperLocal, rotation, upperA);
assembly.upper = upper;

lowerStart = lower.points.E;
lowerEnd = lower.points.D;
upperStart = upper.points.D;
upperEnd = upper.points.E;
lowerCenter = (lowerStart + lowerEnd) / 2;
upperCenter = (upperStart + upperEnd) / 2;
lowerAngle = atan2( ...
    lowerEnd(2) - lowerStart(2), lowerEnd(1) - lowerStart(1));
upperAngle = atan2( ...
    upperEnd(2) - upperStart(2), upperEnd(1) - upperStart(1));
lowerEstimate = struct( ...
    'start', lowerStart, ...
    'end', lowerEnd, ...
    'center', lowerCenter, ...
    'orientation', lowerAngle);
upperEstimate = struct( ...
    'start', upperStart, ...
    'end', upperEnd, ...
    'center', upperCenter, ...
    'orientation', upperAngle);

assembly.symmetryResidual = [ ...
    upperCenter - lowerCenter; ...
    wrapAngle(upperAngle - lowerAngle)];
assembly.sharedLinkEstimates.lower = lowerEstimate;
assembly.sharedLinkEstimates.upper = upperEstimate;
assembly.quality.diagnosticAvailable = true;
assembly.quality.valid = true;

positionMatch = norm(assembly.symmetryResidual(1:2)) <= ...
    geometry.tolerance.symmetryPosition;
angleMatch = abs(assembly.symmetryResidual(3)) <= ...
    geometry.tolerance.symmetryAngle;
if positionMatch && angleMatch && mode == "ideal"
    assembly.sharedLink = lowerEstimate;
    assembly.quality.statusCode = "OK";
elseif mode == "diagnostic"
    if positionMatch && angleMatch
        assembly.quality.statusCode = "OK";
    else
        assembly.quality.valid = false;
        assembly.quality.statusCode = "SHARED_LINK_MISMATCH";
    end
else
    assembly.quality.valid = false;
    assembly.quality.statusCode = "SHARED_LINK_MISMATCH";
end

if assembly.quality.valid && mode == "ideal"
    collisionProfile = ...
        duallink5.validation.validateCollisionConfiguration( ...
            getOption(options, 'collisionProfile', "centerline"), ...
            geometry.collision, true);
    clearance = parallelSharedDistance(lower.points);
    assembly.quality.parallelSharedClearance = clearance;
    if collisionProfile ~= "none" && clearance <= ...
            geometry.collision.parallelSharedClearance
        assembly.quality.valid = false;
        assembly.quality.statusCode = "PARALLEL_SHARED_COLLISION";
        assembly = rmfield(assembly, 'sharedLink');
    end
end
end

function distance = parallelSharedDistance(points)
firstStart = points.Pbeta1(:);
firstEnd = points.Pbeta2(:);
secondStart = points.E(:);
secondEnd = points.D(:);
distance = min([ ...
    pointSegmentDistance(firstStart, secondStart, secondEnd), ...
    pointSegmentDistance(firstEnd, secondStart, secondEnd), ...
    pointSegmentDistance(secondStart, firstStart, firstEnd), ...
    pointSegmentDistance(secondEnd, firstStart, firstEnd)]);
end

function distance = pointSegmentDistance(point, startPoint, endPoint)
segment = endPoint - startPoint;
denominator = dot(segment, segment);
if denominator == 0
    distance = norm(point - startPoint);
    return
end
parameter = dot(point - startPoint, segment) / denominator;
parameter = max(0, min(1, parameter));
distance = norm(point - (startPoint + parameter * segment));
end

function angle = wrapAngle(angle)
angle = atan2(sin(angle), cos(angle));
end

function value = getOption(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end

function invalidMode()
error('duallink5:kinematics:InvalidAssemblyMode', ...
    'mode must be ideal or diagnostic.');
end
