function task = taskPose(assembly, taskSpec)
if ~isstruct(taskSpec) || ~isscalar(taskSpec) || ...
        ~isfield(taskSpec, 'kind')
    invalidTaskSpec('taskSpec.kind is required.');
end
try
    kind = string(taskSpec.kind);
catch
    invalidTaskSpec('kind must be scalar string-convertible text.');
end
includeOrientation = getField( ...
    taskSpec, 'includeOrientation', false);
validInclude = isscalar(includeOrientation) && ...
    (islogical(includeOrientation) || ...
    (isnumeric(includeOrientation) && isreal(includeOrientation) && ...
    isfinite(includeOrientation) && ismember(includeOrientation, [0, 1])));
if ~isscalar(kind) || ismissing(kind) || ~validInclude
    invalidTaskSpec( ...
        'kind must be scalar and includeOrientation must be logical.');
end
includeOrientation = logical(includeOrientation);

if ~isstruct(assembly) || ~isscalar(assembly) || ...
        ~isfield(assembly, 'metadata') || ...
        ~isstruct(assembly.metadata) || ...
        ~isscalar(assembly.metadata) || ...
        ~isfield(assembly.metadata, 'mode') || ...
        ~isfield(assembly.metadata, 'units') || ...
        ~isfield(assembly, 'quality') || ...
        ~isstruct(assembly.quality) || ~isscalar(assembly.quality)
    invalidAssemblyPose();
end
try
    mode = string(assembly.metadata.mode);
catch
    invalidAssemblyPose();
end
if ~isscalar(mode) || ismissing(mode) || ...
        ~ismember(mode, ["ideal", "diagnostic"])
    invalidAssemblyPose();
end

if mode == "diagnostic"
    diagnosticAvailable = readAssemblyFlag( ...
        assembly.quality, 'diagnosticAvailable');
    if ~diagnosticAvailable
        error('duallink5:kinematics:InvalidAssemblyPose', ...
            'Diagnostic side poses are unavailable.');
    end
    try
        side = string(getField(taskSpec, 'side', ""));
    catch
        ambiguousDiagnosticTask();
    end
    if ~isscalar(side) || ismissing(side) || ...
            ~ismember(side, ["lower", "upper"])
        ambiguousDiagnosticTask();
    end
    if ~isfield(assembly, 'sharedLinkEstimates') || ...
            ~isstruct(assembly.sharedLinkEstimates) || ...
            ~isscalar(assembly.sharedLinkEstimates) || ...
            ~isfield(assembly.sharedLinkEstimates, char(side))
        invalidAssemblyPose();
    end
    shared = assembly.sharedLinkEstimates.(char(side));
    if ~isTaskFrame(shared)
        invalidAssemblyPose();
    end
    shared.center = double(shared.center(:));
    shared.orientation = double(shared.orientation);
    pointG = readPointG(assembly, side);
else
    valid = readAssemblyFlag(assembly.quality, 'valid');
    if ~valid || ...
            ~isfield(assembly, 'sharedLink')
        invalidAssemblyPose();
    end
    shared = assembly.sharedLink;
    if ~isTaskFrame(shared)
        invalidAssemblyPose();
    end
    shared.center = double(shared.center(:));
    shared.orientation = double(shared.orientation);
    pointG = readPointG(assembly, "lower");
end

orientationOffset = getField(taskSpec, 'orientationOffset', 0);
if ~(isscalar(orientationOffset) && isnumeric(orientationOffset) && ...
        isreal(orientationOffset) && isfinite(orientationOffset))
    invalidTaskSpec( ...
        'orientationOffset must be a finite scalar in radians.');
end
orientationOffset = double(orientationOffset);

if kind == "sharedCenter"
    position = shared.center;
elseif kind == "pointG"
    position = pointG;
elseif ismember(kind, ["sharedOffset", "marker"])
    if ~isfield(taskSpec, 'offset')
        invalidTaskSpec('%s requires taskSpec.offset.', kind);
    end
    offset = taskSpec.offset;
    if ~isnumeric(offset) || ~isreal(offset) || numel(offset) ~= 2 || ...
            any(~isfinite(offset), 'all')
        invalidTaskSpec('%s requires a finite 2-vector offset.', kind);
    end
    offset = double(offset(:));
    angle = shared.orientation;
    rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    position = shared.center + rotation * offset;
else
    invalidTaskSpec('Unsupported task kind: %s.', kind);
end

task.position = position;
task.orientation = wrapAngle(shared.orientation + orientationOffset);
task.includeOrientation = includeOrientation;
if includeOrientation
    task.vector = [position; task.orientation];
else
    task.vector = position;
end
task.units = assembly.metadata.units;
end

function angle = wrapAngle(angle)
angle = atan2(sin(angle), cos(angle));
end

function value = getField(input, name, defaultValue)
if isfield(input, name)
    value = input.(name);
else
    value = defaultValue;
end
end

function value = readAssemblyFlag(quality, name)
if ~isfield(quality, name)
    invalidAssemblyPose();
end
rawValue = quality.(name);
validFlag = isscalar(rawValue) && ...
    (islogical(rawValue) || ...
    (isnumeric(rawValue) && isreal(rawValue) && ...
    isfinite(rawValue) && ismember(double(rawValue), [0, 1])));
if ~validFlag
    invalidAssemblyPose();
end
value = logical(rawValue);
end

function valid = isTaskFrame(shared)
valid = isstruct(shared) && isscalar(shared) && ...
    isfield(shared, 'center') && ...
    isnumeric(shared.center) && isreal(shared.center) && ...
    numel(shared.center) == 2 && all(isfinite(shared.center), 'all') && ...
    isfield(shared, 'orientation') && ...
    isnumeric(shared.orientation) && isreal(shared.orientation) && ...
    isscalar(shared.orientation) && isfinite(shared.orientation);
end

function pointG = readPointG(assembly, side)
sideName = char(side);
if ~isfield(assembly, sideName) || ...
        ~isstruct(assembly.(sideName)) || ...
        ~isscalar(assembly.(sideName)) || ...
        ~isfield(assembly.(sideName), 'points') || ...
        ~isstruct(assembly.(sideName).points) || ...
        ~isscalar(assembly.(sideName).points) || ...
        ~isfield(assembly.(sideName).points, 'G')
    invalidAssemblyPose();
end
pointG = assembly.(sideName).points.G;
if ~isnumeric(pointG) || ~isreal(pointG) || numel(pointG) ~= 2 || ...
        any(~isfinite(pointG), 'all')
    invalidAssemblyPose();
end
pointG = double(pointG(:));
end

function invalidTaskSpec(message, varargin)
error('duallink5:kinematics:InvalidTaskSpec', message, varargin{:});
end

function invalidAssemblyPose()
error('duallink5:kinematics:InvalidAssemblyPose', ...
    'A valid assembly pose is required.');
end

function ambiguousDiagnosticTask()
error('duallink5:kinematics:AmbiguousDiagnosticTask', ...
    'Diagnostic taskPose requires taskSpec.side=lower or upper.');
end
