function samples = sampleWorkspace(grid, geometry, taskSpec, options)
if nargin < 4
    options = struct();
end
if ~isstruct(grid) || ~isscalar(grid) || ...
        ~isfield(grid, 'theta') || ~isfield(grid, 'phi') || ...
        ~isValidAngleVector(grid.theta) || ...
        ~isValidAngleVector(grid.phi)
    error('duallink5:workspace:InvalidAngleGrid', ...
        'grid.theta and grid.phi must be finite vectors.');
end
if ~isstruct(options) || ~isscalar(options)
    error('duallink5:workspace:InvalidWorkspaceOptions', ...
        'options must be a scalar struct.');
end
try
    topologyProfile = string( ...
        getOption(options, 'topologyProfile', "reference"));
catch
    invalidWorkspaceOptions();
end
if ~isscalar(topologyProfile) || ismissing(topologyProfile) || ...
        ~ismember(topologyProfile, ["reference", "none"])
    invalidWorkspaceOptions();
end
taskSpec = normalizeTaskSpec(taskSpec);
geometry = duallink5.model.validateGeometry(geometry);

[thetaGrid, phiGrid] = ndgrid( ...
    full(double(grid.theta)), full(double(grid.phi)));
gridShape = size(thetaGrid);
samples.x = nan(gridShape);
samples.y = nan(gridShape);
samples.orientation = nan(gridShape);
samples.validMask = false(gridShape);
samples.reasonMap = strings(gridShape);
samples.conditionNumber = nan(gridShape);
samples.thetaGrid = thetaGrid;
samples.phiGrid = phiGrid;

topologyReference = [];
if topologyProfile == "reference"
    referenceQ = geometry.analysis.referenceQ;
    referenceInput = struct('lower', referenceQ, 'upper', referenceQ);
    referenceOptions = struct( ...
        'mode', "ideal", ...
        'branchMode', "fixed", ...
        'branchId', geometry.assembly.defaultBranch);
    referenceAssembly = duallink5.kinematics.forwardAssembly( ...
        referenceInput, geometry, referenceOptions);
    if ~referenceAssembly.quality.valid
        error('duallink5:workspace:InvalidTopologyReference', ...
            'geometry.analysis.referenceQ must form a valid assembly.');
    end
    topologyReference = ...
        duallink5.validation.prepareAssemblyTopologyReference( ...
        referenceAssembly, geometry.tolerance.length);
end

for index = 1:numel(thetaGrid)
    qLogical = [thetaGrid(index), phiGrid(index)];
    q.lower = qLogical;
    q.upper = qLogical;
    assemblyOptions = options;
    if isfield(assemblyOptions, 'topologyProfile')
        assemblyOptions = rmfield(assemblyOptions, 'topologyProfile');
    end
    assemblyOptions.mode = "ideal";
    assembly = duallink5.kinematics.forwardAssembly( ...
        q, geometry, assemblyOptions);
    samples.reasonMap(index) = assembly.quality.statusCode;
    if ~assembly.quality.valid
        continue
    end

    if topologyProfile == "reference"
        topology = ...
            duallink5.validation.compareAssemblyTopologyToReference( ...
            assembly, topologyReference);
        if ~topology.compatible
            samples.reasonMap(index) = topology.statusCode;
            continue
        end
    end

    jacobian = duallink5.kinematics.fiveBarJacobian( ...
        qLogical, assembly.lower, geometry, taskSpec);
    samples.conditionNumber(index) = jacobian.conditionNumber;
    if jacobian.nearSingular
        samples.reasonMap(index) = "NEAR_SINGULAR";
        continue
    end

    task = duallink5.kinematics.taskPose(assembly, taskSpec);
    samples.x(index) = task.position(1);
    samples.y(index) = task.position(2);
    if task.includeOrientation
        samples.orientation(index) = task.orientation;
    end
    samples.validMask(index) = true;
end

samples.metadata.units = geometry.units;
samples.metadata.taskSpec = taskSpec;
samples.metadata.topologyProfile = topologyProfile;
samples.metadata.referenceQ = geometry.analysis.referenceQ;
end

function valid = isValidAngleVector(value)
valid = isnumeric(value) && isreal(value) && isvector(value) && ...
    ~isempty(value) && all(isfinite(value), 'all');
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
    if ~isnumeric(orientationOffset) || ...
            ~isreal(orientationOffset) || ...
            ~isscalar(orientationOffset) || ...
            ~isfinite(orientationOffset)
        invalidTaskSpec( ...
            'orientationOffset must be a finite numeric scalar.');
    end
    taskSpec.orientationOffset = double(orientationOffset);
end
end

function invalidTaskSpec(message, varargin)
error('duallink5:kinematics:InvalidTaskSpec', ...
    message, varargin{:});
end

function value = getOption(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end

function invalidWorkspaceOptions()
error('duallink5:workspace:InvalidWorkspaceOptions', ...
    'topologyProfile must be reference or none.');
end
