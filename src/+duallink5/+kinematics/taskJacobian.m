function result = taskJacobian(q, geometry, taskSpec, options)
if nargin < 4
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error('duallink5:kinematics:InvalidJacobianOptions', ...
        'options must be a scalar struct.');
end

if isnumeric(q)
    if ~isreal(q) || numel(q) ~= 2
        invalidJointVector();
    end
    q = double(q(:).');
    inputKind = "ideal";
elseif isstruct(q) && isscalar(q)
    if ~isfield(q, 'lower') || ~isfield(q, 'upper')
        error('duallink5:kinematics:InvalidAssemblyInput', ...
            'q.lower and q.upper are required.');
    end
    inputKind = "diagnostic";
else
    invalidJointVector();
end
taskSpec = normalizeTaskSpec(taskSpec);

if inputKind == "ideal"
    input.lower = q;
    input.upper = q;
    idealOptions = options;
    idealOptions.mode = "ideal";
    assembly = duallink5.kinematics.forwardAssembly( ...
        input, geometry, idealOptions);
    if ~assembly.quality.valid
        result = invalidResult( ...
            taskSpec, assembly.quality.statusCode, false, assembly);
        return
    end
    lower = duallink5.kinematics.fiveBarJacobian( ...
        q, assembly.lower, geometry, taskSpec);
    result.logical = lower.matrix;
    result.lowerLocal = lower.matrix;
    result.upperLocal = lower.matrix;
    result.closureConditionNumber = ...
        lower.closureConditionNumber;
    result.taskConditionNumber = lower.taskConditionNumber;
    result.taskRowScale = lower.taskRowScale;
    result.conditionNumber = lower.conditionNumber;
    result.nearSingular = lower.nearSingular;
    result.statusCode = lower.statusCode;
    result.assemblyStatusCode = assembly.quality.statusCode;
    result = addAssemblyMetadata(result, assembly);
    return
end

diagnosticOptions = options;
diagnosticOptions.mode = "diagnostic";
assembly = duallink5.kinematics.forwardAssembly( ...
    q, geometry, diagnosticOptions);
if ~isfield(assembly.quality, 'diagnosticAvailable') || ...
        ~assembly.quality.diagnosticAvailable
    result = invalidResult( ...
        taskSpec, assembly.quality.statusCode, true, assembly);
    return
end
lower = duallink5.kinematics.fiveBarJacobian( ...
    q.lower, assembly.lower, geometry, taskSpec);
upper = duallink5.kinematics.fiveBarJacobian( ...
    q.upper, assembly.upperLocal, geometry, taskSpec);
result.logical = [];
result.lowerLocal = lower.matrix;
result.upperLocal = upper.matrix;
result.closureConditionNumber = [ ...
    lower.closureConditionNumber, ...
    upper.closureConditionNumber];
result.taskConditionNumber = [lower.taskConditionNumber, ...
    upper.taskConditionNumber];
result.taskRowScale = lower.taskRowScale;
result.conditionNumber = [lower.conditionNumber, ...
    upper.conditionNumber];
result.nearSingular = lower.nearSingular || upper.nearSingular;
result.assemblyStatusCode = assembly.quality.statusCode;
if result.nearSingular
    result.statusCode = "NEAR_SINGULAR";
else
    result.statusCode = "DIAGNOSTIC_ONLY";
end
result = addAssemblyMetadata(result, assembly);
end

function result = invalidResult( ...
        taskSpec, statusCode, diagnostic, assembly)
includeOrientation = taskSpec.includeOrientation;
rows = 2 + double(includeOrientation);
result.logical = nan(rows, 2);
if diagnostic
    result.logical = [];
end
result.lowerLocal = nan(rows, 2);
result.upperLocal = nan(rows, 2);
conditionNumber = Inf;
if diagnostic
    conditionNumber = [Inf, Inf];
end
result.closureConditionNumber = conditionNumber;
result.taskConditionNumber = conditionNumber;
result.taskRowScale = nan(1, rows);
result.conditionNumber = conditionNumber;
result.nearSingular = statusCode == "NEAR_SINGULAR";
result.statusCode = statusCode;
result.assemblyStatusCode = statusCode;
result = addAssemblyMetadata(result, assembly);
end

function result = addAssemblyMetadata(result, assembly)
result.failingSide = "";
if isfield(assembly.metadata, 'failingSide')
    result.failingSide = string(assembly.metadata.failingSide);
end
result.lowerStatusCode = assembly.lower.quality.statusCode;
result.upperStatusCode = assembly.upperLocal.quality.statusCode;
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

function invalidJointVector()
error('duallink5:kinematics:InvalidJointVector', ...
    'q must be a real numeric two-vector or a diagnostic input struct.');
end

function invalidTaskSpec(message, varargin)
error('duallink5:kinematics:InvalidTaskSpec', ...
    message, varargin{:});
end
