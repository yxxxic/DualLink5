function result = evaluate(q, geometry, options)
if nargin < 3
    options = struct();
end
geometry = duallink5.model.validateGeometry(geometry);
options = normalizeOptions(options, geometry);

if isnumeric(q)
    q = normalizeJointVector(q);
    mode = "ideal";
    lower = evaluateSide(q, geometry, options);
    upper = lower;
    logicalMetrics = lower;
elseif isstruct(q) && isscalar(q) && ...
        isfield(q, 'lower') && isfield(q, 'upper')
    lowerQ = normalizeJointVector(q.lower);
    upperQ = normalizeJointVector(q.upper);
    mode = "diagnostic";
    lower = evaluateSide(lowerQ, geometry, options);
    upper = evaluateSide(upperQ, geometry, options);
    logicalMetrics = [];
else
    invalidJointInput();
end

result.mode = mode;
result.logical = logicalMetrics;
result.lower = lower;
result.upper = upper;
result.nearSingular = lower.near.any || upper.near.any;
result.worstMargin = worstMargin(lower, upper);

if mode == "ideal"
    reference = prepareTopologyReference(geometry, options);
    assessment = assessIdealAssembly( ...
        q, geometry, options, reference);
    result.assemblyValid = assessment.valid;
    result.assemblyStatusCode = assessment.statusCode;
else
    input = struct('lower', lowerQ, 'upper', upperQ);
    assembly = duallink5.kinematics.forwardAssembly( ...
        input, geometry, struct('mode', "diagnostic", ...
        'branchMode', "fixed", 'branchId', options.branchId, ...
        'collisionProfile', options.collisionProfile));
    result.assemblyValid = assembly.quality.valid;
    result.assemblyStatusCode = assembly.quality.statusCode;
end
end

function q = normalizeJointVector(q)
if ~(isnumeric(q) && isreal(q) && isequal(size(q), [1, 2]) && ...
        all(isfinite(q)))
    invalidJointInput();
end
q = double(q);
end

function value = worstMargin(lower, upper)
values = [lower.margins.typeITheta, lower.margins.typeIPhi, ...
    lower.margins.typeII, upper.margins.typeITheta, ...
    upper.margins.typeIPhi, upper.margins.typeII];
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = min(values);
end
end

function invalidJointInput()
error('duallink5:singularity:InvalidJointInput', ...
    ['q must be a finite real 1-by-2 vector or a scalar struct ', ...
     'containing finite real q.lower and q.upper vectors.']);
end
