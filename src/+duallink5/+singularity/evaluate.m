function result = evaluate(q, geometry, options)
if nargin < 3
    options = struct();
end
geometry = duallink5.model.validateGeometry(geometry);
options = normalizeOptions(options, geometry);
if ~(isnumeric(q) && isreal(q) && isequal(size(q), [1, 2]) && ...
        all(isfinite(q)))
    error('duallink5:singularity:InvalidJointInput', ...
        'q must be a finite real 1-by-2 vector or diagnostic struct.');
end
q = double(q);
logicalMetrics = evaluateSide(q, geometry, options);
result.mode = "ideal";
result.logical = logicalMetrics;
result.lower = logicalMetrics;
result.upper = logicalMetrics;
result.assemblyValid = false;
result.assemblyStatusCode = "NOT_EVALUATED";
result.nearSingular = logicalMetrics.near.any;
result.worstMargin = min([logicalMetrics.margins.typeITheta, ...
    logicalMetrics.margins.typeIPhi, logicalMetrics.margins.typeII]);
end
