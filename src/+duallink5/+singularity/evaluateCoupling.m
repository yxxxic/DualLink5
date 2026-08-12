function result = evaluateCoupling(qLower, geometry, options)
%EVALUATECOUPLING Evaluate passive-doublet coupling at one lower pose.
if nargin < 3
    options = struct();
end
qLower = validateQ(qLower);
geometry = duallink5.model.validateGeometry(geometry);
options = normalizeCouplingOptions(options, geometry);
reference = prepareCouplingReference(options, geometry);
context = struct( ...
    'geometry', geometry, ...
    'options', options, ...
    'reference', reference);
result = evaluateCouplingPrepared(qLower, context);
end

function q = validateQ(q)
if ~(isnumeric(q) && isreal(q) && isequal(size(q), [1, 2]) && ...
        all(isfinite(q)))
    error('duallink5:singularity:InvalidCouplingInput', ...
        'qLower must be a finite real numeric 1x2 vector.');
end
q = full(double(q));
end
