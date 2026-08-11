function options = normalizeOptions(options, geometry)
if nargin < 1 || isempty(options)
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    invalidOptions();
end
allowed = {'branchId', 'exactThreshold', 'nearMetricThreshold', ...
    'nearConditionThreshold', 'collisionProfile', 'topologyProfile'};
if ~all(ismember(fieldnames(options), allowed))
    invalidOptions();
end

options.branchId = getOption( ...
    options, 'branchId', geometry.assembly.defaultBranch);
if ~(isnumeric(options.branchId) && isreal(options.branchId) && ...
        isscalar(options.branchId) && isfinite(options.branchId) && ...
        ismember(double(options.branchId), [-1, 1]))
    invalidOptions();
end
options.branchId = int8(options.branchId);

options.exactThreshold = scalarOption( ...
    options, 'exactThreshold', 1e-8);
options.nearMetricThreshold = scalarOption( ...
    options, 'nearMetricThreshold', 0.05);
options.nearConditionThreshold = scalarOption( ...
    options, 'nearConditionThreshold', 100);
if options.exactThreshold >= options.nearMetricThreshold || ...
        options.nearMetricThreshold > 1 || ...
        options.nearConditionThreshold <= 1
    invalidOptions();
end

try
    options.collisionProfile = ...
        duallink5.validation.validateCollisionConfiguration( ...
        getOption(options, 'collisionProfile', "centerline"), ...
        geometry.collision, true);
    options.topologyProfile = string( ...
        getOption(options, 'topologyProfile', "reference"));
catch exception
    if strcmp(exception.identifier, ...
            'duallink5:validation:MissingClearanceGeometry')
        rethrow(exception)
    end
    invalidOptions();
end
if ~isscalar(options.topologyProfile) || ...
        ismissing(options.topologyProfile) || ...
        ~ismember(options.topologyProfile, ["reference", "none"])
    invalidOptions();
end
end

function value = scalarOption(options, name, defaultValue)
value = getOption(options, name, defaultValue);
if ~(isnumeric(value) && isreal(value) && isscalar(value) && ...
        isfinite(value) && value > 0)
    invalidOptions();
end
value = double(value);
end

function value = getOption(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end

function invalidOptions()
error('duallink5:singularity:InvalidOptions', ...
    ['options must define a valid fixed branch, thresholds, ', ...
     'collision profile, and topology profile.']);
end
