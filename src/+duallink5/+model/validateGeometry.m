function geometry = validateGeometry(geometry)
if ~isstruct(geometry) || ~isscalar(geometry)
    fail('geometry must be a scalar struct.');
end

requireStruct(geometry, 'links');
validatePositiveFields(geometry.links, ...
    ["link1", "link2", "link3", "link4", "link5"], 'links');

requireStruct(geometry, 'parallel');
requireStruct(geometry.parallel, 'lengths');
requireStruct(geometry.parallel, 'direction');
validatePositiveFields(geometry.parallel.lengths, ...
    ["E_Palpha1", "Palpha1_Palpha4", ...
     "D_Pbeta2", "Pbeta2_Pbeta3"], 'parallel.lengths');
validateSignedFields(geometry.parallel.direction, ...
    ["alphaAlongLink", "alphaSide", "betaAlongLink", "betaSide"], ...
    'parallel.direction');

requireStruct(geometry, 'assembly');
if ~isfield(geometry.assembly, 'defaultBranch') || ...
        ~isscalar(geometry.assembly.defaultBranch) || ...
        ~isnumeric(geometry.assembly.defaultBranch) || ...
        ~isreal(geometry.assembly.defaultBranch) || ...
        ~ismember(double(geometry.assembly.defaultBranch), [-1, 1])
    fail('assembly.defaultBranch must be -1 or +1.');
end

requireStruct(geometry, 'tolerance');
validatePositiveFields(geometry.tolerance, ...
    ["length", "residual", "symmetryPosition", ...
     "symmetryAngle", "singularityCondition"], 'tolerance');

requireStruct(geometry, 'analysis');
validateRange(geometry.analysis, 'thetaRange');
validateRange(geometry.analysis, 'phiRange');
validateReferenceQ(geometry.analysis);
validateCouplingWarningAngle(geometry.analysis);

requireStruct(geometry, 'collision');
if ~isfield(geometry.collision, 'radius') || ...
        ~isstruct(geometry.collision.radius) || ...
        ~isscalar(geometry.collision.radius) || ...
        ~isfield(geometry.collision, 'layerOffset') || ...
        ~isstruct(geometry.collision.layerOffset) || ...
        ~isscalar(geometry.collision.layerOffset) || ...
        ~isfield(geometry.collision, 'clearance') || ...
        ~(isscalar(geometry.collision.clearance) && ...
          isnumeric(geometry.collision.clearance) && ...
          isreal(geometry.collision.clearance) && ...
          isfinite(geometry.collision.clearance) && ...
          geometry.collision.clearance >= 0) || ...
        ~isfield(geometry.collision, 'parallelSharedClearance') || ...
        ~(isscalar(geometry.collision.parallelSharedClearance) && ...
          isnumeric(geometry.collision.parallelSharedClearance) && ...
          isreal(geometry.collision.parallelSharedClearance) && ...
          isfinite(geometry.collision.parallelSharedClearance) && ...
          geometry.collision.parallelSharedClearance >= 0) || ...
        ~isfield(geometry.collision, 'exemptPairs') || ...
        ~(isstring(geometry.collision.exemptPairs) && ...
          ismatrix(geometry.collision.exemptPairs) && ...
          size(geometry.collision.exemptPairs, 2) == 2)
    fail('collision fields are incomplete or invalid.');
end

validateUnits(geometry);
validateRequiredText(geometry, 'version');
validateRequiredText(geometry, 'convention');
end

function requireStruct(parent, name)
if ~isfield(parent, name) || ~isstruct(parent.(name)) || ...
        ~isscalar(parent.(name))
    fail('%s must be a scalar struct.', name);
end
end

function validatePositiveFields(parent, names, prefix)
for name = names
    fieldName = char(name);
    if ~isfield(parent, fieldName)
        fail('%s.%s is required.', prefix, fieldName);
    end
    value = parent.(fieldName);
    if ~(isscalar(value) && isnumeric(value) && isreal(value) && ...
            isfinite(value) && value > 0)
        fail('%s.%s must be a positive finite numeric scalar.', ...
            prefix, fieldName);
    end
end
end

function validateSignedFields(parent, names, prefix)
for name = names
    fieldName = char(name);
    if ~isfield(parent, fieldName)
        fail('%s.%s is required.', prefix, fieldName);
    end
    value = parent.(fieldName);
    if ~(isscalar(value) && isnumeric(value) && isreal(value) && ...
            isfinite(value) && ismember(value, [-1, 1]))
        fail('%s.%s must be -1 or +1.', prefix, fieldName);
    end
end
end

function validateRange(parent, name)
if ~isfield(parent, name)
    fail('analysis.%s is required.', name);
end
value = parent.(name);
if ~(isnumeric(value) && isreal(value) && isequal(size(value), [1, 2]) && ...
        all(isfinite(value)) && value(1) < value(2))
    fail('analysis.%s must be a finite increasing 1x2 vector.', name);
end
end

function validateReferenceQ(analysis)
if ~isfield(analysis, 'referenceQ')
    fail('analysis.referenceQ is required.');
end
value = analysis.referenceQ;
if ~(isnumeric(value) && isreal(value) && ...
        isequal(size(value), [1, 2]) && all(isfinite(value)))
    fail('analysis.referenceQ must be a finite real numeric 1x2 vector.');
end
end

function validateCouplingWarningAngle(analysis)
if ~isfield(analysis, 'couplingWarningAngle')
    fail('analysis.couplingWarningAngle is required.');
end
value = analysis.couplingWarningAngle;
if ~(isnumeric(value) && isreal(value) && isscalar(value) && ...
        isfinite(value) && value > 0 && value < pi / 2)
    fail(['analysis.couplingWarningAngle must be a finite scalar ', ...
        'strictly between 0 and pi/2.']);
end
end

function validateUnits(geometry)
if ~isfield(geometry, 'units') || ~isstruct(geometry.units) || ...
        ~isscalar(geometry.units) || ...
        ~isfield(geometry.units, 'length') || ...
        ~isfield(geometry.units, 'angle')
    invalidUnits();
end

try
    lengthUnit = string(geometry.units.length);
    angleUnit = string(geometry.units.angle);
catch
    invalidUnits();
end

if ~isscalar(lengthUnit) || ~isscalar(angleUnit) || ...
        ~isequal(lengthUnit, "m") || ~isequal(angleUnit, "rad")
    invalidUnits();
end
end

function validateRequiredText(geometry, name)
if ~isfield(geometry, name)
    fail('%s is required.', name);
end
try
    value = string(geometry.(name));
catch
    fail('%s must be nonempty scalar text.', name);
end
if ~isscalar(value) || ismissing(value) || strlength(value) == 0
    fail('%s must be nonempty scalar text.', name);
end
end

function invalidUnits()
error('duallink5:model:InvalidUnits', ...
    'Kinematics geometry must use length=m and angle=rad.');
end

function fail(message, varargin)
error('duallink5:model:InvalidGeometry', message, varargin{:});
end
