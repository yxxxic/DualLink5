# DualLink5 Singularity Analysis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add tested MATLAB APIs and two publication-oriented figures that classify and display Type I, Type II, and Type III singularities of the ideal DualLink5 mechanism doublet while retaining lower/upper diagnostic analysis.

**Architecture:** Keep single-configuration geometry and singularity metrics in a new `duallink5.singularity` package, use a separate sampler to combine theoretical closure with complete-assembly collision/topology validation, and keep figure rendering in `duallink5.viz`. The `DL5` class remains a thin, read-only facade; a single script wires the APIs into the approved two-figure workflow.

**Tech Stack:** MATLAB R2025a, package functions, `matlab.unittest`, existing DualLink5 kinematics/validation APIs, `tiledlayout`, `imagesc`, `contourc`, `scatter`, and Git.

---

## Workspace Guardrail

The worktree already contains user-owned staged script renames and an
unstaged trajectory edit. Every commit in this plan must use an explicit
pathspec. Do not run broad `git add .`, `git commit -a`, reset, checkout, or
cleanup commands.

Project root:

```text
D:\OneDrive\科研\项目\05-DualLink5\运动学\Matlab
```

Every executable MATLAB command below uses
`D:\Program Files\MATLAB\R2025a\bin\matlab.exe`; keep that same R2025a
executable for every TDD and verification command.

## File Map

- Create `src/+duallink5/+singularity/evaluate.m`: public ideal/diagnostic single-configuration API.
- Create `src/+duallink5/+singularity/sampleSpace.m`: grid sampler and complete-mechanism classification.
- Create `src/+duallink5/+singularity/private/normalizeOptions.m`: one source of option defaults and validation.
- Create `src/+duallink5/+singularity/private/evaluateSide.m`: pure local five-bar geometry and metric calculation.
- Create `src/+duallink5/+singularity/private/assessIdealAssembly.m`: joint-limit, full assembly, collision, and topology checks.
- Create `src/+duallink5/+singularity/private/prepareTopologyReference.m`: cached approved-reference construction.
- Create `src/+duallink5/+singularity/private/extractZeroContours.m`: `contourc` parser for joint curves.
- Create `src/+duallink5/+singularity/private/refineTypeIICurves.m`: project interpolated contours onto exact closure-tangency equations.
- Create `src/+duallink5/+singularity/private/findTypeIIILoci.m`: locate and refine simultaneous Type I/II roots along exact Type II curves.
- Create `src/+duallink5/+singularity/private/mapSingularityCurves.m`: map joint curves to G and mark mechanical adjacency.
- Create `src/+duallink5/+viz/plotJointSingularitySpace.m`: approved Figure 1.
- Create `src/+duallink5/+viz/plotTaskSingularitySpace.m`: approved Figure 2.
- Modify `src/+duallink5/DL5.m`: two read-only facade methods.
- Create `scripts/analyze_singularity_space.m`: direct user entry point.
- Create `tests/singularity/TestSingularityMetrics.m`: metric, threshold, and diagnostic tests.
- Create `tests/singularity/TestSingularitySpace.m`: grid, curve, topology, and mask tests.
- Create `tests/viz/TestSingularityVisualization.m`: graphics contract tests.
- Modify `tests/facade/TestDL5.m`: facade state-safety tests.

### Task 1: Core point-G singularity metrics

**Files:**

- Create: `tests/singularity/TestSingularityMetrics.m`
- Create: `src/+duallink5/+singularity/evaluate.m`
- Create: `src/+duallink5/+singularity/private/normalizeOptions.m`
- Create: `src/+duallink5/+singularity/private/evaluateSide.m`

- [ ] **Step 1: Write the failing reference and singular-type tests**

Create `tests/singularity/TestSingularityMetrics.m` with this initial content:

```matlab
classdef TestSingularityMetrics < matlab.unittest.TestCase
    properties
        Geometry
        RegularOptions
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            testCase.Geometry = duallink5.model.defaultGeometry();
            testCase.RegularOptions = struct( ...
                'collisionProfile', "none", ...
                'topologyProfile', "none");
        end
    end

    methods (Test)
        function referenceConfigurationMatchesIndependentValues(testCase)
            result = duallink5.singularity.evaluate( ...
                deg2rad([85, 30]), testCase.Geometry, ...
                testCase.RegularOptions);
            metrics = result.logical;

            testCase.verifyEqual(metrics.classification, "REGULAR");
            testCase.verifyEqual(metrics.closureStatusCode, "OK");
            testCase.verifyEqual( ...
                metrics.conditionNumber.constraint, ...
                2.72294646944709, 'AbsTol', 1e-11);
            testCase.verifyEqual( ...
                metrics.conditionNumber.task, ...
                2.62563451612049, 'AbsTol', 1e-11);
            testCase.verifyEqual(metrics.margins.typeITheta, ...
                0.996379406563618, 'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.margins.typeIPhi, ...
                0.869963303515486, 'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.margins.typeII, ...
                0.647208200758206, 'AbsTol', 1e-12);
            testCase.verifyEqual(metrics.orientationSensitivity, ...
                1.6337986967138, 'AbsTol', 1e-11);
        end

        function signedIndicatorsMatchGeometricSineDefinitions(testCase)
            q = deg2rad([85, 30]);
            result = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, testCase.RegularOptions);
            metrics = result.logical;
            L = testCase.Geometry.links;
            B = [L.link5; 0];
            E = L.link1 * [cos(q(1)); sin(q(1))];
            C = [L.link5 - L.link2 * cos(q(2)); ...
                L.link2 * sin(q(2))];
            D = metrics.pointG - E + B;
            u = D - C;
            v = D - E;
            eTheta = L.link1 * [-sin(q(1)); cos(q(1))];
            cPhi = L.link2 * [sin(q(2)); cos(q(2))];
            expected = [dot(v, eTheta) / (L.link4 * L.link1), ...
                dot(u, cPhi) / (L.link3 * L.link2), ...
                (u(1) * v(2) - u(2) * v(1)) / ...
                (L.link3 * L.link4)];

            actual = [metrics.signedIndicator.typeITheta, ...
                metrics.signedIndicator.typeIPhi, ...
                metrics.signedIndicator.typeII];
            margins = [metrics.margins.typeITheta, ...
                metrics.margins.typeIPhi, metrics.margins.typeII];
            testCase.verifyEqual(actual, expected, 'AbsTol', 1e-12);
            testCase.verifyEqual(margins, abs(expected), 'AbsTol', 1e-12);
        end

        function classifiesBothTypeISubtypes(testCase)
            thetaCase = duallink5.singularity.evaluate( ...
                deg2rad([45.206365032903925, 79]), ...
                testCase.Geometry, testCase.RegularOptions);
            phiCase = duallink5.singularity.evaluate( ...
                deg2rad([56.98067735693949, 1]), ...
                testCase.Geometry, testCase.RegularOptions);

            testCase.verifyTrue(thetaCase.logical.exact.typeITheta);
            testCase.verifyFalse(thetaCase.logical.exact.typeIPhi);
            testCase.verifyEqual( ...
                thetaCase.logical.classification, "TYPE_I");
            testCase.verifyTrue(phiCase.logical.exact.typeIPhi);
            testCase.verifyFalse(phiCase.logical.exact.typeITheta);
            testCase.verifyEqual( ...
                phiCase.logical.classification, "TYPE_I");
        end

        function retainsOuterAndInnerTangencyPointG(testCase)
            outerQ = [3.1273346915827149, deg2rad(69)];
            innerQ = [0.83109759906031577, deg2rad(56.5)];
            outer = duallink5.singularity.evaluate( ...
                outerQ, testCase.Geometry, testCase.RegularOptions);
            inner = duallink5.singularity.evaluate( ...
                innerQ, testCase.Geometry, testCase.RegularOptions);

            for metrics = {outer.logical, inner.logical}
                value = metrics{1};
                testCase.verifyTrue(value.reachable);
                testCase.verifyTrue(value.exact.typeII);
                testCase.verifyEqual(value.classification, "TYPE_II");
                testCase.verifyEqual( ...
                    value.closureStatusCode, "NEAR_SINGULAR");
                testCase.verifyTrue(all(isfinite(value.pointG)));
                testCase.verifyTrue( ...
                    all(isnan(value.matrices.taskJacobian), 'all'));
                testCase.verifyEqual( ...
                    value.conditionNumber.task, Inf);
            end
        end

        function unreachableClosureRetainsStableStatus(testCase)
            result = duallink5.singularity.evaluate( ...
                deg2rad([180, 90]), testCase.Geometry, ...
                testCase.RegularOptions);

            testCase.verifyFalse(result.logical.reachable);
            testCase.verifyEqual(result.logical.closureStatusCode, ...
                "UNREACHABLE_CLOSURE");
            testCase.verifyEqual(result.logical.classification, ...
                "UNREACHABLE_CLOSURE");
            testCase.verifyTrue(all(isnan(result.logical.pointG)));
        end

        function customGeometryProducesTypeIII(testCase)
            geometry = testCase.Geometry;
            geometry.links.link3 = 30e-3;
            geometry.links.link4 = 32e-3;
            geometry = duallink5.model.validateGeometry(geometry);
            result = duallink5.singularity.evaluate( ...
                [0, 0], geometry, testCase.RegularOptions);

            testCase.verifyTrue(result.logical.exact.typeITheta);
            testCase.verifyTrue(result.logical.exact.typeIPhi);
            testCase.verifyTrue(result.logical.exact.typeII);
            testCase.verifyTrue(result.logical.exact.typeIII);
            testCase.verifyEqual( ...
                result.logical.classification, "TYPE_III");
        end

        function analyticTaskJacobianMatchesCentralDifference(testCase)
            q = deg2rad([85, 30]);
            result = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, testCase.RegularOptions);
            numeric = zeros(2);
            step = 1e-7;
            for column = 1:2
                delta = zeros(1, 2);
                delta(column) = step;
                plus = evaluatePointG(q + delta, testCase.Geometry);
                minus = evaluatePointG(q - delta, testCase.Geometry);
                numeric(:, column) = (plus - minus) / (2 * step);
            end

            testCase.verifyEqual( ...
                result.logical.matrices.taskJacobian, ...
                numeric, 'AbsTol', 1e-6);
        end

        function thresholdBoundariesAreDeterministic(testCase)
            q = deg2rad([85, 30]);
            base = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, testCase.RegularOptions);
            boundary = base.logical.margins.typeII;
            common = struct('collisionProfile', "none", ...
                'topologyProfile', "none", ...
                'exactThreshold', 1e-8, ...
                'nearConditionThreshold', 1e12);
            atBoundary = common;
            atBoundary.nearMetricThreshold = boundary;
            inside = common;
            inside.nearMetricThreshold = boundary + 1e-12;
            outside = common;
            outside.nearMetricThreshold = boundary - 1e-12;

            equalResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, atBoundary);
            insideResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, inside);
            outsideResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, outside);
            testCase.verifyFalse(equalResult.logical.near.typeII);
            testCase.verifyTrue(insideResult.logical.near.typeII);
            testCase.verifyFalse(outsideResult.logical.near.typeII);

            exactAt = common;
            exactAt.exactThreshold = boundary;
            exactAt.nearMetricThreshold = boundary + 0.1;
            exactBelow = exactAt;
            exactBelow.exactThreshold = boundary - 1e-12;
            exactAtResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, exactAt);
            exactBelowResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, exactBelow);
            testCase.verifyTrue(exactAtResult.logical.exact.typeII);
            testCase.verifyFalse(exactBelowResult.logical.exact.typeII);

            conditionBoundary = ...
                base.logical.conditionNumber.constraint;
            conditionAt = struct('collisionProfile', "none", ...
                'topologyProfile', "none", ...
                'exactThreshold', 1e-8, ...
                'nearMetricThreshold', 0.01, ...
                'nearConditionThreshold', conditionBoundary);
            conditionInside = conditionAt;
            conditionInside.nearConditionThreshold = ...
                conditionBoundary - 1e-12;
            conditionAtResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, conditionAt);
            conditionInsideResult = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, conditionInside);
            testCase.verifyFalse( ...
                conditionAtResult.logical.near.typeII);
            testCase.verifyTrue( ...
                conditionInsideResult.logical.near.typeII);
        end
    end
end

function pointG = evaluatePointG(q, geometry)
input = struct('lower', q, 'upper', q);
assembly = duallink5.kinematics.forwardAssembly( ...
    input, geometry, struct('mode', "ideal", ...
    'branchMode', "fixed", ...
    'branchId', geometry.assembly.defaultBranch, ...
    'collisionProfile', "none"));
task = duallink5.kinematics.taskPose(assembly, ...
    struct('kind', "pointG", 'includeOrientation', false));
pointG = task.position;
end
```

- [ ] **Step 2: Run the new test and verify RED**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); run('startup.m'); results=runtests('tests/singularity/TestSingularityMetrics.m'); disp(results); assert(all([results.Passed]));"
```

Expected: FAIL because `duallink5.singularity.evaluate` does not exist.

- [ ] **Step 3: Implement shared option validation**

Create `src/+duallink5/+singularity/private/normalizeOptions.m`:

```matlab
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
```

- [ ] **Step 4: Implement the local five-bar evaluator**

Create `src/+duallink5/+singularity/private/evaluateSide.m`. The file must
contain the following public body and local helpers; do not call
`fiveBarJacobian`, because exact Type II must retain D and G without inverting
`A_c`:

```matlab
function result = evaluateSide(q, geometry, options)
L = geometry.links;
B = [L.link5; 0];
theta = q(1);
phi = q(2);
E = L.link1 * [cos(theta); sin(theta)];
C = [L.link5 - L.link2 * cos(phi); L.link2 * sin(phi)];

result = emptyResult(q);
[candidate, closureStatus] = selectClosureCandidate( ...
    C, E, L, geometry, options);
result.closureStatusCode = closureStatus;
result.closureDistance = norm(E - C);
if isempty(candidate)
    return
end

D = candidate.D;
G = D + E - B;
u = D - C;
v = D - E;
eTheta = L.link1 * [-sin(theta); cos(theta)];
cPhi = L.link2 * [sin(phi); cos(phi)];
crossUV = cross2(u, v);

result.reachable = true;
result.branchId = candidate.branchId;
result.pointG = G;
result.signedIndicator.typeITheta = ...
    dot(v, eTheta) / (L.link4 * L.link1);
result.signedIndicator.typeIPhi = ...
    dot(u, cPhi) / (L.link3 * L.link2);
result.signedIndicator.typeII = ...
    crossUV / (L.link3 * L.link4);
result.margins.typeITheta = ...
    abs(result.signedIndicator.typeITheta);
result.margins.typeIPhi = abs(result.signedIndicator.typeIPhi);
result.margins.typeII = abs(result.signedIndicator.typeII);

constraint = [u.'; v.'];
taskActuation = [dot(u, eTheta), dot(u, cPhi); ...
    2 * dot(v, eTheta), 0];
normalizedConstraint = [u.' / L.link3; v.' / L.link4];
constraintSingularValues = svd(normalizedConstraint);
result.matrices.constraint = constraint;
result.matrices.taskActuation = taskActuation;
result.singularValues.constraint = constraintSingularValues;
result.conditionNumber.constraint = ...
    safeConditionNumber(constraintSingularValues);

exactITheta = result.margins.typeITheta <= ...
    options.exactThreshold;
exactIPhi = result.margins.typeIPhi <= options.exactThreshold;
exactI = exactITheta || exactIPhi;
exactII = result.margins.typeII <= options.exactThreshold;
result.exact = exactFlags(exactITheta, exactIPhi, exactI, exactII);

if exactII
    result.matrices.taskJacobian = nan(2);
    result.singularValues.task = [NaN; NaN];
    result.conditionNumber.task = Inf;
    result.dpsi_dq = [NaN, NaN];
    result.orientationSensitivity = NaN;
else
    taskJacobian = constraint \ taskActuation;
    normalizedTask = taskJacobian / L.link4;
    taskSingularValues = svd(normalizedTask);
    result.matrices.taskJacobian = taskJacobian;
    result.singularValues.task = taskSingularValues;
    result.conditionNumber.task = ...
        safeConditionNumber(taskSingularValues);
    result.dpsi_dq = orientationDerivative( ...
        constraint, u, v, eTheta, cPhi);
    result.orientationSensitivity = norm(result.dpsi_dq, 2);
end

nearITheta = result.margins.typeITheta < ...
    options.nearMetricThreshold;
nearIPhi = result.margins.typeIPhi < ...
    options.nearMetricThreshold;
nearI = nearITheta || nearIPhi;
nearII = result.margins.typeII < options.nearMetricThreshold || ...
    result.conditionNumber.constraint > ...
    options.nearConditionThreshold;
nearTask = result.conditionNumber.task > ...
    options.nearConditionThreshold;
result.near = nearFlags( ...
    result.exact, nearITheta, nearIPhi, nearI, nearII, nearTask);
result.classification = classify(result.exact, result.near);
end

function [candidate, closureStatus] = selectClosureCandidate( ...
        C, E, L, geometry, options)
candidate = [];
distance = norm(E - C);
targets = [L.link3 + L.link4, abs(L.link4 - L.link3)];
[targetResidual, targetIndex] = min(abs(distance - targets));
tangencyTolerance = 32 * eps(max([L.link3, L.link4, distance]));
targetDistance = targets(targetIndex);
if targetDistance > geometry.tolerance.length && ...
        targetResidual <= tangencyTolerance
    direction = (E - C) / distance;
    along = (L.link3^2 - L.link4^2 + targetDistance^2) / ...
        (2 * targetDistance);
    candidate = struct('D', C + along * direction, ...
        'branchId', int8(0));
    closureStatus = "NEAR_SINGULAR";
    return
end

[candidates, closureStatus] = duallink5.kinematics.solveClosure( ...
    C, E, L.link3, L.link4, geometry.tolerance.length);
if isempty(candidates)
    return
end
if closureStatus == "NEAR_SINGULAR"
    candidate = candidates(1);
    return
end
branchIds = [candidates.branchId];
selected = find(branchIds == options.branchId, 1);
if isempty(selected)
    closureStatus = "NO_VALID_BRANCH";
else
    candidate = candidates(selected);
end
end

function result = emptyResult(q)
result.q = q;
result.reachable = false;
result.closureStatusCode = "UNREACHABLE_CLOSURE";
result.closureDistance = NaN;
result.branchId = int8(0);
result.pointG = [NaN; NaN];
result.matrices.constraint = nan(2);
result.matrices.taskActuation = nan(2);
result.matrices.taskJacobian = nan(2);
result.signedIndicator = scalarFields(NaN, NaN, NaN);
result.margins = scalarFields(NaN, NaN, NaN);
result.singularValues.constraint = [NaN; NaN];
result.singularValues.task = [NaN; NaN];
result.conditionNumber.constraint = Inf;
result.conditionNumber.task = Inf;
result.exact = exactFlags(false, false, false, false);
result.near = nearFlags( ...
    result.exact, false, false, false, false, false);
result.dpsi_dq = [NaN, NaN];
result.orientationSensitivity = NaN;
result.classification = "UNREACHABLE_CLOSURE";
end

function fields = scalarFields(thetaValue, phiValue, typeIIValue)
fields.typeITheta = thetaValue;
fields.typeIPhi = phiValue;
fields.typeII = typeIIValue;
end

function flags = exactFlags(typeITheta, typeIPhi, typeI, typeII)
flags.typeITheta = logical(typeITheta);
flags.typeIPhi = logical(typeIPhi);
flags.typeI = logical(typeI);
flags.typeII = logical(typeII);
flags.typeIII = logical(typeI && typeII);
flags.any = logical(typeI || typeII);
end

function flags = nearFlags( ...
        exact, typeITheta, typeIPhi, typeI, typeII, taskCondition)
flags.typeITheta = logical(typeITheta || exact.typeITheta);
flags.typeIPhi = logical(typeIPhi || exact.typeIPhi);
flags.typeI = logical(typeI || exact.typeI);
flags.typeII = logical(typeII || exact.typeII);
flags.typeIII = logical(flags.typeI && flags.typeII);
flags.taskCondition = logical(taskCondition);
flags.any = logical(flags.typeI || flags.typeII || taskCondition);
end

function classification = classify(exact, near)
if exact.typeIII
    classification = "TYPE_III";
elseif exact.typeII
    classification = "TYPE_II";
elseif exact.typeI
    classification = "TYPE_I";
elseif near.typeI && near.typeII
    classification = "NEAR_MULTIPLE";
elseif near.typeII
    classification = "NEAR_TYPE_II";
elseif near.typeI
    classification = "NEAR_TYPE_I";
elseif near.taskCondition
    classification = "NEAR_TASK";
else
    classification = "REGULAR";
end
end

function derivative = orientationDerivative( ...
        constraint, u, v, eTheta, cPhi)
dE = [eTheta, zeros(2, 1)];
dC = [zeros(2, 1), cPhi];
dD = zeros(2, 2);
for column = 1:2
    rightHandSide = [dot(u, dC(:, column)); ...
        dot(v, dE(:, column))];
    dD(:, column) = constraint \ rightHandSide;
end
dLink = dD - dE;
derivative = zeros(1, 2);
for column = 1:2
    derivative(column) = cross2(v, dLink(:, column)) / dot(v, v);
end
end

function value = safeConditionNumber(singularValues)
if numel(singularValues) ~= 2 || ...
        any(~isfinite(singularValues)) || singularValues(2) <= eps
    value = Inf;
else
    value = singularValues(1) / singularValues(2);
end
end

function value = cross2(first, second)
value = first(1) * second(2) - first(2) * second(1);
end
```

- [ ] **Step 5: Implement the minimal ideal public wrapper**

Create `src/+duallink5/+singularity/evaluate.m` with ideal input support. The
diagnostic and complete-assembly sections are added in Task 2.

```matlab
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
```

- [ ] **Step 6: Run the metric test and verify GREEN**

Run the command from Step 2.

Expected: 8 tests pass with the MATLAB-verified fixed configurations and the
production default `exactThreshold = 1e-8`.

- [ ] **Step 7: Commit Task 1 only**

```powershell
git add -- tests/singularity/TestSingularityMetrics.m src/+duallink5/+singularity/evaluate.m src/+duallink5/+singularity/private/normalizeOptions.m src/+duallink5/+singularity/private/evaluateSide.m
git commit --only -m "feat: classify five-bar singularities" -- tests/singularity/TestSingularityMetrics.m src/+duallink5/+singularity/evaluate.m src/+duallink5/+singularity/private/normalizeOptions.m src/+duallink5/+singularity/private/evaluateSide.m
```

### Task 2: Complete-assembly and lower/upper diagnostic evaluation

**Files:**

- Modify: `tests/singularity/TestSingularityMetrics.m`
- Modify: `src/+duallink5/+singularity/evaluate.m`
- Create: `src/+duallink5/+singularity/private/assessIdealAssembly.m`
- Create: `src/+duallink5/+singularity/private/prepareTopologyReference.m`

- [ ] **Step 1: Add failing diagnostic and mechanical-state tests**

Append these methods inside `methods (Test)` in
`tests/singularity/TestSingularityMetrics.m`:

```matlab
function diagnosticInputKeepsBothSideClassifications(testCase)
    q.lower = deg2rad([85, 30]);
    q.upper = deg2rad([45.206365032903925, 79]);
    result = duallink5.singularity.evaluate( ...
        q, testCase.Geometry, testCase.RegularOptions);

    testCase.verifyEqual(result.mode, "diagnostic");
    testCase.verifyEmpty(result.logical);
    testCase.verifyEqual(result.lower.classification, "REGULAR");
    testCase.verifyEqual(result.upper.classification, "TYPE_I");
    testCase.verifyTrue(result.nearSingular);
    testCase.verifyEqual( ...
        result.assemblyStatusCode, "SHARED_LINK_MISMATCH");
end

function topologyAndJointLimitRemainSeparateFromSingularity(testCase)
    topologyResult = duallink5.singularity.evaluate( ...
        deg2rad([13.5, 84]), testCase.Geometry, struct());
    limitResult = duallink5.singularity.evaluate( ...
        [-0.1, 0.5], testCase.Geometry, struct());

    testCase.verifyFalse(topologyResult.assemblyValid);
    testCase.verifyEqual(topologyResult.assemblyStatusCode, ...
        "ASSEMBLY_TOPOLOGY_MISMATCH");
    testCase.verifyFalse(limitResult.assemblyValid);
    testCase.verifyEqual(limitResult.assemblyStatusCode, ...
        "JOINT_LIMIT_VIOLATION");
end

function rejectsMalformedJointInputsAndOptions(testCase)
    malformed = {[1; 2], [1, NaN], true, ...
        struct('lower', [1, 2]), ...
        struct('lower', [1, 2], 'upper', [1; 2])};
    for index = 1:numel(malformed)
        testCase.verifyError( ...
            @()duallink5.singularity.evaluate( ...
            malformed{index}, testCase.Geometry, struct()), ...
            'duallink5:singularity:InvalidJointInput');
    end

    badOptions = {struct('branchId', 0), ...
        struct('exactThreshold', 0.1, ...
        'nearMetricThreshold', 0.05), ...
        struct('nearConditionThreshold', 1), ...
        struct('topologyProfile', "other"), ...
        struct('unsupported', true)};
    for index = 1:numel(badOptions)
        testCase.verifyError( ...
            @()duallink5.singularity.evaluate( ...
            deg2rad([85, 30]), testCase.Geometry, ...
            badOptions{index}), ...
            'duallink5:singularity:InvalidOptions');
    end
end
```

- [ ] **Step 2: Run the focused test and verify RED**

Run the Task 1 test command.

Expected: diagnostic input fails with `InvalidJointInput`, and ideal results
still report `NOT_EVALUATED` for assembly status.

- [ ] **Step 3: Add cached topology-reference construction**

Create `src/+duallink5/+singularity/private/prepareTopologyReference.m`:

```matlab
function reference = prepareTopologyReference(geometry, options)
reference = [];
if options.topologyProfile == "none"
    return
end
referenceQ = geometry.analysis.referenceQ;
input = struct('lower', referenceQ, 'upper', referenceQ);
assemblyOptions = struct( ...
    'mode', "ideal", ...
    'branchMode', "fixed", ...
    'branchId', options.branchId, ...
    'collisionProfile', options.collisionProfile);
assembly = duallink5.kinematics.forwardAssembly( ...
    input, geometry, assemblyOptions);
if ~assembly.quality.valid
    error('duallink5:singularity:InvalidOptions', ...
        'The configured reference assembly is invalid for these options.');
end
reference = duallink5.validation.prepareAssemblyTopologyReference( ...
    assembly, geometry.tolerance.length);
end
```

- [ ] **Step 4: Add complete ideal-assembly assessment**

Create `src/+duallink5/+singularity/private/assessIdealAssembly.m`:

```matlab
function assessment = assessIdealAssembly( ...
        q, geometry, options, topologyReference)
assessment.valid = false;
assessment.statusCode = "UNINITIALIZED";
ranges = [geometry.analysis.thetaRange; geometry.analysis.phiRange];
if any(q < ranges(:, 1).' | q > ranges(:, 2).')
    assessment.statusCode = "JOINT_LIMIT_VIOLATION";
    return
end

input = struct('lower', q, 'upper', q);
assemblyOptions = struct( ...
    'mode', "ideal", ...
    'branchMode', "fixed", ...
    'branchId', options.branchId, ...
    'collisionProfile', options.collisionProfile);
assembly = duallink5.kinematics.forwardAssembly( ...
    input, geometry, assemblyOptions);
assessment.statusCode = assembly.quality.statusCode;
if ~assembly.quality.valid
    return
end

if options.topologyProfile == "reference"
    comparison = ...
        duallink5.validation.compareAssemblyTopologyToReference( ...
        assembly, topologyReference);
    if ~comparison.compatible
        assessment.statusCode = comparison.statusCode;
        return
    end
end
assessment.valid = true;
assessment.statusCode = "OK";
end
```

- [ ] **Step 5: Replace `evaluate.m` with ideal and diagnostic behavior**

Replace the Task 1 wrapper body with:

```matlab
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
```

- [ ] **Step 6: Run focused metric tests and verify GREEN**

Run the Task 1 test command.

Expected: 11 tests pass.

- [ ] **Step 7: Commit Task 2 only**

```powershell
git add -- tests/singularity/TestSingularityMetrics.m src/+duallink5/+singularity/evaluate.m src/+duallink5/+singularity/private/assessIdealAssembly.m src/+duallink5/+singularity/private/prepareTopologyReference.m
git commit --only -m "feat: diagnose doublet singularity sides" -- tests/singularity/TestSingularityMetrics.m src/+duallink5/+singularity/evaluate.m src/+duallink5/+singularity/private/assessIdealAssembly.m src/+duallink5/+singularity/private/prepareTopologyReference.m
```

### Task 3: Grid sampling, exact curves, and actual usable masks

**Files:**

- Create: `tests/singularity/TestSingularitySpace.m`
- Create: `src/+duallink5/+singularity/sampleSpace.m`
- Create: `src/+duallink5/+singularity/private/extractZeroContours.m`
- Create: `src/+duallink5/+singularity/private/refineTypeIICurves.m`
- Create: `src/+duallink5/+singularity/private/findTypeIIILoci.m`
- Create: `src/+duallink5/+singularity/private/mapSingularityCurves.m`

- [ ] **Step 1: Write failing grid-contract tests**

Create `tests/singularity/TestSingularitySpace.m`:

```matlab
classdef TestSingularitySpace < matlab.unittest.TestCase
    properties
        Geometry
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            testCase.Geometry = duallink5.model.defaultGeometry();
        end
    end

    methods (Test)
        function sampleFieldsHaveDocumentedShapes(testCase)
            grid.theta = deg2rad([84, 85, 86]);
            grid.phi = deg2rad([29, 30, 31, 32]);
            samples = duallink5.singularity.sampleSpace( ...
                grid, testCase.Geometry, struct());

            testCase.verifySize(samples.thetaGrid, [3, 4]);
            testCase.verifySize(samples.x, [3, 4]);
            testCase.verifySize(samples.constraintMatrix, [2, 2, 3, 4]);
            testCase.verifySize(samples.taskActuationMatrix, [2, 2, 3, 4]);
            testCase.verifySize(samples.taskJacobian, [2, 2, 3, 4]);
            testCase.verifySize( ...
                samples.singularValues.constraint, [2, 3, 4]);
            testCase.verifySize(samples.singularValues.task, [2, 3, 4]);
            testCase.verifyTrue(all( ...
                samples.safeUsableMask <= ...
                samples.mechanicallyValidMask, 'all'));
            testCase.verifyTrue(all( ...
                samples.mechanicallyValidMask <= ...
                samples.theoreticalReachableMask, 'all'));
        end

        function singlePointGridPreservesMetadataAndHasEmptyCurves(testCase)
            grid.theta = deg2rad(85);
            grid.phi = deg2rad(30);
            options = struct('branchId', int8(-1), ...
                'exactThreshold', 2e-8, ...
                'nearMetricThreshold', 0.08, ...
                'nearConditionThreshold', 250, ...
                'collisionProfile', "none", ...
                'topologyProfile', "none");
            samples = duallink5.singularity.sampleSpace( ...
                grid, testCase.Geometry, options);

            testCase.verifySize(samples.thetaGrid, [1, 1]);
            curveNames = {'typeITheta', 'typeIPhi', ...
                'typeIIOuter', 'typeIIInner', 'typeIII'};
            for index = 1:numel(curveNames)
                testCase.verifyEmpty(samples.curves.(curveNames{index}));
            end
            testCase.verifyEqual(samples.metadata.units, ...
                testCase.Geometry.units);
            testCase.verifyEqual(samples.metadata.branchId, int8(-1));
            testCase.verifyEqual(samples.metadata.exactThreshold, 2e-8);
            testCase.verifyEqual(samples.metadata.nearMetricThreshold, 0.08);
            testCase.verifyEqual( ...
                samples.metadata.nearConditionThreshold, 250);
            testCase.verifyEqual(samples.metadata.collisionProfile, "none");
            testCase.verifyEqual(samples.metadata.topologyProfile, "none");
        end

        function exactTangencyKeepsFiniteTaskPoint(testCase)
            grid.theta = [0.83109759906031577, deg2rad(85)];
            grid.phi = [deg2rad(30), deg2rad(56.5)];
            samples = duallink5.singularity.sampleSpace( ...
                grid, testCase.Geometry, ...
                struct('collisionProfile', "none", ...
                'topologyProfile', "none"));
            row = 1;
            column = 2;

            testCase.verifyTrue( ...
                samples.theoreticalReachableMask(row, column));
            testCase.verifyTrue(samples.exact.typeII(row, column));
            testCase.verifyTrue(isfinite(samples.x(row, column)));
            testCase.verifyTrue(isfinite(samples.y(row, column)));
            testCase.verifyTrue(all(isnan( ...
                samples.taskJacobian(:, :, row, column)), 'all'));
            testCase.verifyFalse( ...
                samples.mechanicallyValidMask(row, column));
        end

        function topologyProfileChangesOnlyTopologyRejection(testCase)
            grid.theta = deg2rad([13.5, 14.5]);
            grid.phi = deg2rad([83, 84]);
            reference = duallink5.singularity.sampleSpace( ...
                grid, testCase.Geometry, ...
                struct('topologyProfile', "reference"));
            unfiltered = duallink5.singularity.sampleSpace( ...
                grid, testCase.Geometry, ...
                struct('topologyProfile', "none"));

            target = reference.thetaGrid == deg2rad(13.5) & ...
                reference.phiGrid == deg2rad(84);
            testCase.verifyEqual(reference.reasonMap(target), ...
                "ASSEMBLY_TOPOLOGY_MISMATCH");
            testCase.verifyFalse(reference.mechanicallyValidMask(target));
            testCase.verifyTrue(unfiltered.mechanicallyValidMask(target));
            testCase.verifyEqual( ...
                reference.theoreticalReachableMask, ...
                unfiltered.theoreticalReachableMask);
        end

        function collisionProfileChangesOnlyCollisionRejection(testCase)
            grid.theta = deg2rad([179, 180]);
            grid.phi = deg2rad([68.25, 69]);
            collision = duallink5.singularity.sampleSpace( ...
                grid, testCase.Geometry, ...
                struct('collisionProfile', "centerline", ...
                'topologyProfile', "none"));
            unfiltered = duallink5.singularity.sampleSpace( ...
                grid, testCase.Geometry, ...
                struct('collisionProfile', "none", ...
                'topologyProfile', "none"));

            target = collision.thetaGrid == deg2rad(180) & ...
                collision.phiGrid == deg2rad(68.25);
            testCase.verifyTrue(collision.theoreticalReachableMask(target));
            testCase.verifyEqual(collision.reasonMap(target), ...
                "PARALLEL_SHARED_COLLISION");
            testCase.verifyFalse(collision.mechanicallyValidMask(target));
            testCase.verifyTrue(unfiltered.mechanicallyValidMask(target));
            testCase.verifyEqual(collision.theoreticalReachableMask, ...
                unfiltered.theoreticalReachableMask);
        end

        function exactTypeIIIGridPointCreatesMappedLocus(testCase)
            geometry = testCase.Geometry;
            geometry.links.link3 = 30e-3;
            geometry.links.link4 = 32e-3;
            geometry = duallink5.model.validateGeometry(geometry);
            grid.theta = 0;
            grid.phi = 0;
            samples = duallink5.singularity.sampleSpace( ...
                grid, geometry, struct('collisionProfile', "none", ...
                'topologyProfile', "none"));

            testCase.verifyTrue(samples.exact.typeIII);
            testCase.verifyNumElements(samples.curves.typeIII, 1);
            testCase.verifyEqual(samples.curves.typeIII.theta, 0);
            testCase.verifyEqual(samples.curves.typeIII.phi, 0);
            testCase.verifyTrue(all(isfinite( ...
                samples.curves.typeIII.pointG), 'all'));
        end

        function nonGridTypeIIIIsRootRefinedFromTypeIICurve(testCase)
            grid.theta = deg2rad(47:1:50);
            grid.phi = deg2rad(55:1:58);
            options = struct('collisionProfile', "none", ...
                'topologyProfile', "none");
            samples = duallink5.singularity.sampleSpace( ...
                grid, testCase.Geometry, options);

            testCase.verifyFalse(any(samples.exact.typeIII, 'all'));
            testCase.verifyNotEmpty(samples.curves.typeIII);
            expected = [0.846282132507182; 0.985169611228796];
            locations = [[samples.curves.typeIII.theta]; ...
                [samples.curves.typeIII.phi]];
            [distance, closest] = min(vecnorm(locations - expected, 2, 1));
            testCase.verifyLessThan(distance, 1e-7);
            q = locations(:, closest).';
            result = duallink5.singularity.evaluate( ...
                q, testCase.Geometry, options);
            testCase.verifyTrue(result.logical.exact.typeIII);
            testCase.verifyEqual(result.logical.closureStatusCode, ...
                "NEAR_SINGULAR");
            tolerance = 32 * eps(max([ ...
                testCase.Geometry.links.link3, ...
                testCase.Geometry.links.link4]));
            testCase.verifyLessThanOrEqual( ...
                abs(result.logical.closureDistance - ...
                abs(testCase.Geometry.links.link4 - ...
                testCase.Geometry.links.link3)), tolerance);
        end

        function samplerExtractsBothTypeISubtypeCurves(testCase)
            grids = { ...
                struct('theta', deg2rad(44:0.25:46.5), ...
                'phi', deg2rad(78:0.25:80)), ...
                struct('theta', deg2rad(56:0.25:58), ...
                'phi', deg2rad(0:0.25:2))};
            names = {'typeITheta', 'typeIPhi'};
            options = struct('collisionProfile', "none", ...
                'topologyProfile', "none");
            for caseIndex = 1:2
                samples = duallink5.singularity.sampleSpace( ...
                    grids{caseIndex}, testCase.Geometry, options);
                curves = samples.curves.(names{caseIndex});
                testCase.verifyNotEmpty(curves);
                for curveIndex = 1:numel(curves)
                    curve = curves(curveIndex);
                    testCase.verifyTrue( ...
                        islogical(curve.adjacentMechanical));
                    testCase.verifySize(curve.pointG, ...
                        [2, numel(curve.theta)]);
                    testCase.verifySize(curve.adjacentMechanical, ...
                        [1, numel(curve.theta)]);
                    for pointIndex = 1:numel(curve.theta)
                        result = duallink5.singularity.evaluate( ...
                            [curve.theta(pointIndex), ...
                            curve.phi(pointIndex)], ...
                            testCase.Geometry, options);
                        testCase.verifyLessThan(abs( ...
                            result.logical.signedIndicator.( ...
                            names{caseIndex})), 1e-4);
                    end
                end
            end
        end

        function denseGridExtractsOuterTypeIICurve(testCase)
            grid.theta = deg2rad(170:0.5:180);
            grid.phi = deg2rad(60:0.5:80);
            samples = duallink5.singularity.sampleSpace( ...
                grid, testCase.Geometry, ...
                struct('collisionProfile', "none", ...
                'topologyProfile', "none"));

            pointCount = sum(arrayfun( ...
                @(curve)numel(curve.theta), ...
                samples.curves.typeIIOuter));
            testCase.verifyGreaterThan(pointCount, 2);
            finiteCurves = arrayfun(@(curve) ...
                all(isfinite(curve.pointG), 'all'), ...
                samples.curves.typeIIOuter);
            testCase.verifyTrue(all(finiteCurves));
            theta = [samples.curves.typeIIOuter.theta];
            phi = [samples.curves.typeIIOuter.phi];
            L = testCase.Geometry.links;
            deltaX = L.link1 * cos(theta) - L.link5 + ...
                L.link2 * cos(phi);
            deltaY = L.link1 * sin(theta) - L.link2 * sin(phi);
            outerTarget = L.link3 + L.link4;
            residual = hypot(deltaX, deltaY) - outerTarget;
            tolerance = 32 * eps(max( ...
                [outerTarget, L.link3, L.link4]));
            testCase.verifyLessThanOrEqual( ...
                max(abs(residual)), tolerance);
        end

        function rejectsMalformedGrid(testCase)
            malformed = { ...
                struct('theta', [0, 1]), ...
                struct('theta', [0, 1], 'phi', [0, NaN]), ...
                struct('theta', [1, 0], 'phi', [0, 1]), ...
                struct('theta', [0, 0], 'phi', [0, 1])};
            for index = 1:numel(malformed)
                testCase.verifyError( ...
                    @()duallink5.singularity.sampleSpace( ...
                    malformed{index}, testCase.Geometry, struct()), ...
                    'duallink5:singularity:InvalidGrid');
            end
        end
    end
end
```

- [ ] **Step 2: Run the sampler test and verify RED**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); run('startup.m'); results=runtests('tests/singularity/TestSingularitySpace.m'); disp(results); assert(all([results.Passed]));"
```

Expected: FAIL because `duallink5.singularity.sampleSpace` does not exist.

- [ ] **Step 3: Add a deterministic `contourc` parser**

Create `src/+duallink5/+singularity/private/extractZeroContours.m`:

```matlab
function curves = extractZeroContours(theta, phi, values)
curves = repmat(struct('theta', [], 'phi', []), 0, 1);
if numel(theta) < 2 || numel(phi) < 2 || ...
        ~any(isfinite(values), 'all')
    return
end
matrix = contourc(theta, phi, values.', [0, 0]);
column = 1;
while column <= size(matrix, 2)
    pointCount = matrix(2, column);
    first = column + 1;
    last = column + pointCount;
    if pointCount >= 2 && last <= size(matrix, 2)
        curve.theta = matrix(1, first:last);
        curve.phi = matrix(2, first:last);
        curves(end + 1, 1) = curve; %#ok<AGROW>
    end
    column = last + 1;
end
end
```

In the same slice, create
`src/+duallink5/+singularity/private/refineTypeIICurves.m`. `contourc`
interpolates between grid nodes, so every returned point must be projected
onto the analytic closure-distance equation before it is mapped to G:

```matlab
function refined = refineTypeIICurves( ...
        curves, targetDistance, geometry, bounds)
template = struct('theta', [], 'phi', []);
refined = repmat(template, 0, 1);
if targetDistance <= geometry.tolerance.length
    return
end
tolerance = 32 * eps(max([targetDistance, ...
    geometry.links.link3, geometry.links.link4]));
for curveIndex = 1:numel(curves)
    source = [curves(curveIndex).theta; curves(curveIndex).phi];
    projected = nan(size(source));
    keep = false(1, size(source, 2));
    for pointIndex = 1:size(source, 2)
        q = source(:, pointIndex);
        iteration = 0;
        while iteration < 15
            iteration = iteration + 1;
            [residual, gradient] = closureResidual( ...
                q, targetDistance, geometry);
            if abs(residual) <= tolerance
                keep(pointIndex) = true;
                break
            end
            denominator = dot(gradient, gradient);
            if ~isfinite(denominator) || denominator <= eps
                break
            end
            correction = residual * gradient.' / denominator;
            if norm(correction) > 0.25
                correction = 0.25 * correction / norm(correction);
            end
            q = min(max(q - correction, bounds(:, 1)), bounds(:, 2));
        end
        [residual, ~] = closureResidual(q, targetDistance, geometry);
        keep(pointIndex) = abs(residual) <= tolerance;
        if keep(pointIndex)
            projected(:, pointIndex) = q;
        end
    end
    projected = projected(:, keep);
    if size(projected, 2) >= 2
        refined(end + 1, 1) = struct( ... %#ok<AGROW>
            'theta', projected(1, :), 'phi', projected(2, :));
    end
end
end

function [residual, gradient] = closureResidual(q, targetDistance, geometry)
L = geometry.links;
theta = q(1);
phi = q(2);
E = L.link1 * [cos(theta); sin(theta)];
C = [L.link5 - L.link2 * cos(phi); L.link2 * sin(phi)];
delta = E - C;
distance = norm(delta);
residual = distance - targetDistance;
if distance <= eps
    gradient = [NaN, NaN];
    return
end
eTheta = L.link1 * [-sin(theta); cos(theta)];
cPhi = L.link2 * [sin(phi); cos(phi)];
gradient = [dot(delta, eTheta), -dot(delta, cPhi)] / distance;
end
```

Also create `src/+duallink5/+singularity/private/findTypeIIILoci.m`.
It detects sign changes of each Type I indicator along the already-refined
Type II curves, adds exact grid hits, and solves the simultaneous normalized
Type I/II equations with a bounded Newton iteration:

```matlab
function loci = findTypeIIILoci( ...
        typeIICurves, targetDistance, geometry, options, bounds, seedPoints)
candidatePoints = reshape(seedPoints, 2, []);
candidateTypes = strings(1, size(candidatePoints, 2));
for index = 1:size(candidatePoints, 2)
    [thetaMetric, phiMetric] = tangentIndicators( ...
        candidatePoints(:, index), targetDistance, geometry);
    if min(abs([thetaMetric, phiMetric])) <= options.exactThreshold
        if abs(thetaMetric) <= abs(phiMetric)
            candidateTypes(index) = "theta";
        else
            candidateTypes(index) = "phi";
        end
    end
end

detectionTolerance = max(options.exactThreshold, 1e-10);
for curveIndex = 1:numel(typeIICurves)
    points = [typeIICurves(curveIndex).theta; ...
        typeIICurves(curveIndex).phi];
    thetaMetric = nan(1, size(points, 2));
    phiMetric = nan(1, size(points, 2));
    for pointIndex = 1:size(points, 2)
        [thetaMetric(pointIndex), phiMetric(pointIndex)] = ...
            tangentIndicators(points(:, pointIndex), ...
            targetDistance, geometry);
    end
    metricValues = [thetaMetric; phiMetric];
    metricTypes = ["theta", "phi"];
    for metricIndex = 1:2
        values = metricValues(metricIndex, :);
        for segment = 1:(numel(values) - 1)
            firstValue = values(segment);
            secondValue = values(segment + 1);
            if abs(firstValue) <= detectionTolerance
                candidatePoints(:, end + 1) = points(:, segment); %#ok<AGROW>
                candidateTypes(end + 1) = metricTypes(metricIndex); %#ok<AGROW>
            elseif firstValue * secondValue < 0
                fraction = firstValue / (firstValue - secondValue);
                candidatePoints(:, end + 1) = ... %#ok<AGROW>
                    points(:, segment) + fraction * ...
                    (points(:, segment + 1) - points(:, segment));
                candidateTypes(end + 1) = metricTypes(metricIndex); %#ok<AGROW>
            end
            if segment == numel(values) - 1 && ...
                    abs(secondValue) <= detectionTolerance
                candidatePoints(:, end + 1) = points(:, segment + 1); %#ok<AGROW>
                candidateTypes(end + 1) = metricTypes(metricIndex); %#ok<AGROW>
            end
        end
    end
end

refinedPoints = zeros(2, 0);
for index = 1:size(candidatePoints, 2)
    if candidateTypes(index) == ""
        continue
    end
    [q, converged] = refineRoot(candidatePoints(:, index), ...
        targetDistance, candidateTypes(index), geometry, options, bounds);
    if converged && (isempty(refinedPoints) || ...
            all(vecnorm(refinedPoints - q, 2, 1) > 1e-8))
        refinedPoints(:, end + 1) = q; %#ok<AGROW>
    end
end
if isempty(refinedPoints)
    loci = repmat(struct('theta', [], 'phi', []), 0, 1);
else
    loci = struct('theta', refinedPoints(1, :), ...
        'phi', refinedPoints(2, :));
end
end

function [q, converged] = refineRoot( ...
        q, targetDistance, type, geometry, options, bounds)
converged = false;
lengthScale = max([geometry.links.link3, ...
    geometry.links.link4, targetDistance]);
closureTolerance = 32 * eps(max([targetDistance, ...
    geometry.links.link3, geometry.links.link4]));
metricTolerance = max(100 * eps, ...
    min(options.exactThreshold, 1e-10));
finiteDifferenceStep = 1e-7;
iteration = 0;
while iteration < 20
    iteration = iteration + 1;
    residual = combinedResidual( ...
        q, targetDistance, type, geometry, lengthScale);
    if abs(residual(1)) <= closureTolerance / lengthScale && ...
            abs(residual(2)) <= metricTolerance
        converged = true;
        return
    end
    jacobian = zeros(2);
    for column = 1:2
        offset = zeros(2, 1);
        offset(column) = finiteDifferenceStep;
        jacobian(:, column) = (combinedResidual( ...
            q + offset, targetDistance, type, geometry, lengthScale) - ...
            combinedResidual(q - offset, targetDistance, type, ...
            geometry, lengthScale)) / (2 * finiteDifferenceStep);
    end
    if any(~isfinite(jacobian), 'all') || rcond(jacobian) <= 1e-12
        return
    end
    direction = jacobian \ residual;
    if norm(direction) > 0.25
        direction = 0.25 * direction / norm(direction);
    end
    accepted = false;
    for backtrack = 0:8
        proposal = min(max(q - 0.5^backtrack * direction, ...
            bounds(:, 1)), bounds(:, 2));
        proposalResidual = combinedResidual( ...
            proposal, targetDistance, type, geometry, lengthScale);
        if norm(proposalResidual) < norm(residual)
            q = proposal;
            accepted = true;
            break
        end
    end
    if ~accepted
        return
    end
end
residual = combinedResidual( ...
    q, targetDistance, type, geometry, lengthScale);
converged = abs(residual(1)) <= closureTolerance / lengthScale && ...
    abs(residual(2)) <= metricTolerance;
end

function residual = combinedResidual( ...
        q, targetDistance, type, geometry, lengthScale)
L = geometry.links;
E = L.link1 * [cos(q(1)); sin(q(1))];
C = [L.link5 - L.link2 * cos(q(2)); L.link2 * sin(q(2))];
[thetaMetric, phiMetric] = tangentIndicators( ...
    q, targetDistance, geometry);
if type == "theta"
    typeIMetric = thetaMetric;
else
    typeIMetric = phiMetric;
end
residual = [(norm(E - C) - targetDistance) / lengthScale; typeIMetric];
end

function [thetaMetric, phiMetric] = ...
        tangentIndicators(q, targetDistance, geometry)
L = geometry.links;
theta = q(1);
phi = q(2);
E = L.link1 * [cos(theta); sin(theta)];
C = [L.link5 - L.link2 * cos(phi); L.link2 * sin(phi)];
delta = E - C;
distance = norm(delta);
if distance <= geometry.tolerance.length || ...
        targetDistance <= geometry.tolerance.length
    thetaMetric = NaN;
    phiMetric = NaN;
    return
end
along = (L.link3^2 - L.link4^2 + targetDistance^2) / ...
    (2 * targetDistance);
D = C + along * delta / distance;
u = D - C;
v = D - E;
eTheta = L.link1 * [-sin(theta); cos(theta)];
cPhi = L.link2 * [sin(phi); cos(phi)];
thetaMetric = dot(v, eTheta) / (L.link4 * L.link1);
phiMetric = dot(u, cPhi) / (L.link3 * L.link2);
end
```

- [ ] **Step 4: Add joint-to-G curve mapping**

Create `src/+duallink5/+singularity/private/mapSingularityCurves.m`:

```matlab
function mapped = mapSingularityCurves( ...
        curves, geometry, options, thetaGrid, phiGrid, mechanicalMask)
mapped = repmat(struct('theta', [], 'phi', [], 'pointG', [], ...
    'adjacentMechanical', []), 0, 1);
for curveIndex = 1:numel(curves)
    curve = curves(curveIndex);
    pointCount = numel(curve.theta);
    pointG = nan(2, pointCount);
    adjacent = false(1, pointCount);
    for pointIndex = 1:pointCount
        q = [curve.theta(pointIndex), curve.phi(pointIndex)];
        side = evaluateSide(q, geometry, options);
        if side.reachable
            pointG(:, pointIndex) = side.pointG;
        end
        [~, row] = min(abs(thetaGrid(:, 1) - q(1)));
        [~, column] = min(abs(phiGrid(1, :) - q(2)));
        rowRange = max(1, row - 1):min(size(mechanicalMask, 1), row + 1);
        columnRange = max(1, column - 1): ...
            min(size(mechanicalMask, 2), column + 1);
        adjacent(pointIndex) = any( ...
            mechanicalMask(rowRange, columnRange), 'all');
    end
    mapped(curveIndex).theta = curve.theta;
    mapped(curveIndex).phi = curve.phi;
    mapped(curveIndex).pointG = pointG;
    mapped(curveIndex).adjacentMechanical = adjacent;
end
end
```

`contourc` vertices are only linearly interpolated approximations. Before G is
computed, `sampleSpace` uses `refineTypeIICurves` to make every retained Type
II vertex satisfy the analytic closure-distance equation to a scale-aware
machine-precision tolerance. `findTypeIIILoci` then refines both
the Type II residual and the relevant Type I indicator; `mapTangencyContours`
only maps these already-refined joint coordinates and never hides residual by
loosening the global closure tolerance.

- [ ] **Step 5: Implement grid sampling and stored maps**

Create `src/+duallink5/+singularity/sampleSpace.m`. Use the following complete
top-level flow; the named local allocation/copy helpers must initialize every
documented field with the exact grid dimensions shown here.

```matlab
function samples = sampleSpace(grid, geometry, options)
if nargin < 3
    options = struct();
end
geometry = duallink5.model.validateGeometry(geometry);
options = normalizeOptions(options, geometry);
[theta, phi] = validateGrid(grid);
[thetaGrid, phiGrid] = ndgrid(theta, phi);
gridShape = size(thetaGrid);
samples = allocateSamples(thetaGrid, phiGrid);
topologyReference = prepareTopologyReference(geometry, options);

for linearIndex = 1:numel(thetaGrid)
    [row, column] = ind2sub(gridShape, linearIndex);
    q = [thetaGrid(row, column), phiGrid(row, column)];
    side = evaluateSide(q, geometry, options);
    samples = copySide(samples, side, row, column);
    assessment = assessIdealAssembly( ...
        q, geometry, options, topologyReference);
    samples.reasonMap(row, column) = assessment.statusCode;
    samples.mechanicallyValidMask(row, column) = assessment.valid;
    samples.safeUsableMask(row, column) = ...
        assessment.valid && ~side.near.any;
    if assessment.valid && side.near.any
        samples.reasonMap(row, column) = "NEAR_SINGULAR";
    end
end

samples.curves = buildCurves( ...
    samples, theta, phi, geometry, options);
samples.metadata.units = geometry.units;
samples.metadata.displayLengthUnit = "mm";
samples.metadata.displayAngleUnit = "deg";
samples.metadata.referenceQ = geometry.analysis.referenceQ;
samples.metadata.geometryVersion = geometry.version;
samples.metadata.branchId = options.branchId;
samples.metadata.collisionProfile = options.collisionProfile;
samples.metadata.topologyProfile = options.topologyProfile;
samples.metadata.exactThreshold = options.exactThreshold;
samples.metadata.nearMetricThreshold = options.nearMetricThreshold;
samples.metadata.nearConditionThreshold = ...
    options.nearConditionThreshold;
end

function [theta, phi] = validateGrid(grid)
if ~isstruct(grid) || ~isscalar(grid) || ...
        ~isfield(grid, 'theta') || ~isfield(grid, 'phi') || ...
        ~isIncreasingFiniteVector(grid.theta) || ...
        ~isIncreasingFiniteVector(grid.phi)
    error('duallink5:singularity:InvalidGrid', ...
        ['grid.theta and grid.phi must be nonempty, finite, real, ', ...
         'strictly increasing vectors.']);
end
theta = full(double(grid.theta(:).'));
phi = full(double(grid.phi(:).'));
end

function valid = isIncreasingFiniteVector(value)
valid = isnumeric(value) && isreal(value) && isvector(value) && ...
    ~isempty(value) && all(isfinite(value), 'all') && ...
    all(diff(double(value(:))) > 0);
end

function samples = allocateSamples(thetaGrid, phiGrid)
shape = size(thetaGrid);
samples.thetaGrid = thetaGrid;
samples.phiGrid = phiGrid;
samples.x = nan(shape);
samples.y = nan(shape);
samples.orientationSensitivity = nan(shape);
samples.closureDistance = nan(shape);
samples.branchId = zeros(shape, 'int8');
samples.theoreticalReachableMask = false(shape);
samples.mechanicallyValidMask = false(shape);
samples.safeUsableMask = false(shape);
samples.reasonMap = strings(shape);
samples.classificationMap = strings(shape);
samples.constraintMatrix = nan([2, 2, shape]);
samples.taskActuationMatrix = nan([2, 2, shape]);
samples.taskJacobian = nan([2, 2, shape]);
samples.singularValues.constraint = nan([2, shape]);
samples.singularValues.task = nan([2, shape]);
names = {'typeITheta', 'typeIPhi', 'typeII'};
for index = 1:numel(names)
    name = names{index};
    samples.signedIndicator.(name) = nan(shape);
    samples.margins.(name) = nan(shape);
end
samples.conditionNumber.constraint = inf(shape);
samples.conditionNumber.task = inf(shape);
flagNames = {'typeITheta', 'typeIPhi', 'typeI', 'typeII', ...
    'typeIII', 'any'};
for index = 1:numel(flagNames)
    name = flagNames{index};
    samples.exact.(name) = false(shape);
    samples.near.(name) = false(shape);
end
samples.near.taskCondition = false(shape);
end

function samples = copySide(samples, side, row, column)
samples.theoreticalReachableMask(row, column) = side.reachable;
samples.classificationMap(row, column) = side.classification;
samples.branchId(row, column) = side.branchId;
samples.closureDistance(row, column) = side.closureDistance;
samples.x(row, column) = side.pointG(1);
samples.y(row, column) = side.pointG(2);
samples.orientationSensitivity(row, column) = ...
    side.orientationSensitivity;
samples.constraintMatrix(:, :, row, column) = ...
    side.matrices.constraint;
samples.taskActuationMatrix(:, :, row, column) = ...
    side.matrices.taskActuation;
samples.taskJacobian(:, :, row, column) = ...
    side.matrices.taskJacobian;
samples.singularValues.constraint(:, row, column) = ...
    side.singularValues.constraint;
samples.singularValues.task(:, row, column) = ...
    side.singularValues.task;
metricNames = {'typeITheta', 'typeIPhi', 'typeII'};
for index = 1:numel(metricNames)
    name = metricNames{index};
    samples.signedIndicator.(name)(row, column) = ...
        side.signedIndicator.(name);
    samples.margins.(name)(row, column) = side.margins.(name);
end
samples.conditionNumber.constraint(row, column) = ...
    side.conditionNumber.constraint;
samples.conditionNumber.task(row, column) = ...
    side.conditionNumber.task;
flagNames = {'typeITheta', 'typeIPhi', 'typeI', 'typeII', ...
    'typeIII', 'any'};
for index = 1:numel(flagNames)
    name = flagNames{index};
    samples.exact.(name)(row, column) = side.exact.(name);
    samples.near.(name)(row, column) = side.near.(name);
end
samples.near.taskCondition(row, column) = ...
    side.near.taskCondition;
end
```

Implement `buildCurves` in the same file with these exact inputs and outputs:

```matlab
function curves = buildCurves(samples, theta, phi, geometry, options)
typeIThetaValues = samples.signedIndicator.typeITheta;
typeIPhiValues = samples.signedIndicator.typeIPhi;
typeIThetaValues(~samples.theoreticalReachableMask) = NaN;
typeIPhiValues(~samples.theoreticalReachableMask) = NaN;
typeITheta = extractZeroContours(theta, phi, typeIThetaValues);
typeIPhi = extractZeroContours(theta, phi, typeIPhiValues);

outerResidual = samples.closureDistance - ...
    (geometry.links.link3 + geometry.links.link4);
innerResidual = samples.closureDistance - ...
    abs(geometry.links.link4 - geometry.links.link3);
typeIIOuter = extractZeroContours(theta, phi, outerResidual);
typeIIInner = extractZeroContours(theta, phi, innerResidual);

outerTarget = geometry.links.link3 + geometry.links.link4;
innerTarget = abs(geometry.links.link4 - geometry.links.link3);
bounds = [theta([1, end]); phi([1, end])];
typeIIOuter = refineTypeIICurves( ...
    typeIIOuter, outerTarget, geometry, bounds);
typeIIInner = refineTypeIICurves( ...
    typeIIInner, innerTarget, geometry, bounds);

exactIndices = find(samples.exact.typeIII & ...
    isfinite(samples.closureDistance));
sampledTypeIII = [reshape(samples.thetaGrid(exactIndices), 1, []); ...
    reshape(samples.phiGrid(exactIndices), 1, [])];
sampledDistance = reshape(samples.closureDistance(exactIndices), 1, []);
outerSeed = abs(sampledDistance - outerTarget) <= ...
    abs(sampledDistance - innerTarget);
typeIIIOuter = findTypeIIILoci(typeIIOuter, outerTarget, ...
    geometry, options, bounds, sampledTypeIII(:, outerSeed));
typeIIIInner = findTypeIIILoci(typeIIInner, innerTarget, ...
    geometry, options, bounds, sampledTypeIII(:, ~outerSeed));

curves.typeITheta = mapSingularityCurves( ...
    typeITheta, geometry, options, samples.thetaGrid, ...
    samples.phiGrid, samples.mechanicallyValidMask);
curves.typeIPhi = mapSingularityCurves( ...
    typeIPhi, geometry, options, samples.thetaGrid, ...
    samples.phiGrid, samples.mechanicallyValidMask);
curves.typeIIOuter = mapTangencyContours( ...
    typeIIOuter, outerTarget, geometry, samples);
curves.typeIIInner = mapTangencyContours( ...
    typeIIInner, innerTarget, geometry, samples);
curves.typeIII = [mapTangencyContours( ...
    typeIIIOuter, outerTarget, geometry, samples); ...
    mapTangencyContours(typeIIIInner, innerTarget, geometry, samples)];
end

function mapped = mapTangencyContours( ...
        curves, targetDistance, geometry, samples)
template = struct('theta', [], 'phi', [], 'pointG', [], ...
    'adjacentMechanical', []);
if targetDistance <= geometry.tolerance.length
    mapped = repmat(template, 0, 1);
    return
end
mapped = repmat(template, numel(curves), 1);
L = geometry.links;
B = [L.link5; 0];
for curveIndex = 1:numel(curves)
    curve = curves(curveIndex);
    pointCount = numel(curve.theta);
    pointG = nan(2, pointCount);
    adjacent = false(1, pointCount);
    for pointIndex = 1:pointCount
        thetaValue = curve.theta(pointIndex);
        phiValue = curve.phi(pointIndex);
        E = L.link1 * [cos(thetaValue); sin(thetaValue)];
        C = [L.link5 - L.link2 * cos(phiValue); ...
            L.link2 * sin(phiValue)];
        delta = E - C;
        actualDistance = norm(delta);
        if actualDistance > geometry.tolerance.length
            unitCE = delta / actualDistance;
            along = (L.link3^2 - L.link4^2 + ...
                targetDistance^2) / (2 * targetDistance);
            D = C + along * unitCE;
            pointG(:, pointIndex) = D + E - B;
        end
        [~, row] = min(abs( ...
            samples.thetaGrid(:, 1) - thetaValue));
        [~, column] = min(abs( ...
            samples.phiGrid(1, :) - phiValue));
        rowRange = max(1, row - 1): ...
            min(size(samples.mechanicallyValidMask, 1), row + 1);
        columnRange = max(1, column - 1): ...
            min(size(samples.mechanicallyValidMask, 2), column + 1);
        adjacent(pointIndex) = any(samples.mechanicallyValidMask( ...
            rowRange, columnRange), 'all');
    end
    mapped(curveIndex).theta = curve.theta;
    mapped(curveIndex).phi = curve.phi;
    mapped(curveIndex).pointG = pointG;
    mapped(curveIndex).adjacentMechanical = adjacent;
end
end
```

- [ ] **Step 6: Run sampler tests and verify GREEN**

Run the command from Step 2.

Expected: 10 tests pass. Record elapsed time for the dense curve test; it must
remain below 30 seconds on the current machine so focused TDD remains usable.

- [ ] **Step 7: Run metric and sampler tests together**

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); run('startup.m'); results=[runtests('tests/singularity/TestSingularityMetrics.m'),runtests('tests/singularity/TestSingularitySpace.m')]; disp(results); assert(all([results.Passed]));"
```

Expected: all singularity tests pass.

- [ ] **Step 8: Commit Task 3 only**

```powershell
git add -- tests/singularity/TestSingularitySpace.m src/+duallink5/+singularity/sampleSpace.m src/+duallink5/+singularity/private/extractZeroContours.m src/+duallink5/+singularity/private/refineTypeIICurves.m src/+duallink5/+singularity/private/findTypeIIILoci.m src/+duallink5/+singularity/private/mapSingularityCurves.m
git commit --only -m "feat: sample singularity spaces" -- tests/singularity/TestSingularitySpace.m src/+duallink5/+singularity/sampleSpace.m src/+duallink5/+singularity/private/extractZeroContours.m src/+duallink5/+singularity/private/refineTypeIICurves.m src/+duallink5/+singularity/private/findTypeIIILoci.m src/+duallink5/+singularity/private/mapSingularityCurves.m
```

### Task 4: Joint-space singularity figure

**Files:**

- Create: `tests/viz/TestSingularityVisualization.m`
- Create: `src/+duallink5/+viz/plotJointSingularitySpace.m`

- [ ] **Step 1: Write failing joint-figure tests**

Create `tests/viz/TestSingularityVisualization.m`:

```matlab
classdef TestSingularityVisualization < matlab.unittest.TestCase
    properties
        Samples
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            geometry = duallink5.model.defaultGeometry();
            grid.theta = deg2rad(0:15:180);
            grid.phi = deg2rad(0:15:90);
            testCase.Samples = duallink5.singularity.sampleSpace( ...
                grid, geometry, struct());
        end
    end

    methods (Test)
        function jointPlotReturnsLayerHandlesAndRestoresAxes(testCase)
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];
            axesHandles(1).NextPlot = 'replacechildren';
            axesHandles(2).NextPlot = 'replace';

            handles = duallink5.viz.plotJointSingularitySpace( ...
                testCase.Samples, axesHandles);

            required = {'conditionImage', 'categoryImage', 'nearBand', ...
                'typeITheta', 'typeIPhi', 'typeIIOuter', ...
                'typeIIInner', 'reference', 'conditionColorbar', ...
                'categoryColorbar', 'legend'};
            testCase.verifyTrue(all(isfield(handles, required)));
            testCase.verifyTrue(isgraphics(handles.conditionImage));
            testCase.verifyTrue(isgraphics(handles.categoryImage));
            testCase.verifyTrue(isgraphics(handles.reference));
            testCase.verifyEqual( ...
                axesHandles(1).NextPlot, 'replacechildren');
            testCase.verifyEqual(axesHandles(2).NextPlot, 'replace');
            testCase.verifyEqual(axesHandles(1).XLabel.String, ...
                '\theta [deg]');
            testCase.verifyEqual(axesHandles(1).YLabel.String, ...
                '\phi [deg]');
            clear cleanup
        end

        function jointPlotRejectsBadAxesAndSamples(testCase)
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            oneAxes = axes(figureHandle);
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                testCase.Samples, oneAxes), ...
                'duallink5:viz:InvalidGraphicsHandle');

            delete(oneAxes);
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];
            malformed = rmfield(testCase.Samples, 'classificationMap');
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                malformed, axesHandles), ...
                'duallink5:viz:InvalidSingularityPlotInput');

            malformed = testCase.Samples;
            malformed.conditionNumber.task = strings( ...
                size(malformed.thetaGrid));
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                malformed, axesHandles), ...
                'duallink5:viz:InvalidSingularityPlotInput');

            malformed = testCase.Samples;
            malformed.mechanicallyValidMask = double( ...
                malformed.mechanicallyValidMask);
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                malformed, axesHandles), ...
                'duallink5:viz:InvalidSingularityPlotInput');

            malformed = testCase.Samples;
            malformed.metadata.referenceQ(1) = NaN;
            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                malformed, axesHandles), ...
                'duallink5:viz:InvalidSingularityPlotInput');
            clear cleanup
        end

        function jointPlotRejectsMalformedCurveContract(testCase)
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];
            malformed = testCase.Samples;
            malformed.curves.typeITheta = struct( ...
                'theta', [0, 1], 'phi', 0, ...
                'pointG', zeros(2, 2), ...
                'adjacentMechanical', true(1, 2));

            testCase.verifyError( ...
                @()duallink5.viz.plotJointSingularitySpace( ...
                malformed, axesHandles), ...
                'duallink5:viz:InvalidSingularityPlotInput');
            clear cleanup
        end

        function jointPlotSupportsSinglePointGrid(testCase)
            geometry = duallink5.model.defaultGeometry();
            grid.theta = deg2rad(85);
            grid.phi = deg2rad(30);
            samples = duallink5.singularity.sampleSpace( ...
                grid, geometry, struct('collisionProfile', "none", ...
                'topologyProfile', "none"));
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];

            duallink5.viz.plotJointSingularitySpace(samples, axesHandles);

            testCase.verifyGreaterThan(diff(axesHandles(1).XLim), 0);
            testCase.verifyGreaterThan(diff(axesHandles(1).YLim), 0);
            clear cleanup
        end

        function jointLimitViolationUsesUnreachableCategory(testCase)
            samples = testCase.Samples;
            target = find(samples.theoreticalReachableMask, 1);
            [row, column] = ind2sub(size(samples.thetaGrid), target);
            samples.reasonMap(target) = "JOINT_LIMIT_VIOLATION";
            samples.mechanicallyValidMask(target) = false;
            samples.safeUsableMask(target) = false;
            figureHandle = figure('Visible', 'off');
            cleanup = onCleanup(@()close(figureHandle));
            layout = tiledlayout(figureHandle, 1, 2);
            axesHandles = [nexttile(layout), nexttile(layout)];

            handles = duallink5.viz.plotJointSingularitySpace( ...
                samples, axesHandles);

            testCase.verifyEqual( ...
                handles.categoryImage.CData(column, row), 0);
            clear cleanup
        end
    end
end
```

- [ ] **Step 2: Run the visualization test and verify RED**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); run('startup.m'); results=runtests('tests/viz/TestSingularityVisualization.m'); disp(results); assert(all([results.Passed]));"
```

Expected: FAIL because `plotJointSingularitySpace` does not exist.

- [ ] **Step 3: Implement Figure 1 with explicit axes ownership**

Create `src/+duallink5/+viz/plotJointSingularitySpace.m` with this structure:

```matlab
function handles = plotJointSingularitySpace(samples, axesHandles)
axesHandles = validateAxes(axesHandles);
validateSamples(samples);
thetaDegrees = rad2deg(samples.thetaGrid(:, 1));
phiDegrees = rad2deg(samples.phiGrid(1, :));
originalNextPlot = string({axesHandles.NextPlot});
cleanup = onCleanup(@()restoreAxes(axesHandles, originalNextPlot));
set(axesHandles, 'NextPlot', 'add');

conditionData = log10(samples.conditionNumber.task);
conditionData(~samples.theoreticalReachableMask | ...
    ~isfinite(conditionData)) = NaN;
handles.conditionImage = imagesc(axesHandles(1), ...
    thetaDegrees, phiDegrees, conditionData.');
set(axesHandles(1), 'YDir', 'normal');
colormap(axesHandles(1), turbo(256));
handles.conditionColorbar = colorbar(axesHandles(1));
handles.conditionColorbar.Label.String = 'log_{10} \kappa(J_G)';

nearMask = samples.near.any & samples.theoreticalReachableMask;
handles.nearBand = scatter(axesHandles(1), ...
    rad2deg(samples.thetaGrid(nearMask)), ...
    rad2deg(samples.phiGrid(nearMask)), 12, ...
    [0.49, 0.18, 0.82], 'filled', ...
    'MarkerFaceAlpha', 0.22, 'MarkerEdgeAlpha', 0.22);
handles.typeITheta = plotJointCurves(axesHandles(1), ...
    samples.curves.typeITheta, [1.00, 0.58, 0.00], '-', 1.8);
handles.typeIPhi = plotJointCurves(axesHandles(1), ...
    samples.curves.typeIPhi, [1.00, 0.58, 0.00], ':', 1.8);
handles.typeIIOuter = plotJointCurves(axesHandles(1), ...
    samples.curves.typeIIOuter, [0.84, 0.16, 0.16], '--', 2.0);
handles.typeIIInner = plotJointCurves(axesHandles(1), ...
    samples.curves.typeIIInner, [0.84, 0.16, 0.16], '-.', 2.0);
referenceDegrees = rad2deg(samples.metadata.referenceQ);
handles.reference = plot(axesHandles(1), referenceDegrees(1), ...
    referenceDegrees(2), 'wo', 'MarkerEdgeColor', [0.07, 0.09, 0.12], ...
    'MarkerFaceColor', 'w', 'LineWidth', 1.4, 'MarkerSize', 7);

category = buildCategoryMap(samples);
handles.categoryImage = imagesc(axesHandles(2), ...
    thetaDegrees, phiDegrees, category.');
set(axesHandles(2), 'YDir', 'normal');
colormap(axesHandles(2), [ ...
    0.90, 0.91, 0.93; 0.73, 0.90, 0.98; ...
    0.29, 0.35, 0.43; 0.66, 0.33, 0.85; ...
    0.96, 0.68, 0.18; 0.31, 0.78, 0.47]);
clim(axesHandles(2), [-0.5, 5.5]);
handles.categoryColorbar = colorbar(axesHandles(2));
handles.categoryColorbar.Ticks = 0:5;
handles.categoryColorbar.TickLabels = { ...
    'unreachable', 'theoretical', 'collision', ...
    'topology', 'near singular', 'safe usable'};
plot(axesHandles(2), referenceDegrees(1), referenceDegrees(2), ...
    'ko', 'MarkerFaceColor', 'w', 'LineWidth', 1.2);

for index = 1:2
    xlabel(axesHandles(index), '\theta [deg]');
    ylabel(axesHandles(index), '\phi [deg]');
    xlim(axesHandles(index), paddedLimits(thetaDegrees));
    ylim(axesHandles(index), paddedLimits(phiDegrees));
    grid(axesHandles(index), 'on');
end
title(axesHandles(1), 'Conditioning and singular curves');
title(axesHandles(2), 'Configuration validity and safe domain');

legendItems = [ ...
    plot(axesHandles(1), NaN, NaN, '-', ...
        'Color', [1.00, 0.58, 0.00], 'LineWidth', 1.8), ...
    plot(axesHandles(1), NaN, NaN, '--', ...
        'Color', [0.84, 0.16, 0.16], 'LineWidth', 2.0), ...
    plot(axesHandles(1), NaN, NaN, 's', ...
        'Color', [0.49, 0.18, 0.82], ...
        'MarkerFaceColor', [0.49, 0.18, 0.82]), ...
    handles.reference];
handles.legend = legend(axesHandles(1), legendItems, ...
    {'Type I', 'Type II', 'near singular', 'reference'}, ...
    'Location', 'best');
clear cleanup
end

function category = buildCategoryMap(samples)
category = zeros(size(samples.thetaGrid));
category(samples.theoreticalReachableMask) = 1;
collision = contains(samples.reasonMap, "COLLISION") | ...
    contains(samples.reasonMap, "CLEARANCE");
category(collision) = 2;
category(samples.reasonMap == "ASSEMBLY_TOPOLOGY_MISMATCH") = 3;
category(samples.mechanicallyValidMask & samples.near.any) = 4;
category(samples.safeUsableMask) = 5;
category(samples.reasonMap == "JOINT_LIMIT_VIOLATION") = 0;
end

function limits = paddedLimits(values)
limits = [values(1), values(end)];
if limits(1) == limits(2)
    halfWidth = max(1, 0.01 * abs(limits(1)));
    limits = limits + [-halfWidth, halfWidth];
end
end

function handles = plotJointCurves(axesHandle, curves, color, style, width)
handles = gobjects(0);
for index = 1:numel(curves)
    handles(end + 1) = plot(axesHandle, ... %#ok<AGROW>
        rad2deg(curves(index).theta), ...
        rad2deg(curves(index).phi), style, ...
        'Color', color, 'LineWidth', width);
end
end
```

Append these local validators and cleanup helper to the same file:

```matlab
function axesHandles = validateAxes(axesHandles)
if ~isvector(axesHandles) || numel(axesHandles) ~= 2 || ...
        ~all(isgraphics(axesHandles, 'axes'), 'all')
    error('duallink5:viz:InvalidGraphicsHandle', ...
        'axesHandles must contain exactly two live axes.');
end
axesHandles = axesHandles(:).';
end

function validateSamples(samples)
required = {'thetaGrid', 'phiGrid', 'theoreticalReachableMask', ...
    'mechanicallyValidMask', 'safeUsableMask', 'reasonMap', ...
    'classificationMap', 'conditionNumber', 'near', 'curves', 'metadata'};
if ~isstruct(samples) || ~isscalar(samples) || ...
        ~all(isfield(samples, required))
    invalidPlotInput();
end
shape = size(samples.thetaGrid);
sameSize = isequal(size(samples.phiGrid), shape) && ...
    isequal(size(samples.theoreticalReachableMask), shape) && ...
    isequal(size(samples.mechanicallyValidMask), shape) && ...
    isequal(size(samples.safeUsableMask), shape) && ...
    isequal(size(samples.reasonMap), shape) && ...
    isequal(size(samples.classificationMap), shape);
validNested = isstruct(samples.conditionNumber) && ...
    isscalar(samples.conditionNumber) && ...
    isfield(samples.conditionNumber, 'task') && ...
    isnumeric(samples.conditionNumber.task) && ...
    isreal(samples.conditionNumber.task) && ...
    isequal(size(samples.conditionNumber.task), shape) && ...
    isstruct(samples.near) && isscalar(samples.near) && ...
    isfield(samples.near, 'any') && ...
    islogical(samples.near.any) && ...
    isequal(size(samples.near.any), shape) && ...
    isstruct(samples.curves) && isscalar(samples.curves) && ...
    all(isfield(samples.curves, ...
    {'typeITheta', 'typeIPhi', 'typeIIOuter', 'typeIIInner'})) && ...
    all(cellfun(@(name)validJointCurveCollection(samples.curves.(name)), ...
    {'typeITheta', 'typeIPhi', 'typeIIOuter', 'typeIIInner'})) && ...
    isstruct(samples.metadata) && isscalar(samples.metadata) && ...
    isfield(samples.metadata, 'referenceQ') && ...
    isnumeric(samples.metadata.referenceQ) && ...
    isreal(samples.metadata.referenceQ) && ...
    isequal(size(samples.metadata.referenceQ), [1, 2]) && ...
    all(isfinite(samples.metadata.referenceQ));
validMasks = islogical(samples.theoreticalReachableMask) && ...
    islogical(samples.mechanicallyValidMask) && ...
    islogical(samples.safeUsableMask);
if ~sameSize || ~validNested || ~validMasks || ...
        ~isnumeric(samples.thetaGrid) || ~isreal(samples.thetaGrid) || ...
        ~isnumeric(samples.phiGrid) || ~isreal(samples.phiGrid) || ...
        any(~isfinite(samples.thetaGrid), 'all') || ...
        any(~isfinite(samples.phiGrid), 'all') || ...
        ~isstring(samples.reasonMap) || ...
        ~isstring(samples.classificationMap)
    invalidPlotInput();
end
end

function valid = validJointCurveCollection(curves)
valid = isstruct(curves);
if ~valid
    return
end
for index = 1:numel(curves)
    curve = curves(index);
    valid = all(isfield(curve, {'theta', 'phi'})) && ...
        isnumeric(curve.theta) && isreal(curve.theta) && ...
        isvector(curve.theta) && all(isfinite(curve.theta), 'all') && ...
        isnumeric(curve.phi) && isreal(curve.phi) && ...
        isvector(curve.phi) && all(isfinite(curve.phi), 'all') && ...
        numel(curve.theta) == numel(curve.phi);
    if ~valid
        return
    end
end
end

function restoreAxes(axesHandles, originalNextPlot)
for index = 1:numel(axesHandles)
    if isgraphics(axesHandles(index), 'axes')
        axesHandles(index).NextPlot = char(originalNextPlot(index));
    end
end
end

function invalidPlotInput()
error('duallink5:viz:InvalidSingularityPlotInput', ...
    'samples do not satisfy the joint singularity plot contract.');
end
```

- [ ] **Step 4: Run visualization tests and verify GREEN**

Run the command from Step 2.

Expected: 5 tests pass.

- [ ] **Step 5: Commit Task 4 only**

```powershell
git add -- tests/viz/TestSingularityVisualization.m src/+duallink5/+viz/plotJointSingularitySpace.m
git commit --only -m "feat: plot joint singularity space" -- tests/viz/TestSingularityVisualization.m src/+duallink5/+viz/plotJointSingularitySpace.m
```

### Task 5: Point-G and orientation-sensitivity figure

**Files:**

- Modify: `tests/viz/TestSingularityVisualization.m`
- Create: `src/+duallink5/+viz/plotTaskSingularitySpace.m`

- [ ] **Step 1: Add failing task-space figure tests**

Append inside `methods (Test)` in
`tests/viz/TestSingularityVisualization.m`:

```matlab
function taskPlotReturnsWorkspaceCurveAndSensitivityHandles(testCase)
    figureHandle = figure('Visible', 'off');
    cleanup = onCleanup(@()close(figureHandle));
    layout = tiledlayout(figureHandle, 1, 2);
    axesHandles = [nexttile(layout), nexttile(layout)];
    axesHandles(1).NextPlot = 'replacechildren';
    axesHandles(2).NextPlot = 'replace';

    handles = duallink5.viz.plotTaskSingularitySpace( ...
        testCase.Samples, axesHandles);

    required = {'theoretical', 'safe', 'typeITheta', 'typeIPhi', ...
        'typeIIOuter', 'typeIIInner', 'typeIII', 'typeIIIStatus', ...
        'orientation', 'orientationColorbar', 'legend'};
    testCase.verifyTrue(all(isfield(handles, required)));
    testCase.verifyTrue(isgraphics(handles.theoretical));
    testCase.verifyTrue(isgraphics(handles.safe));
    testCase.verifyTrue(isgraphics(handles.orientation));
    testCase.verifyEqual(axesHandles(1).DataAspectRatio, [1, 1, 1]);
    testCase.verifyEqual(axesHandles(2).DataAspectRatio, [1, 1, 1]);
    testCase.verifyEqual(axesHandles(1).NextPlot, 'replacechildren');
    testCase.verifyEqual(axesHandles(2).NextPlot, 'replace');
    testCase.verifyEqual(axesHandles(1).XLabel.String, 'x_G [mm]');
    testCase.verifyEqual(axesHandles(1).YLabel.String, 'y_G [mm]');
    clear cleanup
end

function taskPlotRejectsMalformedCoordinates(testCase)
    figureHandle = figure('Visible', 'off');
    cleanup = onCleanup(@()close(figureHandle));
    layout = tiledlayout(figureHandle, 1, 2);
    axesHandles = [nexttile(layout), nexttile(layout)];
    malformed = testCase.Samples;
    first = find(malformed.theoreticalReachableMask, 1);
    malformed.x(first) = NaN;

    testCase.verifyError( ...
        @()duallink5.viz.plotTaskSingularitySpace( ...
        malformed, axesHandles), ...
        'duallink5:viz:InvalidSingularityPlotInput');
    clear cleanup
end

function taskPlotRejectsMalformedCurveContract(testCase)
    figureHandle = figure('Visible', 'off');
    cleanup = onCleanup(@()close(figureHandle));
    layout = tiledlayout(figureHandle, 1, 2);
    axesHandles = [nexttile(layout), nexttile(layout)];
    malformed = testCase.Samples;
    malformed.curves.typeITheta = struct( ...
        'theta', [0, 1], 'phi', [0, 1], ...
        'pointG', zeros(2, 2), ...
        'adjacentMechanical', true(1, 1));

    testCase.verifyError( ...
        @()duallink5.viz.plotTaskSingularitySpace( ...
        malformed, axesHandles), ...
        'duallink5:viz:InvalidSingularityPlotInput');
    clear cleanup
end

function taskPlotMapsExactTypeIIIPoint(testCase)
    geometry = duallink5.model.defaultGeometry();
    geometry.links.link3 = 30e-3;
    geometry.links.link4 = 32e-3;
    geometry = duallink5.model.validateGeometry(geometry);
    grid.theta = 0;
    grid.phi = 0;
    samples = duallink5.singularity.sampleSpace( ...
        grid, geometry, struct('collisionProfile', "none", ...
        'topologyProfile', "none"));
    figureHandle = figure('Visible', 'off');
    cleanup = onCleanup(@()close(figureHandle));
    layout = tiledlayout(figureHandle, 1, 2);
    axesHandles = [nexttile(layout), nexttile(layout)];

    handles = duallink5.viz.plotTaskSingularitySpace( ...
        samples, axesHandles);

    testCase.verifyNumElements(handles.typeIII.XData, 1);
    testCase.verifyEqual(handles.typeIIIStatus.String, '');
    clear cleanup
end

function taskPlotAnnotatesAbsentTypeIII(testCase)
    geometry = duallink5.model.defaultGeometry();
    grid.theta = deg2rad(85);
    grid.phi = deg2rad(30);
    samples = duallink5.singularity.sampleSpace( ...
        grid, geometry, struct('collisionProfile', "none", ...
        'topologyProfile', "none"));
    figureHandle = figure('Visible', 'off');
    cleanup = onCleanup(@()close(figureHandle));
    layout = tiledlayout(figureHandle, 1, 2);
    axesHandles = [nexttile(layout), nexttile(layout)];

    handles = duallink5.viz.plotTaskSingularitySpace( ...
        samples, axesHandles);

    testCase.verifyEmpty(handles.typeIII.XData);
    testCase.verifyEqual(handles.typeIIIStatus.String, ...
        'No Type III locus in sampled range');
    clear cleanup
end


function taskPlotDistinguishesTheoreticalAndMechanicalCurveStyles(testCase)
    geometry = duallink5.model.defaultGeometry();
    grid.theta = deg2rad(43:1:48);
    grid.phi = deg2rad(77:1:82);
    samples = duallink5.singularity.sampleSpace( ...
        grid, geometry, struct('collisionProfile', "none", ...
        'topologyProfile', "none"));
    figureHandle = figure('Visible', 'off');
    cleanup = onCleanup(@()close(figureHandle));
    layout = tiledlayout(figureHandle, 1, 2);
    axesHandles = [nexttile(layout), nexttile(layout)];

    handles = duallink5.viz.plotTaskSingularitySpace( ...
        samples, axesHandles);

    testCase.verifyNotEmpty(handles.typeITheta);
    styles = string(arrayfun(@(item)item.LineStyle, ...
        handles.typeITheta, 'UniformOutput', false));
    testCase.verifyTrue(any(styles == "--"));
    testCase.verifyTrue(any(styles == "-"));
    legendText = string(handles.legend.String);
    testCase.verifyTrue(any(legendText == "theoretical-only curve"));
    testCase.verifyTrue(any(legendText == "mechanically adjacent curve"));
    clear cleanup
end
```

- [ ] **Step 2: Run visualization tests and verify RED**

Run the Task 4 visualization command.

Expected: the six new tests fail because `plotTaskSingularitySpace` does not
exist, while the Figure 1 tests remain green.

- [ ] **Step 3: Implement Figure 2**

Create `src/+duallink5/+viz/plotTaskSingularitySpace.m`:

```matlab
function handles = plotTaskSingularitySpace(samples, axesHandles)
axesHandles = validateAxes(axesHandles);
validateSamples(samples);
scale = 1e3;
originalNextPlot = string({axesHandles.NextPlot});
cleanup = onCleanup(@()restoreAxes(axesHandles, originalNextPlot));
set(axesHandles, 'NextPlot', 'add');

theoretical = samples.theoreticalReachableMask;
safe = samples.safeUsableMask;
handles.theoretical = scatter(axesHandles(1), ...
    scale * samples.x(theoretical), scale * samples.y(theoretical), ...
    8, [0.58, 0.79, 0.96], 'filled', ...
    'MarkerFaceAlpha', 0.22, 'MarkerEdgeAlpha', 0.12);
handles.safe = scatter(axesHandles(1), ...
    scale * samples.x(safe), scale * samples.y(safe), ...
    9, [0.22, 0.72, 0.39], 'filled', ...
    'MarkerFaceAlpha', 0.32, 'MarkerEdgeAlpha', 0.18);
handles.typeITheta = plotTaskCurves(axesHandles(1), ...
    samples.curves.typeITheta, [1.00, 0.58, 0.00], '--');
handles.typeIPhi = plotTaskCurves(axesHandles(1), ...
    samples.curves.typeIPhi, [1.00, 0.58, 0.00], ':');
handles.typeIIOuter = plotTaskCurves(axesHandles(1), ...
    samples.curves.typeIIOuter, [0.84, 0.16, 0.16], '--');
handles.typeIIInner = plotTaskCurves(axesHandles(1), ...
    samples.curves.typeIIInner, [0.84, 0.16, 0.16], '-.');
typeIII = collectCurvePoints(samples.curves.typeIII);
handles.typeIII = scatter(axesHandles(1), ...
    scale * typeIII(1, :), scale * typeIII(2, :), ...
    35, [0.49, 0.18, 0.82], 'filled');
if isempty(typeIII)
    typeIIIStatus = 'No Type III locus in sampled range';
else
    typeIIIStatus = '';
end
handles.typeIIIStatus = text(axesHandles(1), 0.02, 0.98, ...
    typeIIIStatus, 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'Color', [0.49, 0.18, 0.82]);

orientationMask = samples.mechanicallyValidMask & ...
    isfinite(samples.orientationSensitivity) & ...
    samples.orientationSensitivity > 0;
orientationColor = log10(samples.orientationSensitivity(orientationMask));
handles.orientation = scatter(axesHandles(2), ...
    scale * samples.x(orientationMask), ...
    scale * samples.y(orientationMask), 12, orientationColor, 'filled');
colormap(axesHandles(2), turbo(256));
handles.orientationColorbar = colorbar(axesHandles(2));
handles.orientationColorbar.Label.String = ...
    'log_{10} ||\partial\psi/\partialq||_2';

for index = 1:2
    xlabel(axesHandles(index), 'x_G [mm]');
    ylabel(axesHandles(index), 'y_G [mm]');
    axis(axesHandles(index), 'equal');
    grid(axesHandles(index), 'on');
end
title(axesHandles(1), 'Point-G singularity loci and safe workspace');
title(axesHandles(2), 'Shared-link orientation sensitivity');

legendItems = [handles.theoretical, handles.safe, ...
    plot(axesHandles(1), NaN, NaN, 's', 'LineStyle', 'none', ...
        'Color', [1.00, 0.58, 0.00], ...
        'MarkerFaceColor', [1.00, 0.58, 0.00]), ...
    plot(axesHandles(1), NaN, NaN, 's', 'LineStyle', 'none', ...
        'Color', [0.84, 0.16, 0.16], ...
        'MarkerFaceColor', [0.84, 0.16, 0.16]), ...
    scatter(axesHandles(1), NaN, NaN, 35, ...
        [0.49, 0.18, 0.82], 'filled'), ...
    plot(axesHandles(1), NaN, NaN, '--', ...
        'Color', [0.20, 0.22, 0.26], 'LineWidth', 1.2), ...
    plot(axesHandles(1), NaN, NaN, '-', ...
        'Color', [0.20, 0.22, 0.26], 'LineWidth', 2.2)];
handles.legend = legend(axesHandles(1), legendItems, ...
    {'theoretical workspace', 'safe usable workspace', ...
    'Type I', 'Type II', 'Type III', ...
    'theoretical-only curve', 'mechanically adjacent curve'}, ...
    'Location', 'best');
clear cleanup
end

function handles = plotTaskCurves(axesHandle, curves, color, style)
handles = gobjects(0);
for index = 1:numel(curves)
    points = 1e3 * curves(index).pointG;
    valid = all(isfinite(points), 1);
    theoreticalPoints = points;
    theoreticalPoints(:, ~valid) = NaN;
    handles(end + 1) = plot(axesHandle, ... %#ok<AGROW>
        theoreticalPoints(1, :), theoreticalPoints(2, :), ...
        style, 'Color', color, 'LineWidth', 1.2);
    adjacentPoints = theoreticalPoints;
    adjacentPoints(:, ~curves(index).adjacentMechanical) = NaN;
    handles(end + 1) = plot(axesHandle, ... %#ok<AGROW>
        adjacentPoints(1, :), adjacentPoints(2, :), '-', ...
        'Color', color, 'LineWidth', 2.2);
end
end

function points = collectCurvePoints(curves)
points = zeros(2, 0);
for index = 1:numel(curves)
    finite = all(isfinite(curves(index).pointG), 1);
    points = [points, curves(index).pointG(:, finite)]; %#ok<AGROW>
end
end
```

Append these local functions to the same file:

```matlab
function axesHandles = validateAxes(axesHandles)
if ~isvector(axesHandles) || numel(axesHandles) ~= 2 || ...
        ~all(isgraphics(axesHandles, 'axes'), 'all')
    error('duallink5:viz:InvalidGraphicsHandle', ...
        'axesHandles must contain exactly two live axes.');
end
axesHandles = axesHandles(:).';
end

function validateSamples(samples)
required = {'x', 'y', 'orientationSensitivity', ...
    'theoreticalReachableMask', 'mechanicallyValidMask', ...
    'safeUsableMask', 'exact', 'curves'};
if ~isstruct(samples) || ~isscalar(samples) || ...
        ~all(isfield(samples, required))
    invalidPlotInput();
end
shape = size(samples.x);
sameSize = isequal(size(samples.y), shape) && ...
    isequal(size(samples.orientationSensitivity), shape) && ...
    isequal(size(samples.theoreticalReachableMask), shape) && ...
    isequal(size(samples.mechanicallyValidMask), shape) && ...
    isequal(size(samples.safeUsableMask), shape);
validNested = isstruct(samples.exact) && isscalar(samples.exact) && ...
    isfield(samples.exact, 'typeIII') && ...
    islogical(samples.exact.typeIII) && ...
    isequal(size(samples.exact.typeIII), shape) && ...
    isstruct(samples.curves) && isscalar(samples.curves) && ...
    all(isfield(samples.curves, ...
    {'typeITheta', 'typeIPhi', 'typeIIOuter', 'typeIIInner', 'typeIII'})) && ...
    all(cellfun(@(name)validCurveCollection(samples.curves.(name)), ...
    {'typeITheta', 'typeIPhi', 'typeIIOuter', 'typeIIInner', 'typeIII'}));
validMasks = islogical(samples.theoreticalReachableMask) && ...
    islogical(samples.mechanicallyValidMask) && ...
    islogical(samples.safeUsableMask);
theoretical = samples.theoreticalReachableMask;
if ~sameSize || ~validNested || ~validMasks || ...
        ~isnumeric(samples.x) || ~isreal(samples.x) || ...
        ~isnumeric(samples.y) || ~isreal(samples.y) || ...
        ~isnumeric(samples.orientationSensitivity) || ...
        ~isreal(samples.orientationSensitivity) || ...
        any(~isfinite(samples.x(theoretical))) || ...
        any(~isfinite(samples.y(theoretical)))
    invalidPlotInput();
end
end


function valid = validCurveCollection(curves)
valid = isstruct(curves);
if ~valid
    return
end
required = {'theta', 'phi', 'pointG', 'adjacentMechanical'};
for index = 1:numel(curves)
    curve = curves(index);
    if ~all(isfield(curve, required)) || ...
            ~isnumeric(curve.theta) || ~isreal(curve.theta) || ...
            ~isvector(curve.theta) || ...
            ~isnumeric(curve.phi) || ~isreal(curve.phi) || ...
            ~isvector(curve.phi)
        valid = false;
        return
    end
    pointCount = numel(curve.theta);
    valid = numel(curve.phi) == pointCount && ...
        all(isfinite(curve.theta), 'all') && ...
        all(isfinite(curve.phi), 'all') && ...
        isnumeric(curve.pointG) && isreal(curve.pointG) && ...
        isequal(size(curve.pointG), [2, pointCount]) && ...
        ~any(isinf(curve.pointG), 'all') && ...
        islogical(curve.adjacentMechanical) && ...
        isequal(size(curve.adjacentMechanical), [1, pointCount]);
    if ~valid
        return
    end
end
end

function restoreAxes(axesHandles, originalNextPlot)
for index = 1:numel(axesHandles)
    if isgraphics(axesHandles(index), 'axes')
        axesHandles(index).NextPlot = char(originalNextPlot(index));
    end
end
end

function invalidPlotInput()
error('duallink5:viz:InvalidSingularityPlotInput', ...
    'samples do not satisfy the task singularity plot contract.');
end
```

- [ ] **Step 4: Run all visualization tests and verify GREEN**

Run the Task 4 visualization command.

Expected: 11 tests pass.

- [ ] **Step 5: Commit Task 5 only**

```powershell
git add -- tests/viz/TestSingularityVisualization.m src/+duallink5/+viz/plotTaskSingularitySpace.m
git commit --only -m "feat: plot task singularity space" -- tests/viz/TestSingularityVisualization.m src/+duallink5/+viz/plotTaskSingularitySpace.m
```

### Task 6: Read-only facade methods and runnable analysis script

**Files:**

- Modify: `tests/facade/TestDL5.m`
- Modify: `src/+duallink5/DL5.m`
- Create: `scripts/analyze_singularity_space.m`

- [ ] **Step 1: Add failing facade state-safety tests**

Append inside `methods (Test)` in `tests/facade/TestDL5.m`:

```matlab
function singularityAnalysisDoesNotMutateCommittedState(testCase)
    robot = duallink5.DL5();
    previousQ = robot.Q;
    previousAssembly = robot.Assembly;
    previousPointG = robot.PointG;
    previousValid = robot.IsValid;
    previousStatus = robot.StatusCode;

    result = robot.singularity(struct());

    testCase.verifyEqual(result.mode, "ideal");
    testCase.verifyEqual(result.logical.classification, "REGULAR");
    testCase.verifyEqual(robot.Q, previousQ);
    testCase.verifyEqual(robot.Assembly, previousAssembly);
    testCase.verifyEqual(robot.PointG, previousPointG);
    testCase.verifyEqual(robot.IsValid, previousValid);
    testCase.verifyEqual(robot.StatusCode, previousStatus);
end

function singularitySamplingDoesNotMutateCommittedState(testCase)
    robot = duallink5.DL5();
    previousQ = robot.Q;
    previousAssembly = robot.Assembly;
    grid.theta = deg2rad([84, 85, 86]);
    grid.phi = deg2rad([29, 30, 31]);

    samples = robot.sampleSingularitySpace(grid, struct());

    testCase.verifySize(samples.thetaGrid, [3, 3]);
    testCase.verifyEqual(robot.Q, previousQ);
    testCase.verifyEqual(robot.Assembly, previousAssembly);
end
```

- [ ] **Step 2: Run facade tests and verify RED**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); run('startup.m'); results=runtests('tests/facade/TestDL5.m'); disp(results); assert(all([results.Passed]));"
```

Expected: the two new tests fail because the facade methods do not exist.

- [ ] **Step 3: Add two thin public methods to `DL5`**

Insert these methods after the existing `plot` method and before dependent
property getters in `src/+duallink5/DL5.m`:

```matlab
function result = singularity(robot, options)
    if nargin < 2
        options = struct();
    end
    result = duallink5.singularity.evaluate( ...
        robot.Q, robot.Geometry, options);
end

function samples = sampleSingularitySpace(robot, grid, options)
    if nargin < 3
        options = struct();
    end
    samples = duallink5.singularity.sampleSpace( ...
        grid, robot.Geometry, options);
end
```

- [ ] **Step 4: Run facade tests and verify GREEN**

Run the command from Step 2.

Expected: all facade tests pass.

- [ ] **Step 5: Create the direct analysis script**

Create `scripts/analyze_singularity_space.m`:

```matlab
analyzeSingularitySpaceExample();

function analyzeSingularitySpaceExample()
originalPath = path;
pathCleanup = onCleanup(@()path(originalPath));
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptDir);
run(fullfile(projectRoot, 'startup.m'));

geometry = duallink5.model.defaultGeometry();
grid.theta = deg2rad(0:1:180);
grid.phi = deg2rad(0:1:90);
options = struct( ...
    'exactThreshold', 1e-8, ...
    'nearMetricThreshold', 0.05, ...
    'nearConditionThreshold', 100, ...
    'collisionProfile', "centerline", ...
    'topologyProfile', "reference");

timer = tic;
samples = duallink5.singularity.sampleSpace( ...
    grid, geometry, options);
elapsed = toc(timer);

jointFigure = figure('Name', 'DualLink5 joint singularity space');
jointLayout = tiledlayout(jointFigure, 1, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
jointAxes = [nexttile(jointLayout), nexttile(jointLayout)];
duallink5.viz.plotJointSingularitySpace(samples, jointAxes);

taskFigure = figure('Name', 'DualLink5 task singularity space');
taskLayout = tiledlayout(taskFigure, 1, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
taskAxes = [nexttile(taskLayout), nexttile(taskLayout)];
duallink5.viz.plotTaskSingularitySpace(samples, taskAxes);

theoreticalCount = nnz(samples.theoreticalReachableMask);
mechanicalCount = nnz(samples.mechanicallyValidMask);
safeCount = nnz(samples.safeUsableMask);
if mechanicalCount == 0
    safeFraction = NaN;
else
    safeFraction = safeCount / mechanicalCount;
end
summary = struct( ...
    'theoreticalCount', theoreticalCount, ...
    'mechanicalCount', mechanicalCount, ...
    'safeCount', safeCount, ...
    'nearTypeICount', nnz(samples.near.typeI), ...
    'nearTypeIICount', nnz(samples.near.typeII));
jointFigure.UserData = summary;
taskFigure.UserData = summary;
fprintf('DualLink5 singularity sampling: %.3f s\n', elapsed);
fprintf('Theoretical/mechanical/safe samples: %d / %d / %d\n', ...
    theoreticalCount, mechanicalCount, safeCount);
fprintf('Exact Type I/II/III: %d / %d / %d\n', ...
    nnz(samples.exact.typeI), nnz(samples.exact.typeII), ...
    nnz(samples.exact.typeIII));
fprintf('Near Type I/II/multiple: %d / %d / %d\n', ...
    nnz(samples.near.typeI), nnz(samples.near.typeII), ...
    nnz(samples.near.typeIII));
fprintf('Safe fraction of mechanically valid samples: %.3f\n', ...
    safeFraction);
clear pathCleanup
end
```

- [ ] **Step 6: Smoke-test the script without visible windows**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); set(groot,'defaultFigureVisible','off'); run('scripts/analyze_singularity_space.m'); figures=findall(groot,'Type','figure'); assert(numel(figures)==2); assert(all(arrayfun(@(f)numel(findall(f,'Type','axes'))>=2,figures))); summary=figures(1).UserData; assert(summary.theoreticalCount>0&&summary.mechanicalCount>0&&summary.safeCount>0&&summary.nearTypeICount>0&&summary.nearTypeIICount>0); axesHandles=[findall(figures(1),'Type','axes');findall(figures(2),'Type','axes')]; assert(all(arrayfun(@(a)all(isfinite(a.CLim))&&diff(a.CLim)>0,axesHandles))); close(figures);"
```

Expected: exit code 0, two figures created, and the printed counts are
nonzero for theoretical, mechanical, safe, Type I-near, and Type II-near
samples.

- [ ] **Step 7: Commit Task 6 only**

```powershell
git add -- tests/facade/TestDL5.m src/+duallink5/DL5.m scripts/analyze_singularity_space.m
git commit --only -m "feat: expose singularity analysis workflow" -- tests/facade/TestDL5.m src/+duallink5/DL5.m scripts/analyze_singularity_space.m
```

### Task 7: Regression, static analysis, and rendered-result inspection

**Files:**

- Modify only files introduced by Tasks 1-6 when a failing check demonstrates
  a defect.

- [ ] **Step 1: Run the focused regression set**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); run('startup.m'); results=[runtests('tests/singularity','IncludeSubfolders',true),runtests('tests/kinematics/TestJacobian.m'),runtests('tests/kinematics/TestClosure.m'),runtests('tests/validation/TestAssemblyTopology.m'),runtests('tests/workspace/TestWorkspace.m'),runtests('tests/viz/TestSingularityVisualization.m'),runtests('tests/facade/TestDL5.m')]; disp(results); assert(all([results.Passed]));"
```

Expected: every focused test passes with no warning or error output.

- [ ] **Step 2: Run the complete test suite**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); run('startup.m'); results=runtests('tests','IncludeSubfolders',true); disp(results); assert(all([results.Passed]));"
```

Expected: all project tests pass.

- [ ] **Step 3: Run Code Analyzer on every changed production file**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); files={'src/+duallink5/+singularity/evaluate.m','src/+duallink5/+singularity/sampleSpace.m','src/+duallink5/+singularity/private/normalizeOptions.m','src/+duallink5/+singularity/private/evaluateSide.m','src/+duallink5/+singularity/private/assessIdealAssembly.m','src/+duallink5/+singularity/private/prepareTopologyReference.m','src/+duallink5/+singularity/private/extractZeroContours.m','src/+duallink5/+singularity/private/refineTypeIICurves.m','src/+duallink5/+singularity/private/findTypeIIILoci.m','src/+duallink5/+singularity/private/mapSingularityCurves.m','src/+duallink5/+viz/plotJointSingularitySpace.m','src/+duallink5/+viz/plotTaskSingularitySpace.m','src/+duallink5/DL5.m','scripts/analyze_singularity_space.m'}; hasMessages=false; for index=1:numel(files); messages=checkcode(files{index},'-id'); if ~isempty(messages); fprintf('%s\n',files{index}); disp(struct2table(messages)); hasMessages=true; end; end; assert(~hasMessages);"
```

Expected: exit code 0 and no analyzer messages.

- [ ] **Step 4: Render both figures to temporary PNG files and inspect them**

Run this exact render command:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); set(groot,'defaultFigureVisible','off'); run('scripts/analyze_singularity_space.m'); joint=findall(groot,'Type','figure','Name','DualLink5 joint singularity space'); task=findall(groot,'Type','figure','Name','DualLink5 task singularity space'); assert(isscalar(joint)&&isscalar(task)); jointPath=[tempname,'.png']; taskPath=[tempname,'.png']; exportgraphics(joint,jointPath,'Resolution',160); exportgraphics(task,taskPath,'Resolution',160); fprintf('%s\n%s\n',jointPath,taskPath); close([joint,task]);"
```

Inspect the two uniquely named PNG paths printed by MATLAB with the local
image viewer and verify:

- Figure 1 has two panels, increasing theta/phi axes, a finite condition
  colorbar, distinct Type I/II curves, reference marker, and categorical safe
  region;
- Figure 2 has two equal-aspect panels, millimetre axes, theoretical and safe
  point-G layers, singular curves, and a finite orientation-sensitivity
  colorbar; and
- labels and legends do not overlap the plotted data.

After inspection, resolve both printed paths and verify that their parent is
the system temporary directory. Delete only those two explicit PNG files.

- [ ] **Step 5: Close any verification defect with a committed rerun loop**

If Steps 1-4 expose a defect, first add or tighten the smallest focused test,
then edit only a file introduced or modified in Tasks 1-6. Rerun the focused
test, followed by Steps 1-4 in full. Before committing, use `git diff --name-only
--` with the explicit Task 1-6 production/test pathspecs to confirm every
changed path is in scope. Stage only the paths actually changed, then run
`git commit --only -m "fix: satisfy singularity verification" --` followed by
that same explicit path list. Repeat Steps 1-4 from the committed tree. Do not
use a broad add, `commit -a`, or include any pre-existing script change.

If Steps 1-4 pass without a repair, do not create an empty verification commit.

- [ ] **Step 6: Verify scope and worktree preservation**

Run:

```powershell
git status --short
git log --oneline -7
```

Expected: the pre-existing user script renames and trajectory edit remain in
their original staged/unstaged states, no temporary render is present in the
repository, and singularity work is contained in the explicit task commits.
