# DualLink5 Kinematics Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the script-coupled DualLink5 MATLAB code with a tested SI-unit package for five-bar and symmetric dual-assembly kinematics, then migrate callers and remove obsolete length-optimization assets.

**Architecture:** Implement pure functions under `src/+duallink5`, with explicit `geometry`, `q`, `pose`, and `taskSpec` structs. Build and validate the single five-bar solver first, compose the upper/lower assembly next, then layer Jacobian, inverse kinematics, workspace analysis, visualization, and experiment adapters without reverse dependencies.

**Tech Stack:** MATLAB R2025a, `matlab.unittest`, MATLAB package folders, Git, MATLAB Code Analyzer.

**Approved design:** `docs/superpowers/specs/2026-07-30-duallink5-kinematics-refactor-design.md`

---

## File map

### New source files

```text
startup.m
src/+duallink5/+model/defaultGeometry.m
src/+duallink5/+model/validateGeometry.m
src/+duallink5/+kinematics/solveClosure.m
src/+duallink5/+kinematics/computeParallelPoints.m
src/+duallink5/+kinematics/forwardFiveBar.m
src/+duallink5/+kinematics/transformFiveBarPose.m
src/+duallink5/+kinematics/forwardAssembly.m
src/+duallink5/+kinematics/taskPose.m
src/+duallink5/+kinematics/fiveBarJacobian.m
src/+duallink5/+kinematics/taskJacobian.m
src/+duallink5/+kinematics/inverseKinematics.m
src/+duallink5/+validation/assessFiveBarPose.m
src/+duallink5/+validation/isCollisionFree.m
src/+duallink5/+validation/private/segmentsIntersect.m
src/+duallink5/+validation/private/segmentDistance.m
src/+duallink5/+workspace/sampleWorkspace.m
src/+duallink5/+workspace/analyzeWorkspace.m
src/+duallink5/+workspace/largestRectangleInMask.m
src/+duallink5/+viz/plotMechanism.m
src/+duallink5/+viz/plotWorkspaceResult.m
src/+duallink5/+viz/exportFigure.m
Experiment/+duallink5exp/computeGTrajectory.m
examples/plot_fixed_pose.m
examples/plot_continuous_trajectory.m
examples/analyze_fixed_workspace.m
docs/kinematics-conventions.md
```

### New tests

```text
tests/model/TestGeometry.m
tests/kinematics/TestClosure.m
tests/kinematics/TestForwardFiveBar.m
tests/validation/TestPoseValidation.m
tests/kinematics/TestAssembly.m
tests/kinematics/TestJacobian.m
tests/kinematics/TestInverseKinematics.m
tests/workspace/TestWorkspace.m
tests/viz/TestVisualization.m
tests/experiment/TestComputeGTrajectory.m
```

### Existing callers to modify

```text
Experiment/plotModeledG.m
Experiment/exp0612.m
Experiment/exp0618.m
Experiment/exp0417.m
Experiment/exp0514.m
.gitignore
```

### Legacy files removed only after migration and full verification

The exact deletion command appears in Task 11. `Kinematics/公式.pptx`, `Kinematics/Figure.png`, `Plot/1.png`, and all `Experiment/实验数据/` files remain.

---

### Task 1: Project bootstrap and immutable geometry contract

**Files:**
- Create: `startup.m`
- Create: `src/+duallink5/+model/defaultGeometry.m`
- Create: `src/+duallink5/+model/validateGeometry.m`
- Create: `tests/model/TestGeometry.m`

- [ ] **Step 1: Write the failing geometry tests**

Create `tests/model/TestGeometry.m`:

```matlab
classdef TestGeometry < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(~)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
        end
    end

    methods (Test)
        function defaultLengthsUseSIAndApprovedMapping(testCase)
            g = duallink5.model.defaultGeometry();

            testCase.verifyEqual(g.links.link1, 80e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link2, 62e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link3, 69e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link4, 80e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link5, 80e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.units.length, "m");
            testCase.verifyEqual(g.units.angle, "rad");
        end

        function parallelDimensionsUseEndpointNames(testCase)
            g = duallink5.model.defaultGeometry();
            L = g.parallel.lengths;

            testCase.verifyEqual(L.E_Palpha1, 30e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(L.Palpha1_Palpha4, 60e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(L.D_Pbeta2, 30e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(L.Pbeta2_Pbeta3, 60e-3, 'AbsTol', 1e-15);
        end

        function invalidUnitIsRejected(testCase)
            g = duallink5.model.defaultGeometry();
            g.units.length = "mm";

            testCase.verifyError( ...
                @() duallink5.model.validateGeometry(g), ...
                'duallink5:model:InvalidUnits');
        end

        function missingDirectionFieldIsRejected(testCase)
            g=duallink5.model.defaultGeometry();
            g.parallel.direction=rmfield( ...
                g.parallel.direction,'betaSide');
            testCase.verifyError( ...
                @()duallink5.model.validateGeometry(g), ...
                'duallink5:model:InvalidGeometry');
        end
    end
end
```

- [ ] **Step 2: Run the test and verify the package is missing**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "results=runtests('tests/model/TestGeometry.m'); assert(all([results.Passed]))"
```

Expected: FAIL because `startup.m` or `duallink5.model.defaultGeometry` does not exist.

- [ ] **Step 3: Add the explicit source bootstrap**

Create `startup.m`:

```matlab
projectRoot = fileparts(mfilename('fullpath'));
sourceDir = fullfile(projectRoot, 'src');

pathEntries = strsplit(path, pathsep);
if ~any(strcmp(pathEntries, sourceDir))
    addpath(sourceDir);
end

clear projectRoot sourceDir pathEntries
```

- [ ] **Step 4: Implement the geometry factory and validator**

Create `src/+duallink5/+model/defaultGeometry.m`:

```matlab
function geometry = defaultGeometry()
geometry.version = "1.0.0";

geometry.links.link1 = 80e-3;  % AE
geometry.links.link2 = 62e-3;  % BC
geometry.links.link3 = 69e-3;  % CD
geometry.links.link4 = 80e-3;  % DE
geometry.links.link5 = 80e-3;  % AB

geometry.parallel.lengths.E_Palpha1 = 30e-3;
geometry.parallel.lengths.Palpha1_Palpha4 = 60e-3;
geometry.parallel.lengths.D_Pbeta2 = 30e-3;
geometry.parallel.lengths.Pbeta2_Pbeta3 = 60e-3;

geometry.parallel.direction.alphaAlongLink = -1;
geometry.parallel.direction.alphaSide = 1;
geometry.parallel.direction.betaAlongLink = -1;
geometry.parallel.direction.betaSide = 1;

geometry.assembly.defaultBranch = int8(-1);
geometry.tolerance.length = 1e-9;
geometry.tolerance.residual = 1e-9;
geometry.tolerance.symmetryPosition = 1e-8;
geometry.tolerance.symmetryAngle = 1e-8;
geometry.tolerance.singularityCondition = 1e10;

geometry.analysis.thetaRange = [0, pi];
geometry.analysis.phiRange = [0, pi/2];

geometry.collision.radius = struct();
geometry.collision.layerOffset = struct();
geometry.collision.clearance = 0;
geometry.collision.exemptPairs = strings(0, 2);

geometry.units.length = "m";
geometry.units.angle = "rad";
geometry.convention = "A-origin; +x=A-to-B; +y=mechanism-interior; q=[theta,phi]";

geometry = duallink5.model.validateGeometry(geometry);
end
```

Create `src/+duallink5/+model/validateGeometry.m`:

```matlab
function geometry = validateGeometry(geometry)
if ~isstruct(geometry) || ~isscalar(geometry)
    fail('geometry must be a scalar struct.');
end

requireStruct(geometry,'links');
validatePositiveFields(geometry.links, ...
    ["link1","link2","link3","link4","link5"], 'links');

requireStruct(geometry,'parallel');
requireStruct(geometry.parallel,'lengths');
requireStruct(geometry.parallel,'direction');
validatePositiveFields(geometry.parallel.lengths, ...
    ["E_Palpha1","Palpha1_Palpha4", ...
     "D_Pbeta2","Pbeta2_Pbeta3"], 'parallel.lengths');
validateSignedFields(geometry.parallel.direction, ...
    ["alphaAlongLink","alphaSide","betaAlongLink","betaSide"], ...
    'parallel.direction');

requireStruct(geometry,'assembly');
if ~isfield(geometry.assembly,'defaultBranch') || ...
        ~isscalar(geometry.assembly.defaultBranch) || ...
        ~isnumeric(geometry.assembly.defaultBranch) || ...
        ~ismember(double(geometry.assembly.defaultBranch),[-1,1])
    fail('assembly.defaultBranch must be -1 or +1.');
end

requireStruct(geometry,'tolerance');
validatePositiveFields(geometry.tolerance, ...
    ["length","residual","symmetryPosition", ...
     "symmetryAngle","singularityCondition"], 'tolerance');

requireStruct(geometry,'analysis');
validateRange(geometry.analysis,'thetaRange');
validateRange(geometry.analysis,'phiRange');

requireStruct(geometry,'collision');
if ~isfield(geometry.collision,'radius') || ...
        ~isstruct(geometry.collision.radius) || ...
        ~isfield(geometry.collision,'layerOffset') || ...
        ~isstruct(geometry.collision.layerOffset) || ...
        ~isfield(geometry.collision,'clearance') || ...
        ~(isscalar(geometry.collision.clearance) && ...
          isnumeric(geometry.collision.clearance) && ...
          isfinite(geometry.collision.clearance) && ...
          geometry.collision.clearance>=0) || ...
        ~isfield(geometry.collision,'exemptPairs') || ...
        ~(isstring(geometry.collision.exemptPairs) && ...
          size(geometry.collision.exemptPairs,2)==2)
    fail('collision fields are incomplete or invalid.');
end

requireStruct(geometry,'units');
if ~isfield(geometry.units,'length') || ~isfield(geometry.units,'angle')
    error('duallink5:model:InvalidUnits', ...
        'Kinematics geometry must use length=m and angle=rad.');
end
lengthUnit=string(geometry.units.length);
angleUnit=string(geometry.units.angle);
if ~isscalar(lengthUnit) || ~isscalar(angleUnit) || ...
        lengthUnit~="m" || angleUnit~="rad"
    error('duallink5:model:InvalidUnits', ...
        'Kinematics geometry must use length=m and angle=rad.');
end
if ~isfield(geometry,'version') || ~isfield(geometry,'convention')
    fail('version and convention are required.');
end
version=string(geometry.version);
convention=string(geometry.convention);
if ~isscalar(version) || strlength(version)==0 || ...
        ~isscalar(convention) || strlength(convention)==0
    fail('version and convention must be nonempty scalar text.');
end
end

function requireStruct(parent,name)
if ~isfield(parent,name) || ~isstruct(parent.(name)) || ...
        ~isscalar(parent.(name))
    fail('%s must be a struct.',name);
end
end

function validatePositiveFields(parent,names,prefix)
for name=names
    fieldName=char(name);
    if ~isfield(parent,fieldName)
        fail('%s.%s is required.',prefix,fieldName);
    end
    value=parent.(fieldName);
    if ~(isscalar(value) && isnumeric(value) && isfinite(value) && value>0)
        fail('%s.%s must be a positive finite scalar.',prefix,fieldName);
    end
end
end

function validateSignedFields(parent,names,prefix)
for name=names
    fieldName=char(name);
    if ~isfield(parent,fieldName) || ...
            ~isscalar(parent.(fieldName)) || ...
            ~isnumeric(parent.(fieldName)) || ...
            ~isfinite(parent.(fieldName)) || ...
            ~ismember(parent.(fieldName),[-1,1])
        fail('%s.%s must be -1 or +1.',prefix,fieldName);
    end
end
end

function validateRange(parent,name)
if ~isfield(parent,name)
    fail('analysis.%s is required.',name);
end
value=parent.(name);
if ~(isnumeric(value) && isequal(size(value),[1,2]) && ...
        all(isfinite(value)) && value(1)<value(2))
    fail('analysis.%s must be a finite increasing 1x2 vector.',name);
end
end

function fail(message,varargin)
error('duallink5:model:InvalidGeometry',message,varargin{:});
end
```

- [ ] **Step 5: Run the geometry tests**

Run the Step 2 command again.

Expected: 4 tests pass, 0 tests fail.

- [ ] **Step 6: Commit the geometry contract**

```powershell
git add startup.m src/+duallink5/+model tests/model/TestGeometry.m
git commit -m "feat: define DualLink5 geometry contract"
```

---

### Task 2: Circle-closure solver and branch identity

**Files:**
- Create: `src/+duallink5/+kinematics/solveClosure.m`
- Create: `tests/kinematics/TestClosure.m`

- [ ] **Step 1: Write the failing closure tests**

Create `tests/kinematics/TestClosure.m`:

```matlab
classdef TestClosure < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(~)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
        end
    end

    methods (Test)
        function returnsBothSignedBranches(testCase)
            C = [0; 0];
            E = [1; 0];
            [candidates, statusCode] = ...
                duallink5.kinematics.solveClosure(C, E, 1, 1, 1e-12);

            testCase.verifyEqual(statusCode, "OK");
            testCase.verifyNumElements(candidates, 2);
            testCase.verifyEqual(sort([candidates.branchId]), int8([-1, 1]));
            for index=1:numel(candidates)
                candidate=candidates(index);
                testCase.verifyEqual(norm(candidate.D - C), 1, 'AbsTol', 1e-12);
                testCase.verifyEqual(norm(candidate.D - E), 1, 'AbsTol', 1e-12);
                testCase.verifyLessThan(candidate.closureResidual, 1e-12);
            end
        end

        function unreachableCirclesReturnStatus(testCase)
            [candidates, statusCode] = ...
                duallink5.kinematics.solveClosure([0;0], [3;0], 1, 1, 1e-12);

            testCase.verifyEmpty(candidates);
            testCase.verifyEqual(statusCode, "UNREACHABLE_CLOSURE");
        end

        function tangentCirclesReportSingularity(testCase)
            [candidates, statusCode] = ...
                duallink5.kinematics.solveClosure([0;0], [2;0], 1, 1, 1e-12);

            testCase.verifyNumElements(candidates, 1);
            testCase.verifyEqual(candidates.branchId, int8(0));
            testCase.verifyEqual(statusCode, "NEAR_SINGULAR");
        end
    end
end
```

- [ ] **Step 2: Run the closure tests and verify failure**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); results=runtests('tests/kinematics/TestClosure.m'); assert(all([results.Passed]))"
```

Expected: FAIL because `solveClosure` is undefined.

- [ ] **Step 3: Implement the complete circle-intersection solver**

Create `src/+duallink5/+kinematics/solveClosure.m`:

```matlab
function [candidates, statusCode] = solveClosure(C, E, radiusCD, radiusDE, tolerance)
C = C(:);
E = E(:);

if numel(C) ~= 2 || numel(E) ~= 2 || ...
        any(~isfinite([C; E])) || ...
        ~(isscalar(radiusCD) && isfinite(radiusCD) && radiusCD>0) || ...
        ~(isscalar(radiusDE) && isfinite(radiusDE) && radiusDE>0) || ...
        ~(isscalar(tolerance) && isfinite(tolerance) && tolerance>0)
    error('duallink5:kinematics:InvalidClosureInput', ...
        ['Circle centers must be finite 2-vectors; radii and ', ...
         'length tolerance must be positive finite scalars.']);
end

delta = E - C;
distance = norm(delta);
template = struct('D', zeros(2,1), 'branchId', int8(0), ...
    'closureResidual', Inf);
candidates = repmat(template, 0, 1);

if distance < tolerance || ...
        distance > radiusCD + radiusDE + tolerance || ...
        distance < abs(radiusCD - radiusDE) - tolerance
    statusCode = "UNREACHABLE_CLOSURE";
    return
end

unitCE = delta / distance;
along = (radiusCD^2 - radiusDE^2 + distance^2) / (2 * distance);
heightSquared = radiusCD^2 - along^2;
squaredTolerance = tolerance*max([radiusCD,radiusDE,distance,tolerance]);

if heightSquared < -squaredTolerance
    statusCode = "UNREACHABLE_CLOSURE";
    return
end

height = sqrt(max(0, heightSquared));
basePoint = C + along * unitCE;
normal = [-unitCE(2); unitCE(1)];

if height <= tolerance
    candidates = template;
    candidates.D = basePoint;
    candidates.branchId = int8(0);
    candidates.closureResidual = residual(basePoint, C, E, radiusCD, radiusDE);
    statusCode = "NEAR_SINGULAR";
    return
end

points = [basePoint + height * normal, basePoint - height * normal];
candidates = repmat(template, 2, 1);
for index = 1:2
    D = points(:, index);
    branch = sign(cross2(delta, D - C));
    candidates(index).D = D;
    candidates(index).branchId = int8(branch);
    candidates(index).closureResidual = residual(D, C, E, radiusCD, radiusDE);
end
statusCode = "OK";
end

function value = residual(D, C, E, radiusCD, radiusDE)
value = max(abs([norm(D-C)-radiusCD, norm(D-E)-radiusDE]));
end

function value = cross2(a, b)
value = a(1) * b(2) - a(2) * b(1);
end
```

- [ ] **Step 4: Run the closure tests**

Run the Step 2 command again.

Expected: 3 tests pass, 0 tests fail.

- [ ] **Step 5: Commit the closure solver**

```powershell
git add src/+duallink5/+kinematics/solveClosure.m tests/kinematics/TestClosure.m
git commit -m "feat: solve five-bar closure branches"
```

---

### Task 3: Structured single-five-bar forward kinematics

**Files:**
- Create: `src/+duallink5/+kinematics/computeParallelPoints.m`
- Create: `src/+duallink5/+kinematics/forwardFiveBar.m`
- Create: `tests/kinematics/TestForwardFiveBar.m`

- [ ] **Step 1: Write the failing forward-kinematics tests**

Create `tests/kinematics/TestForwardFiveBar.m`:

```matlab
classdef TestForwardFiveBar < matlab.unittest.TestCase
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
        function matchesVerifiedLegacyPoseInSI(testCase)
            q = deg2rad([85, 52.93]);
            pose = duallink5.kinematics.forwardFiveBar(q, testCase.Geometry);

            testCase.verifyTrue(pose.quality.valid);
            testCase.verifyEqual(pose.quality.branchId, int8(-1));
            testCase.verifyEqual(pose.points.C, ...
                [42.627001951220; 49.469778823499] * 1e-3, 'AbsTol', 1e-11);
            testCase.verifyEqual(pose.points.D, ...
                [82.631791324193; 105.689142277954] * 1e-3, 'AbsTol', 1e-11);
            testCase.verifyEqual(pose.points.E, ...
                [6.972459419813; 79.695575847340] * 1e-3, 'AbsTol', 1e-11);
            testCase.verifyEqual(pose.points.G, ...
                [9.604250744006; 185.384718125293] * 1e-3, 'AbsTol', 1e-11);
        end

        function allLinkAndParallelDistancesClose(testCase)
            g = testCase.Geometry;
            pose = duallink5.kinematics.forwardFiveBar(deg2rad([85, 52.93]), g);
            p = pose.points;

            actual = [norm(p.E-p.A), norm(p.C-p.B), norm(p.D-p.C), ...
                norm(p.D-p.E), norm(p.B-p.A)];
            expected = [g.links.link1, g.links.link2, g.links.link3, ...
                g.links.link4, g.links.link5];
            testCase.verifyEqual(actual, expected, 'AbsTol', 1e-10);
            testCase.verifyEqual(norm(p.Palpha1-p.E), ...
                g.parallel.lengths.E_Palpha1, 'AbsTol', 1e-10);
            testCase.verifyEqual(norm(p.Palpha4-p.Palpha1), ...
                g.parallel.lengths.Palpha1_Palpha4, 'AbsTol', 1e-10);
            testCase.verifyEqual(norm(p.Pbeta2-p.D), ...
                g.parallel.lengths.D_Pbeta2, 'AbsTol', 1e-10);
            testCase.verifyEqual(norm(p.Pbeta3-p.Pbeta2), ...
                g.parallel.lengths.Pbeta2_Pbeta3, 'AbsTol', 1e-10);
        end

        function continuousModeFollowsPreviousBranch(testCase)
            q = deg2rad([85,52.93]);
            previous = duallink5.kinematics.forwardFiveBar(q, ...
                testCase.Geometry,struct('branchId',int8(1), ...
                'collisionProfile',"none"));
            options = struct('branchMode',"continuous", ...
                'previousPose',previous,'maxContinuityCost',5e-3, ...
                'collisionProfile',"none");

            pose = duallink5.kinematics.forwardFiveBar( ...
                q+deg2rad([0.1,-0.1]),testCase.Geometry,options);

            testCase.verifyTrue(pose.quality.valid);
            testCase.verifyEqual(pose.quality.branchId,int8(1));
            testCase.verifyLessThan(pose.quality.continuityCost,5e-3);
        end

        function excessiveContinuityJumpReturnsStatus(testCase)
            q=deg2rad([85,52.93]);
            previous=duallink5.kinematics.forwardFiveBar(q, ...
                testCase.Geometry,struct('collisionProfile',"none"));
            options=struct('branchMode',"continuous", ...
                'previousPose',previous,'maxContinuityCost',1e-12, ...
                'collisionProfile',"none");

            pose=duallink5.kinematics.forwardFiveBar( ...
                q+deg2rad([0.1,-0.1]),testCase.Geometry,options);

            testCase.verifyFalse(pose.quality.valid);
            testCase.verifyEqual(pose.quality.statusCode, ...
                "BRANCH_DISCONTINUITY");
        end

        function invalidBranchConfigurationThrows(testCase)
            options=struct('branchMode',"fixed",'branchId',int8(7));
            testCase.verifyError(@()duallink5.kinematics.forwardFiveBar( ...
                deg2rad([85,52.93]),testCase.Geometry,options), ...
                'duallink5:kinematics:InvalidBranchConfiguration');
        end

        function nonfiniteInputReturnsStatus(testCase)
            pose = duallink5.kinematics.forwardFiveBar([NaN, 0.5], testCase.Geometry);
            testCase.verifyFalse(pose.quality.valid);
            testCase.verifyEqual(pose.quality.statusCode, "NONFINITE_INPUT");
        end
    end
end
```

- [ ] **Step 2: Run the tests and verify failure**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); results=runtests('tests/kinematics/TestForwardFiveBar.m'); assert(all([results.Passed]))"
```

Expected: FAIL because `forwardFiveBar` is undefined.

- [ ] **Step 3: Implement endpoint-named parallel points**

Create `src/+duallink5/+kinematics/computeParallelPoints.m`:

```matlab
function points = computeParallelPoints(A, C, D, E, geometry)
A = A(:); C = C(:); D = D(:); E = E(:);
tolerance = geometry.tolerance.length;

uAE = unitVector(E - A, tolerance, 'AE');
uCD = unitVector(D - C, tolerance, 'CD');
L = geometry.parallel.lengths;
direction = geometry.parallel.direction;

alphaShift = direction.alphaAlongLink * L.E_Palpha1 * uAE;
points.Palpha1 = E + alphaShift;
points.Palpha2 = D + alphaShift;
alphaSide = direction.alphaSide * L.Palpha1_Palpha4 * uAE;
points.Palpha3 = points.Palpha2 + alphaSide;
points.Palpha4 = points.Palpha1 + alphaSide;

betaShift = direction.betaAlongLink * L.D_Pbeta2 * uCD;
points.Pbeta1 = E + betaShift;
points.Pbeta2 = D + betaShift;
betaSide = direction.betaSide * L.Pbeta2_Pbeta3 * uCD;
points.Pbeta3 = points.Pbeta2 + betaSide;
points.Pbeta4 = points.Pbeta1 + betaSide;
end

function unit = unitVector(vector, tolerance, label)
lengthValue = norm(vector);
if lengthValue <= tolerance
    error('duallink5:kinematics:DegenerateDirection', ...
        '%s direction is degenerate.', label);
end
unit = vector / lengthValue;
end
```

- [ ] **Step 4: Implement structured forward kinematics**

Create `src/+duallink5/+kinematics/forwardFiveBar.m`:

```matlab
function pose = forwardFiveBar(q, geometry, options)
if nargin < 3
    options = struct();
end
geometry = duallink5.model.validateGeometry(geometry);
q = q(:).';

pose = invalidPose("UNINITIALIZED");
pose.metadata.units = geometry.units;
pose.metadata.convention = geometry.convention;
pose.metadata.geometryVersion = geometry.version;
pose.metadata.q = q;
if numel(q) ~= 2
    error('duallink5:kinematics:InvalidJointVector', ...
        'q must contain [theta, phi].');
end
branchMode = string(getOption(options,'branchMode',"fixed"));
branchValue = getOption(options,'branchId',geometry.assembly.defaultBranch);
previousPose = getOption(options,'previousPose',[]);
maxContinuityCost = getOption(options,'maxContinuityCost',Inf);
validFixedBranch=isscalar(branchValue) && isnumeric(branchValue) && ...
    isfinite(branchValue) && ismember(double(branchValue),[-1,1]);
validPrevious=~isempty(previousPose) && isstruct(previousPose) && ...
    isfield(previousPose,'points') && isfield(previousPose.points,'D') && ...
    isnumeric(previousPose.points.D) && numel(previousPose.points.D)==2 && ...
    all(isfinite(previousPose.points.D));
if ~isscalar(branchMode) || ~ismember(branchMode,["fixed","continuous"]) || ...
        (branchMode=="fixed" && ~validFixedBranch) || ...
        (branchMode=="continuous" && ~validPrevious)
    error('duallink5:kinematics:InvalidBranchConfiguration', ...
        'Use fixed with branchId +/-1, or continuous with previousPose.');
end
if ~(isscalar(maxContinuityCost) && isnumeric(maxContinuityCost) && ...
        ~isnan(maxContinuityCost) && maxContinuityCost>0)
    error('duallink5:kinematics:InvalidBranchConfiguration', ...
        'maxContinuityCost must be positive or Inf.');
end
if validFixedBranch
    branchId=int8(branchValue);
else
    branchId=geometry.assembly.defaultBranch;
end
if any(~isfinite(q))
    pose.quality.statusCode = "NONFINITE_INPUT";
    return
end

theta = q(1);
phi = q(2);
L = geometry.links;
A = [0; 0];
B = [L.link5; 0];
E = L.link1 * [cos(theta); sin(theta)];
C = [L.link5 - L.link2*cos(phi); L.link2*sin(phi)];

[candidates, closureStatus] = duallink5.kinematics.solveClosure( ...
    C, E, L.link3, L.link4, geometry.tolerance.length);
if isempty(candidates)
    pose.quality.statusCode = closureStatus;
    return
end
if closureStatus == "NEAR_SINGULAR"
    pose.quality.branchId = candidates(1).branchId;
    pose.quality.closureResidual = candidates(1).closureResidual;
    pose.quality.statusCode = "NEAR_SINGULAR";
    return
end

[candidate, continuityCost, selectStatus] = selectCandidate( ...
    candidates, branchMode, branchId, previousPose, maxContinuityCost);
if isempty(candidate)
    pose.quality.statusCode = selectStatus;
    return
end

D = candidate.D;
G = D + E - B;
parallel = duallink5.kinematics.computeParallelPoints(A, C, D, E, geometry);

points = struct('A', A, 'B', B, 'C', C, 'D', D, 'E', E, 'G', G);
parallelNames = fieldnames(parallel);
for index = 1:numel(parallelNames)
    points.(parallelNames{index}) = parallel.(parallelNames{index});
end

pose.points = points;
pose.sharedLink.start = E;
pose.sharedLink.end = D;
pose.sharedLink.center = (D + E) / 2;
pose.sharedLink.orientation = atan2(D(2)-E(2), D(1)-E(1));
pose.quality.valid = true;
pose.quality.branchId = candidate.branchId;
pose.quality.closureResidual = candidate.closureResidual;
pose.quality.continuityCost = continuityCost;
pose.quality.collisionFree = NaN;
pose.quality.clearanceModelApplied = false;
pose.quality.statusCode = closureStatus;
end

function [candidate, cost, status] = selectCandidate( ...
        candidates, branchMode, branchId, previousPose, maxCost)
candidate = [];
cost = NaN;
status = "NO_VALID_BRANCH";

if branchMode == "continuous" && ~isempty(previousPose)
    costs = arrayfun(@(item) norm(item.D - previousPose.points.D), candidates);
    [cost, index] = min(costs);
    if cost > maxCost
        status = "BRANCH_DISCONTINUITY";
        candidate = [];
        return
    end
    candidate = candidates(index);
    status = "OK";
    return
end

index = find([candidates.branchId] == branchId, 1, 'first');
if ~isempty(index)
    candidate = candidates(index);
    cost = 0;
    status = "OK";
end
end

function value = getOption(options, name, defaultValue)
if isfield(options, name)
    value = options.(name);
else
    value = defaultValue;
end
end

function pose = invalidPose(statusCode)
pose.points = struct();
pose.sharedLink = struct('start', [], 'end', [], 'center', [], 'orientation', NaN);
pose.quality = struct('valid', false, 'branchId', int8(0), ...
    'closureResidual', Inf, 'continuityCost', NaN, ...
    'collisionFree', NaN, 'clearanceModelApplied', false, ...
    'statusCode', string(statusCode));
pose.metadata = struct();
end
```

- [ ] **Step 5: Run the forward-kinematics tests**

Run the Step 2 command again.

Expected: 6 tests pass, including the verified legacy pose, continuity selection, discontinuity rejection, and branch-configuration errors.

- [ ] **Step 6: Commit the structured forward model**

```powershell
git add src/+duallink5/+kinematics/computeParallelPoints.m src/+duallink5/+kinematics/forwardFiveBar.m tests/kinematics/TestForwardFiveBar.m
git commit -m "feat: add structured five-bar forward kinematics"
```

---

### Task 4: Pose validation and unambiguous collision semantics

**Files:**
- Create: `src/+duallink5/+validation/assessFiveBarPose.m`
- Create: `src/+duallink5/+validation/isCollisionFree.m`
- Create: `src/+duallink5/+validation/private/segmentsIntersect.m`
- Create: `src/+duallink5/+validation/private/segmentDistance.m`
- Modify: `src/+duallink5/+kinematics/forwardFiveBar.m`
- Create: `tests/validation/TestPoseValidation.m`

- [ ] **Step 1: Write failing validation tests**

Create `tests/validation/TestPoseValidation.m`:

```matlab
classdef TestPoseValidation < matlab.unittest.TestCase
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
        function verifiedPoseIsCollisionFree(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                deg2rad([85, 52.93]), testCase.Geometry);
            testCase.verifyTrue(pose.quality.valid);
            testCase.verifyTrue(pose.quality.collisionFree);
            testCase.verifyEqual(pose.quality.statusCode, "OK");
        end

        function crossingCenterlinesAreRejected(testCase)
            pose.points = struct( ...
                'A', [-1;0], 'B', [1;0], 'C', [0;-1], ...
                'D', [0;1], 'E', [2;2]);
            [safe, details] = duallink5.validation.isCollisionFree( ...
                pose, testCase.Geometry, "centerline");
            testCase.verifyFalse(safe);
            testCase.verifyNotEmpty(details.collidingPairs);
        end

        function coincidentNonJointEndpointsAreRejected(testCase)
            pose.points = struct( ...
                'A',[0;0],'B',[2;0],'C',[1;0], ...
                'D',[1;1],'E',[1;0]);
            [safe,details]=duallink5.validation.isCollisionFree( ...
                pose,testCase.Geometry,"centerline");

            testCase.verifyFalse(safe);
            testCase.verifyTrue(any(all( ...
                details.collidingPairs==["link1","link2"],2)));
        end

        function physicalProfileRequiresRadii(testCase)
            pose = duallink5.kinematics.forwardFiveBar( ...
                deg2rad([85, 52.93]), testCase.Geometry, ...
                struct('collisionProfile', "none"));
            testCase.verifyError( ...
                @() duallink5.validation.isCollisionFree( ...
                    pose, testCase.Geometry, "physicalClearance"), ...
                'duallink5:validation:MissingClearanceGeometry');
        end
    end
end
```

- [ ] **Step 2: Run validation tests and verify failure**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); results=runtests('tests/validation/TestPoseValidation.m'); assert(all([results.Passed]))"
```

Expected: FAIL because `duallink5.validation.isCollisionFree` does not exist.

- [ ] **Step 3: Implement robust segment primitives**

Create `src/+duallink5/+validation/private/segmentsIntersect.m`:

```matlab
function tf = segmentsIntersect(p1, p2, q1, q2, tolerance)
o1 = cross2(p2-p1, q1-p1);
o2 = cross2(p2-p1, q2-p1);
o3 = cross2(q2-q1, p1-q1);
o4 = cross2(q2-q1, p2-q1);
orientationTolerance = tolerance*max( ...
    [norm(p2-p1),norm(q2-q1),tolerance]);

properCrossing = ...
    ((o1>orientationTolerance && o2<-orientationTolerance) || ...
     (o1<-orientationTolerance && o2>orientationTolerance)) && ...
    ((o3>orientationTolerance && o4<-orientationTolerance) || ...
     (o3<-orientationTolerance && o4>orientationTolerance));
tf = properCrossing || ...
    (abs(o1)<=orientationTolerance && onSegment(p1,p2,q1,tolerance)) || ...
    (abs(o2)<=orientationTolerance && onSegment(p1,p2,q2,tolerance)) || ...
    (abs(o3)<=orientationTolerance && onSegment(q1,q2,p1,tolerance)) || ...
    (abs(o4)<=orientationTolerance && onSegment(q1,q2,p2,tolerance));
end

function tf = onSegment(a, b, p, tolerance)
tf = all(p >= min(a,b)-tolerance) && all(p <= max(a,b)+tolerance);
end

function value = cross2(a, b)
value = a(1)*b(2) - a(2)*b(1);
end
```

Create `src/+duallink5/+validation/private/segmentDistance.m`:

```matlab
function distance = segmentDistance(p1, p2, q1, q2, tolerance)
if segmentsIntersect(p1, p2, q1, q2, tolerance)
    distance = 0;
    return
end
distance = min([pointDistance(p1,q1,q2,tolerance), ...
    pointDistance(p2,q1,q2,tolerance), ...
    pointDistance(q1,p1,p2,tolerance), ...
    pointDistance(q2,p1,p2,tolerance)]);
end

function distance = pointDistance(point, a, b, tolerance)
ab = b-a;
if dot(ab,ab) <= tolerance^2
    distance = norm(point-a);
    return
end
t = max(0, min(1, dot(point-a,ab)/dot(ab,ab)));
distance = norm(point-(a+t*ab));
end
```

- [ ] **Step 4: Implement collision checking and pose assessment**

Create `src/+duallink5/+validation/isCollisionFree.m`:

```matlab
function [safe, details] = isCollisionFree(pose, geometry, profile)
geometry=duallink5.model.validateGeometry(geometry);
profile = string(profile);
if ~isscalar(profile) || ...
        ~ismember(profile,["centerline","physicalClearance"])
    error('duallink5:validation:InvalidCollisionProfile', ...
        'profile must be centerline or physicalClearance.');
end
p = pose.points;
segments = [ ...
    segment("link1",p.A,p.E,"A","E"), ...
    segment("link2",p.B,p.C,"B","C"), ...
    segment("link3",p.C,p.D,"C","D"), ...
    segment("link4",p.E,p.D,"E","D"), ...
    segment("link5",p.A,p.B,"A","B")];

pairBuffer = strings(nchoosek(numel(segments),2),2);
pairCount = 0;
details.clearanceModelApplied = profile=="physicalClearance";
if details.clearanceModelApplied
    validatePhysicalGeometry(geometry.collision);
end
safe = true;

for first = 1:numel(segments)-1
    for second = first+1:numel(segments)
        if shareTopologicalJoint(segments(first),segments(second))
            continue
        end
        if isExempt(segments(first).name, segments(second).name, ...
                geometry.collision.exemptPairs)
            continue
        end

        if profile == "centerline"
            collision = segmentsIntersect(segments(first).p1, segments(first).p2, ...
                segments(second).p1, segments(second).p2, geometry.tolerance.length);
        elseif profile == "physicalClearance"
            firstName=char(segments(first).name);
            secondName=char(segments(second).name);
            required = geometry.collision.radius.(firstName) + ...
                geometry.collision.radius.(secondName) + ...
                geometry.collision.clearance;
            planarDistance = segmentDistance( ...
                segments(first).p1, segments(first).p2, ...
                segments(second).p1, segments(second).p2, geometry.tolerance.length);
            layerDistance = geometry.collision.layerOffset.(firstName) - ...
                geometry.collision.layerOffset.(secondName);
            actual = hypot(planarDistance,layerDistance);
            collision = actual < required;
        else
            error('duallink5:validation:InvalidCollisionProfile', ...
                'profile must be centerline or physicalClearance.');
        end

        if collision
            safe = false;
            pairCount=pairCount+1;
            pairBuffer(pairCount,:) = ...
                [segments(first).name,segments(second).name];
        end
    end
end
details.collidingPairs=pairBuffer(1:pairCount,:);
end

function value = segment(name,p1,p2,joint1,joint2)
value = struct('name',string(name),'p1',p1(:),'p2',p2(:), ...
    'joints',[string(joint1),string(joint2)]);
end

function tf = shareTopologicalJoint(a,b)
tf=any(a.joints==b.joints(1)) || any(a.joints==b.joints(2));
end

function tf = isExempt(first, second, exemptPairs)
tf = any((exemptPairs(:,1)==first & exemptPairs(:,2)==second) | ...
    (exemptPairs(:,1)==second & exemptPairs(:,2)==first));
end

function validatePhysicalGeometry(collision)
names = {'link1','link2','link3','link4','link5'};
for index=1:numel(names)
    name=names{index};
    validRadius=isfield(collision.radius,name) && ...
        isscalar(collision.radius.(name)) && ...
        isnumeric(collision.radius.(name)) && ...
        isfinite(collision.radius.(name)) && collision.radius.(name)>=0;
    validLayer=isfield(collision.layerOffset,name) && ...
        isscalar(collision.layerOffset.(name)) && ...
        isnumeric(collision.layerOffset.(name)) && ...
        isfinite(collision.layerOffset.(name));
    if ~validRadius || ~validLayer
        error('duallink5:validation:MissingClearanceGeometry', ...
            ['physicalClearance requires nonnegative link radii ', ...
             'and finite layer offsets for link1 through link5.']);
    end
end
end
```

Create `src/+duallink5/+validation/assessFiveBarPose.m`:

```matlab
function pose = assessFiveBarPose(pose, geometry, profile)
if ~pose.quality.valid
    return
end
if pose.quality.closureResidual > geometry.tolerance.residual
    pose.quality.valid=false;
    pose.quality.statusCode="UNREACHABLE_CLOSURE";
    return
end
if profile=="none"
    pose.quality.statusCode="OK";
    return
end

[safe, details] = duallink5.validation.isCollisionFree(pose, geometry, profile);
pose.quality.collisionFree = safe;
pose.quality.clearanceModelApplied = details.clearanceModelApplied;
pose.quality.collisionDetails = details;

if ~safe
    pose.quality.valid = false;
    pose.quality.statusCode = "SELF_COLLISION";
else
    pose.quality.statusCode = "OK";
end
end
```

- [ ] **Step 5: Connect validation to `forwardFiveBar`**

Immediately after `geometry = duallink5.model.validateGeometry(geometry);`, validate the configuration even when the runtime pose later proves invalid:

```matlab
collisionProfile=string(getOption(options,'collisionProfile',"centerline"));
if ~isscalar(collisionProfile) || ...
        ~ismember(collisionProfile,["none","centerline","physicalClearance"])
    error('duallink5:validation:InvalidCollisionProfile', ...
        'collisionProfile must be none, centerline, or physicalClearance.');
end
```

At the end of `forwardFiveBar`, immediately before its final `end`, add:

```matlab
pose = duallink5.validation.assessFiveBarPose( ...
    pose, geometry, collisionProfile);
```

- [ ] **Step 6: Run validation and existing kinematics tests**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); results=runtests({'tests/validation','tests/kinematics'}); assert(all([results.Passed]))"
```

Expected: all closure, FK, and validation tests pass.

- [ ] **Step 7: Commit validation semantics**

```powershell
git add src/+duallink5/+validation src/+duallink5/+kinematics/forwardFiveBar.m tests/validation/TestPoseValidation.m
git commit -m "feat: validate poses and collision semantics"
```

---

### Task 5: Symmetric upper/lower assembly and configurable task frame

**Files:**
- Create: `src/+duallink5/+kinematics/transformFiveBarPose.m`
- Create: `src/+duallink5/+kinematics/forwardAssembly.m`
- Create: `src/+duallink5/+kinematics/taskPose.m`
- Create: `tests/kinematics/TestAssembly.m`

- [ ] **Step 1: Write failing assembly tests**

Create `tests/kinematics/TestAssembly.m`:

```matlab
classdef TestAssembly < matlab.unittest.TestCase
    properties
        Geometry
        Q
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            testCase.Geometry = duallink5.model.defaultGeometry();
            testCase.Q = deg2rad([85, 52.93]);
        end
    end

    methods (Test)
        function idealSymmetryClosesSharedLink(testCase)
            q.lower = testCase.Q;
            q.upper = testCase.Q;
            assembly = duallink5.kinematics.forwardAssembly( ...
                q, testCase.Geometry, struct('mode', "ideal"));

            testCase.verifyTrue(assembly.quality.valid);
            testCase.verifyLessThan(norm(assembly.symmetryResidual), 1e-10);
            testCase.verifyEqual(assembly.lower.points.E, ...
                assembly.upper.points.D, 'AbsTol', 1e-10);
            testCase.verifyEqual(assembly.lower.points.D, ...
                assembly.upper.points.E, 'AbsTol', 1e-10);
        end

        function diagnosticModeReportsPerturbation(testCase)
            q.lower = testCase.Q;
            q.upper = testCase.Q + deg2rad([0.2, -0.1]);
            assembly = duallink5.kinematics.forwardAssembly( ...
                q, testCase.Geometry, struct('mode', "diagnostic"));

            lowerEstimate=assembly.sharedLinkEstimates.lower;
            upperEstimate=assembly.sharedLinkEstimates.upper;
            expected=[upperEstimate.center-lowerEstimate.center; ...
                atan2(sin(upperEstimate.orientation-lowerEstimate.orientation), ...
                cos(upperEstimate.orientation-lowerEstimate.orientation))];
            testCase.verifyEqual(assembly.symmetryResidual,expected,'AbsTol',1e-12);
            testCase.verifyEqual(assembly.symmetryResidual, ...
                [2.88896613817112e-4;-5.48531152552295e-5; ...
                 8.19584059540190e-4],'AbsTol',1e-12);
            testCase.verifyGreaterThan(assembly.symmetryResidual(1),0);
            testCase.verifyLessThan(assembly.symmetryResidual(2),0);
            testCase.verifyGreaterThan(assembly.symmetryResidual(3),0);
            testCase.verifyEqual(assembly.metadata.symmetryResidualUnits, ...
                ["m";"m";"rad"]);
            testCase.verifyEqual(assembly.quality.statusCode, "SHARED_LINK_MISMATCH");
            testCase.verifyFalse(assembly.quality.valid);
            testCase.verifyTrue(assembly.quality.diagnosticAvailable);
            testCase.verifyFalse(isfield(assembly,'sharedLink'));
            testCase.verifyError(@()duallink5.kinematics.taskPose( ...
                assembly,struct('kind',"sharedCenter")), ...
                'duallink5:kinematics:AmbiguousDiagnosticTask');
        end

        function taskSpecSelectsCenterAndOffset(testCase)
            q.lower = testCase.Q;
            q.upper = testCase.Q;
            assembly = duallink5.kinematics.forwardAssembly(q, testCase.Geometry);

            center = duallink5.kinematics.taskPose( ...
                assembly, struct('kind', "sharedCenter", 'includeOrientation', true));
            offset = duallink5.kinematics.taskPose(assembly, ...
                struct('kind', "sharedOffset", 'offset', [10e-3;0], ...
                'includeOrientation', true));
            marker = duallink5.kinematics.taskPose(assembly, ...
                struct('kind',"marker",'offset',[10e-3;0], ...
                'orientationOffset',0.2,'includeOrientation',true));

            testCase.verifyEqual(center.position, assembly.sharedLink.center, 'AbsTol', 1e-12);
            testCase.verifyEqual(norm(offset.position-center.position), 10e-3, 'AbsTol', 1e-12);
            testCase.verifyEqual(marker.position,offset.position,'AbsTol',1e-12);
            testCase.verifyEqual(atan2(sin(marker.orientation-center.orientation), ...
                cos(marker.orientation-center.orientation)),0.2,'AbsTol',1e-12);
        end

        function previousAssemblyControlsBothContinuousBranches(testCase)
            q.lower=testCase.Q;
            q.upper=testCase.Q;
            previous=duallink5.kinematics.forwardAssembly(q,testCase.Geometry, ...
                struct('mode',"ideal"));
            q.lower=q.lower+deg2rad([0.1,-0.1]);
            q.upper=q.lower;
            options=struct('mode',"ideal",'branchMode',"continuous", ...
                'previousAssembly',previous,'maxContinuityCost',5e-3);

            assembly=duallink5.kinematics.forwardAssembly( ...
                q,testCase.Geometry,options);

            testCase.verifyTrue(assembly.quality.valid);
            testCase.verifyEqual(assembly.lower.quality.branchId, ...
                previous.lower.quality.branchId);
            testCase.verifyEqual(assembly.upperLocal.quality.branchId, ...
                previous.upperLocal.quality.branchId);
        end
    end
end
```

- [ ] **Step 2: Run assembly tests and verify failure**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); results=runtests('tests/kinematics/TestAssembly.m'); assert(all([results.Passed]))"
```

Expected: FAIL because `forwardAssembly` is undefined.

- [ ] **Step 3: Implement pose transformation**

Create `src/+duallink5/+kinematics/transformFiveBarPose.m`:

```matlab
function transformed = transformFiveBarPose(pose, rotation, translation)
if ~isequal(size(rotation), [2,2]) || numel(translation) ~= 2
    error('duallink5:kinematics:InvalidTransform', ...
        'rotation must be 2x2 and translation must be a 2-vector.');
end
translation = translation(:);
transformed = pose;

names = fieldnames(pose.points);
for index = 1:numel(names)
    name = names{index};
    transformed.points.(name) = rotation * pose.points.(name) + translation;
end

transformed.sharedLink.start = rotation * pose.sharedLink.start + translation;
transformed.sharedLink.end = rotation * pose.sharedLink.end + translation;
transformed.sharedLink.center = rotation * pose.sharedLink.center + translation;
vector = transformed.sharedLink.end - transformed.sharedLink.start;
transformed.sharedLink.orientation = atan2(vector(2), vector(1));
transformed.metadata.rotation = rotation;
transformed.metadata.translation = translation;
end
```

- [ ] **Step 4: Implement the symmetric composition**

Create `src/+duallink5/+kinematics/forwardAssembly.m`:

```matlab
function assembly = forwardAssembly(q, geometry, options)
if nargin < 3
    options = struct();
end
mode = string(getOption(options, 'mode', "ideal"));
if ~isscalar(mode) || ~ismember(mode,["ideal","diagnostic"])
    error('duallink5:kinematics:InvalidAssemblyMode', ...
        'mode must be ideal or diagnostic.');
end
if ~isstruct(q) || ~isscalar(q) || ...
        ~isfield(q,'lower') || ~isfield(q,'upper')
    error('duallink5:kinematics:InvalidAssemblyInput', ...
        'q.lower and q.upper are required.');
end

lowerOptions = options;
upperOptions = options;
if isfield(options,'previousAssembly')
    previous=options.previousAssembly;
    if ~isstruct(previous) || ~isfield(previous,'lower') || ...
            ~isfield(previous,'upperLocal')
        error('duallink5:kinematics:InvalidBranchConfiguration', ...
            'previousAssembly must contain lower and upperLocal poses.');
    end
    lowerOptions.previousPose=previous.lower;
    upperOptions.previousPose=previous.upperLocal;
end

lower = duallink5.kinematics.forwardFiveBar(q.lower, geometry, lowerOptions);
upperLocal = duallink5.kinematics.forwardFiveBar(q.upper, geometry, upperOptions);

assembly = struct();
assembly.lower = lower;
assembly.upperLocal = upperLocal;
assembly.quality.valid = false;
assembly.quality.diagnosticAvailable = false;
assembly.quality.statusCode = "UNINITIALIZED";
assembly.symmetryResidual = [NaN;NaN;NaN];
assembly.metadata.mode = mode;
assembly.metadata.units = geometry.units;
assembly.metadata.symmetryResidualUnits = ["m";"m";"rad"];

if ~lower.quality.valid
    assembly.quality.statusCode = lower.quality.statusCode;
    assembly.metadata.failingSide="lower";
    return
end
if ~upperLocal.quality.valid
    assembly.quality.statusCode = upperLocal.quality.statusCode;
    assembly.metadata.failingSide="upper";
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
lowerCenter = (lowerStart + lowerEnd)/2;
upperCenter = (upperStart + upperEnd)/2;
lowerAngle = atan2(lowerEnd(2)-lowerStart(2), lowerEnd(1)-lowerStart(1));
upperAngle = atan2(upperEnd(2)-upperStart(2), upperEnd(1)-upperStart(1));
lowerEstimate=struct('start',lowerStart,'end',lowerEnd, ...
    'center',lowerCenter,'orientation',lowerAngle);
upperEstimate=struct('start',upperStart,'end',upperEnd, ...
    'center',upperCenter,'orientation',upperAngle);

assembly.symmetryResidual = [upperCenter-lowerCenter; wrapAngle(upperAngle-lowerAngle)];
assembly.sharedLinkEstimates.lower = lowerEstimate;
assembly.sharedLinkEstimates.upper = upperEstimate;
assembly.quality.diagnosticAvailable = true;
assembly.quality.valid = true;

positionMatch=norm(assembly.symmetryResidual(1:2)) <= ...
    geometry.tolerance.symmetryPosition;
angleMatch=abs(assembly.symmetryResidual(3)) <= ...
    geometry.tolerance.symmetryAngle;
if positionMatch && angleMatch && mode=="ideal"
    assembly.sharedLink=lowerEstimate;
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
```

- [ ] **Step 5: Implement configurable task coordinates**

Create `src/+duallink5/+kinematics/taskPose.m`:

```matlab
function task = taskPose(assembly, taskSpec)
if ~isstruct(taskSpec) || ~isscalar(taskSpec) || ~isfield(taskSpec,'kind')
    error('duallink5:kinematics:InvalidTaskSpec', ...
        'taskSpec.kind is required.');
end
kind = string(taskSpec.kind);
includeOrientation = getField(taskSpec, 'includeOrientation', false);
validInclude=isscalar(includeOrientation) && ...
    (islogical(includeOrientation) || ...
     (isnumeric(includeOrientation) && isfinite(includeOrientation) && ...
      ismember(includeOrientation,[0,1])));
if ~isscalar(kind) || ~validInclude
    error('duallink5:kinematics:InvalidTaskSpec', ...
        'kind must be scalar and includeOrientation must be logical.');
end
includeOrientation=logical(includeOrientation);

mode=string(assembly.metadata.mode);
if mode=="diagnostic"
    if ~isfield(assembly.quality,'diagnosticAvailable') || ...
            ~assembly.quality.diagnosticAvailable
        error('duallink5:kinematics:InvalidAssemblyPose', ...
            'Diagnostic side poses are unavailable.');
    end
    side=string(getField(taskSpec,'side',""));
    if ~isscalar(side) || ~ismember(side,["lower","upper"])
        error('duallink5:kinematics:AmbiguousDiagnosticTask', ...
            'Diagnostic taskPose requires taskSpec.side=lower or upper.');
    end
    shared=assembly.sharedLinkEstimates.(char(side));
    if side=="lower"
        pointG=assembly.lower.points.G;
    else
        pointG=assembly.upper.points.G;
    end
else
    if ~assembly.quality.valid
        error('duallink5:kinematics:InvalidAssemblyPose', ...
            'A valid ideal assembly is required.');
    end
    shared=assembly.sharedLink;
    pointG=assembly.lower.points.G;
end

orientationOffset=getField(taskSpec,'orientationOffset',0);
if ~(isscalar(orientationOffset) && isnumeric(orientationOffset) && ...
        isfinite(orientationOffset))
    error('duallink5:kinematics:InvalidTaskSpec', ...
        'orientationOffset must be a finite scalar in radians.');
end

if kind == "sharedCenter"
    position = shared.center;
elseif kind == "pointG"
    position = pointG;
elseif ismember(kind,["sharedOffset","marker"])
    if ~isfield(taskSpec,'offset')
        error('duallink5:kinematics:InvalidTaskSpec', ...
            '%s requires taskSpec.offset.',kind);
    end
    offset = taskSpec.offset(:);
    if ~isnumeric(offset) || numel(offset) ~= 2 || any(~isfinite(offset))
        error('duallink5:kinematics:InvalidTaskSpec', ...
            '%s requires a finite 2-vector offset.',kind);
    end
    angle = shared.orientation;
    rotation = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    position = shared.center + rotation*offset;
else
    error('duallink5:kinematics:InvalidTaskSpec', ...
        'Unsupported task kind: %s.', kind);
end

task.position = position;
task.orientation = wrapAngle(shared.orientation+orientationOffset);
task.includeOrientation = includeOrientation;
if includeOrientation
    task.vector = [position; task.orientation];
else
    task.vector = position;
end
task.units = assembly.metadata.units;
end

function angle=wrapAngle(angle)
angle=atan2(sin(angle),cos(angle));
end

function value = getField(input, name, defaultValue)
if isfield(input, name)
    value = input.(name);
else
    value = defaultValue;
end
end
```

- [ ] **Step 6: Run assembly tests**

Run the Step 2 command again.

Expected: 4 tests pass, including exact ideal link reversal, diagnostic ambiguity protection, marker extrinsics, and two-sided continuity.

- [ ] **Step 7: Commit assembly composition**

```powershell
git add src/+duallink5/+kinematics/transformFiveBarPose.m src/+duallink5/+kinematics/forwardAssembly.m src/+duallink5/+kinematics/taskPose.m tests/kinematics/TestAssembly.m
git commit -m "feat: compose symmetric DualLink5 assembly"
```

---

### Task 6: Analytic task Jacobian and singularity reporting

**Files:**
- Create: `src/+duallink5/+kinematics/fiveBarJacobian.m`
- Create: `src/+duallink5/+kinematics/taskJacobian.m`
- Create: `tests/kinematics/TestJacobian.m`

- [ ] **Step 1: Write the failing Jacobian tests**

Create `tests/kinematics/TestJacobian.m`:

```matlab
classdef TestJacobian < matlab.unittest.TestCase
    properties
        Geometry
        Q
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            testCase.Geometry = duallink5.model.defaultGeometry();
            testCase.Q = deg2rad([85, 52.93]);
        end
    end

    methods (Test)
        function analyticCenterJacobianMatchesCentralDifference(testCase)
            spec = struct('kind', "sharedCenter", 'includeOrientation', true);
            result = duallink5.kinematics.taskJacobian( ...
                testCase.Q, testCase.Geometry, spec, struct());
            numeric = centralDifference(testCase.Q, testCase.Geometry, spec);

            testCase.verifyEqual(result.logical, numeric, 'AbsTol', 1e-6);
            testCase.verifyFalse(result.nearSingular);
        end

        function diagnosticInputReturnsPerSideJacobians(testCase)
            q.lower = testCase.Q;
            q.upper = testCase.Q + deg2rad([0.2, -0.1]);
            spec = struct('kind', "sharedCenter", 'includeOrientation', true);
            result = duallink5.kinematics.taskJacobian( ...
                q, testCase.Geometry, spec, struct('mode', "diagnostic"));

            testCase.verifySize(result.lowerLocal, [3,2]);
            testCase.verifySize(result.upperLocal, [3,2]);
            testCase.verifyEmpty(result.logical);
        end

        function positionOnlySingularResultHasCorrectShape(testCase)
            pose=duallink5.kinematics.forwardFiveBar( ...
                testCase.Q,testCase.Geometry,struct('collisionProfile',"none"));
            pose.points.C=[0;0];
            pose.points.E=[80e-3;0];
            pose.points.D=[40e-3;0];
            spec=struct('kind',"sharedCenter",'includeOrientation',false);

            result=duallink5.kinematics.fiveBarJacobian( ...
                testCase.Q,pose,testCase.Geometry,spec);

            testCase.verifySize(result.matrix,[2,2]);
            testCase.verifyTrue(all(isnan(result.matrix),'all'));
            testCase.verifyTrue(result.nearSingular);
            testCase.verifyEqual(result.statusCode,"NEAR_SINGULAR");
        end

        function invalidAssemblyStatusPropagates(testCase)
            spec=struct('kind',"sharedCenter",'includeOrientation',false);
            result=duallink5.kinematics.taskJacobian( ...
                [NaN,0.5],testCase.Geometry,spec,struct());

            testCase.verifyEqual(result.statusCode,"NONFINITE_INPUT");
            testCase.verifySize(result.logical,[2,2]);
            testCase.verifyTrue(all(isnan(result.logical),'all'));
        end
    end
end

function J = centralDifference(q, geometry, spec)
step = 1e-7;
J = zeros(3,2);
for column = 1:2
    delta = zeros(1,2);
    delta(column) = step;
    plus = idealTask(q+delta, geometry, spec);
    minus = idealTask(q-delta, geometry, spec);
    difference = plus.vector-minus.vector;
    difference(3) = atan2(sin(difference(3)),cos(difference(3)));
    J(:,column) = difference/(2*step);
end
end

function task = idealTask(q, geometry, spec)
input.lower = q;
input.upper = q;
assembly = duallink5.kinematics.forwardAssembly(input, geometry);
task = duallink5.kinematics.taskPose(assembly, spec);
end
```

- [ ] **Step 2: Run the Jacobian tests and verify failure**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); results=runtests('tests/kinematics/TestJacobian.m'); assert(all([results.Passed]))"
```

Expected: FAIL because `taskJacobian` is undefined.

- [ ] **Step 3: Implement constraint-differentiated five-bar Jacobian**

Create `src/+duallink5/+kinematics/fiveBarJacobian.m`:

```matlab
function result = fiveBarJacobian(q, pose, geometry, taskSpec)
q = q(:).';
theta = q(1);
phi = q(2);
L = geometry.links;
C = pose.points.C;
D = pose.points.D;
E = pose.points.E;

dE = [-L.link1*sin(theta), 0; L.link1*cos(theta), 0];
dC = [0, L.link2*sin(phi); 0, L.link2*cos(phi)];
constraint = [(D-C).'; (D-E).'];
closureConditionNumber = cond(constraint);
includeOrientation=isfield(taskSpec,'includeOrientation') && ...
    taskSpec.includeOrientation;
taskDimension=2+double(includeOrientation);

if ~isfinite(closureConditionNumber) || ...
        closureConditionNumber >= geometry.tolerance.singularityCondition
    result.matrix = nan(taskDimension,2);
    result.closureConditionNumber = closureConditionNumber;
    result.taskConditionNumber = Inf;
    result.conditionNumber = Inf;
    result.taskRowScale=[ones(1,taskDimension-includeOrientation), ...
        repmat(L.link4,1,double(includeOrientation))];
    result.nearSingular = true;
    result.statusCode = "NEAR_SINGULAR";
    return
end

dD = zeros(2,2);
for column = 1:2
    rightHandSide = [(D-C).'*dC(:,column); (D-E).'*dE(:,column)];
    dD(:,column) = constraint\rightHandSide;
end

dP = (dD+dE)/2;
dG = dD+dE;
linkVector = D-E;
dLink = dD-dE;
dPsi = zeros(1,2);
for column = 1:2
    dPsi(column) = cross2(linkVector,dLink(:,column))/dot(linkVector,linkVector);
end

kind = string(taskSpec.kind);
if kind == "sharedCenter"
    positionJacobian = dP;
elseif kind == "pointG"
    positionJacobian = dG;
elseif ismember(kind,["sharedOffset","marker"])
    offset = taskSpec.offset(:);
    psi = pose.sharedLink.orientation;
    derivativeRotation = [-sin(psi),-cos(psi);cos(psi),-sin(psi)];
    positionJacobian = dP + derivativeRotation*offset*dPsi;
else
    error('duallink5:kinematics:InvalidTaskSpec', ...
        'Unsupported task kind: %s.', kind);
end

if includeOrientation
    result.matrix = [positionJacobian; dPsi];
else
    result.matrix = positionJacobian;
end
scaledTaskMatrix=result.matrix;
taskRowScale=ones(1,taskDimension);
if includeOrientation
    taskRowScale(end)=L.link4;
    scaledTaskMatrix(end,:)=L.link4*scaledTaskMatrix(end,:);
end
taskConditionNumber=cond(scaledTaskMatrix);
result.closureConditionNumber=closureConditionNumber;
result.taskConditionNumber=taskConditionNumber;
result.taskRowScale=taskRowScale;
result.conditionNumber=max(closureConditionNumber,taskConditionNumber);
result.nearSingular=~isfinite(taskConditionNumber) || ...
    taskConditionNumber>=geometry.tolerance.singularityCondition;
if result.nearSingular
    result.statusCode="NEAR_SINGULAR";
else
    result.statusCode="OK";
end
end

function value = cross2(a,b)
value = a(1)*b(2)-a(2)*b(1);
end
```

- [ ] **Step 4: Implement the assembly-level Jacobian API**

Create `src/+duallink5/+kinematics/taskJacobian.m`:

```matlab
function result = taskJacobian(q, geometry, taskSpec, options)
if nargin < 4
    options = struct();
end

if isnumeric(q)
    q = q(:).';
    input.lower = q;
    input.upper = q;
    idealOptions=options;
    idealOptions.mode="ideal";
    assembly = duallink5.kinematics.forwardAssembly(input,geometry,idealOptions);
    if ~assembly.quality.valid
        result=invalidResult(taskSpec,assembly.quality.statusCode,false);
        return
    end
    lower = duallink5.kinematics.fiveBarJacobian( ...
        q, assembly.lower, geometry, taskSpec);
    result.logical = lower.matrix;
    result.lowerLocal = lower.matrix;
    result.upperLocal = lower.matrix;
    result.closureConditionNumber=lower.closureConditionNumber;
    result.taskConditionNumber=lower.taskConditionNumber;
    result.taskRowScale=lower.taskRowScale;
    result.conditionNumber = lower.conditionNumber;
    result.nearSingular = lower.nearSingular;
    result.statusCode = lower.statusCode;
    result.assemblyStatusCode=assembly.quality.statusCode;
    return
end

diagnosticOptions=options;
diagnosticOptions.mode="diagnostic";
assembly = duallink5.kinematics.forwardAssembly(q,geometry,diagnosticOptions);
if ~isfield(assembly.quality,'diagnosticAvailable') || ...
        ~assembly.quality.diagnosticAvailable
    result=invalidResult(taskSpec,assembly.quality.statusCode,true);
    return
end
lower = duallink5.kinematics.fiveBarJacobian( ...
    q.lower, assembly.lower, geometry, taskSpec);
upper = duallink5.kinematics.fiveBarJacobian( ...
    q.upper, assembly.upperLocal, geometry, taskSpec);
result.logical = [];
result.lowerLocal = lower.matrix;
result.upperLocal = upper.matrix;
result.closureConditionNumber=[lower.closureConditionNumber, ...
    upper.closureConditionNumber];
result.taskConditionNumber=[lower.taskConditionNumber, ...
    upper.taskConditionNumber];
result.taskRowScale=lower.taskRowScale;
result.conditionNumber = [lower.conditionNumber, upper.conditionNumber];
result.nearSingular = lower.nearSingular || upper.nearSingular;
result.assemblyStatusCode=assembly.quality.statusCode;
if result.nearSingular
    result.statusCode="NEAR_SINGULAR";
else
    result.statusCode="DIAGNOSTIC_ONLY";
end
end

function result=invalidResult(taskSpec,statusCode,diagnostic)
includeOrientation=isfield(taskSpec,'includeOrientation') && ...
    taskSpec.includeOrientation;
rows=2+double(includeOrientation);
result.logical=nan(rows,2);
if diagnostic
    result.logical=[];
end
result.lowerLocal=nan(rows,2);
result.upperLocal=nan(rows,2);
result.closureConditionNumber=Inf;
result.taskConditionNumber=Inf;
result.taskRowScale=nan(1,rows);
result.conditionNumber=Inf;
result.nearSingular=statusCode=="NEAR_SINGULAR";
result.statusCode=statusCode;
result.assemblyStatusCode=statusCode;
end
```

- [ ] **Step 5: Run Jacobian and assembly tests**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); results=runtests({'tests/kinematics/TestAssembly.m','tests/kinematics/TestJacobian.m'}); assert(all([results.Passed]))"
```

Expected: all tests pass; analytic and central-difference Jacobians agree within `1e-6`.

- [ ] **Step 6: Commit Jacobian support**

```powershell
git add src/+duallink5/+kinematics/fiveBarJacobian.m src/+duallink5/+kinematics/taskJacobian.m tests/kinematics/TestJacobian.m
git commit -m "feat: add analytic task Jacobian"
```

---

### Task 7: Inverse kinematics with forward validation

**Files:**
- Create: `src/+duallink5/+kinematics/inverseKinematics.m`
- Create: `tests/kinematics/TestInverseKinematics.m`

- [ ] **Step 1: Write failing inverse-kinematics tests**

Create `tests/kinematics/TestInverseKinematics.m`:

```matlab
classdef TestInverseKinematics < matlab.unittest.TestCase
    properties
        Geometry
        Q
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
            testCase.Geometry = duallink5.model.defaultGeometry();
            testCase.Q = deg2rad([85, 52.93]);
        end
    end

    methods (Test)
        function roundTripSharedCenter(testCase)
            pose = duallink5.kinematics.forwardFiveBar(testCase.Q, testCase.Geometry);
            target = struct('kind', "sharedCenter", 'position', pose.sharedLink.center);
            solutions = duallink5.kinematics.inverseKinematics( ...
                target, testCase.Geometry, struct('collisionProfile', "centerline"));

            testCase.verifyNumElements(solutions,2);
            testCase.verifyEqual(sort([solutions.branchId]),int8([-1,-1]));
            testCase.verifyEqual(sort(rad2deg(arrayfun( ...
                @(item)item.q(2),solutions))), ...
                [52.93,129.922883026159],'AbsTol',1e-9);
            errors = arrayfun(@(item) norm(item.q-testCase.Q), solutions);
            testCase.verifyLessThan(min(errors), 1e-7);
            testCase.verifyTrue(all([solutions.valid]));
            testCase.verifyTrue(all([solutions.collisionFree]));
            testCase.verifyLessThan(max([solutions.closureResidual]),1e-9);
            for index=1:numel(solutions)
                candidate=solutions(index);
                pose=duallink5.kinematics.forwardFiveBar( ...
                    candidate.q,testCase.Geometry, ...
                    struct('branchId',candidate.branchId));
                testCase.verifyLessThan( ...
                    norm(pose.sharedLink.center-target.position),1e-8);
            end
            testCase.verifyLessThan(max([solutions.forwardResidual]), 1e-8);
        end

        function pointGTargetUsesApprovedMidpointRelation(testCase)
            pose = duallink5.kinematics.forwardFiveBar(testCase.Q, testCase.Geometry);
            target = struct('kind', "pointG", 'position', pose.points.G);
            solutions = duallink5.kinematics.inverseKinematics( ...
                target, testCase.Geometry, struct());
            testCase.verifyNotEmpty(solutions);
            testCase.verifyLessThan(min([solutions.forwardResidual]), 1e-8);
        end

        function unreachableTargetReturnsEmpty(testCase)
            target = struct('kind', "sharedCenter", 'position', [1;1]);
            solutions = duallink5.kinematics.inverseKinematics( ...
                target, testCase.Geometry, struct());
            testCase.verifyEmpty(solutions);
        end
    end
end
```

- [ ] **Step 2: Run IK tests and verify failure**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); results=runtests('tests/kinematics/TestInverseKinematics.m'); assert(all([results.Passed]))"
```

Expected: FAIL because `inverseKinematics` is undefined.

- [ ] **Step 3: Implement signed analytic IK and mandatory FK filtering**

Create `src/+duallink5/+kinematics/inverseKinematics.m`:

```matlab
function solutions = inverseKinematics(target, geometry, options)
if nargin < 3
    options = struct();
end
geometry = duallink5.model.validateGeometry(geometry);
kind = string(target.kind);
position = target.position(:);
if numel(position) ~= 2 || any(~isfinite(position))
    error('duallink5:kinematics:InvalidIKTarget', ...
        'IK target position must be a finite 2-vector.');
end

A = [0;0];
B = [geometry.links.link5;0];
if kind == "sharedCenter"
    P = position;
elseif kind == "pointG"
    P = (position+B)/2;
else
    error('duallink5:kinematics:UnsupportedIKTask', ...
        'IK currently supports sharedCenter and pointG targets.');
end

EList = circleIntersections(A, geometry.links.link1, ...
    P, geometry.links.link4/2, geometry.tolerance.length);
template = struct('q', zeros(1,2), 'branchId', int8(0), ...
    'valid', false, 'forwardResidual', Inf, 'closureResidual',Inf, ...
    'collisionFree',false,'nearSingular',false, ...
    'statusCode',"UNINITIALIZED",'pose', struct());
solutionBuffer = repmat(template, 4, 1);
solutionCount=0;
forwardTolerance=getOption(options,'forwardTolerance',1e-8);
if ~(isscalar(forwardTolerance) && isfinite(forwardTolerance) && ...
        forwardTolerance>0)
    error('duallink5:kinematics:InvalidIKOptions', ...
        'forwardTolerance must be a positive finite scalar.');
end

for eIndex = 1:size(EList,2)
    E = EList(:,eIndex);
    D = 2*P-E;
    CList = circleIntersections(B, geometry.links.link2, ...
        D, geometry.links.link3, geometry.tolerance.length);

    for cIndex = 1:size(CList,2)
        C = CList(:,cIndex);
        theta = atan2(E(2),E(1));
        phi = atan2(C(2), geometry.links.link5-C(1));
        branch = int8(sign(cross2(E-C,D-C)));
        if branch==0
            continue
        end
        forwardOptions = options;
        forwardOptions.branchId = branch;
        forwardOptions.branchMode = "fixed";
        pose = duallink5.kinematics.forwardFiveBar( ...
            [theta,phi], geometry, forwardOptions);
        if ~pose.quality.valid
            continue
        end
        if kind == "sharedCenter"
            predicted = pose.sharedLink.center;
        else
            predicted = pose.points.G;
        end
        forwardResidual = norm(predicted-position);
        if forwardResidual > forwardTolerance
            continue
        end

        item = template;
        item.q = [theta,phi];
        item.branchId = branch;
        item.valid = true;
        item.forwardResidual = forwardResidual;
        item.closureResidual=pose.quality.closureResidual;
        item.collisionFree=pose.quality.collisionFree;
        item.nearSingular=pose.quality.statusCode=="NEAR_SINGULAR";
        item.statusCode=pose.quality.statusCode;
        item.pose = pose;
        solutionCount=solutionCount+1;
        solutionBuffer(solutionCount)=item;
    end
end
solutions = deduplicate(solutionBuffer(1:solutionCount),forwardTolerance);
end

function points = circleIntersections(center1, radius1, center2, radius2, tolerance)
delta = center2-center1;
distance = norm(delta);
points = zeros(2,0);
if distance < tolerance || distance > radius1+radius2+tolerance || ...
        distance < abs(radius1-radius2)-tolerance
    return
end
unit = delta/distance;
along = (radius1^2-radius2^2+distance^2)/(2*distance);
heightSquared = radius1^2-along^2;
squaredTolerance=tolerance*max([radius1,radius2,distance,tolerance]);
if heightSquared < -squaredTolerance
    return
end
height = sqrt(max(0,heightSquared));
base = center1+along*unit;
normal = [-unit(2);unit(1)];
if height <= tolerance
    points = base;
else
    points = [base+height*normal,base-height*normal];
end
end

function uniqueSolutions = deduplicate(solutions, tolerance)
if isempty(solutions)
    uniqueSolutions=solutions;
    return
end
uniqueBuffer=repmat(solutions(1),numel(solutions),1);
uniqueCount=0;
for index = 1:numel(solutions)
    current=uniqueBuffer(1:uniqueCount);
    if uniqueCount==0 || all(arrayfun( ...
            @(item) norm(wrapVector(item.q-solutions(index).q))>tolerance, ...
            current))
        uniqueCount=uniqueCount+1;
        uniqueBuffer(uniqueCount) = solutions(index);
    end
end
uniqueSolutions=uniqueBuffer(1:uniqueCount);
end

function value = wrapVector(value)
value = atan2(sin(value),cos(value));
end

function value = cross2(a,b)
value = a(1)*b(2)-a(2)*b(1);
end

function value=getOption(options,name,defaultValue)
if isfield(options,name), value=options.(name); else, value=defaultValue; end
end
```

- [ ] **Step 4: Run IK and FK tests**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); results=runtests({'tests/kinematics/TestForwardFiveBar.m','tests/kinematics/TestInverseKinematics.m'}); assert(all([results.Passed]))"
```

Expected: all tests pass; every returned IK candidate has forward residual below `1e-8 m`.

- [ ] **Step 5: Commit inverse kinematics**

```powershell
git add src/+duallink5/+kinematics/inverseKinematics.m tests/kinematics/TestInverseKinematics.m
git commit -m "feat: add validated inverse kinematics"
```

---

### Task 8: Fixed-geometry workspace analysis

**Files:**
- Create: `src/+duallink5/+workspace/sampleWorkspace.m`
- Create: `src/+duallink5/+workspace/analyzeWorkspace.m`
- Create: `src/+duallink5/+workspace/largestRectangleInMask.m`
- Create: `tests/workspace/TestWorkspace.m`

- [ ] **Step 1: Write failing workspace tests**

Create `tests/workspace/TestWorkspace.m`:

```matlab
classdef TestWorkspace < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(~)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
        end
    end

    methods (Test)
        function largestRectangleFindsKnownMask(testCase)
            mask = logical([1 1 0;1 1 1;1 1 1]);
            result = duallink5.workspace.largestRectangleInMask(mask, 2, 3);
            testCase.verifyEqual(result.areaCells, 6);
            testCase.verifyEqual(result.area, 36);
        end

        function emptyMaskHasZeroPhysicalExtent(testCase)
            result=duallink5.workspace.largestRectangleInMask( ...
                false(3,4),2,3);
            testCase.verifyEqual(result.areaCells,0);
            testCase.verifyEqual(result.area,0);
            testCase.verifyEqual(result.width,0);
            testCase.verifyEqual(result.height,0);
        end

        function samplerReturnsReasonMapAndFiniteTasks(testCase)
            g = duallink5.model.defaultGeometry();
            grid.theta = linspace(deg2rad(70),deg2rad(100),9);
            grid.phi = linspace(deg2rad(40),deg2rad(65),9);
            spec = struct('kind',"pointG",'includeOrientation',false);
            samples = duallink5.workspace.sampleWorkspace(grid,g,spec,struct());

            testCase.verifySize(samples.validMask,[9,9]);
            testCase.verifySize(samples.reasonMap,[9,9]);
            testCase.verifyTrue(any(samples.validMask,'all'));
            testCase.verifyTrue(all(isfinite(samples.x(samples.validMask))));
            testCase.verifyTrue(all(isfinite( ...
                samples.conditionNumber(samples.validMask))));
        end

        function fixedMechanismWorkspaceRegression(testCase)
            g=duallink5.model.defaultGeometry();
            grid.theta=linspace(deg2rad(80),deg2rad(90),9);
            grid.phi=linspace(deg2rad(48),deg2rad(58),9);
            spec=struct('kind',"pointG",'includeOrientation',false);
            samples=duallink5.workspace.sampleWorkspace( ...
                grid,g,spec,struct());
            result=duallink5.workspace.analyzeWorkspace( ...
                samples,struct('alpha',Inf,'gridSize',[40,40]));

            testCase.verifyEqual(nnz(samples.validMask),81);
            testCase.verifyEqual(result.area, ...
                4.844403225084295e-4,'AbsTol',1e-12);
            testCase.verifyEqual(result.maxRectangle.area, ...
                1.515848848459369e-4,'AbsTol',1e-12);
        end


        function squareSamplesGiveAreaAndRectangleRegression(testCase)
            samples.x=[0,1;0,1;NaN,NaN];
            samples.y=[0,0;1,1;NaN,NaN];
            samples.validMask=logical([1,1;1,1;0,0]);
            samples.reasonMap=["OK","OK";"OK","OK"; ...
                "NEAR_SINGULAR","NONFINITE_INPUT"];
            samples.conditionNumber=[2,3;4,5;Inf,NaN];
            samples.metadata=struct('units', ...
                struct('length',"m",'angle',"rad"));

            result=duallink5.workspace.analyzeWorkspace( ...
                samples,struct('alpha',Inf,'gridSize',[20,20]));

            testCase.verifyEqual(result.area,1,'AbsTol',1e-12);
            testCase.verifyEqual(result.maxRectangle.area,1,'AbsTol',1e-12);
            testCase.verifyEqual(result.maxRectangle.bounds, ...
                [0,1,0,1],'AbsTol',1e-12);
            testCase.verifyEqual(result.nearSingularCount,1);
        end
    end
end
```

- [ ] **Step 2: Run workspace tests and verify failure**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); results=runtests('tests/workspace/TestWorkspace.m'); assert(all([results.Passed]))"
```

Expected: FAIL because workspace package functions are undefined.

- [ ] **Step 3: Implement workspace sampling from one FK path**

Create `src/+duallink5/+workspace/sampleWorkspace.m`:

```matlab
function samples = sampleWorkspace(grid, geometry, taskSpec, options)
if nargin<4, options=struct(); end
if ~isfield(grid,'theta') || ~isfield(grid,'phi') || ...
        ~isvector(grid.theta) || ~isvector(grid.phi) || ...
        any(~isfinite(grid.theta)) || any(~isfinite(grid.phi))
    error('duallink5:workspace:InvalidAngleGrid', ...
        'grid.theta and grid.phi must be finite vectors.');
end
[thetaGrid,phiGrid] = ndgrid(grid.theta,grid.phi);
sizeGrid = size(thetaGrid);
samples.x = nan(sizeGrid);
samples.y = nan(sizeGrid);
samples.orientation = nan(sizeGrid);
samples.validMask = false(sizeGrid);
samples.reasonMap = strings(sizeGrid);
samples.conditionNumber = nan(sizeGrid);
samples.thetaGrid = thetaGrid;
samples.phiGrid = phiGrid;

for index = 1:numel(thetaGrid)
    qLogical = [thetaGrid(index),phiGrid(index)];
    q.lower=qLogical;
    q.upper=qLogical;
    assemblyOptions=options;
    assemblyOptions.mode="ideal";
    assembly=duallink5.kinematics.forwardAssembly( ...
        q,geometry,assemblyOptions);
    samples.reasonMap(index) = assembly.quality.statusCode;
    if ~assembly.quality.valid
        continue
    end
    jacobian=duallink5.kinematics.fiveBarJacobian( ...
        qLogical,assembly.lower,geometry,taskSpec);
    samples.conditionNumber(index)=jacobian.conditionNumber;
    if jacobian.nearSingular
        samples.reasonMap(index)="NEAR_SINGULAR";
        continue
    end
    task=duallink5.kinematics.taskPose(assembly,taskSpec);
    samples.x(index) = task.position(1);
    samples.y(index) = task.position(2);
    if task.includeOrientation
        samples.orientation(index)=task.orientation;
    end
    samples.validMask(index) = true;
end
samples.metadata.units = geometry.units;
samples.metadata.taskSpec = taskSpec;
end
```

- [ ] **Step 4: Implement the single rectangle algorithm**

Create `src/+duallink5/+workspace/largestRectangleInMask.m`:

```matlab
function result = largestRectangleInMask(mask, dx, dy)
if ~(ismatrix(mask) && isscalar(dx) && isfinite(dx) && dx>0 && ...
        isscalar(dy) && isfinite(dy) && dy>0)
    error('duallink5:workspace:InvalidRectangleInput', ...
        'mask must be 2-D and dx/dy must be positive finite scalars.');
end
mask = logical(mask);
heights = zeros(1,size(mask,2));
best = struct('areaCells',0,'top',0,'bottom',0,'left',0,'right',0);

for row = 1:size(mask,1)
    heights(mask(row,:)) = heights(mask(row,:))+1;
    heights(~mask(row,:)) = 0;
    candidate = histogramRectangle(heights,row);
    if candidate.areaCells > best.areaCells
        best = candidate;
    end
end

result = best;
result.area = best.areaCells*dx*dy;
if best.areaCells==0
    result.width=0;
    result.height=0;
else
    result.width = (best.right-best.left+1)*dx;
    result.height = (best.bottom-best.top+1)*dy;
end
end

function best = histogramRectangle(heights,row)
extended = [heights,0];
stack = zeros(1,numel(extended));
stackSize = 0;
best = struct('areaCells',0,'top',0,'bottom',0,'left',0,'right',0);

for index = 1:numel(extended)
    while stackSize>0 && extended(stack(stackSize))>extended(index)
        height = extended(stack(stackSize));
        stackSize = stackSize-1;
        if stackSize==0
            left = 1;
        else
            left = stack(stackSize)+1;
        end
        right = index-1;
        area = height*(right-left+1);
        if area>best.areaCells
            best.areaCells = area;
            best.bottom = row;
            best.top = row-height+1;
            best.left = left;
            best.right = right;
        end
    end
    stackSize = stackSize+1;
    stack(stackSize) = index;
end
end
```

- [ ] **Step 5: Implement workspace metrics from sampled points**

Create `src/+duallink5/+workspace/analyzeWorkspace.m`:

```matlab
function result = analyzeWorkspace(samples, options)
if nargin<2, options=struct(); end
validX = samples.x(samples.validMask);
validY = samples.y(samples.validMask);
if numel(validX)<3
    error('duallink5:workspace:InsufficientSamples', ...
        'At least three valid task samples are required.');
end
if max(validX)==min(validX) || max(validY)==min(validY)
    error('duallink5:workspace:InsufficientSpan', ...
        'Valid task samples must span nonzero x and y ranges.');
end

alpha = getOption(options,'alpha',Inf);
if ~(isscalar(alpha) && isnumeric(alpha) && ~isnan(alpha) && alpha>0)
    error('duallink5:workspace:InvalidAlpha', ...
        'alpha must be a positive scalar or Inf.');
end
shape = alphaShape(validX,validY,alpha);
result.area = area(shape);
result.boundaryShape = shape;

gridSize = getOption(options,'gridSize',[150,150]);
if ~(isnumeric(gridSize) && isequal(size(gridSize),[1,2]) && ...
        all(isfinite(gridSize)) && all(gridSize>=2) && ...
        all(gridSize==fix(gridSize)))
    error('duallink5:workspace:InvalidGridSize', ...
        'gridSize must contain two integers greater than or equal to 2.');
end
result.xEdges=linspace(min(validX),max(validX),gridSize(1)+1);
result.yEdges=linspace(min(validY),max(validY),gridSize(2)+1);
result.xGrid=(result.xEdges(1:end-1)+result.xEdges(2:end))/2;
result.yGrid=(result.yEdges(1:end-1)+result.yEdges(2:end))/2;
[X,Y] = meshgrid(result.xGrid,result.yGrid);
result.insideMask = inShape(shape,X,Y);
dx = result.xEdges(2)-result.xEdges(1);
dy = result.yEdges(2)-result.yEdges(1);
result.maxRectangle = duallink5.workspace.largestRectangleInMask( ...
    result.insideMask,dx,dy);
if result.maxRectangle.areaCells==0
    result.maxRectangle.bounds=[NaN,NaN,NaN,NaN];
else
    rectangleInfo=result.maxRectangle;
    result.maxRectangle.bounds=[ ...
        result.xEdges(rectangleInfo.left), ...
        result.xEdges(rectangleInfo.right+1), ...
        result.yEdges(rectangleInfo.top), ...
        result.yEdges(rectangleInfo.bottom+1)];
end
statusCodes = unique(samples.reasonMap(:));
counts = arrayfun(@(code)nnz(samples.reasonMap(:)==code),statusCodes);
result.reasonCounts = table(statusCodes,counts, ...
    'VariableNames',{'statusCode','count'});
result.nearSingularCount=nnz(samples.reasonMap=="NEAR_SINGULAR");
if isfield(samples,'conditionNumber')
    finiteConditions=samples.conditionNumber(isfinite(samples.conditionNumber));
else
    finiteConditions=[];
end
result.singularity.nearCount=result.nearSingularCount;
if isempty(finiteConditions)
    result.singularity.maxCondition=NaN;
    result.singularity.medianCondition=NaN;
else
    result.singularity.maxCondition=max(finiteConditions);
    result.singularity.medianCondition=median(finiteConditions);
end
result.metadata = samples.metadata;
end

function value = getOption(options,name,defaultValue)
if isfield(options,name), value=options.(name); else, value=defaultValue; end
end
```

- [ ] **Step 6: Run workspace tests**

Run the Step 2 command again.

Expected: 5 tests pass, including zero-area handling, the fixed-mechanism area/rectangle regression, and the square integration fixture.

- [ ] **Step 7: Commit workspace analysis**

```powershell
git add src/+duallink5/+workspace tests/workspace/TestWorkspace.m
git commit -m "feat: analyze fixed-geometry workspace"
```

---

### Task 9: Side-effect-free visualization and runnable examples

**Files:**
- Create: `src/+duallink5/+viz/plotMechanism.m`
- Create: `src/+duallink5/+viz/plotWorkspaceResult.m`
- Create: `src/+duallink5/+viz/exportFigure.m`
- Create: `examples/plot_fixed_pose.m`
- Create: `examples/plot_continuous_trajectory.m`
- Create: `examples/analyze_fixed_workspace.m`
- Create: `tests/viz/TestVisualization.m`

- [ ] **Step 1: Write the failing visualization test**

Create `tests/viz/TestVisualization.m`:

```matlab
classdef TestVisualization < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(~)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot,'startup.m'));
        end
    end

    methods (Test)
        function plotUsesProvidedAxesWithoutChangingRootDefaults(testCase)
            g = duallink5.model.defaultGeometry();
            pose = duallink5.kinematics.forwardFiveBar(deg2rad([85,52.93]),g);
            originalFont = get(groot,'defaultAxesFontName');
            figureHandle = figure('Visible','off');
            cleanup = onCleanup(@() close(figureHandle));
            axesHandle = axes(figureHandle);

            handles = duallink5.viz.plotMechanism(pose,axesHandle,struct());

            testCase.verifyNotEmpty(handles.links);
            testCase.verifyEqual(get(groot,'defaultAxesFontName'),originalFont);
            testCase.verifyTrue(all([handles.links.Parent]==axesHandle));
            clear cleanup
        end

        function workspacePlotAndExporterUseExplicitHandles(testCase)
            samples.x=[0,1;0,1];
            samples.y=[0,0;1,1];
            samples.validMask=true(2);
            samples.reasonMap=repmat("OK",2);
            samples.metadata=struct('units', ...
                struct('length',"m",'angle',"rad"));
            result=duallink5.workspace.analyzeWorkspace( ...
                samples,struct('alpha',Inf,'gridSize',[10,10]));
            figureHandle=figure('Visible','off');
            figureCleanup=onCleanup(@()close(figureHandle));
            axesHandle=axes(figureHandle);
            outputFile=[tempname,'.png'];
            fileCleanup=onCleanup(@()deleteIfExists(outputFile));

            handles=duallink5.viz.plotWorkspaceResult( ...
                samples,result,axesHandle);
            duallink5.viz.exportFigure( ...
                figureHandle,outputFile,struct('resolution',100));

            testCase.verifyEqual(handles.boundary.Parent,axesHandle);
            testCase.verifyTrue(isfile(outputFile));
            clear fileCleanup figureCleanup
        end
    end
end

function deleteIfExists(fileName)
if isfile(fileName), delete(fileName); end
end
```

- [ ] **Step 2: Run visualization test and verify failure**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); results=runtests('tests/viz/TestVisualization.m'); assert(all([results.Passed]))"
```

Expected: FAIL because `plotMechanism` is undefined.

- [ ] **Step 3: Implement axes-scoped plotting**

Create `src/+duallink5/+viz/plotMechanism.m`:

```matlab
function handles = plotMechanism(pose,axesHandle,options)
if nargin<3, options=struct(); end
lineWidth = getOption(options,'lineWidth',2);
showLabels = getOption(options,'showLabels',true);
p = pose.points;

holdState = ishold(axesHandle);
hold(axesHandle,'on');
segments = {p.A,p.E; p.B,p.C; p.C,p.D; p.E,p.D; p.A,p.B};
handles.links = gobjects(5,1);
for index=1:5
    startPoint=segments{index,1};
    endPoint=segments{index,2};
    handles.links(index)=plot(axesHandle, ...
        [startPoint(1),endPoint(1)],[startPoint(2),endPoint(2)], ...
        '-o','LineWidth',lineWidth);
end
handles.sharedCenter = plot(axesHandle,pose.sharedLink.center(1), ...
    pose.sharedLink.center(2),'kp','MarkerFaceColor','y');

if showLabels
    names={'A','B','C','D','E','G'};
    handles.labels=gobjects(numel(names),1);
    for index=1:numel(names)
        point=p.(names{index});
        handles.labels(index)=text(axesHandle,point(1),point(2), ...
            ['  ',names{index}],'Interpreter','none');
    end
end
axis(axesHandle,'equal');
xlabel(axesHandle,'x [m]'); ylabel(axesHandle,'y [m]');
if ~holdState, hold(axesHandle,'off'); end
end

function value=getOption(options,name,defaultValue)
if isfield(options,name), value=options.(name); else, value=defaultValue; end
end
```

Create `src/+duallink5/+viz/plotWorkspaceResult.m`:

```matlab
function handles = plotWorkspaceResult(samples,result,axesHandle)
holdState=ishold(axesHandle); hold(axesHandle,'on');
handles.samples=scatter(axesHandle,samples.x(samples.validMask), ...
    samples.y(samples.validMask),8,'.');
handles.boundary=plot(result.boundaryShape,'Parent',axesHandle, ...
    'FaceAlpha',0.08,'EdgeColor',[0 0.45 0.74]);
rectangleInfo=result.maxRectangle;
handles.rectangle=gobjects(0);
if rectangleInfo.areaCells>0
    bounds=rectangleInfo.bounds;
    handles.rectangle=rectangle(axesHandle,'Position',[bounds(1),bounds(3), ...
        bounds(2)-bounds(1),bounds(4)-bounds(3)], ...
        'EdgeColor','r','LineWidth',2);
end
axis(axesHandle,'equal'); xlabel(axesHandle,'x [m]'); ylabel(axesHandle,'y [m]');
if ~holdState, hold(axesHandle,'off'); end
end
```

Create `src/+duallink5/+viz/exportFigure.m`:

```matlab
function exportFigure(figureHandle,fileName,options)
if nargin<3, options=struct(); end
resolution=getOption(options,'resolution',300);
exportgraphics(figureHandle,fileName,'Resolution',resolution);
end

function value=getOption(options,name,defaultValue)
if isfield(options,name), value=options.(name); else, value=defaultValue; end
end
```

- [ ] **Step 4: Add examples that run from any current directory**

Create `examples/plot_fixed_pose.m`:

```matlab
exampleDir=fileparts(mfilename('fullpath'));
projectRoot=fileparts(exampleDir);
run(fullfile(projectRoot,'startup.m'));

geometry=duallink5.model.defaultGeometry();
pose=duallink5.kinematics.forwardFiveBar(deg2rad([85,52.93]),geometry);
figureHandle=figure;
axesHandle=axes(figureHandle);
duallink5.viz.plotMechanism(pose,axesHandle,struct('showLabels',true));
title(axesHandle,'DualLink5 fixed pose');
```

Create `examples/plot_continuous_trajectory.m`:

```matlab
exampleDir=fileparts(mfilename('fullpath'));
projectRoot=fileparts(exampleDir);
run(fullfile(projectRoot,'startup.m'));

geometry=duallink5.model.defaultGeometry();
theta=linspace(deg2rad(80),deg2rad(90),80);
phi=linspace(deg2rad(48),deg2rad(58),80);
trajectory=nan(2,numel(theta));
previousPose=[];
for index=1:numel(theta)
    if isempty(previousPose)
        options=struct('collisionProfile',"none");
    else
        options=struct('branchMode',"continuous", ...
            'previousPose',previousPose,'maxContinuityCost',5e-3, ...
            'collisionProfile',"none");
    end
    pose=duallink5.kinematics.forwardFiveBar( ...
        [theta(index),phi(index)],geometry,options);
    if ~pose.quality.valid
        error('duallink5:example:InvalidTrajectory', ...
            'Trajectory failed at sample %d: %s.', ...
            index,pose.quality.statusCode);
    end
    trajectory(:,index)=pose.points.G;
    previousPose=pose;
end

figureHandle=figure;
axesHandle=axes(figureHandle);
plot(axesHandle,trajectory(1,:),trajectory(2,:),'LineWidth',1.5);
hold(axesHandle,'on');
duallink5.viz.plotMechanism(previousPose,axesHandle, ...
    struct('showLabels',false));
title(axesHandle,'Continuous branch trajectory');
```

Create `examples/analyze_fixed_workspace.m`:

```matlab
exampleDir=fileparts(mfilename('fullpath'));
projectRoot=fileparts(exampleDir);
run(fullfile(projectRoot,'startup.m'));

geometry=duallink5.model.defaultGeometry();
grid.theta=linspace(geometry.analysis.thetaRange(1),geometry.analysis.thetaRange(2),121);
grid.phi=linspace(geometry.analysis.phiRange(1),geometry.analysis.phiRange(2),121);
taskSpec=struct('kind',"pointG",'includeOrientation',false);
samples=duallink5.workspace.sampleWorkspace(grid,geometry,taskSpec,struct());
result=duallink5.workspace.analyzeWorkspace(samples,struct('gridSize',[180,180]));
figureHandle=figure;
axesHandle=axes(figureHandle);
duallink5.viz.plotWorkspaceResult(samples,result,axesHandle);
title(axesHandle,sprintf('Workspace area %.6f m^2',result.area));
```

- [ ] **Step 5: Run visualization tests and all examples headlessly from another current directory**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "projectRoot=pwd; run(fullfile(projectRoot,'startup.m')); results=runtests(fullfile(projectRoot,'tests','viz','TestVisualization.m')); assert(all([results.Passed])); set(groot,'defaultFigureVisible','off'); previousDir=pwd; cd(tempdir); run(fullfile(projectRoot,'examples','plot_fixed_pose.m')); close all; run(fullfile(projectRoot,'examples','plot_continuous_trajectory.m')); close all; run(fullfile(projectRoot,'examples','analyze_fixed_workspace.m')); close all; cd(previousDir)"
```

Expected: both visualization tests pass and all three examples exit without error while `pwd` is outside the project.

- [ ] **Step 6: Commit visualization and examples**

```powershell
git add src/+duallink5/+viz examples tests/viz/TestVisualization.m
git commit -m "feat: add side-effect-free kinematics visualization"
```

---

### Task 10: Migrate experiment trajectory callers to SI kinematics

**Files:**
- Create: `Experiment/+duallink5exp/computeGTrajectory.m`
- Create: `tests/experiment/TestComputeGTrajectory.m`
- Modify: `Experiment/plotModeledG.m:1-45,106-124`
- Modify: `Experiment/exp0612.m:1-10,215-242`
- Modify: `Experiment/exp0618.m:1-10,218-245`
- Modify: `Experiment/exp0417.m:1-2,36-51`
- Modify: `Experiment/exp0514.m:1-4,135`

- [ ] **Step 1: Write the failing experiment-adapter test**

Create `tests/experiment/TestComputeGTrajectory.m`:

```matlab
classdef TestComputeGTrajectory < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addPaths(~)
            testDir=fileparts(mfilename('fullpath'));
            projectRoot=fileparts(fileparts(testDir));
            run(fullfile(projectRoot,'startup.m'));
            addpath(fullfile(projectRoot,'Experiment'));
        end
    end

    methods (Test)
        function convertsDegreeStreamsAndPreservesInvalidRows(testCase)
            geometry=duallink5.model.defaultGeometry();
            theta=[85;NaN]; phi=[52.93;50];
            result=duallink5exp.computeGTrajectory(theta,phi,geometry);

            testCase.verifyEqual(result.x(1),9.604250744006e-3,'AbsTol',1e-11);
            testCase.verifyEqual(result.y(1),185.384718125293e-3,'AbsTol',1e-11);
            testCase.verifyTrue(isnan(result.x(2)));
            testCase.verifyEqual(result.status(2),"NONFINITE_INPUT");
        end
    end
end
```

- [ ] **Step 2: Run adapter test and verify failure**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); addpath('Experiment'); results=runtests('tests/experiment/TestComputeGTrajectory.m'); assert(all([results.Passed]))"
```

Expected: FAIL because `duallink5exp.computeGTrajectory` is undefined.

- [ ] **Step 3: Implement the explicit degree-to-SI adapter**

Create `Experiment/+duallink5exp/computeGTrajectory.m`:

```matlab
function result = computeGTrajectory(thetaDeg,phiDeg,geometry)
% Inputs are calibrated logical theta/phi angles in degrees. This adapter
% deliberately does not infer device channels, signs, zero offsets, or gearing.
thetaLogicalDeg=thetaDeg(:);
phiLogicalDeg=phiDeg(:);
if numel(thetaLogicalDeg)~=numel(phiLogicalDeg)
    error('duallink5exp:SizeMismatch', ...
        'thetaDeg and phiDeg must have the same length.');
end

result.x=nan(size(thetaLogicalDeg));
result.y=nan(size(phiLogicalDeg));
result.status=strings(size(thetaLogicalDeg));
for index=1:numel(thetaLogicalDeg)
    if ~isfinite(thetaLogicalDeg(index)) || ~isfinite(phiLogicalDeg(index))
        result.status(index)="NONFINITE_INPUT";
        continue
    end
    q=deg2rad([thetaLogicalDeg(index),phiLogicalDeg(index)]);
    pose=duallink5.kinematics.forwardFiveBar(q,geometry);
    result.status(index)=pose.quality.statusCode;
    if pose.quality.valid
        result.x(index)=pose.points.G(1);
        result.y(index)=pose.points.G(2);
    end
end
result.units.position="m";
result.units.inputAngle="deg";
end
```

- [ ] **Step 4: Replace script bootstrapping and local FK loops**

At the top of each of `Experiment/plotModeledG.m`, `Experiment/exp0612.m`, and `Experiment/exp0618.m`, replace the `addpath('../...')` and `run('../Definition/set_parameter.m')` block with:

```matlab
experimentDir=fileparts(mfilename('fullpath'));
projectRoot=fileparts(experimentDir);
run(fullfile(projectRoot,'startup.m'));
addpath(experimentDir);
geometry=duallink5.model.defaultGeometry();
```

In both `Experiment/exp0612.m` and `Experiment/exp0618.m`, replace the two local-helper calls with:

```matlab
% Historical 2026-06 logs are preserved with their existing assumption:
% channel 1 is logical theta, channel 2 is logical phi, both already in deg.
% Future logs must replace these aliases with a versioned sign/zero/gear
% calibration before calling computeGTrajectory.
thetaMtLogicalDeg=mt_angle1_f;
phiMtLogicalDeg=mt_angle2_f;
thetaRsLogicalDeg=angle_data.rs_id1_deg;
phiRsLogicalDeg=angle_data.rs_id2_deg;

trajectoryMt=duallink5exp.computeGTrajectory( ...
    thetaMtLogicalDeg,phiMtLogicalDeg,geometry);
x_g_mt=trajectoryMt.x;
y_g_mt=trajectoryMt.y;

trajectoryRs=duallink5exp.computeGTrajectory( ...
    thetaRsLogicalDeg,phiRsLogicalDeg,geometry);
x_g_rs=trajectoryRs.x;
y_g_rs=trajectoryRs.y;
```

In `Experiment/plotModeledG.m`, replace its two calls with:

```matlab
% extractExperimentAngles has already applied this legacy file's historical
% channel transforms; name the resulting logical convention explicitly.
thetaEncoderLogicalDeg=angle1;
phiEncoderLogicalDeg=angle2;
thetaMotorLogicalDeg=ft_pos1_interp;
phiMotorLogicalDeg=ft_pos2_interp;

trajectoryAngle=duallink5exp.computeGTrajectory( ...
    thetaEncoderLogicalDeg,phiEncoderLogicalDeg,geometry);
x_G_angle=trajectoryAngle.x;
y_G_angle=trajectoryAngle.y;

trajectoryFt=duallink5exp.computeGTrajectory( ...
    thetaMotorLogicalDeg,phiMotorLogicalDeg,geometry);
x_G_ft=trajectoryFt.x;
y_G_ft=trajectoryFt.y;
```

Remove the three duplicated local `computeGTrajectory` functions after all six call sites use the package helper.

In `Experiment/plotModeledG.m`, replace both boundary-extrapolating interpolations with:

```matlab
ft_pos1_interp=interp1( ...
    t_sec(data.idx_id1),ft_pos1(data.idx_id1),t_sec,'linear',NaN);
ft_pos2_interp=interp1( ...
    t_sec(data.idx_id2),ft_pos2(data.idx_id2),t_sec,'linear',NaN);
```

Keep experiment calculations in SI. In `readOpticalRecord` for `exp0612.m`, convert the historical millimetre coordinates after applying the existing offsets:

```matlab
optical.x=(T.("aimooe_coord_position.x")-40)*1e-3;
optical.y=(T.("aimooe_coord_position.y")-30)*1e-3;
optical.z=T.("aimooe_coord_position.z")*1e-3;
optical.units.position="m";
```

Use the same conversion in `exp0618.m`, preserving its actual x offset of `36 mm`:

```matlab
optical.x=(T.("aimooe_coord_position.x")-36)*1e-3;
optical.y=(T.("aimooe_coord_position.y")-30)*1e-3;
optical.z=T.("aimooe_coord_position.z")*1e-3;
optical.units.position="m";
```

`calcTrajectoryError` therefore returns metres. Convert only at presentation boundaries:

```matlab
fprintf('%s error: mean=%.3f mm, rmse=%.3f mm, max=%.3f mm, n=%d\n', ...
    label,1e3*stats.mean,1e3*stats.rmse,1e3*stats.max,stats.n);

fprintf('First optical point after offset: x = %.4f mm, y = %.4f mm\n', ...
    1e3*first_optical.x(1),1e3*first_optical.y(1));
```

In both `plotComparison` functions, pass `1e3*optical.x`, `1e3*x_g_mt`, and `1e3*x_g_rs` (and the corresponding y arrays) to `plotValid`, keeping the axes labelled in millimetres. Correct the `exp0618` legend to `optical x-36, y-30`. In `plotModeledG.m`, likewise pass `1e3*x_G_angle`, `1e3*y_G_angle`, `1e3*x_G_ft`, and `1e3*y_G_ft` to `plotTimeGradientLine`.

Replace the three remaining `saveFigIEEE` calls with the new explicit exporter:

```matlab
% exp0612.m and exp0618.m
duallink5.viz.exportFigure(fig,fullfile(output_dir,output_name), ...
    struct('resolution',300));

% plotModeledG.m, after assigning fig=figure('Color','w')
duallink5.viz.exportFigure(fig,fullfile(experimentDir,'6.png'), ...
    struct('resolution',300));
```

Also make data roots independent of `pwd`: use `fullfile(experimentDir,'实验数据','0612')` and `fullfile(experimentDir,'实验数据','0618')`; in `plotModeledG.m`, resolve its active file under `fullfile(experimentDir,'实验数据','0529',...)`. Set `exp0618` to the three pairs actually present by trimming the label/title arrays to three entries and assigning `expected_pairs=numel(experiment_labels)`.

Remove the final dependencies on the soon-to-be-deleted `Plot/saveFigIEEE.m` outside those three scripts. In `Experiment/exp0417.m`, replace the relative Plot path, capture the created figure, and export explicitly:

```matlab
experimentDir=fileparts(mfilename('fullpath'));
projectRoot=fileparts(experimentDir);
run(fullfile(projectRoot,'startup.m'));
% ...
figureHandle=figure;
% existing plotting calls
duallink5.viz.exportFigure(figureHandle, ...
    fullfile(experimentDir,'4.png'),struct('resolution',300));
```

In `Experiment/exp0514.m`, remove `addpath('../Plot')` and the stale commented `saveFigIEEE` line; that script has no active export call to replace.

- [ ] **Step 5: Run adapter and script parse checks**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); addpath('Experiment'); results=runtests('tests/experiment/TestComputeGTrajectory.m'); assert(all([results.Passed])); files={'Experiment/plotModeledG.m','Experiment/exp0612.m','Experiment/exp0618.m','Experiment/exp0417.m','Experiment/exp0514.m'}; total=0; for k=1:numel(files), messages=checkcode(files{k},'-id'); total=total+numel(messages); for j=1:numel(messages), fprintf('%s:%d %s %s\n',files{k},messages(j).line,messages(j).id,messages(j).message); end; end; assert(total==0,'Experiment Code Analyzer reported messages')"
```

Expected: adapter test passes and Code Analyzer reports zero messages for all five migrated experiment scripts.

- [ ] **Step 6: Confirm old API calls are gone from active experiment scripts**

Run:

```powershell
$matches=rg -n --glob '*.m' "modelingfx|set_parameter|run\('\.\./Definition|addpath\('\.\./Plot|saveFigIEEE" Experiment
if ($LASTEXITCODE -eq 0) { $matches; throw 'Legacy experiment API remains' }
if ($LASTEXITCODE -ne 1) { throw 'rg failed unexpectedly' }
```

Expected: no matches.

- [ ] **Step 7: Commit the experiment migration**

```powershell
git add Experiment/+duallink5exp Experiment/plotModeledG.m Experiment/exp0612.m Experiment/exp0618.m Experiment/exp0417.m Experiment/exp0514.m tests/experiment/TestComputeGTrajectory.m
git commit -m "refactor: migrate experiments to SI kinematics API"
```

---

### Task 11: Remove legacy implementations and length-optimization assets

**Files:**
- Create: `docs/kinematics-conventions.md`
- Modify: `.gitignore`
- Delete: legacy files listed below

- [ ] **Step 1: Write the permanent conventions document**

Create `docs/kinematics-conventions.md` with this content:

```markdown
# DualLink5 kinematics conventions

## Main links

| Link | Endpoints | Length |
|---|---|---:|
| link1 | AE | 80e-3 m |
| link2 | BC | 62e-3 m |
| link3 | CD | 69e-3 m |
| link4 | DE | 80e-3 m |
| link5 | AB | 80e-3 m |

The local frame places A at the origin, +x from A to B, and +y toward the
interior of that five-bar. theta rotates A→B toward A→E. phi rotates B→A
toward B→C. Inputs are radians and positions are metres.

The upper mechanism uses the same local convention and is transformed by a
pi rotation into the assembly frame. Ideal symmetry therefore means
q.upper == q.lower.

Parallel linkage distances are named by endpoints:
E_Palpha1=30e-3 m, Palpha1_Palpha4=60e-3 m,
D_Pbeta2=30e-3 m, and Pbeta2_Pbeta3=60e-3 m.

## Experiment boundary

The kinematics core accepts only logical, calibrated angles in radians and
returns positions in metres. `duallink5exp.computeGTrajectory` is a legacy
boundary adapter: its inputs must already be calibrated logical theta/phi in
degrees, and its outputs remain metres. Device channel selection, sign, zero,
gear ratio, encoder wrapping, time synchronization, and optical extrinsics
belong to the experiment/calibration layer. Historical optical millimetres are
converted to metres on import; conversion back to millimetres is display-only.
```

- [ ] **Step 2: Delete the approved optimization and duplicate assets**

Run this exact PowerShell command from the project root:

```powershell
$targets = @(
  'Plot/optimize_two_links.m',
  'Plot/optimize_two_links.asv',
  'Plot/normalizeScore.m',
  'Plot/plotMetricMap.m',
  'Kinematics/findBestLength.m',
  'Kinematics/normalizeMap.m',
  'Plot/two_link_optimization_result.mat',
  'Plot/two_link_optimization_l_drives_l_driven.mat',
  'Plot/two_link_optimization_l_base_l_drives.mat',
  'Plot/normalized_score_l_drives_l_driven.mat',
  'Plot/normalized_score_l_base_l_drives.mat',
  'Kinematics/two_link_optimization_result.mat',
  'Plot/工作空间.fig',
  'Plot/最大矩形.fig',
  'Plot/面积比.fig',
  'Plot/综合指标.fig',
  'Plot/工作空间.png',
  'Plot/最大矩形.png',
  'Plot/calArea.mlx',
  'Plot/workspace.mlx'
)
$resolvedRoot = (Resolve-Path '.').Path
if ((Split-Path $resolvedRoot -Leaf) -ne 'Matlab' -or -not (Test-Path -LiteralPath (Join-Path $resolvedRoot '.git'))) {
  throw 'Deletion guard: run only from the intended Matlab repository root'
}
foreach ($target in $targets) {
  $absolute = [IO.Path]::GetFullPath((Join-Path $resolvedRoot $target))
  $relative = [IO.Path]::GetRelativePath($resolvedRoot,$absolute)
  if ($relative -eq '..' -or $relative.StartsWith("..$([IO.Path]::DirectorySeparatorChar)")) {
    throw "Deletion target escaped project root: $absolute"
  }
  if (Test-Path -LiteralPath $absolute) {
    Remove-Item -LiteralPath $absolute -Force
  }
}
```

Before execution, verify `$resolvedRoot` equals the intended `Matlab` project root. These are the only generated/optimization targets removed in this step.

- [ ] **Step 3: Delete replaced legacy source files**

After Task 10 proves no active caller uses them, delete:

```powershell
$targets = @(
  'Definition/set_parameter.m',
  'Kinematics/modeling.m',
  'Kinematics/modelingfx.m',
  'Kinematics/computeAllPoints.m',
  'Kinematics/inverse_kinematics_from_P.m',
  'Kinematics/inversekinematic.m',
  'Kinematics/isValidPose.m',
  'Kinematics/check_interference.m',
  'Kinematics/calcPalphaPbetaFromED.m',
  'Kinematics/evaluateWorkspaceMetrics.m',
  'Kinematics/calcMaxWorkspaceRectangle.m',
  'Plot/largestRectangleInMask.m',
  'Plot/plotWorkspace.m',
  'Plot/plot_mech.m',
  'Plot/saveFigIEEE.m',
  'Plot/workspacerect.m'
)
$resolvedRoot = (Resolve-Path '.').Path
if ((Split-Path $resolvedRoot -Leaf) -ne 'Matlab' -or -not (Test-Path -LiteralPath (Join-Path $resolvedRoot '.git'))) {
  throw 'Deletion guard: run only from the intended Matlab repository root'
}
foreach ($target in $targets) {
  $absolute = [IO.Path]::GetFullPath((Join-Path $resolvedRoot $target))
  $relative = [IO.Path]::GetRelativePath($resolvedRoot,$absolute)
  if ($relative -eq '..' -or $relative.StartsWith("..$([IO.Path]::DirectorySeparatorChar)")) {
    throw "Deletion target escaped project root: $absolute"
  }
  if (Test-Path -LiteralPath $absolute) {
    Remove-Item -LiteralPath $absolute -Force
  }
}
```

Do not delete `Kinematics/公式.pptx`, `Kinematics/Figure.png`, `Plot/1.png`, or anything under `Experiment/实验数据/`.

- [ ] **Step 4: Simplify `.gitignore` after generated files are removed**

Keep the rules that protect local state and raw experiments:

```gitignore
.superpowers/
*.asv
Experiment/实验数据/
```

Remove ignore patterns that named optimization outputs once those files no longer exist.

- [ ] **Step 5: Verify no old symbol remains in active MATLAB code**

Run:

```powershell
$matches=rg -n --glob '*.m' --glob '*.mlx' "l_base|l_drivel|l_drives|l_driven|l_trans|modelingfx|set_parameter|check_interference|isValidPose|findBestLength|normalizeMap|saveFigIEEE" src Experiment examples tests startup.m
if ($LASTEXITCODE -eq 0) { $matches; throw 'Legacy symbol remains' }
if ($LASTEXITCODE -ne 1) { throw 'rg failed unexpectedly' }
```

Expected: no matches in active source, tests, examples, or experiment scripts. Matches in the approved design and implementation plan are acceptable because those are historical documentation.

- [ ] **Step 6: Commit cleanup and permanent documentation**

```powershell
git add docs/kinematics-conventions.md .gitignore
git add -u -- Definition Kinematics Plot
git diff --cached --name-status
git commit -m "refactor: remove legacy kinematics and optimization assets"
```

---

### Task 12: Full verification and final refactor audit

**Files:**
- Modify only if verification exposes a specific defect in earlier tasks

- [ ] **Step 1: Run the complete MATLAB test suite**

Run:

```powershell
$preflight=@(git status --porcelain)
if ($preflight.Count -gt 0) { $preflight; throw 'Task 12 must start clean' }
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "run('startup.m'); addpath('Experiment'); results=runtests('tests','IncludeSubfolders',true); disp(results); assert(all([results.Passed]),'Test suite failed'); assert(~any([results.Incomplete]),'Test suite contains incomplete tests')"
```

Expected: every test passes with 0 failures and 0 incomplete tests.

- [ ] **Step 2: Run Code Analyzer on every active source file**

Run:

```powershell
& 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "files=[dir(fullfile('src','**','*.m'));dir(fullfile('Experiment','+duallink5exp','*.m'));dir(fullfile('examples','*.m'));dir('startup.m');dir(fullfile('Experiment','plotModeledG.m'));dir(fullfile('Experiment','exp0612.m'));dir(fullfile('Experiment','exp0618.m'));dir(fullfile('Experiment','exp0417.m'));dir(fullfile('Experiment','exp0514.m'))]; total=0; for k=1:numel(files), messages=checkcode(fullfile(files(k).folder,files(k).name),'-id'); total=total+numel(messages); for j=1:numel(messages), fprintf('%s:%d %s %s\n',files(k).name,messages(j).line,messages(j).id,messages(j).message); end; end; assert(total==0,'Code Analyzer reported messages')"
```

Expected: command exits 0 and prints no Code Analyzer messages.

- [ ] **Step 3: Verify examples from an unrelated current directory**

Run:

```powershell
$project = (Resolve-Path '.').Path
Push-Location $env:TEMP
try {
  & 'D:\Program Files\MATLAB\R2025a\bin\matlab.exe' -batch "set(groot,'defaultFigureVisible','off'); run(fullfile('$($project.Replace("'","''"))','examples','plot_fixed_pose.m')); close all; run(fullfile('$($project.Replace("'","''"))','examples','plot_continuous_trajectory.m')); close all; run(fullfile('$($project.Replace("'","''"))','examples','analyze_fixed_workspace.m')); close all"
  if ($LASTEXITCODE -ne 0) { throw 'Example verification failed' }
} finally {
  Pop-Location
}
```

Expected: all three examples exit 0 without depending on MATLAB's initial folder.

- [ ] **Step 4: Run repository-wide legacy and unit scans**

Run:

```powershell
$legacy=rg -n --glob '*.m' --glob '*.mlx' "run\('\.\./|addpath\('\.\./|modelingfx|set_parameter|saveFigIEEE|l_drivel|l_drives|l_driven|l_trans|l_base" src Experiment examples tests startup.m
$legacyExit=$LASTEXITCODE
if ($legacyExit -eq 0) { $legacy; throw 'Legacy API or symbol remains' }
if ($legacyExit -ne 1) { throw 'Legacy rg scan failed unexpectedly' }

$degreeTrig=rg -n --glob '*.m' "cosd|sind|acosd|atan2d" src
$degreeExit=$LASTEXITCODE
if ($degreeExit -eq 0) { $degreeTrig; throw 'Degree trigonometry remains in SI core' }
if ($degreeExit -ne 1) { throw 'Unit rg scan failed unexpectedly' }
```

Expected: no matches. The new core uses explicit paths, SI lengths, and radian trigonometry.

- [ ] **Step 5: Commit a verification correction only when Steps 1–4 exposed one**

If verification required a concrete correction, inspect every changed path, reject generated/raw-data changes, stage the reviewed correction, commit it, and then restart Task 12 from Step 1:

```powershell
$tracked=@(git diff --name-only)
$untracked=@(git ls-files --others --exclude-standard)
$verifiedCorrections=@(($tracked+$untracked) | Sort-Object -Unique)
if ($verifiedCorrections.Count -gt 0) {
  $verifiedCorrections
  if ($verifiedCorrections -match '^Experiment/实验数据/') {
    throw 'Raw experiment data must not be staged'
  }
  git add -- $verifiedCorrections
  git diff --cached --check
  if ($LASTEXITCODE -ne 0) { throw 'Staged correction failed diff check' }
  git commit -m "fix: resolve final kinematics verification finding"
  Write-Host 'Correction committed: rerun Task 12 Steps 1-4 now.'
}
```

If no correction was required, do not create an empty commit. Any correction commit invalidates the earlier evidence, so Steps 1–4 must run again before Step 6.

- [ ] **Step 6: Assert a clean repository and review the complete committed diff**

Run only after the latest execution of Steps 1–4 passes:

```powershell
git diff --check 57e6b24..HEAD
if ($LASTEXITCODE -ne 0) { throw 'Committed diff has whitespace errors' }
$finalStatus=@(git status --porcelain)
if ($finalStatus.Count -gt 0) { $finalStatus; throw 'Worktree is not clean' }
$rawChanges=@(git diff --name-only 57e6b24..HEAD | Where-Object { $_ -like 'Experiment/实验数据/*' })
if ($rawChanges.Count -gt 0) { $rawChanges; throw 'Raw data entered the diff' }
git diff 57e6b24..HEAD --stat
git log --oneline --decorate 57e6b24..HEAD
```

Expected: clean worktree, no whitespace errors, one focused commit per task, and no raw experiment data in the committed diff.

---

## Execution checkpoints

After Tasks 3, 7, 10, and 12, pause for a focused review:

1. **Task 3:** Verify the new SI FK matches the trusted legacy pose and link constraints.
2. **Task 7:** Inspect every IK candidate and its FK residual; reject any unvalidated branch.
3. **Task 10:** Confirm experiment angle channels still map to logical theta/phi with explicit degree conversion.
4. **Task 12:** Review tests, analyzer output, examples, deletion list, and Git diff before declaring completion.
