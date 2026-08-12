# DualLink5 Doublet Coupling Singularity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a complete rigid passive-doublet kinematic solver and use it to detect, sample, and visualize the upper-loop internal-tangency coupling singularity observed near `phi = 0 deg` under single-sided actuation.

**Architecture:** Keep the existing ideal symmetric `forwardAssembly` and single-five-bar singularity APIs unchanged. Add a separate passive-doublet forward solver that constructs the upper circle centers from the two physical parallelograms and solves `G_k` as a real two-circle closure; build SVD-based coupling metrics, a cached-reference grid sampler, a dedicated visualization, facade wrappers, and one runnable analysis script on top of that solver.

**Tech Stack:** MATLAB R2025a, package folders (`+duallink5`), `matlab.unittest`, SVD, existing closure/topology/collision utilities, Git.

---

## File structure

### Production files

- Modify `src/+duallink5/+model/defaultGeometry.m`: add the configurable `10 deg` coupling warning angle.
- Modify `src/+duallink5/+model/validateGeometry.m`: validate the new positive finite angle in `(0, pi/2)`.
- Create `src/+duallink5/+kinematics/forwardPassiveDoublet.m`: solve the complete single-sided passive doublet, retain exact tangency, select a stable upper closure branch, and report geometric residuals.
- Create `src/+duallink5/+singularity/evaluateCoupling.m`: compute `A_G`, `B_G`, SVD metrics, classification, and mechanical assembly status for one `qLower`.
- Create `src/+duallink5/+singularity/private/normalizeCouplingOptions.m`: validate shared coupling analysis options and prepare stable defaults.
- Create `src/+duallink5/+singularity/private/prepareCouplingReference.m`: solve and cache the approved reference assembly and its passive orientation sign/topology signature.
- Create `src/+duallink5/+singularity/private/assessCouplingAssembly.m`: keep geometry, collision, passive-orientation topology, and warning states separate.
- Create `src/+duallink5/+singularity/sampleCouplingSpace.m`: sample joint space without recomputing the reference inside the loop.
- Create `src/+duallink5/+viz/plotCouplingSingularitySpace.m`: render joint space, task space, and three complete representative mechanisms.
- Modify `src/+duallink5/DL5.m`: add two read-only facade methods.
- Create `scripts/analyze_coupling_singularity.m`: run the approved analysis and create one five-panel figure.
- Modify `docs/kinematics-conventions.md`: document passive-doublet point identity, closure, branch, and metric conventions.

### Test files

- Modify `tests/model/TestGeometry.m`: cover the new geometry setting and invalid values.
- Create `tests/kinematics/TestPassiveDoublet.m`: cover reference agreement, exact tangency, negative-side symmetry, branches, residuals, and invalid inputs.
- Create `tests/singularity/TestCouplingMetrics.m`: cover matrices, SVD, finite-difference velocity transmission, exact/near classification, topology, and options.
- Create `tests/singularity/TestCouplingSpace.m`: cover grid contracts, masks, cached reference behavior, positive/negative danger bands, and per-sample failure isolation.
- Create `tests/viz/TestCouplingVisualization.m`: cover plot contracts, axes restoration, layers, labels, and self-motion arrows.
- Modify `tests/facade/TestDL5.m`: prove the new analysis methods do not mutate the facade state.

## Shared contracts

The following names and meanings are fixed across all tasks:

```matlab
result.kinematics                 % forwardPassiveDoublet output
result.coupling.constraintMatrix  % normalized A_G, 2x2
result.coupling.actuationMatrix   % B_G, 2x2, units m/rad
result.coupling.margin            % abs(cross2(uAG,uCG)), [0,1]
result.coupling.singularValues    % descending 2x1 vector
result.coupling.sigmaMin          % singularValues(2)
result.coupling.conditionNumber   % sigmaMax/sigmaMin or Inf
result.coupling.passiveDirection  % unit 2x1 right singular vector
result.exactCouplingSingular      % margin <= exactThreshold
result.nearCouplingSingular       % metric or condition threshold
result.engineeringWarning         % abs(phi) <= warning angle
result.theoreticallyReachable
result.mechanicallyValid
result.statusCode
```

The exact theoretical singularity remains a geometrically reachable result. It is not converted into `UNREACHABLE_CLOSURE` merely because the two upper circle candidates coalesce.

---

### Task 1: Geometry setting and complete passive-doublet forward kinematics

**Files:**
- Modify: `src/+duallink5/+model/defaultGeometry.m`
- Modify: `src/+duallink5/+model/validateGeometry.m`
- Create: `src/+duallink5/+kinematics/forwardPassiveDoublet.m`
- Modify: `tests/model/TestGeometry.m`
- Create: `tests/kinematics/TestPassiveDoublet.m`

- [ ] **Step 1: Add failing geometry-setting tests**

Add these test methods to `TestGeometry`:

```matlab
function couplingWarningAngleUsesApprovedValue(testCase)
    geometry = duallink5.model.defaultGeometry();
    testCase.verifyEqual(geometry.analysis.couplingWarningAngle, ...
        deg2rad(10), 'AbsTol', 1e-15);
end

function invalidCouplingWarningAnglesAreRejected(testCase)
    geometry = duallink5.model.defaultGeometry();
    invalid = {0, -1, pi / 2, Inf, NaN, [0.1, 0.2]};
    for index = 1:numel(invalid)
        candidate = geometry;
        candidate.analysis.couplingWarningAngle = invalid{index};
        testCase.verifyError( ...
            @()duallink5.model.validateGeometry(candidate), ...
            'duallink5:model:InvalidGeometry');
    end
end
```

- [ ] **Step 2: Run the focused geometry RED test**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests('tests/model/TestGeometry.m'); assertSuccess(r);"
```

Expected: the approved-value test fails because `couplingWarningAngle` does not exist.

- [ ] **Step 3: Add and validate the geometry setting**

Add to `defaultGeometry` after `referenceQ`:

```matlab
geometry.analysis.couplingWarningAngle = deg2rad(10);
```

Add to `validateGeometry` after `validateReferenceQ`:

```matlab
validateCouplingWarningAngle(geometry.analysis);
```

Add the local validator:

```matlab
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
```

- [ ] **Step 4: Add failing passive-doublet kinematics tests**

Create `TestPassiveDoublet` with setup using `startup.m`, default geometry, and `collisionProfile="none"`. Add these tests:

```matlab
function referencePoseMatchesIdealAssembly(testCase)
    q = deg2rad([85, 30]);
    passive = duallink5.kinematics.forwardPassiveDoublet( ...
        q, testCase.Geometry, struct('collisionProfile', "none"));
    ideal = duallink5.kinematics.forwardAssembly( ...
        struct('lower', q, 'upper', q), testCase.Geometry, ...
        struct('collisionProfile', "none"));

    testCase.verifyTrue(passive.quality.theoreticallyReachable);
    testCase.verifyEqual(passive.quality.statusCode, "OK");
    for name = ["A", "B", "C", "D", "E"]
        field = char(name);
        testCase.verifyEqual(passive.upper.points.(field), ...
            ideal.upper.points.(field), 'AbsTol', 1e-11);
    end
    testCase.verifyEqual(passive.points.G, ...
        ideal.upper.points.B, 'AbsTol', 1e-11);
    testCase.verifyLessThanOrEqual( ...
        passive.residuals.maximum, 1e-11);
end

function zeroPhiRetainsInternalTangency(testCase)
    q = deg2rad([85, 0]);
    result = duallink5.kinematics.forwardPassiveDoublet( ...
        q, testCase.Geometry, struct('collisionProfile', "none"));

    Aupper = result.upper.points.A;
    Cupper = result.upper.points.C;
    G = result.points.G;
    testCase.verifyTrue(result.quality.theoreticallyReachable);
    testCase.verifyEqual(result.quality.closureStatusCode, ...
        "INTERNAL_TANGENCY");
    testCase.verifyEqual(norm(Aupper - Cupper), 18e-3, ...
        'AbsTol', 1e-12);
    testCase.verifyLessThanOrEqual(abs(cross2( ...
        G - Aupper, G - Cupper)), 1e-14);
    testCase.verifyEqual(result.quality.passiveBranchId, int8(0));
end

function positiveAndNegativePhiHaveSymmetricCouplingGeometry(testCase)
    positive = duallink5.kinematics.forwardPassiveDoublet( ...
        deg2rad([85, 5]), testCase.Geometry, ...
        struct('collisionProfile', "none"));
    negative = duallink5.kinematics.forwardPassiveDoublet( ...
        deg2rad([85, -5]), testCase.Geometry, ...
        struct('collisionProfile', "none"));
    testCase.verifyTrue(positive.quality.theoreticallyReachable);
    testCase.verifyTrue(negative.quality.theoreticallyReachable);
    positiveMargin = passiveMargin(positive);
    negativeMargin = passiveMargin(negative);
    testCase.verifyEqual(positiveMargin, negativeMargin, ...
        'AbsTol', 1e-12);
    testCase.verifyEqual(norm(positive.upper.points.A - ...
        positive.upper.points.C), ...
        norm(negative.upper.points.A - negative.upper.points.C), ...
        'AbsTol', 1e-12);
    testCase.verifyEqual(positive.quality.passiveOrientationSign, ...
        -negative.quality.passiveOrientationSign);
end

function invalidInputsUseStableErrors(testCase)
    testCase.verifyError(@()duallink5.kinematics.forwardPassiveDoublet( ...
        [1; 2], testCase.Geometry), ...
        'duallink5:kinematics:InvalidPassiveDoubletInput');
    testCase.verifyError(@()duallink5.kinematics.forwardPassiveDoublet( ...
        [1, 2], testCase.Geometry, struct('branchMode', "bad")), ...
        'duallink5:kinematics:InvalidPassiveDoubletOptions');
end
```

Define file-local `cross2` and `passiveMargin` helpers in the test; `passiveMargin` normalizes the two vectors from the upper circle centers to `G` and returns their absolute 2-D cross product. Add separate assertions for all four alpha and four beta side-length residuals, rigid-line residuals, branch continuity using `previousPose`, and an unreachable custom geometry status.

- [ ] **Step 5: Run the kinematics RED test**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests('tests/kinematics/TestPassiveDoublet.m'); assert(~all([r.Passed]));"
```

Expected: failure with `MATLAB:undefinedVarOrClass` for `forwardPassiveDoublet`.

- [ ] **Step 6: Implement the complete forward solver**

Implement `forwardPassiveDoublet(qLower, geometry, options)` with this order:

```matlab
geometry = duallink5.model.validateGeometry(geometry);
qLower = validateQ(qLower);
options = validateOptions(options);
lower = duallink5.kinematics.forwardFiveBar(qLower, geometry, ...
    struct('branchMode', options.lowerBranchMode, ...
           'branchId', options.lowerBranchId, ...
           'collisionProfile', options.collisionProfile));
```

If the lower pose is invalid, return a typed invalid result with its status. For a valid lower pose, use the confirmed rigid-parallelogram relations:

```matlab
p = lower.points;
Aupper = p.D + (p.E - p.A);
Cupper = p.E + (p.D - p.C);
```

Solve `G` from circles centered at `Aupper` and `Cupper` with radii `link5` and `link2`. Do not pass the ordinary `geometry.tolerance.length` to a path that discards tangency. Detect inner/outer tangency using:

```matlab
distance = norm(Cupper - Aupper);
targets = [abs(L.link5 - L.link2), L.link5 + L.link2];
tangentTolerance = 32 * eps(max([distance, targets, L.link5, L.link2]));
```

At tangency construct the unique point analytically, set `passiveBranchId=int8(0)`, and retain finite points. Away from tangency return two candidates with branch sign:

```matlab
sign(cross2(Cupper - Aupper, candidateG - Aupper))
```

For fixed mode, select the candidate closest to the lower ideal-symmetry prediction `p.G`; this prediction is only a branch selector and residual check. Record the selected circle-side sign as `passiveOrientationSign`. For continuous mode, select the candidate closest to `options.previousPose.points.G`, enforce a positive `maxContinuityCost`, and report `BRANCH_DISCONTINUITY` if exceeded.

Construct the upper pose explicitly:

```matlab
upper.points.A = Aupper;
upper.points.B = G;
upper.points.C = Cupper;
upper.points.D = p.E;
upper.points.E = p.D;
upper.points.G = p.B;
```

Copy the confirmed physical `Palpha` and `Pbeta` points from the lower pose. Also return the standard assembly fields `lower`, `upper`, and `sharedLink`, so the existing `plotAssembly` can render the result without an adapter. Return residual groups for upper link lengths, alpha/beta parallelogram opposite-side equality and parallelism, rigid-line membership, shared points, and ideal symmetry; set `residuals.maximum` to their maximum absolute finite value.

- [ ] **Step 7: Run Task 1 tests and static analysis**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests({'tests/model/TestGeometry.m','tests/kinematics/TestPassiveDoublet.m'}); assertSuccess(r); files={'src/+duallink5/+model/defaultGeometry.m','src/+duallink5/+model/validateGeometry.m','src/+duallink5/+kinematics/forwardPassiveDoublet.m'}; for k=1:numel(files), assert(isempty(checkcode(files{k},'-id'))); end"
```

Expected: all Task 1 tests pass and all three production files have zero Code Analyzer messages.

- [ ] **Step 8: Commit Task 1 only**

```powershell
git add -- src/+duallink5/+model/defaultGeometry.m src/+duallink5/+model/validateGeometry.m src/+duallink5/+kinematics/forwardPassiveDoublet.m tests/model/TestGeometry.m tests/kinematics/TestPassiveDoublet.m
git commit -m "feat: solve passive doublet closure"
```

---

### Task 2: Single-configuration coupling singularity metrics

**Files:**
- Create: `src/+duallink5/+singularity/evaluateCoupling.m`
- Create: `src/+duallink5/+singularity/private/normalizeCouplingOptions.m`
- Create: `src/+duallink5/+singularity/private/prepareCouplingReference.m`
- Create: `src/+duallink5/+singularity/private/assessCouplingAssembly.m`
- Create: `tests/singularity/TestCouplingMetrics.m`

- [ ] **Step 1: Write failing metric and classification tests**

Create `TestCouplingMetrics` and include:

```matlab
function referenceMetricsMatchIndependentGeometry(testCase)
    q = deg2rad([85, 30]);
    result = duallink5.singularity.evaluateCoupling( ...
        q, testCase.Geometry, testCase.Options);
    A = result.kinematics.upper.points.A;
    C = result.kinematics.upper.points.C;
    G = result.kinematics.points.G;
    uAG = (G - A) / norm(G - A);
    uCG = (G - C) / norm(G - C);
    expectedMatrix = [uAG.'; uCG.'];
    expectedMargin = abs(cross2(uAG, uCG));

    testCase.verifyEqual(result.coupling.constraintMatrix, ...
        expectedMatrix, 'AbsTol', 1e-12);
    testCase.verifyEqual(result.coupling.margin, ...
        expectedMargin, 'AbsTol', 1e-12);
    testCase.verifyFalse(result.exactCouplingSingular);
    testCase.verifyFalse(result.engineeringWarning);
end

function zeroPhiHasPassiveSelfMotion(testCase)
    result = duallink5.singularity.evaluateCoupling( ...
        deg2rad([85, 0]), testCase.Geometry, testCase.Options);
    linkDirection = result.kinematics.points.G - ...
        result.kinematics.upper.points.A;
    linkDirection = linkDirection / norm(linkDirection);

    testCase.verifyTrue(result.theoreticallyReachable);
    testCase.verifyTrue(result.exactCouplingSingular);
    testCase.verifyEqual(result.coupling.sigmaMin, 0, ...
        'AbsTol', 1e-12);
    testCase.verifyEqual(result.coupling.conditionNumber, Inf);
    testCase.verifyLessThanOrEqual(abs(dot( ...
        linkDirection, result.coupling.passiveDirection)), 1e-12);
    testCase.verifyEqual(result.statusCode, "COUPLING_SINGULAR");
end

function warningBandIncludesBothBoundaries(testCase)
    values = [-10, -5, 0, 5, 10];
    for phi = values
        result = duallink5.singularity.evaluateCoupling( ...
            deg2rad([85, phi]), testCase.Geometry, testCase.Options);
        testCase.verifyTrue(result.engineeringWarning);
    end
    outside = duallink5.singularity.evaluateCoupling( ...
        deg2rad([85, 10.01]), testCase.Geometry, testCase.Options);
    testCase.verifyFalse(outside.engineeringWarning);
end
```

Add tests that compare `B_G` and `A_G\B_G` to central differences at `(85 deg,30 deg)`, verify SVD ordering and unit passive direction, distinguish theoretical reachability from mechanical topology mismatch, and assert stable errors for malformed q/options.

- [ ] **Step 2: Run the focused RED test**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests('tests/singularity/TestCouplingMetrics.m'); assert(~all([r.Passed]));"
```

Expected: failure because `evaluateCoupling` is undefined.

- [ ] **Step 3: Implement shared option normalization**

`normalizeCouplingOptions(options, geometry)` must reject non-scalar structs, unsupported fields, invalid thresholds, invalid collision/topology profiles, invalid branch modes, and malformed cached references. Defaults:

```matlab
options.collisionProfile = "centerline";
options.topologyProfile = "reference";
options.exactThreshold = 1e-8;
options.nearMetricThreshold = 0.05;
options.nearConditionThreshold = 100;
options.lowerBranchId = geometry.assembly.defaultBranch;
options.lowerBranchMode = "fixed";
options.referenceQ = geometry.analysis.referenceQ;
options.couplingReference = [];
```

Require `exactThreshold < nearMetricThreshold < 1` and `nearConditionThreshold > 1`. Use error ID `duallink5:singularity:InvalidCouplingOptions` for all public option-contract failures.

- [ ] **Step 4: Implement cached reference preparation and assembly assessment**

`prepareCouplingReference` must solve the reference q with collision disabled, require a regular passive branch, and store:

```matlab
reference.q
reference.passiveOrientationSign
reference.topology = duallink5.validation.prepareAssemblyTopologyReference(...)
reference.geometryVersion
```

`assessCouplingAssembly` must:

1. return theoretical failure status unchanged when closure is unreachable;
2. keep exact coupling singularity geometrically reachable;
3. compare the candidate passive orientation sign with the nonzero reference sign and report `PASSIVE_ASSEMBLY_TOPOLOGY_MISMATCH` when opposite;
4. compare the full assembly topology signature when `topologyProfile="reference"`;
5. run the existing lower five-bar collision check directly; build a collision-only upper pose with finite points and `quality.valid=true`, then run the existing upper five-bar collision check for enabled profiles;
6. return independent booleans and a single precedence-ordered status without erasing the coupling metrics.

At exact tangency `passiveOrientationSign=0`; treat it as the branch-coalescence boundary, not as the opposite branch. Enforce `geometry.analysis.thetaRange` and `phiRange` only in the mechanical layer, so out-of-limit negative-phi samples remain visible in the theoretical layer with `JOINT_LIMIT_VIOLATION` as their mechanical reason.

- [ ] **Step 5: Implement `evaluateCoupling`**

Build normalized constraints:

```matlab
uAG = (G - Aupper) / L.link5;
uCG = (G - Cupper) / L.link2;
AG = [uAG.'; uCG.'];
[~, singularMatrix, V] = svd(AG);
singularValues = diag(singularMatrix);
passiveDirection = V(:, end);
margin = abs(cross2(uAG, uCG));
```

Derive lower point velocities analytically. With `u = D-C`, `v = D-E`:

```matlab
dE = [L.link1 * [-sin(theta); cos(theta)], zeros(2,1)];
dC = [zeros(2,1), L.link2 * [sin(phi); cos(phi)]];
lowerConstraint = [u.'; v.'];
dD(:,j) = lowerConstraint \ [dot(u,dC(:,j)); dot(v,dE(:,j))];
dAupper = dD + dE;
dCupper = dD + dE - dC;
BG = [uAG.' * dAupper; uCG.' * dCupper];
```

If the lower constraint is singular, retain coupling geometry but return `BG=NaN(2)` and velocity transmission `NaN(2)` with a distinct status. If `exactCouplingSingular`, do not solve `AG\BG`; otherwise expose `taskJacobian=AG\BG`.

Classification precedence:

```text
theoretical closure failure
lower closure singularity
exact coupling singularity
mechanical invalidity
near coupling singularity
engineering warning
OK
```

The boolean fields remain independent even when the displayed status uses this precedence.

- [ ] **Step 6: Run Task 2 tests and related regression**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests({'tests/singularity/TestCouplingMetrics.m','tests/kinematics/TestPassiveDoublet.m','tests/singularity/TestSingularityMetrics.m','tests/kinematics/TestJacobian.m'}); assertSuccess(r); files={'src/+duallink5/+singularity/evaluateCoupling.m','src/+duallink5/+singularity/private/normalizeCouplingOptions.m','src/+duallink5/+singularity/private/prepareCouplingReference.m','src/+duallink5/+singularity/private/assessCouplingAssembly.m'}; for k=1:numel(files), assert(isempty(checkcode(files{k},'-id'))); end"
```

Expected: all focused and existing singularity/Jacobian tests pass, with zero Code Analyzer messages.

- [ ] **Step 7: Commit Task 2 only**

```powershell
git add -- src/+duallink5/+singularity/evaluateCoupling.m src/+duallink5/+singularity/private/normalizeCouplingOptions.m src/+duallink5/+singularity/private/prepareCouplingReference.m src/+duallink5/+singularity/private/assessCouplingAssembly.m tests/singularity/TestCouplingMetrics.m
git commit -m "feat: evaluate doublet coupling singularity"
```

---

### Task 3: Coupling singularity-space sampling

**Files:**
- Create: `src/+duallink5/+singularity/sampleCouplingSpace.m`
- Create: `tests/singularity/TestCouplingSpace.m`

- [ ] **Step 1: Write failing sampler tests**

Create tests for a small grid containing negative phi, zero, positive phi, and an unreachable custom case:

```matlab
function samplesBothSidesAndExactLine(testCase)
    grid.theta = deg2rad([84, 85, 86]);
    grid.phi = deg2rad([-15, -10, -5, 0, 5, 10, 15]);
    samples = duallink5.singularity.sampleCouplingSpace( ...
        grid, testCase.Geometry, struct('collisionProfile', "none"));

    testCase.verifySize(samples.thetaGrid, [3, 7]);
    zeroColumn = 4;
    testCase.verifyTrue(all(samples.exactCouplingSingular(:, zeroColumn)));
    testCase.verifyTrue(all(samples.engineeringWarning(:, 2:6), 'all'));
    testCase.verifyFalse(any(samples.engineeringWarning(:, [1,7]), 'all'));
    testCase.verifyEqual(samples.couplingMargin(:, 3), ...
        samples.couplingMargin(:, 5), 'AbsTol', 1e-11);
end

function referenceIsPreparedOnceOutsideLoop(testCase)
    grid.theta = deg2rad([84, 85]);
    grid.phi = deg2rad([-5, 0, 5]);
    samples = duallink5.singularity.sampleCouplingSpace( ...
        grid, testCase.Geometry, struct('collisionProfile', "none"));
    testCase.verifyEqual(samples.metadata.referenceBuildCount, 1);
    testCase.verifyEqual(samples.metadata.referenceQ, ...
        testCase.Geometry.analysis.referenceQ);
end

function malformedGridUsesStableError(testCase)
    bad.theta = [1, 0];
    bad.phi = [0, 1];
    testCase.verifyError(@() ...
        duallink5.singularity.sampleCouplingSpace( ...
            bad, testCase.Geometry), ...
        'duallink5:singularity:InvalidCouplingGrid');
end
```

Also test single-element axes, nonuniform grids, theoretical masks independent of mechanical masks, exact singular samples retaining finite `x/y/passiveDirection`, reason-map consistency, and invalid samples remaining `NaN` without stopping the loop.

- [ ] **Step 2: Run the sampler RED test**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests('tests/singularity/TestCouplingSpace.m'); assert(~all([r.Passed]));"
```

Expected: failure because `sampleCouplingSpace` is undefined.

- [ ] **Step 3: Implement validated sampling and allocation**

Accept nonempty finite real strictly increasing vectors for `grid.theta` and `grid.phi`, including single-element vectors. Build `ndgrid`, allocate fixed-size arrays, and prepare the coupling reference exactly once before the loop.

The public sample contract must include:

```matlab
samples.thetaGrid
samples.phiGrid
samples.x
samples.y
samples.couplingMargin
samples.sigmaMin
samples.conditionNumber
samples.constraintMatrix        % 2x2xNthetaxNphi
samples.actuationMatrix         % 2x2xNthetaxNphi
samples.passiveDirection        % 2xNthetaxNphi
samples.theoreticalReachableMask
samples.mechanicallyValidMask
samples.exactCouplingSingular
samples.nearCouplingSingular
samples.engineeringWarning
samples.safeUsableMask
samples.reasonMap
samples.passiveOrientationSign
samples.metadata
```

Inside the loop pass the cached reference through options to `evaluateCoupling`. Catch only expected per-configuration statuses represented by the public result; input/programming errors must still throw. Copy each field into its preallocated location. Define `safeUsableMask` as mechanically valid and neither exact/near coupling singular nor inside the engineering warning band.

Set metadata for SI storage, mm/deg display, thresholds, reference q/sign, branch policy, grid shape, geometry version, profiles, and `referenceBuildCount=1`.

- [ ] **Step 4: Run Task 3 tests and a performance smoke test**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests({'tests/singularity/TestCouplingSpace.m','tests/singularity/TestCouplingMetrics.m'}); assertSuccess(r); g=duallink5.model.defaultGeometry(); grid.theta=deg2rad(60:2:120); grid.phi=deg2rad(-30:2:30); tic; s=duallink5.singularity.sampleCouplingSpace(grid,g,struct('collisionProfile','none')); elapsed=toc; assert(elapsed<30); assert(any(s.exactCouplingSingular,'all')); assert(isempty(checkcode('src/+duallink5/+singularity/sampleCouplingSpace.m','-id')));"
```

Expected: tests pass, the 31x31 smoke grid completes under 30 seconds on the current machine, and static analysis is clean.

- [ ] **Step 5: Commit Task 3 only**

```powershell
git add -- src/+duallink5/+singularity/sampleCouplingSpace.m tests/singularity/TestCouplingSpace.m
git commit -m "feat: sample doublet coupling space"
```

---

### Task 4: Dedicated coupling singularity visualization

**Files:**
- Create: `src/+duallink5/+viz/plotCouplingSingularitySpace.m`
- Create: `tests/viz/TestCouplingVisualization.m`

- [ ] **Step 1: Write failing visualization tests**

Create an invisible figure with a `2x3` tiled layout and five axes. Use a small real sample result. Test:

```matlab
function rendersApprovedFivePanels(testCase)
    [figureHandle, axesHandles] = testCase.makeAxes();
    cleanup = onCleanup(@()close(figureHandle));
    original = string({axesHandles.NextPlot});
    handles = duallink5.viz.plotCouplingSingularitySpace( ...
        testCase.Samples, axesHandles, struct());

    testCase.verifyTrue(isgraphics(handles.jointMap));
    testCase.verifyTrue(isgraphics(handles.taskScatter));
    testCase.verifySize(handles.poseArrows, [3, 1]);
    testCase.verifyTrue(all(isgraphics(handles.poseArrows)));
    testCase.verifyEqual(string({axesHandles.NextPlot}), original);
    testCase.verifyEqual(axesHandles(1).XLabel.String, '\theta [deg]');
    testCase.verifyEqual(axesHandles(1).YLabel.String, '\phi [deg]');
    testCase.verifyNotEmpty(findobj(axesHandles(1), ...
        'DisplayName', '\phi = 0 deg theoretical singularity'));
    clear cleanup
end

function passiveArrowsUseComputedDirections(testCase)
    [figureHandle, axesHandles] = testCase.makeAxes();
    cleanup = onCleanup(@()close(figureHandle));
    handles = duallink5.viz.plotCouplingSingularitySpace( ...
        testCase.Samples, axesHandles, struct());
    for index = 1:3
        direction = handles.representativeDirections(:, index);
        testCase.verifyEqual(norm(direction), 1, 'AbsTol', 1e-12);
        testCase.verifyEqual(abs(dot(direction, ...
            handles.sourceDirections(:, index))), 1, 'AbsTol', 1e-12);
    end
    clear cleanup
end
```

Add stable error-contract tests for duplicate/dead axes, malformed sample arrays, non-`ndgrid` grids, inconsistent masks/reasons, invalid representative angles, and error-path `NextPlot` restoration. Confirm figure count returns to baseline after each test.

- [ ] **Step 2: Run the visualization RED test**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests('tests/viz/TestCouplingVisualization.m'); assert(~all([r.Passed]));"
```

Expected: failure because `plotCouplingSingularitySpace` is undefined.

- [ ] **Step 3: Implement strict validation and axes ownership**

Require exactly five distinct live axes. Validate every nested sample field, numeric/logical type, finite grid, grid separability, shape, mask implication, reason consistency, and metadata thresholds. Throw only:

```text
duallink5:viz:InvalidCouplingPlotInput
```

for public plot-contract errors. Save all five `NextPlot` values and restore them with `onCleanup` on normal and error paths.

- [ ] **Step 4: Implement the approved layers**

Use nonuniform cell edges and flat `surface` rendering for the joint map. Plot `log10(conditionNumber)` with finite positive color limits even for constant data. Overlay:

- a named solid `phi=0 deg` theoretical line;
- two named dashed `phi=+/-10 deg` engineering boundaries;
- translucent engineering-band fill;
- gray markers/mask for theoretical or mechanical invalidity;
- the configured reference point.

Use a task-space scatter of finite theoretical `G_k` points colored by the same coupling metric. Keep meters internally and display mm.

For the three representative axes, select the nearest theoretically reachable samples to:

```matlab
options.representativeQ = deg2rad([85,30; 85,10; 85,0]);
```

Re-solve each pose with `collisionProfile="none"`, call the existing `plotAssembly`, fix equal axes to one shared stable extent, and draw a bidirectional `quiver` pair at `G_k` using the computed `passiveDirection`. Titles must identify `regular`, `engineering warning`, and `exact coupling singularity` without claiming a gravity direction.

- [ ] **Step 5: Run visualization and regression tests**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); before=numel(findall(groot,'Type','figure')); r=runtests({'tests/viz/TestCouplingVisualization.m','tests/viz/TestSingularityVisualization.m','tests/viz/TestVisualization.m'}); assertSuccess(r); after=numel(findall(groot,'Type','figure')); assert(after==before); assert(isempty(checkcode('src/+duallink5/+viz/plotCouplingSingularitySpace.m','-id')));"
```

Expected: all visualization tests pass, no figure leak occurs, and static analysis is clean.

- [ ] **Step 6: Commit Task 4 only**

```powershell
git add -- src/+duallink5/+viz/plotCouplingSingularitySpace.m tests/viz/TestCouplingVisualization.m
git commit -m "feat: plot doublet coupling singularity"
```

---

### Task 5: Facade, runnable script, and user-facing conventions

**Files:**
- Modify: `src/+duallink5/DL5.m`
- Modify: `tests/facade/TestDL5.m`
- Create: `scripts/analyze_coupling_singularity.m`
- Modify: `docs/kinematics-conventions.md`

- [ ] **Step 1: Add failing facade state-safety tests**

Add:

```matlab
function couplingAnalysisDoesNotMutateCommittedState(testCase)
    robot = duallink5.DL5();
    before = struct('Q', robot.Q, 'Assembly', robot.Assembly, ...
        'PointG', robot.PointG, 'IsValid', robot.IsValid, ...
        'StatusCode', robot.StatusCode);
    result = robot.couplingSingularity( ...
        deg2rad([85, 0]), struct('collisionProfile', "none"));
    testCase.verifyTrue(result.exactCouplingSingular);
    testCase.verifyEqual(robot.Q, before.Q);
    testCase.verifyEqual(robot.Assembly, before.Assembly);
    testCase.verifyEqual(robot.PointG, before.PointG);
    testCase.verifyEqual(robot.IsValid, before.IsValid);
    testCase.verifyEqual(robot.StatusCode, before.StatusCode);
end

function couplingSamplingDoesNotMutateCommittedState(testCase)
    robot = duallink5.DL5();
    beforeQ = robot.Q;
    grid.theta = deg2rad([84, 85, 86]);
    grid.phi = deg2rad([-5, 0, 5]);
    samples = robot.sampleCouplingSingularitySpace( ...
        grid, struct('collisionProfile', "none"));
    testCase.verifySize(samples.thetaGrid, [3, 3]);
    testCase.verifyEqual(robot.Q, beforeQ);
end
```

- [ ] **Step 2: Run the facade RED test**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests('tests/facade/TestDL5.m'); assert(~all([r.Passed]));"
```

Expected: the two new methods are undefined.

- [ ] **Step 3: Add thin read-only facade wrappers**

Add public methods to `DL5`:

```matlab
function result = couplingSingularity(robot, qLower, options)
    if nargin < 3
        options = struct();
    end
    result = duallink5.singularity.evaluateCoupling( ...
        qLower, robot.Geometry, options);
end

function samples = sampleCouplingSingularitySpace( ...
        robot, grid, options)
    if nargin < 3
        options = struct();
    end
    samples = duallink5.singularity.sampleCouplingSpace( ...
        grid, robot.Geometry, options);
end
```

These methods must not read or write `robot.Q` unless the user passes it explicitly as `qLower`.

- [ ] **Step 4: Create the runnable analysis script**

The script must preserve caller path, run `startup.m`, create a default geometry, and sample:

```matlab
grid.theta = deg2rad(0:1:180);
grid.phi = deg2rad(-30:1:30);
options = struct('collisionProfile', "centerline", ...
    'topologyProfile', "reference");
```

Create a `2x3` tiled layout with the joint map spanning the left column, task map in the upper middle/right region, and three representative poses along the remaining tiles. If MATLAB tile spanning cannot express the approved layout cleanly, create five explicitly positioned axes in one figure; the public plotter still receives five axes.

Print summary counts and the reference metrics with `fprintf`, including theoretical, mechanical, exact, near, warning, and safe sample counts. Keep the figure open for interactive use; on an exception, close the partially created figure and rethrow.

- [ ] **Step 5: Document the conventions**

Add a section to `docs/kinematics-conventions.md` that explicitly states:

- `G_k = B_(k+1)`;
- passive upper centers come from the two physical parallelograms;
- `phi=0` makes the upper circles internally tangent for the approved 80/62 mm lengths;
- `passiveDirection` has sign ambiguity and is displayed bidirectionally;
- `+/-10 deg` is an empirical configurable warning band, not the theoretical singularity definition;
- coupling singularity results are separate from the existing single-five-bar Type-I/II results.

- [ ] **Step 6: Run facade tests, static analysis, and a headless script smoke test**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests('tests/facade/TestDL5.m'); assertSuccess(r); before=numel(findall(groot,'Type','figure')); run('scripts/analyze_coupling_singularity.m'); figures=findall(groot,'Type','figure'); assert(numel(figures)==before+1); assert(numel(findall(figures(1),'Type','axes'))>=5); close(figures(1)); assert(numel(findall(groot,'Type','figure'))==before); assert(isempty(checkcode('src/+duallink5/DL5.m','-id'))); assert(isempty(checkcode('scripts/analyze_coupling_singularity.m','-id')));"
```

Expected: facade tests pass, exactly one analysis figure is created and closed by the smoke test, and both production files are Code Analyzer clean.

- [ ] **Step 7: Commit Task 5 only**

```powershell
git add -- src/+duallink5/DL5.m tests/facade/TestDL5.m scripts/analyze_coupling_singularity.m docs/kinematics-conventions.md
git commit -m "feat: expose doublet coupling analysis"
```

---

### Task 6: Cross-module review, full verification, and final integration commit

**Files:**
- Review every production and test file changed in Tasks 1-5.
- Do not stage or modify the user's pre-existing script renames, `scripts/plot_continuous_trajectory.m`, or `Kinematics/公式/` exports.

- [ ] **Step 1: Perform a specification-compliance review**

Check every requirement in `docs/superpowers/specs/2026-08-12-doublet-coupling-singularity-design.md` against the actual API. Specifically verify:

- exact inner tangency returns finite `G` and a finite passive direction;
- both physical parallelograms participate in construction and residual checks;
- the existing ideal `forwardAssembly` remains unchanged;
- positive/negative theoretical sides remain visible;
- reference topology is built once per sample call;
- angle warning and SVD near-singular flags remain independent;
- all exclusions in the spec remain excluded.

Record each discrepancy as an actionable file/line finding, add a failing regression test, then apply the smallest correction.

- [ ] **Step 2: Perform a numerical-quality review**

Check machine-scale tangency tolerance, division-by-zero protection, branch continuity, SVD sign ambiguity, condition number handling, NaN contracts, exact-threshold boundaries, and finite-difference agreement. Add regression tests before any correction.

- [ ] **Step 3: Run all focused coupling tests**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests({'tests/model/TestGeometry.m','tests/kinematics/TestPassiveDoublet.m','tests/singularity/TestCouplingMetrics.m','tests/singularity/TestCouplingSpace.m','tests/viz/TestCouplingVisualization.m','tests/facade/TestDL5.m'}); disp(table(r)); assertSuccess(r);"
```

Expected: all focused tests pass with zero incomplete tests.

- [ ] **Step 4: Run the full project test suite**

Run:

```powershell
matlab.exe -batch "cd('D:/OneDrive/科研/项目/05-DualLink5/运动学/Matlab'); r=runtests('tests','IncludeSubfolders',true); disp(table(r)); assertSuccess(r);"
```

Expected: all existing and new tests pass with zero failures and zero incomplete tests.

- [ ] **Step 5: Run Code Analyzer on every changed production file**

Run one MATLAB batch that calls `checkcode(file,'-id')` on:

```text
src/+duallink5/+model/defaultGeometry.m
src/+duallink5/+model/validateGeometry.m
src/+duallink5/+kinematics/forwardPassiveDoublet.m
src/+duallink5/+singularity/evaluateCoupling.m
src/+duallink5/+singularity/private/normalizeCouplingOptions.m
src/+duallink5/+singularity/private/prepareCouplingReference.m
src/+duallink5/+singularity/private/assessCouplingAssembly.m
src/+duallink5/+singularity/sampleCouplingSpace.m
src/+duallink5/+viz/plotCouplingSingularitySpace.m
src/+duallink5/DL5.m
scripts/analyze_coupling_singularity.m
```

Expected: zero messages from every file.

- [ ] **Step 6: Render and visually inspect the final figure**

Run the analysis script headlessly, export the figure to a unique PNG under the system temporary directory, close MATLAB figures, then inspect the PNG at original resolution. Verify:

- the `phi=0 deg` theoretical line is clear;
- the `+/-10 deg` engineering band is symmetric;
- negative and positive theoretical regions appear;
- task points and colorbar are legible;
- all three complete mechanisms are visible at a common fixed scale;
- the exact-singularity arrow is bidirectional and perpendicular to the collinear upper links;
- labels and legend do not hide critical data.

Delete only the unique PNG created by this verification after inspection.

- [ ] **Step 7: Verify Git scope and commit review fixes only if needed**

Run:

```powershell
git status --short
git diff --check
git log --oneline -8
```

Ensure feature commits contain only authorized files. If review produced fixes, commit their explicit pathspecs as:

```powershell
git commit -m "fix: harden doublet coupling analysis"
```

If no fixes were needed, do not create an empty commit. Preserve all pre-existing user changes exactly.
