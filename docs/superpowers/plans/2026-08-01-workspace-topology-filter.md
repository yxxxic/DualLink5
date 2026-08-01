# Workspace Topology Filter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove workspace samples whose complete projected link-crossing topology differs from the approved `theta = 85 deg`, `phi = 30 deg` assembly.

**Architecture:** Keep direct kinematics capable of constructing alternative mathematical assembly modes. Add a pure validation component that builds named assembly segments, computes open-interior crossing signatures, and compares a candidate with a reference. `sampleWorkspace` constructs the reference once, rejects incompatible samples with `ASSEMBLY_TOPOLOGY_MISMATCH`, and exposes an explicit diagnostic opt-out.

**Tech Stack:** MATLAB R2025a, package folders under `src/+duallink5`, `matlab.unittest`, Git.

---

## File Structure

- Modify `src/+duallink5/+model/defaultGeometry.m`: store the approved reference joint vector.
- Modify `src/+duallink5/+model/validateGeometry.m`: validate the reference vector.
- Create `src/+duallink5/+validation/compareAssemblyTopology.m`: public comparison API and result contract.
- Create `src/+duallink5/+validation/private/assemblyTopologySignature.m`: build named segments and compute proper-intersection signature.
- Modify `src/+duallink5/+workspace/sampleWorkspace.m`: construct the reference once and reject mismatched samples.
- Modify `examples/analyze_fixed_workspace.m`: explicitly run the reference topology profile.
- Modify tests under `tests/model`, `tests/validation`, and `tests/workspace`: prove configuration, comparison, rejection, opt-out, and regression behavior.

### Task 0: Preserve and Commit the Verified Workspace Baseline

**Files:**
- Existing Codex changes only: `src/+duallink5/+kinematics/forwardAssembly.m`, `src/+duallink5/+model/defaultGeometry.m`, `src/+duallink5/+model/validateGeometry.m`, `src/+duallink5/+viz/plotWorkspaceResult.m`, `src/+duallink5/+workspace/analyzeWorkspace.m`, `tests/kinematics/TestAssembly.m`, `tests/model/TestGeometry.m`, `tests/workspace/TestWorkspace.m`
- Preserve without staging: `examples/plot_fixed_pose.m`

- [ ] **Step 1: Verify the existing baseline changes**

Run:

```powershell
matlab -batch "run('startup.m'); r=runtests({'tests/model/TestGeometry.m','tests/kinematics/TestAssembly.m','tests/workspace/TestWorkspace.m','tests/viz/TestVisualization.m'}); assertSuccess(r);"
```

Expected: all selected tests pass.

- [ ] **Step 2: Stage only the verified baseline files**

Run:

```powershell
git add -- src/+duallink5/+kinematics/forwardAssembly.m src/+duallink5/+model/defaultGeometry.m src/+duallink5/+model/validateGeometry.m src/+duallink5/+viz/plotWorkspaceResult.m src/+duallink5/+workspace/analyzeWorkspace.m tests/kinematics/TestAssembly.m tests/model/TestGeometry.m tests/workspace/TestWorkspace.m
git diff --cached --check
git diff --cached --name-only
```

Expected: the eight listed files are staged and `examples/plot_fixed_pose.m` is absent.

- [ ] **Step 3: Commit the baseline**

```powershell
git commit -m "fix: preserve physical workspace boundary"
```

### Task 1: Configure the Approved Reference Pose

**Files:**
- Modify: `src/+duallink5/+model/defaultGeometry.m`
- Modify: `src/+duallink5/+model/validateGeometry.m`
- Test: `tests/model/TestGeometry.m`

- [ ] **Step 1: Write failing geometry tests**

Add to `TestGeometry`:

```matlab
function topologyReferenceUsesApprovedPose(testCase)
    g = duallink5.model.defaultGeometry();
    testCase.verifyEqual(g.analysis.referenceQ, ...
        deg2rad([85, 30]), 'AbsTol', 1e-15);
end

function malformedTopologyReferencesAreRejected(testCase)
    g = duallink5.model.defaultGeometry();
    malformed = {[85, 30, 0], [85; 30], [85, NaN], "85,30"};
    for index = 1:numel(malformed)
        candidate = g;
        candidate.analysis.referenceQ = malformed{index};
        testCase.verifyError( ...
            @() duallink5.model.validateGeometry(candidate), ...
            'duallink5:model:InvalidGeometry');
    end
end
```

- [ ] **Step 2: Run tests and verify RED**

```powershell
matlab -batch "run('startup.m'); r=runtests('tests/model/TestGeometry.m'); assertSuccess(r);"
```

Expected: failure because `analysis.referenceQ` does not exist and is not validated.

- [ ] **Step 3: Add the reference configuration**

In `defaultGeometry.m` add:

```matlab
geometry.analysis.referenceQ = deg2rad([85, 30]);
```

In `validateGeometry.m`, after validating the two analysis ranges, add:

```matlab
if ~isfield(geometry.analysis, 'referenceQ') || ...
        ~isnumeric(geometry.analysis.referenceQ) || ...
        ~isreal(geometry.analysis.referenceQ) || ...
        ~isequal(size(geometry.analysis.referenceQ), [1, 2]) || ...
        any(~isfinite(geometry.analysis.referenceQ))
    fail('analysis.referenceQ must be a finite real 1x2 vector.');
end
```

- [ ] **Step 4: Run tests and verify GREEN**

Run the Task 1 test command. Expected: all `TestGeometry` tests pass.

- [ ] **Step 5: Commit**

```powershell
git add -- src/+duallink5/+model/defaultGeometry.m src/+duallink5/+model/validateGeometry.m tests/model/TestGeometry.m
git commit -m "feat: configure workspace topology reference"
```

### Task 2: Implement Topology Signature Comparison

**Files:**
- Create: `src/+duallink5/+validation/compareAssemblyTopology.m`
- Create: `src/+duallink5/+validation/private/assemblyTopologySignature.m`
- Create: `tests/validation/TestAssemblyTopology.m`

- [ ] **Step 1: Write failing comparison tests**

Create `TestAssemblyTopology.m` with setup matching the existing validation tests and these cases:

```matlab
function referenceAndNearbyPoseAreCompatible(testCase)
    reference = makeAssembly(testCase.Geometry, [85, 30]);
    nearby = makeAssembly(testCase.Geometry, [85, 31]);
    result = duallink5.validation.compareAssemblyTopology( ...
        nearby, reference, testCase.Geometry);
    testCase.verifyTrue(result.compatible);
    testCase.verifyEqual(result.statusCode, "OK");
    testCase.verifyEmpty(result.addedCrossings);
    testCase.verifyEmpty(result.removedCrossings);
end

function sparseModesAreRejected(testCase)
    reference = makeAssembly(testCase.Geometry, [85, 30]);
    sparseQ = {[13.5, 84], [166.5, 72.75]};
    for index = 1:numel(sparseQ)
        candidate = makeAssembly(testCase.Geometry, sparseQ{index});
        result = duallink5.validation.compareAssemblyTopology( ...
            candidate, reference, testCase.Geometry);
        testCase.verifyFalse(result.compatible);
        testCase.verifyEqual(result.statusCode, ...
            "ASSEMBLY_TOPOLOGY_MISMATCH");
        testCase.verifyGreaterThan( ...
            size(result.addedCrossings, 1) + ...
            size(result.removedCrossings, 1), 0);
    end
end

function assembly = makeAssembly(geometry, qDegrees)
q = deg2rad(qDegrees);
jointInput = struct('lower', q, 'upper', q);
assembly = duallink5.kinematics.forwardAssembly( ...
    jointInput, geometry, struct('collisionProfile', "none"));
end
```

Also test malformed scalar inputs and require
`duallink5:validation:InvalidAssemblyTopology`:

```matlab
function malformedAssembliesAreRejected(testCase)
    reference = makeAssembly(testCase.Geometry, [85, 30]);
    malformed = {42, repmat(reference, 1, 2), ...
        rmfield(reference, 'upper')};
    for index = 1:numel(malformed)
        testCase.verifyError(@() ...
            duallink5.validation.compareAssemblyTopology( ...
            malformed{index}, reference, testCase.Geometry), ...
            'duallink5:validation:InvalidAssemblyTopology');
    end
end
```

- [ ] **Step 2: Run tests and verify RED**

```powershell
matlab -batch "run('startup.m'); r=runtests('tests/validation/TestAssemblyTopology.m'); assertSuccess(r);"
```

Expected: error that `compareAssemblyTopology` is undefined.

- [ ] **Step 3: Implement named segment and signature construction**

Create `private/assemblyTopologySignature.m`. It must:

```matlab
function signature = assemblyTopologySignature(assembly, tolerance)
segments = buildSegments(assembly);
pairs = strings(nchoosek(numel(segments), 2), 2);
pairCount = 0;
for first = 1:numel(segments) - 1
    for second = first + 1:numel(segments)
        if any(segments(first).joints == segments(second).joints(1)) || ...
                any(segments(first).joints == segments(second).joints(2))
            continue
        end
        if properInteriorIntersection(segments(first), ...
                segments(second), tolerance)
            pairCount = pairCount + 1;
            pairs(pairCount, :) = sort( ...
                [segments(first).name, segments(second).name]);
        end
    end
end
signature = sortrows(pairs(1:pairCount, :), [1, 2]);
end
```

`buildSegments` must produce the 17 stable names described in the spec:

```matlab
lower.link5, lower.link1, lower.link2, lower.link3,
shared.link4,
upper.link5, upper.link1, upper.link2, upper.link3,
alpha.12, alpha.23, alpha.34, alpha.41,
beta.12, beta.23, beta.34, beta.41
```

Use joint IDs such as `lower.A`, `shared.E`, `alpha.1`, and `upper.A` so only true topological endpoints are skipped. Validate every required point as a finite real two-vector.

Implement open-interior intersection parametrically:

```matlab
r = first.p2 - first.p1;
s = second.p2 - second.p1;
denominator = cross2(r, s);
orientationTolerance = tolerance * ...
    max([norm(r), norm(s), tolerance]);
if abs(denominator) <= orientationTolerance
    intersects = false;
    return
end
t = cross2(second.p1 - first.p1, s) / denominator;
u = cross2(second.p1 - first.p1, r) / denominator;
parameterTolerance = tolerance / ...
    max([norm(r), norm(s), tolerance]);
intersects = t > parameterTolerance && ...
    t < 1 - parameterTolerance && ...
    u > parameterTolerance && u < 1 - parameterTolerance;
```

- [ ] **Step 4: Implement the public comparison contract**

Create `compareAssemblyTopology.m`:

```matlab
function result = compareAssemblyTopology(candidate, reference, geometry)
geometry = duallink5.model.validateGeometry(geometry);
try
    candidateSignature = assemblyTopologySignature( ...
        candidate, geometry.tolerance.length);
    referenceSignature = assemblyTopologySignature( ...
        reference, geometry.tolerance.length);
catch exception
    if exception.identifier == "duallink5:validation:InvalidAssemblyTopology"
        rethrow(exception)
    end
    error('duallink5:validation:InvalidAssemblyTopology', ...
        'Assemblies must contain finite complete assembly points.');
end
result.candidateSignature = candidateSignature;
result.referenceSignature = referenceSignature;
result.addedCrossings = setdiff( ...
    candidateSignature, referenceSignature, 'rows');
result.removedCrossings = setdiff( ...
    referenceSignature, candidateSignature, 'rows');
result.compatible = isempty(result.addedCrossings) && ...
    isempty(result.removedCrossings);
if result.compatible
    result.statusCode = "OK";
else
    result.statusCode = "ASSEMBLY_TOPOLOGY_MISMATCH";
end
end
```

Use string matrices, not tables, for signatures and update the test assertions to use `size(...,1)` rather than `height`.

- [ ] **Step 5: Run tests and verify GREEN**

Run the Task 2 test command. Expected: all topology tests pass, including the two sparse-pose regressions.

- [ ] **Step 6: Run related validation and visualization tests**

```powershell
matlab -batch "run('startup.m'); r=runtests({'tests/validation','tests/viz/TestVisualization.m'}); assertSuccess(r);"
```

Expected: all tests pass.

- [ ] **Step 7: Commit**

```powershell
git add -- src/+duallink5/+validation/compareAssemblyTopology.m src/+duallink5/+validation/private/assemblyTopologySignature.m tests/validation/TestAssemblyTopology.m
git commit -m "feat: compare complete assembly topology"
```

### Task 3: Enforce Reference Topology in Workspace Sampling

**Files:**
- Modify: `src/+duallink5/+workspace/sampleWorkspace.m`
- Modify: `tests/workspace/TestWorkspace.m`

- [ ] **Step 1: Write failing workspace tests**

Add tests that sample one pose at a time:

```matlab
function samplerRejectsSparseTopologyModes(testCase)
    geometry = duallink5.model.defaultGeometry();
    taskSpec = struct('kind', "pointG");
    sparseQ = {[13.5, 84], [166.5, 72.75]};
    for index = 1:numel(sparseQ)
        grid.theta = deg2rad(sparseQ{index}(1));
        grid.phi = deg2rad(sparseQ{index}(2));
        samples = duallink5.workspace.sampleWorkspace( ...
            grid, geometry, taskSpec, struct());
        testCase.verifyFalse(samples.validMask);
        testCase.verifyEqual(samples.reasonMap, ...
            "ASSEMBLY_TOPOLOGY_MISMATCH");
        testCase.verifyTrue(isnan(samples.x));
        testCase.verifyTrue(isnan(samples.y));
    end
end

function topologyDiagnosticOptOutRetainsSparsePose(testCase)
    geometry = duallink5.model.defaultGeometry();
    grid.theta = deg2rad(13.5);
    grid.phi = deg2rad(84);
    samples = duallink5.workspace.sampleWorkspace( ...
        grid, geometry, struct('kind', "pointG"), ...
        struct('topologyProfile', "none", ...
        'collisionProfile', "none"));
    testCase.verifyTrue(samples.validMask);
    testCase.verifyEqual(samples.reasonMap, "OK");
end
```

Add malformed-profile tests for non-scalar and unsupported values, expecting
`duallink5:workspace:InvalidWorkspaceOptions`.

- [ ] **Step 2: Run tests and verify RED**

```powershell
matlab -batch "run('startup.m'); r=runtests('tests/workspace/TestWorkspace.m'); assertSuccess(r);"
```

Expected: sparse poses remain valid because topology is not yet checked.

- [ ] **Step 3: Normalize the topology profile and construct the reference once**

At the start of `sampleWorkspace`, normalize:

```matlab
topologyProfile = getOption(options, 'topologyProfile', "reference");
try
    topologyProfile = string(topologyProfile);
catch
    invalidOptions();
end
if ~isscalar(topologyProfile) || ismissing(topologyProfile) || ...
        ~ismember(topologyProfile, ["reference", "none"])
    invalidOptions();
end
```

Before the sample loop, construct the reference only for the reference profile:

```matlab
referenceAssembly = [];
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
end
```

- [ ] **Step 4: Reject mismatches before Jacobian evaluation**

Immediately after candidate assembly validity:

```matlab
if topologyProfile == "reference"
    topology = duallink5.validation.compareAssemblyTopology( ...
        assembly, referenceAssembly, geometry);
    if ~topology.compatible
        samples.reasonMap(index) = topology.statusCode;
        continue
    end
end
```

Store `topologyProfile` and `geometry.analysis.referenceQ` in sample metadata.

- [ ] **Step 5: Run tests and verify GREEN**

Run the Task 3 test command. Expected: all workspace tests pass.

- [ ] **Step 6: Commit**

```powershell
git add -- src/+duallink5/+workspace/sampleWorkspace.m tests/workspace/TestWorkspace.m
git commit -m "feat: filter workspace by assembly topology"
```

### Task 4: Example, Full Regression, and Visual Verification

**Files:**
- Modify: `examples/analyze_fixed_workspace.m`
- Test: all `tests`

- [ ] **Step 1: Make the example intent explicit**

Change the sampling call to:

```matlab
workspaceOptions = struct('topologyProfile', "reference");
samples = duallink5.workspace.sampleWorkspace( ...
    grid, geometry, taskSpec, workspaceOptions);
```

Do not modify `examples/plot_fixed_pose.m`.

- [ ] **Step 2: Run MATLAB code analysis on changed production files**

```powershell
matlab -batch "files={'src/+duallink5/+model/defaultGeometry.m','src/+duallink5/+model/validateGeometry.m','src/+duallink5/+validation/compareAssemblyTopology.m','src/+duallink5/+validation/private/assemblyTopologySignature.m','src/+duallink5/+workspace/sampleWorkspace.m','examples/analyze_fixed_workspace.m'}; issues=[]; for k=1:numel(files), issues=[issues; checkcode(files{k},'-id')]; end; assert(isempty(issues));"
```

Expected: no analyzer findings.

- [ ] **Step 3: Run the complete test suite**

```powershell
matlab -batch "run('startup.m'); r=runtests('tests'); assertSuccess(r);"
```

Expected: every test passes.

- [ ] **Step 4: Render and inspect the standard workspace**

```powershell
matlab -batch "run('examples/analyze_fixed_workspace.m'); exportgraphics(gcf,fullfile(tempdir,'duallink5-topology-filtered-workspace.png'),'Resolution',160);"
```

Expected visual result: the two sparse lower regions associated with `[13.5,84]` and `[166.5,72.75]` are absent; the filled mesh and maximum rectangle contain only retained samples.

- [ ] **Step 5: Commit the example**

```powershell
git add -- examples/analyze_fixed_workspace.m
git commit -m "docs: demonstrate topology-filtered workspace"
```

- [ ] **Step 6: Confirm the user-owned file remains unstaged**

```powershell
git status --short
```

Expected: `examples/plot_fixed_pose.m` remains modified but unstaged; no topology implementation files remain uncommitted.
