# DualLink5 Singularity Analysis Design

## Objective

Add a dedicated MATLAB singularity-analysis workflow for the planar
five-bar mechanism doublet. The workflow must distinguish Type I, Type II,
and Type III singularities, show both joint-space and point-G task-space
singular sets, and separate ideal geometric reachability from the actual
collision- and topology-constrained mechanism domain.

The standard entry script generates two publication-oriented figures:

1. a joint-space singularity and configuration-validity figure; and
2. a point-G singularity and shared-link orientation-sensitivity figure.

## Approved Scope

The main singularity space uses ideal symmetric motion,
`q.upper == q.lower == [theta, phi]`. Because the upper five-bar is a
180-degree rigid transform of the lower five-bar, both sides have the same
geometric singular set in this mode.

The analysis also accepts diagnostic inputs with different lower and upper
joint angles and reports each side separately. It does not compress the four
diagnostic angles into a misleading two-dimensional plot.

The primary task is translation of point G, with output `[xG, yG]`. Shared-
link orientation sensitivity is a diagnostic quantity and is not treated as
a third independently controlled degree of freedom.

This design does not add the four-motor elastic redundant-actuation
Jacobian, internal-force null space, spring-stiffness model, or load-sharing
controller. Those require a separately approved motor-side/link-side
coordinate and compliance model.

## Kinematic Model

The local five-bar convention remains the one already implemented by
`duallink5.kinematics.forwardFiveBar`:

```matlab
A = [0; 0];
B = [l5; 0];
E = l1 * [cos(theta); sin(theta)];
C = [l5 - l2 * cos(phi); l2 * sin(phi)];
```

Let

```text
u = D - C
v = D - E
e_theta = dE/dtheta = l1 [-sin(theta), cos(theta)]^T
c_phi   = dC/dphi   = l2 [ sin(phi),   cos(phi)]^T
```

The differentiated loop closure is

```text
A_c D_dot = B_D q_dot
```

with

```text
A_c = [u^T; v^T]
B_D = [0, u^T c_phi; v^T e_theta, 0].
```

The project geometry defines `G = D + E - B`, so point-G velocity satisfies

```text
A_c G_dot = B_G q_dot
```

where

```text
B_G = [u^T e_theta, u^T c_phi;
       2 v^T e_theta, 0]
J_G = A_c \ B_G
```

when `A_c` is nonsingular.

## Singularity Definitions

### Type I: inverse-kinematic or actuation-transmission singularity

Type I occurs when `det(B_G) == 0`. Since

```text
det(B_G) = -2 (u^T c_phi) (v^T e_theta),
```

the two subtypes are:

- Type I-theta: points A, E, and D are collinear; and
- Type I-phi: points B, C, and D are collinear.

### Type II: direct-kinematic or constraint singularity

Type II occurs when `det(A_c) == 0`, meaning C, D, and E are collinear. It is
the inner- or outer-tangency boundary of the two closure circles:

```text
norm(E - C) = l3 + l4
```

or

```text
norm(E - C) = abs(l4 - l3).
```

For the default geometry, these distances are 149 mm and 11 mm.

### Type III

Type III occurs when Type I and Type II are simultaneously true. It is stored
as its own classification while retaining the underlying Type I and Type II
boolean flags.

## Dimensionless Margins and Numerical Thresholds

The analysis first computes three signed normalized indicators:

```text
gITheta = (v^T e_theta) / (l4 l1)
gIPhi   = (u^T c_phi)   / (l3 l2)
gII     = cross2(u,v)   / (l3 l4).
```

Their signs support robust zero-contour extraction. The corresponding
geometric margins are their absolute values:

```text
sITheta = abs(v^T e_theta) / (l4 l1)
sIPhi   = abs(u^T c_phi)   / (l3 l2)
sII     = abs(cross2(u,v)) / (l3 l4).
```

All three lie in `[0, 1]` up to floating-point roundoff and approach zero at
their corresponding singularity.

The default thresholds are:

- `exactThreshold = 1e-8` for mathematical classification;
- `nearMetricThreshold = 0.05` for an engineering near-singular band; and
- `nearConditionThreshold = 100` for a condition-number warning.

An exact subtype flag is set when the relevant normalized margin is no larger
than `exactThreshold`. The Type I-theta and Type I-phi near flags use their
respective margin threshold. The Type II near flag uses either its margin
threshold or the normalized constraint-matrix condition threshold. A separate
task warning is set when the normalized point-G Jacobian condition number
exceeds `nearConditionThreshold`. `near.any` is the union of all geometric and
condition-number warnings. Exact points are also near points.

The thresholds are analysis options rather than replacements for
`geometry.tolerance.singularityCondition`. The geometry tolerance continues
to protect numerical kinematics, while the new thresholds describe a useful
engineering safety margin.

The constraint matrix is row-normalized before singular-value analysis:

```text
A_bar = diag([1/l3, 1/l4]) A_c.
```

The point-G Jacobian is normalized by `l4` before reporting dimensionless
singular values. Uniform scaling does not change its condition number.

At Type II, `J_G` is not formed by inversion. Its entries and task singular
values are `NaN`, its task condition number is `Inf`, and the valid Type II
classification is retained.

## Reachability and Usability Layers

The sampler stores three different masks instead of overloading one
`validMask`:

- `theoreticalReachableMask`: the selected five-bar branch closes
  geometrically, including exact inner and outer tangency;
- `mechanicallyValidMask`: the complete doublet passes joint-range,
  assembly, collision, clearance, and approved-reference-topology checks;
- `safeUsableMask`: `mechanicallyValidMask` with `near.any` removed.

`safeUsableMask` must be a subset of `mechanicallyValidMask`, which must be a
subset of `theoreticalReachableMask`. Exact tangency is retained only in the
theoretical layer and therefore does not violate these subset relations.

Per-sample mechanical reasons remain available in `reasonMap`. At minimum,
the categorical plot distinguishes:

- unreachable or joint-limit violation;
- theoretically reachable;
- collision or clearance violation;
- assembly-topology mismatch;
- mechanically valid but near singular; and
- safe usable.

The approved topology reference remains `theta = 85 deg`, `phi = 30 deg`.
It is prepared once before grid sampling and reused for every candidate.

## Public API

### Single-configuration analysis

```matlab
result = duallink5.singularity.evaluate(q, geometry, options);
```

Ideal input is a finite real `1-by-2` vector `[theta, phi]`. Diagnostic input
is a scalar struct containing finite real `1-by-2` vectors `q.lower` and
`q.upper`.

The top-level result contains:

- `mode`: `"ideal"` or `"diagnostic"`;
- `logical`: the ideal logical-side metrics, or empty in diagnostic mode;
- `lower` and `upper`: per-side metric structs;
- `assemblyValid` and `assemblyStatusCode`;
- `nearSingular`: true when either available side is near singular; and
- `worstMargin`: the minimum of all available Type I and Type II margins.

Each per-side metric struct contains:

- `q`, `reachable`, `closureStatusCode`, `branchId`, and `pointG`;
- `matrices.constraint`, `matrices.taskActuation`, and
  `matrices.taskJacobian`;
- `signedIndicator.typeITheta`, `signedIndicator.typeIPhi`, and
  `signedIndicator.typeII`;
- `margins.typeITheta`, `margins.typeIPhi`, and `margins.typeII`;
- normalized constraint and task singular values;
- `conditionNumber.constraint` and `conditionNumber.task`;
- exact and near subtype flags, including Type I, Type II, Type III, and any;
- `classification`; and
- `orientationSensitivity`, equal to `norm(dpsi_dq, 2)` when defined.

Per-side `classification` is one of:

- `UNREACHABLE_CLOSURE`;
- `REGULAR`;
- `NEAR_TYPE_I`;
- `NEAR_TYPE_II`;
- `NEAR_MULTIPLE`;
- `NEAR_TASK`;
- `TYPE_I`;
- `TYPE_II`; or
- `TYPE_III`.

### Grid analysis

```matlab
samples = duallink5.singularity.sampleSpace( ...
    grid, geometry, options);
```

`grid.theta` and `grid.phi` are finite, real, nonempty, strictly increasing
vectors in radians. Samples outside the configured joint ranges are retained
in the grid and classified as joint-limit violations.

The sampled result contains:

- `thetaGrid`, `phiGrid`, `x`, `y`, and `orientationSensitivity`;
- the three reachability/usability masks;
- `reasonMap`, `classificationMap`, and `branchId`;
- nested scalar maps for signed indicators, margins, condition numbers, exact
  flags, and near flags;
- `constraintMatrix`, `taskActuationMatrix`, and `taskJacobian` as
  `2-by-2-by-nTheta-by-nPhi` arrays;
- normalized singular-value arrays; and
- metadata containing units, thresholds, reference configuration, branch,
  collision profile, topology profile, and geometry version.

### Options

Supported option fields are:

- `branchId`, defaulting to `geometry.assembly.defaultBranch`;
- `exactThreshold`, defaulting to `1e-8`;
- `nearMetricThreshold`, defaulting to `0.05`;
- `nearConditionThreshold`, defaulting to `100`;
- `collisionProfile`, defaulting to `"centerline"`; and
- `topologyProfile`, defaulting to `"reference"`.

Thresholds must be finite positive scalars,
`exactThreshold < nearMetricThreshold <= 1`, and
`nearConditionThreshold > 1`. `branchId` must be `-1` or `1`.
`topologyProfile` accepts `"reference"` or `"none"`.

## Tangency Preservation

The existing `forwardFiveBar` intentionally returns early for
`NEAR_SINGULAR` closure and therefore does not retain D or G. The singularity
sampler does not weaken that production-kinematics safety behavior.

Instead, it calls the existing circle-closure solver directly for theoretical
geometry. At tangency, the solver's single candidate D is retained for
classification and for `G = D + E - B`. For regular closure, the configured
fixed branch is selected. Complete-mechanism validity continues to use
`forwardAssembly`, so the theoretical boundary cannot be mistaken for a safe
commandable configuration.

## Sampling Algorithm

For every angle-grid entry, the sampler performs these operations in order:

1. classify the joint-range state;
2. construct A, B, C, and E and solve the local five-bar closure;
3. retain a tangent D or select the configured regular branch;
4. compute G, the three matrices, normalized margins, singular values,
   condition numbers, exact flags, near flags, and orientation sensitivity;
5. evaluate the ideal complete doublet with the fixed branch;
6. apply collision and clearance checks;
7. compare the full assembly to the cached reference topology; and
8. populate the mechanical-valid and safe-usable masks.

The standard script uses a 1-degree grid:

```matlab
grid.theta = deg2rad(0:1:180);
grid.phi = deg2rad(0:1:90);
```

The grid is user-editable; a 0.5-degree grid is appropriate for final
publication rendering. The mathematical Type II line is also extracted from
the analytic closure-distance equation, so its existence and location do not
depend on a grid point landing exactly on zero.

Geometry and options are validated once at the public boundary. The topology
reference is prepared once outside the nested loop. The theoretical scalar
maps use array operations where their formulas are independent, while
full-assembly validation remains deterministic and does not require Parallel
Computing Toolbox.

## Diagnostic Lower/Upper Analysis

For diagnostic input, each local five-bar is analyzed independently even if
the two sides do not form one coincident shared link. This preserves useful
per-side singularity information when the complete diagnostic assembly has
`SHARED_LINK_MISMATCH`.

The top-level result reports the existing complete-assembly status separately
from singularity classification. A mismatch is not relabeled as a
singularity, and a singular side is not hidden by a mismatch.

The two-dimensional sampler remains ideal-only. A four-angle diagnostic
space is outside the approved visualization scope.

## Visualization

The visualization layer adds two functions following the project's explicit
axes-handle convention:

```matlab
handles = duallink5.viz.plotJointSingularitySpace( ...
    samples, axesHandles);

handles = duallink5.viz.plotTaskSingularitySpace( ...
    samples, axesHandles);
```

`axesHandles` is a live `1-by-2` axes array. Both functions restore every
axes object's original `NextPlot` value and return handles to all created
graphics objects.

### Figure 1: Joint-Space Singularity Map

Panel (a) shows:

- `log10(cond(J_G))` over the theoretical regular domain;
- orange Type I curves, separated in returned handles by theta and phi
  subtype;
- red dashed Type II inner- and outer-tangency curves;
- purple engineering near-singular bands; and
- the approved reference configuration as a labelled marker.

Panel (b) shows the categorical mechanism domain:

- light gray for unreachable or outside joint limits;
- light blue for theoretical reachability;
- dark gray for collision or clearance rejection;
- purple for assembly-topology mismatch;
- a warning color for mechanically valid near-singular samples; and
- green for the safe usable domain.

Both axes use degrees and retain equal, increasing theta and phi directions.

### Figure 2: Task-Space Singularity and Orientation Sensitivity

Panel (a) shows:

- the theoretical point-G domain in light blue;
- the actual safe point-G domain in green;
- mapped Type I, Type II, and Type III loci in orange, red, and purple; and
- dashed versus solid styles to distinguish theoretical-only boundaries from
  near-boundary samples adjacent to the mechanically valid domain.

Panel (b) maps `log10(norm(dpsi_dq, 2))` onto finite mechanically valid point-G
samples. Undefined exact Type II values are omitted from the color data rather
than allowed to distort the color scale.

Both task-space panels display millimetres and use equal data aspect ratio.

## User Entry Script

The directly runnable entry point is:

```text
scripts/analyze_singularity_space.m
```

It performs project startup, constructs the default geometry and 1-degree
grid, samples the singularity space, creates two figures with two panels each,
calls the visualization functions, and prints:

- elapsed sampling time;
- theoretical, mechanically valid, and safe sample counts;
- exact and near Type I/II/III counts; and
- the safe fraction of the mechanically valid samples.

The script does not automatically overwrite image or MAT files. Users can
export figures through the existing visualization/export workflow after
inspection.

## DL5 Facade

The `duallink5.DL5` facade adds two read-only methods:

```matlab
result = robot.singularity(options);
samples = robot.sampleSingularitySpace(grid, options);
```

`robot.singularity` analyzes the committed logical joint vector. The sampling
method forwards `robot.Geometry`. Neither method changes `Q`, `Assembly`,
`PointG`, `IsValid`, or `StatusCode`, including when analysis rejects an input.

## Error Handling

Malformed analysis inputs use stable singularity-package identifiers:

- `duallink5:singularity:InvalidJointInput`;
- `duallink5:singularity:InvalidGrid`;
- `duallink5:singularity:InvalidOptions`.

Malformed visualization inputs follow the existing visualization-package
convention:

- `duallink5:viz:InvalidGraphicsHandle`; and
- `duallink5:viz:InvalidSingularityPlotInput`.

Unreachable closure, joint-limit violation, collision, topology mismatch, and
singularity are expected analysis results and do not throw exceptions.

Plotting rejects inconsistent array sizes, missing required fields, nonfinite
coordinates selected by a plotted mask, and dead or nonscalar axes arrays.

## File Structure

Production changes are confined to:

- `src/+duallink5/+singularity/evaluate.m`;
- `src/+duallink5/+singularity/sampleSpace.m`;
- focused helpers under `src/+duallink5/+singularity/private/`;
- `src/+duallink5/+viz/plotJointSingularitySpace.m`;
- `src/+duallink5/+viz/plotTaskSingularitySpace.m`;
- `src/+duallink5/DL5.m`; and
- `scripts/analyze_singularity_space.m`.

The existing workspace API remains compatible and does not become a wrapper
around singularity sampling.

## Verification Strategy

Implementation follows red-green-refactor test-driven development.

### Metric tests

Tests verify:

1. the reference configuration `(85 deg, 30 deg)` is regular and reproduces
   independently calculated constraint and task condition numbers;
2. analytic point-G Jacobians match central finite differences at regular
   configurations;
3. fixed configurations exercise Type I-theta, Type I-phi, Type II outer
   tangency, Type II inner tangency, and Type III;
4. normalized margins equal their geometric sine definitions;
5. threshold equality and just-inside/just-outside behavior are deterministic;
6. Type II returns a finite tangent G, a valid Type II flag, `NaN` task
   Jacobian, and `Inf` task condition number; and
7. diagnostic inputs retain separate lower and upper classifications and the
   complete-assembly mismatch status.

### Sampling tests

Tests verify:

1. every scalar and matrix map has the documented grid shape;
2. `safeUsableMask` is contained in `mechanicallyValidMask` and mechanical
   samples are theoretically reachable;
3. exact tangency remains present in the theoretical outputs;
4. collision and topology mismatch receive separate reasons;
5. `topologyProfile = "none"` changes only topology filtering;
6. reference configuration metadata and thresholds are preserved; and
7. malformed grids and options use stable error identifiers.

### Visualization and facade tests

Tests verify:

1. both plotters require two live axes and reject malformed samples;
2. expected image, line, scatter, patch, marker, legend, and colorbar handles
   are created;
3. exact and near singular layers use distinct handles;
4. degrees, millimetres, and equal task-space aspect ratio are applied;
5. original axes `NextPlot` state is restored on success and failure;
6. figures can be closed without leaked graphics objects; and
7. both facade methods leave the committed robot state unchanged.

### Final verification

The focused singularity, visualization, facade, kinematics, validation, and
workspace tests run first. The complete MATLAB test suite then runs, followed
by Code Analyzer on every new or modified production file and a manual render
of the standard singularity script.

## Acceptance Criteria

The feature is accepted when:

1. Type I, Type II, and Type III are separately identifiable from stored
   geometric metrics rather than inferred from one aggregate condition
   number;
2. exact Type II tangency is visible in both joint and point-G spaces;
3. theoretical reachability, mechanical validity, and safe usability remain
   separately inspectable;
4. ideal and lower/upper diagnostic evaluations behave as documented;
5. the standard script produces the approved two-figure layout;
6. the facade remains state-safe;
7. no existing workspace or kinematics public behavior regresses; and
8. all automated and visual verification steps pass.
