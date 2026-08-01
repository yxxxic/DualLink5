# DualLink5 Workspace Topology Filter Design

## Objective

Exclude mathematically closed but physically different crossed assembly modes
from the `pointG` workspace. The approved reference assembly is
`theta = 85 deg`, `phi = 30 deg`, and the projected link-crossing topology
must remain unchanged during motion.

## Scope

This change adds a reusable assembly-topology comparison and applies it to
workspace sampling. It does not remove alternative assembly modes from direct
kinematics, because retaining them is useful for diagnostics and plotting. It
does not attempt to model three-dimensional layer offsets or actuator housing
geometry.

## Reference Configuration

`duallink5.model.defaultGeometry` stores the reference joint vector as
`geometry.analysis.referenceQ = deg2rad([85, 30])`. Geometry validation
requires a finite real two-element vector.

Workspace sampling constructs one ideal reference assembly before entering
the angle-grid loop. Failure to construct the configured reference assembly is
a configuration error rather than a per-sample rejection.

## Topology Representation

The topology checker uses the same projected segments shown by
`duallink5.viz.plotAssembly`:

- lower five-bar: `AB`, `AE`, `BC`, and `CD`;
- upper five-bar: `AB`, `AE`, `BC`, and `CD`;
- shared link: `ED`;
- alpha parallelogram: its four boundary segments;
- beta parallelogram: its four boundary segments.

Every segment has a stable unique name. For each unordered pair of segments,
the checker records whether their open interiors intersect. Pairs sharing a
topological endpoint are excluded. Endpoint-only contact and contact within
the configured length tolerance do not count as a crossing. This prevents
normal revolute joints and numerical noise from changing the signature.

The sorted set of crossing segment-name pairs is the topology signature. A
candidate is compatible only when its complete signature equals the reference
signature.

## API and Data Flow

A validation-layer function computes and compares signatures without changing
the assembly:

```matlab
comparison = duallink5.validation.compareAssemblyTopology( ...
    candidateAssembly, referenceAssembly, geometry);
```

The result contains:

- `compatible`: logical scalar;
- `candidateSignature` and `referenceSignature`;
- `addedCrossings` and `removedCrossings` for diagnostics;
- `statusCode`: `OK` or `ASSEMBLY_TOPOLOGY_MISMATCH`.

`sampleWorkspace` computes the reference once, then performs the topology
comparison after geometric assembly validation and before Jacobian/task-pose
evaluation. An incompatible sample remains `NaN` in task coordinates, is
false in `validMask`, and receives `ASSEMBLY_TOPOLOGY_MISMATCH` in
`reasonMap`.

Workspace options accept `topologyProfile = "reference"` or `"none"`.
`"reference"` is the default and enforces the approved physical assembly
mode. `"none"` is an explicit diagnostic escape hatch for studying all
mathematical modes; it is never used by the standard workspace example.

## Error Handling

- Malformed `analysis.referenceQ` produces
  `duallink5:model:InvalidGeometry`.
- A reference pose that cannot form a valid ideal assembly produces
  `duallink5:workspace:InvalidTopologyReference`.
- Unsupported topology profiles produce
  `duallink5:workspace:InvalidWorkspaceOptions`.
- Malformed assemblies passed directly to the topology checker produce a
  stable validation error.

## Verification

Tests must prove the following behavior:

1. The default reference is exactly `deg2rad([85, 30])`.
2. The reference pose and the nearby `[85, 31] deg` pose have compatible
   topology.
3. The right sparse pose `[13.5, 84] deg` is rejected.
4. The left sparse pose `[166.5, 72.75] deg` is rejected.
5. Workspace rejection uses `ASSEMBLY_TOPOLOGY_MISMATCH` and does not emit
   finite task coordinates.
6. `topologyProfile = "none"` retains a mathematically valid sparse pose for
   diagnostics.
7. Endpoint-only contact is not treated as an interior crossing.
8. Existing geometry, kinematics, visualization, and workspace tests remain
   green.

The standard workspace example is then rendered to verify visually that both
sparse lower regions are absent and the filled boundary contains only retained
samples.
