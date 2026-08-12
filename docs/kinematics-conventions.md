# DualLink5 kinematics conventions

## Units and parameter editing

Edit the default link and parallelogram lengths in
`src/+duallink5/+model/defaultGeometry.m`. Values are written as
`80 * mm` for readability and converted immediately to SI metres. The
kinematics, Jacobians, and experiment interfaces continue to use metres and
radians internally. Visualization functions display lengths in millimetres
and workspace areas in square millimetres.

## Link geometry

| Link | Segment | Length (m) |
| --- | --- | ---: |
| link1 | AE | `80e-3` |
| link2 | BC | `62e-3` |
| link3 | CD | `69e-3` |
| link4 | DE | `80e-3` |
| link5 | AB | `80e-3` |

## Local frame and generalized coordinates

The local frame has its origin at A, with +x directed from A to B and +y directed toward the mechanism interior. `theta` is measured from A→B toward A→E, and `phi` is measured from B→A toward B→C. Angles are in radians and lengths are in metres.

The upper mechanism uses the same local convention transformed by pi. For ideal symmetric motion, `q.upper == q.lower`.

## Endpoint geometry

| Distance | Value (m) |
| --- | ---: |
| E→Palpha1 | `30e-3` |
| Palpha1→Palpha4 | `60e-3` |
| D→Pbeta2 | `30e-3` |
| Pbeta2→Pbeta3 | `60e-3` |

## Experiment boundary

The core API uses calibrated logical coordinates in radians and metres. The legacy `computeGTrajectory` adapter accepts calibrated logical `theta`/`phi` in degrees and outputs metres.

Device channel mapping, sign, zero, gearing, wrap handling, time synchronization, and optical extrinsics remain in the calibration layer. Optical data is converted from millimetres to metres on import; millimetres are used only for display.

The legacy adapter's `collisionProfile='none'` setting preserves historical geometry trajectories. Collision screening is a separate operation.

## Passive-doublet coupling conventions

For doublet index `k`, the task point of the lower mechanism is the next
mechanism's upper input point: `G_k = B_(k+1)`. The passive upper closure is
therefore solved as two circles centered at `A_(k+1)` and `C_(k+1)`, rather
than by copying an ideal symmetric pose.

The alpha and beta parallelograms retain their physical rigid-body
ownership. `Palpha1` and `Palpha4` belong to the lower `A-E` body, while
`Palpha2` and `Palpha3` belong to the upper body through `D` and
`A_(k+1)`. `Pbeta2` and `Pbeta3` belong to the lower `C-D` body, while
`Pbeta1` and `Pbeta4` belong to the upper body through `E` and
`C_(k+1)`. These rigid translations give

```text
A_(k+1) = D_k + (E_k - A_k)
C_(k+1) = E_k + (D_k - C_k)
```

With the approved upper radii `80 mm` and `62 mm`, `phi = 0` gives a
center distance of `18 mm = |80 - 62|`. The circles are internally tangent,
so this is the theoretical coupling-singularity geometry.

The normalized coupling constraint matrix `A_G` has row directions from
`A_(k+1)` and `C_(k+1)` to `G_k`. Its coupling margin is the absolute
normalized two-dimensional cross product of those directions, and its
smallest singular value `sigmaMin` measures rank loss. `passiveDirection`
is the right singular vector associated with `sigmaMin`; its sign is
mathematically ambiguous, so plots show the same direction with arrows in
both senses.

The default `+/-10 deg` band around `phi = 0` is a configurable empirical
engineering warning. It is not the theoretical definition of coupling
singularity. Doublet coupling classifications are also separate from the
existing Type-I, Type-II, and Type-III classifications for a single
five-bar mechanism. This analysis phase contains kinematics only; gravity
and other dynamics are outside its scope.
