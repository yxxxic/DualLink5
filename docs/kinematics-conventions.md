# DualLink5 kinematics conventions

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
