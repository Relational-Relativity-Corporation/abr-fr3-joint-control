# abr-fr3-joint-control

ABR relational field control — FR3 joint 1 demonstration with PyO3/Blender visualization.

**Metatron Dynamics, Inc.** — relationalrelativity.dev

## Status

V0.2.2 — Verifier PASS (2026-10-01)
- V-1 CLOSED: actuator driven by established kernel output only
- V-2 CLOSED: B conformance test calls canonical operator_b
- 14/14 conformance tests pass

## Architecture

```
abr-kernel (canonical, Verifier-passed Rust)
    ↓ Cargo dependency
abr-fr3-joint-control (this repo)
    src/declaration.rs   — M declarations, declared topology
    src/bridge.rs        — PyO3 bridge exposing advance_relational_step
    src/lib.rs           — crate root
    tests/operator_conformance.rs — 14 Origin-declared conformance tests
    blender/abr_fr3_demo.py      — Blender demonstration script
```

## Build

```powershell
# Set Python to match Blender's bundled version
$env:PYO3_PYTHON = "C:\path\to\python.exe"
cargo build --release

# Copy to Blender (run as administrator)
Copy-Item target\release\abr_fr3.dll -Destination target\release\abr_fr3.pyd -Force
Copy-Item target\release\abr_fr3.pyd -Destination "C:\Program Files\Blender Foundation\Blender 5.2\5.2\python\lib\site-packages\abr_fr3.pyd"
```

## Test

```powershell
cargo test
```

Expected: 14 passed, 0 failed, 0 ignored.

## Declared Field

Three nodes, three edges — change-based observable field:

- x[ΔP_prior] → x[ΔP] → x[Δτ] → x[Δθ]
- Interior locus: x[Δτ] — ABR kernel applies, R is not optional
- Rest state: all change quantities zero — declared ground before motion

M declarations from FR3 URDF and Product Manual R02210/1.5:
- τ_max = 87.0 N·m, ω_max = 2.0 rad/s, P_max = 174.0 W
- θ_range = ±2.3093 rad, ρ_base = 1.0 (neutral pose floor)

## Demonstration

Load `blender/abr_fr3_demo.py` in Blender's Text Editor and run (Alt+P).

The arm executes a 90-frame declared sequence:
- Rest (5): field at zero, arm at neutral
- Free motion (30): cosine ease-in to 45°
- Contact (20): arm held, field sustains nonzero R
- Release (30): cosine ease-out to 0°
- Rest (5): field returns to zero

Field values A[τ→θ], B[ΔP→τ], R[τ→θ], and joint command update
live in the viewport as the animation plays.
