# ABR FR3 Joint 1 — Blender Demonstration Script
# Metatron Dynamics, Inc. — relationalrelativity.dev
# V0.3.0 — clean declared motion, live field overlay
#
# Requires: abr_fr3.pyd installed in Blender's site-packages
# Build: cargo build --release (with PYO3_PYTHON set to Blender's Python)
# Copy: abr_fr3.dll → abr_fr3.pyd → Blender site-packages
#
# Motion declaration:
#   Rest (5 frames):    field at zero, arm at neutral — declared ground state
#   Free (30 frames):   cosine ease-in to 45° — field advances, positive R
#   Contact (20 frames): arm held at 45° — field sustains nonzero R at Δθ=0
#   Release (30 frames): cosine ease-out to 0° — field winds down
#   Rest (5 frames):    field returns to zero — clean declared end
#
# Field overlay displays per frame:
#   Phase label (color-coded)
#   A[τ→θ] — directed contrast at causal terminal edge
#   B[ΔP→τ] — accumulated relational state at interior edge
#   R[τ→θ] — resolved field at causal terminal edge
#   Cmd — physical torque command in N·m

import bpy
import math
import blf
import abr_fr3
from abr_fr3 import advance_relational_step

# ── Clear everything ──────────────────────────────────────────────────────────

for obj in bpy.data.objects:
    obj.animation_data_clear()
for action in bpy.data.actions:
    bpy.data.actions.remove(action)

# Remove any existing draw handler
if hasattr(bpy.types.Scene, 'abr_handler'):
    try:
        bpy.types.SpaceView3D.draw_handler_remove(
            bpy.types.Scene.abr_handler, 'WINDOW')
    except Exception:
        pass

bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete()

# ── Build FR3 arm geometry ────────────────────────────────────────────────────
# Simplified geometry — correct proportions from FR3 URDF
# Link lengths: joint1 riser 0.333m, link2 0.316m

bpy.ops.mesh.primitive_cylinder_add(radius=0.1, depth=0.05, location=(0, 0, 0.025))
bpy.context.active_object.name = "FR3_Base"

bpy.ops.mesh.primitive_cylinder_add(radius=0.04, depth=0.333, location=(0, 0, 0.2165))
bpy.context.active_object.name = "FR3_Link1"

bpy.ops.mesh.primitive_uv_sphere_add(radius=0.055, location=(0, 0, 0.383))
joint1 = bpy.context.active_object
joint1.name = "FR3_Joint1"

bpy.ops.mesh.primitive_cylinder_add(
    radius=0.035, depth=0.316,
    location=(0.158, 0, 0.383),
    rotation=(0, math.pi/2, 0))
bpy.context.active_object.name = "FR3_Link2"

bpy.ops.mesh.primitive_uv_sphere_add(radius=0.04, location=(0.316, 0, 0.383))
bpy.context.active_object.name = "FR3_EE"

# Parent arm to joint so children rotate with it
bpy.ops.object.select_all(action='DESELECT')
for name in ["FR3_Link2", "FR3_EE"]:
    bpy.data.objects[name].select_set(True)
joint1.select_set(True)
bpy.context.view_layer.objects.active = joint1
bpy.ops.object.parent_set(type='OBJECT', keep_transform=True)

# ── Declared motion and observable sequences ──────────────────────────────────
# Motion: pure cosine curves — deterministic, no conditional logic
# Observables: declared change quantities matching the motion phases
# Both sequences are 90 frames total

TARGET_ANGLE = math.radians(45)
REST         = 5
FREE         = 30
CONTACT      = 20
RELEASE      = 30
REST_END     = 5
TOTAL        = REST + FREE + CONTACT + RELEASE + REST_END  # 90

angles      = []
observables = []
phases      = []

# Rest — field at zero, arm at neutral (declared ground)
for i in range(REST):
    angles.append(0.0)
    observables.append((0.0, 0.0, 0.0, 0.0))
    phases.append("rest")

# Free motion — cosine ease-in, torque always ahead of position
for i in range(FREE):
    t = (i + 1) / FREE
    ease = 0.5 * (1 - math.cos(math.pi * t))
    angles.append(TARGET_ANGLE * ease)
    observables.append((0.12*t, 0.12*t, 0.10*t, 0.04*t))
    phases.append("free")

# Contact — arm held at TARGET_ANGLE, position change zero
for i in range(CONTACT):
    angles.append(TARGET_ANGLE)
    observables.append((0.12, 0.12, 0.10, 0.0))
    phases.append("contact")

# Release — cosine ease-out back to zero
for i in range(RELEASE):
    t = (i + 1) / RELEASE
    ease = 0.5 * (1 - math.cos(math.pi * t))
    angles.append(TARGET_ANGLE * (1.0 - ease))
    observables.append((0.12*(1-t), 0.12*(1-t), 0.10*(1-t), 0.04*(1-t)))
    phases.append("release")

# Rest end — field returns to zero, arm at declared neutral
for i in range(REST_END):
    angles.append(0.0)
    observables.append((0.0, 0.0, 0.0, 0.0))
    phases.append("rest")

# ── Run ABR — store field values, set keyframes ───────────────────────────────

field_data = {}  # frame → {phase, a2, b1, r2, cmd}

joint1 = bpy.data.objects["FR3_Joint1"]
joint1.rotation_euler[2] = 0.0

prior = None

for frame, (angle, obs, phase) in enumerate(zip(angles, observables, phases)):
    result = advance_relational_step(obs[0], obs[1], obs[2], obs[3], prior)

    field_data[frame + 1] = {
        "phase": phase,
        "a2":    result.a[2],   # A at causal terminal edge (Δτ → Δθ)
        "b1":    result.b[1],   # B at interior edge (ΔP → Δτ)
        "r2":    result.r[2],   # R at causal terminal edge
        "cmd":   result.joint_delta,
    }

    joint1.rotation_euler[2] = angle
    joint1.keyframe_insert(
        data_path="rotation_euler", index=2, frame=frame + 1)

    # Set LINEAR interpolation immediately — no Blender bezier smoothing
    if joint1.animation_data and joint1.animation_data.action:
        for layer in joint1.animation_data.action.layers:
            for strip in layer.strips:
                if hasattr(strip, 'channelbags'):
                    for cb in strip.channelbags:
                        for fc in cb.fcurves:
                            if fc.keyframe_points:
                                fc.keyframe_points[-1].interpolation = 'LINEAR'

    prior = result.e_spatial

# ── Viewport overlay — field values per frame ─────────────────────────────────

PHASE_COLORS = {
    "rest":    (0.5, 0.5, 0.5, 1.0),   # grey
    "free":    (0.2, 0.8, 0.2, 1.0),   # green
    "contact": (1.0, 0.4, 0.1, 1.0),   # orange
    "release": (0.2, 0.6, 1.0, 1.0),   # blue
}

def draw_field_overlay():
    frame = bpy.context.scene.frame_current
    data  = field_data.get(frame)
    if data is None:
        return

    font_id = 0
    blf.size(font_id, 18)

    # Phase label — color coded
    color = PHASE_COLORS.get(data["phase"], (1.0, 1.0, 1.0, 1.0))
    blf.color(font_id, *color)
    blf.position(font_id, 30, 220, 0)
    blf.draw(font_id, f"Phase:   {data['phase'].upper()}")

    # Field values — white
    blf.color(font_id, 1.0, 1.0, 1.0, 0.9)

    blf.position(font_id, 30, 190, 0)
    blf.draw(font_id, f"A[τ→θ]:  {data['a2']:+.4f}")

    blf.position(font_id, 30, 165, 0)
    blf.draw(font_id, f"B[ΔP→τ]: {data['b1']:+.4f}")

    blf.position(font_id, 30, 140, 0)
    blf.draw(font_id, f"R[τ→θ]:  {data['r2']:+.4f}")

    blf.position(font_id, 30, 110, 0)
    blf.draw(font_id, f"Cmd:     {data['cmd']:+.3f} N·m")

    blf.position(font_id, 30, 80, 0)
    blf.draw(font_id, f"Frame:   {frame} / {TOTAL}")

# Register draw handler
handler = bpy.types.SpaceView3D.draw_handler_add(
    draw_field_overlay, (), 'WINDOW', 'POST_PIXEL')
bpy.types.Scene.abr_handler = handler

# Set scene range
bpy.context.scene.frame_start = 1
bpy.context.scene.frame_end   = TOTAL
bpy.context.scene.frame_set(1)

print(f"ABR FR3 Demo — {TOTAL} frames")
print(f"Target angle: {math.degrees(TARGET_ANGLE):.0f}°")
print("Field overlay active. Press Space to play.")
print("To remove overlay: bpy.types.SpaceView3D.draw_handler_remove("
      "bpy.types.Scene.abr_handler, 'WINDOW')")
