"""
Full-robot LAYOUT + fold-pose study (parametric, primitive-volume level).

Turns DESIGN.md v1.0 into geometry to prove two things before detailed CAD:
  1. the robot stows inside the 15" (381 mm) VEXU start cube, and
  2. the center-pivot + telescoping-slide arm reaches the front floor (deployed)
     and the rear floor (rear-deployed) as intended.

Volumes are primitives (boxes/cylinders) standing in for real subsystems; the
exact fold angles + link lengths are refined in the next iteration's swept-volume
study. Run:  python -m robot.layout

World frame: X = right, +Y = forward (muzzle when deployed front), Z = up;
origin on the floor at the robot's footprint center.
"""

from __future__ import annotations

import os
import numpy as np
import cadquery as cq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

HERE = os.path.dirname(__file__)
PREV = os.path.join(HERE, "previews")
os.makedirs(PREV, exist_ok=True)

# --- robot parameters (mm) -------------------------------------------------
FRAME = 360.0            # square frame footprint (inside the 381 mm cube)
DECK_Z = 100.0           # drive-deck height
WHEEL_D = 82.5           # 3.25" omni
WHEEL_T = 25.0
PIVOT = (0.0, -30.0, 110.0)   # shoulder pivot (x, y, z); axis along X
ARM_BASE = 30.0          # pivot -> head carriage, retracted
SLIDE_MAX = 150.0        # telescoping-slide stroke
HEAD = (210.0, 180.0, 130.0)  # belt-head envelope (X across belts, Y along arm, Z)
FLY_D = 90.0             # flywheel launcher diameter
BALL_TIP = 157.0         # tri-ball tip-to-tip
CUBE = 381.0             # 15" VEXU start cube


def _box(dx, dy, dz, c=(0, 0, 0)):
    return cq.Workplane("XY").box(dx, dy, dz).translate(c)


def _cyl(d, h, c=(0, 0, 0), axis="Z"):
    w = cq.Workplane("XY").circle(d / 2).extrude(h).translate((0, 0, -h / 2))
    if axis == "X":
        w = w.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "Y":
        w = w.rotate((0, 0, 0), (1, 0, 0), 90)
    return w.translate(c)


def _static():
    """Chassis, X-drive, electronics, pivot — the parts that don't move."""
    P = []
    P.append(("Chassis", _box(FRAME, FRAME, DECK_Z, (0, 0, DECK_Z / 2)), "#8f9bad", 0.22))
    c = FRAME / 2 - 30
    for (sx, sy) in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
        ang = 45 if sx * sy > 0 else -45
        w = (_cyl(WHEEL_D, WHEEL_T, axis="Y").rotate((0, 0, 0), (0, 0, 1), ang)
             .translate((sx * c, sy * c, WHEEL_D / 2)))
        P.append(("X-drive omni (x4)", w, "#2b2f36", 1.0))
    P.append(("Battery", _box(90, 90, 45, (-95, -95, DECK_Z + 22)), "#c24234", 1.0))
    P.append(("V5 Brain", _box(120, 80, 40, (95, -95, DECK_Z + 20)), "#586170", 1.0))
    P.append(("Air reservoir", _box(60, 170, 60, (0, -80, DECK_Z + 30)), "#41b06a", 1.0))
    P.append(("Shoulder pivot", _cyl(20, 250, PIVOT, axis="X"), "#e0872f", 1.0))
    return P


def _arm(slide):
    """Arm assembly in its LOCAL frame: pivot at origin, arm along +Y."""
    L = ARM_BASE + slide
    A = []
    A.append(("Fold arm + slide", _box(60, L, 55, (0, L / 2, 0)), "#e0872f", 1.0))
    hy = L + HEAD[1] / 2
    A.append(("Belt head (intake/hold/push)", _box(*HEAD, (0, hy, 0)), "#3d6fb4", 1.0))
    A.append(("Flywheel launcher", _cyl(FLY_D, 130, (0, L + HEAD[1], 0), axis="X"), "#e0b020", 1.0))
    return A, L + HEAD[1] + FLY_D / 2   # also return the mouth reach along +Y (local)


def _place(parts, phi, pivot):
    out = []
    for (n, s, col, a) in parts:
        out.append((n, s.rotate((0, 0, 0), (1, 0, 0), phi).translate(pivot), col, a))
    return out


def _triball(center):
    """Rough 3-lobe proxy = three fused spheres in a plane."""
    r = BALL_TIP / 2 * 0.62
    off = BALL_TIP / 2 - r
    b = None
    for k in range(3):
        a = np.radians(90 + 120 * k)
        s = cq.Workplane("XY").sphere(r).translate(
            (center[0] + off * np.cos(a), center[1] + off * np.sin(a), center[2]))
        b = s if b is None else b.union(s)
    return b


def _add(ax, solid, color, alpha, acc):
    verts, tris = solid.val().tessellate(0.6)
    V = np.array([[p.x, p.y, p.z] for p in verts])
    F = np.array(tris)
    tri = V[F]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nl = np.linalg.norm(n, axis=1, keepdims=True); nl[nl == 0] = 1; n = n / nl
    L = np.array([0.3, -0.5, 0.8]); L = L / np.linalg.norm(L)
    sh = 0.45 + 0.55 * np.clip(np.abs(n @ L), 0, 1)
    base = np.array([int(color[i:i + 2], 16) / 255 for i in (1, 3, 5)])
    ax.add_collection3d(Poly3DCollection(tri, facecolors=np.clip(sh[:, None] * base, 0, 1),
                                         edgecolors="none", alpha=alpha))
    acc.append(V)


def _cube_edges(ax):
    h = CUBE / 2
    pts = [(-h, -h, 0), (h, -h, 0), (h, h, 0), (-h, h, 0),
           (-h, -h, CUBE), (h, -h, CUBE), (h, h, CUBE), (-h, h, CUBE)]
    E = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
         (0, 4), (1, 5), (2, 6), (3, 7)]
    segs = [[pts[a], pts[b]] for (a, b) in E]
    ax.add_collection3d(Line3DCollection(segs, colors="#d04040", linewidths=1.1, linestyles="--"))


def render_pose(title, phi, slide, path, show_cube=False, show_ball=False):
    fig = plt.figure(figsize=(6.6, 6.2))
    ax = fig.add_subplot(111, projection="3d")
    acc = []
    parts = list(_static())
    arm, reach = _arm(slide)
    parts += _place(arm, phi, PIVOT)
    legend = {}
    for (n, s, col, a) in parts:
        _add(ax, s, col, a, acc)
        legend.setdefault(n, col)
    if show_ball:
        pr = np.radians(phi)
        mouth = (PIVOT[0], PIVOT[1] + (reach + 55) * np.cos(pr), PIVOT[2] + (reach + 55) * np.sin(pr))
        _add(ax, _triball(mouth), "#f0c02a", 0.5, acc)
        legend["Tri-ball"] = "#f0c02a"
    if show_cube:
        _cube_edges(ax)

    P = np.vstack(acc)
    ctr = np.array([0, 0, CUBE / 2.0])
    r = CUBE / 2 + 60
    ax.set_xlim(ctr[0] - r, ctr[0] + r); ax.set_ylim(ctr[1] - r, ctr[1] + r); ax.set_zlim(0, 2 * r)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.view_init(elev=16, azim=-58)
    ax.set_axis_off()
    ax.set_title(title, fontsize=12, pad=0)
    handles = [Patch(facecolor=c, edgecolor="none", label=l) for l, c in legend.items()]
    if show_cube:
        handles.append(Patch(facecolor="none", edgecolor="#d04040", label='15" start cube'))
    ax.legend(handles=handles, loc="upper left", fontsize=7.5, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return path


def build_all():
    out = []
    out.append(render_pose("Stowed — legal start (inside the 15\" cube)",
                            phi=0, slide=0,
                            path=os.path.join(PREV, "pose_stowed.png"), show_cube=True))
    out.append(render_pose("Deployed FRONT — floor intake / launch",
                            phi=-40, slide=SLIDE_MAX,
                            path=os.path.join(PREV, "pose_front.png"), show_ball=True))
    out.append(render_pose("Deployed REAR — works off the back",
                            phi=220, slide=SLIDE_MAX,
                            path=os.path.join(PREV, "pose_rear.png"), show_ball=True))
    return out


if __name__ == "__main__":
    for p in build_all():
        print(" ", p)
    print("Done.")
