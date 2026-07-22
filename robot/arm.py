"""
Phase 3 — fold arm + swept-volume study (parametric).

Models the shoulder (dual pivot blocks), the telescoping slide + link, and the
belt-head carriage, then renders the head GHOSTED through its full swing over the
chassis to prove the swept path clears the corner drive wheels (the tight spot the
phase-1 layout flagged). Run:  python -m robot.arm

Key result to read off the render: the head swings in the CENTRAL fore-aft plane
(x≈0), so it passes BETWEEN the corner wheels; it only gets low near the wheels at
the two end poses, where the ~210 mm head sits in the ~240 mm inter-wheel gap
(~15 mm/side clearance). World frame: X right, +Y forward, Z up; floor at z=0.
"""

from __future__ import annotations

import os
import numpy as np
import cadquery as cq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(__file__)
PREV = os.path.join(HERE, "previews")
os.makedirs(PREV, exist_ok=True)

# --- chassis context (from phase 2) ---------------------------------------
FRAME = 360.0
DECK_Z = 25.0            # deck top height
WHEEL_D = 82.5
CORNER = FRAME / 2 - 44  # wheel/motor corner offset (=136)

# --- arm parameters (mm) --------------------------------------------------
PIVOT = (0.0, 0.0, 125.0)   # shoulder axis (x,y,z); axis along X — raised for clearance
BLOCK_X = 118.0             # pivot bearing blocks sit just outside the head width
ARM_LEN = 120.0             # pivot -> carriage, retracted
SLIDE = 150.0               # telescoping stroke
HEAD = (210.0, 180.0, 130.0)  # belt-head envelope (X, Y-along-arm, Z)

# fold angles (deg) about +X, dir=(0,cos,sin): -40 front-down … +220 rear-down
STOW, FRONT, REAR = 8.0, -40.0, 220.0


def _box(dx, dy, dz, c=(0, 0, 0)):
    return cq.Workplane("XY").box(dx, dy, dz).translate(c)


def _cyl(d, h, c=(0, 0, 0), axis="Z"):
    w = cq.Workplane("XY").circle(d / 2).extrude(h).translate((0, 0, -h / 2))
    if axis == "X":
        w = w.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "Y":
        w = w.rotate((0, 0, 0), (1, 0, 0), 90)
    return w.translate(c)


def _context():
    """Non-moving stuff: deck outline, 4 corner wheels, the 2 pivot blocks."""
    P = []
    P.append(("Deck", _box(FRAME - 20, FRAME - 20, 6, (0, 0, DECK_Z - 3)), "#586170", 1.0))
    for (sx, sy) in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
        ang = 45 if sx * sy > 0 else -45
        w = (_cyl(WHEEL_D, 25, axis="Y").rotate((0, 0, 0), (0, 0, 1), ang)
             .translate((sx * CORNER, sy * CORNER, WHEEL_D / 2)))
        P.append(("Corner wheel (x4)" if (sx, sy) == (1, 1) else "_w", w, "#23262b", 1.0))
    for sx in (1, -1):
        blk = _box(26, 44, PIVOT[2] - DECK_Z + 20,
                   (sx * BLOCK_X, PIVOT[1], (DECK_Z + PIVOT[2]) / 2))
        P.append(("Pivot blocks" if sx > 0 else "_b", blk, "#8f9bad", 1.0))
    P.append(("Pivot shaft", _cyl(12, 2 * BLOCK_X + 20, PIVOT, axis="X"), "#e0872f", 1.0))
    return P


def _arm(slide):
    """Arm+head in local frame: pivot at origin, arm along +Y."""
    L = ARM_LEN + slide
    A = []
    A.append(_box(50, L, 44, (0, L / 2, 0)))                       # slide/link
    A.append(_box(*HEAD, (0, L + HEAD[1] / 2, 0)))                 # head
    A.append(_cyl(90, 130, (0, L + HEAD[1], 0), axis="X"))         # flywheel
    return A


def _place(solids, phi):
    return [s.rotate((0, 0, 0), (1, 0, 0), phi).translate(PIVOT) for s in solids]


def _add(ax, solid, color, alpha, acc):
    verts, tris = solid.val().tessellate(0.5)
    V = np.array([[p.x, p.y, p.z] for p in verts]); F = np.array(tris)
    tri = V[F]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nl = np.linalg.norm(n, axis=1, keepdims=True); nl[nl == 0] = 1; n = n / nl
    Ld = np.array([0.3, -0.5, 0.8]); Ld = Ld / np.linalg.norm(Ld)
    sh = 0.45 + 0.55 * np.clip(np.abs(n @ Ld), 0, 1)
    base = np.array([int(color[i:i + 2], 16) / 255 for i in (1, 3, 5)])
    ax.add_collection3d(Poly3DCollection(tri, facecolors=np.clip(sh[:, None] * base, 0, 1),
                                         edgecolors="none", alpha=alpha))
    acc.append(V)


def render():
    fig = plt.figure(figsize=(7.2, 6.6))
    ax = fig.add_subplot(111, projection="3d")
    acc = []
    legend = {}
    for (n, s, c, a) in _context():
        _add(ax, s, c, a, acc)
        if not n.startswith("_"):
            legend[n] = c

    # Ghost the swing through the transition (slide retracted while swinging).
    ghosts = [-40, 10, 60, 110, 160, 220]
    for phi in ghosts:
        for s in _place(_arm(0.0), phi):
            _add(ax, s, "#3d6fb4", 0.16, acc)
    legend["Head swept path"] = "#3d6fb4"
    # Solid at the two deployed ends (slide extended to the floor).
    for phi in (FRONT, REAR):
        for s in _place(_arm(SLIDE), phi):
            _add(ax, s, "#e0872f", 0.9, acc)
    legend["Deployed (front + rear)"] = "#e0872f"

    P = np.vstack(acc)
    ctr = np.array([0, 0, 190]); r = 330
    ax.set_xlim(-r, r); ax.set_ylim(ctr[1] - r, ctr[1] + r); ax.set_zlim(0, 2 * r)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.view_init(elev=14, azim=-62)
    ax.set_axis_off()
    ax.set_title("Phase 3 — fold arm swept-volume (clears the corner wheels)", fontsize=12, pad=0)
    ax.legend(handles=[Patch(facecolor=c, edgecolor="none", label=l) for l, c in legend.items()],
              loc="upper left", fontsize=8, framealpha=0.9)
    fig.tight_layout()
    path = os.path.join(PREV, "arm_swept.png")
    fig.savefig(path, dpi=125, bbox_inches="tight")
    plt.close(fig)
    return path


if __name__ == "__main__":
    print(" ", render())
    print("Done.")
