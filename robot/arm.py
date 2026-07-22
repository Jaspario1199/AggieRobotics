"""
Fold-arm kinematics -- the SINGLE SOURCE OF TRUTH for the robot layout.

Post-gate revision (gate G7 caught 0.0 mm head-wheel clearance; see
docs/DESIGN.md "Gate-driven refinement"):

  * The head is ~227 mm wide; the gap between the corner wheels is ~196 mm, so
    the head can NEVER pass between the wheels -- every swing must clear them
    VERTICALLY. The pivot therefore moved forward and up to (y=155, z=221).
  * ARM_LEN = 0: the carriage/pivot sits at the head's rear face, so the stowed
    head tip (y = 155 - 333 = -178) stays inside the 15" cube (gate G8).
  * SLIDE REMOVED. A straight arm from a high pivot cannot reach the REAR floor:
    the head mid-section would pass z ~= 34 mm over the rear wheels (top 82.5) --
    provably, bottom(y=-136) >= 92 requires pitch >= -0.7 deg, i.e. horizontal.
    Rear floor intake is instead a 180 deg yaw of the holonomic base (~0.3 s).
    The stow pose still works the rear directly: mouth faces rearward at
    ~200 mm height for rear LAUNCH / HOLD / feed.

Poses:
  STOW  = 180 deg -- start-legal fold, head inverted over the deck, mouth rear.
  FRONT = -25 deg -- mouth centre at ball height (~80 mm), ramp at the floor.

Run `python -m robot.arm` to render the swept study.
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

# --- chassis context (matches robot/chassis.py) ----------------------------
FRAME = 360.0
WHEEL_D = 82.5
WHEEL_T = 25.0
CORNER = FRAME / 2 - 44          # wheel centres at (+/-136, +/-136)
DECK_Z = 25.0

# --- head envelope (from the cad/ assembly, gates-verified) ----------------
HEAD_W = 227.0                   # across belts (deck width 2 x 113.5)
HEAD_ALONG = 333.0               # rear face -> flare tip + flywheel allowance
HEAD_N_LO, HEAD_N_HI = -93.0, 128.0   # normal extents about the channel plane
HEAD_N = HEAD_N_HI - HEAD_N_LO
HEAD_N_CTR = (HEAD_N_HI + HEAD_N_LO) / 2

# --- arm ------------------------------------------------------------------
PIVOT = (0.0, 155.0, 221.0)      # shoulder axis (along X)
STOW = 180.0                     # start pose (in-cube), mouth rearward
FRONT = -25.0                    # floor-intake / forward-launch pose
SWEEP = list(range(int(FRONT), int(STOW) + 1, 15)) + [STOW]

CUBE = 381.0                     # 15" start cube


def head_at(phi: float) -> cq.Workplane:
    """Head envelope box at fold angle phi (deg about +X through the pivot)."""
    box = (cq.Workplane("XY")
           .box(HEAD_W, HEAD_ALONG, HEAD_N)
           .translate((0, HEAD_ALONG / 2, HEAD_N_CTR)))
    return box.rotate((0, 0, 0), (1, 0, 0), phi).translate(PIVOT)


def wheels() -> list[cq.Workplane]:
    out = []
    for (sx, sy) in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
        ang = 45 if sx * sy > 0 else -45
        w = (cq.Workplane("XY").circle(WHEEL_D / 2).extrude(WHEEL_T)
             .translate((0, 0, -WHEEL_T / 2))
             .rotate((0, 0, 0), (1, 0, 0), 90)
             .rotate((0, 0, 0), (0, 0, 1), ang)
             .translate((sx * CORNER, sy * CORNER, WHEEL_D / 2)))
        out.append(w)
    return out


def context() -> list[tuple[str, cq.Workplane, str]]:
    """Non-moving stuff for renders: deck, wheels, pivot tower."""
    parts = [("Deck", cq.Workplane("XY").box(FRAME - 20, FRAME - 20, 6)
              .translate((0, 0, DECK_Z)), "#586170")]
    for i, w in enumerate(wheels()):
        parts.append(("Corner wheel (x4)" if i == 0 else "_w", w, "#23262b"))
    for sx in (1, -1):
        blk = (cq.Workplane("XY").box(24, 40, PIVOT[2] - DECK_Z)
               .translate((sx * (HEAD_W / 2 + 14), PIVOT[1],
                           (DECK_Z + PIVOT[2]) / 2)))
        parts.append(("Pivot tower" if sx > 0 else "_t", blk, "#8f9bad"))
    parts.append(("Pivot shaft",
                  cq.Workplane("XY").circle(6).extrude(HEAD_W + 60)
                  .translate((0, 0, -(HEAD_W + 60) / 2))
                  .rotate((0, 0, 0), (0, 1, 0), 90).translate(PIVOT), "#e0872f"))
    return parts


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
    ax.add_collection3d(Poly3DCollection(
        tri, facecolors=np.clip(sh[:, None] * base, 0, 1),
        edgecolors="none", alpha=alpha))
    acc.append(V)


def render() -> str:
    fig = plt.figure(figsize=(7.2, 6.6))
    ax = fig.add_subplot(111, projection="3d")
    acc = []
    legend = {}
    for (n, s, c) in context():
        _add(ax, s, c, 1.0, acc)
        if not n.startswith("_"):
            legend[n] = c
    for phi in SWEEP[1:-1]:
        _add(ax, head_at(phi), "#3d6fb4", 0.10, acc)
    legend["Head swept path"] = "#3d6fb4"
    for phi in (FRONT, STOW):
        _add(ax, head_at(phi), "#e0872f", 0.85, acc)
    legend["End poses (front / stow)"] = "#e0872f"

    ax.set_xlim(-420, 420); ax.set_ylim(-350, 490); ax.set_zlim(0, 840)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.view_init(elev=12, azim=-62)
    ax.set_axis_off()
    ax.set_title("Fold-arm swept study — head always clears the wheels",
                 fontsize=12, pad=0)
    ax.legend(handles=[Patch(facecolor=c, edgecolor="none", label=l)
                       for l, c in legend.items()],
              loc="upper left", fontsize=8, framealpha=0.9)
    fig.tight_layout()
    path = os.path.join(PREV, "arm_swept.png")
    fig.savefig(path, dpi=125, bbox_inches="tight")
    plt.close(fig)
    return path


if __name__ == "__main__":
    print(" ", render())
    print("Done.")
