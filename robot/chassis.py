"""
Phase 2 — X-drive chassis (parametric).

The square VEX aluminum C-channel frame, the four 45° omni + V5 corner drive
modules, and the top deck the arm/electronics mount to. Renders a colour-coded
chassis view.  Run:  python -m robot.chassis

World frame: X = right, +Y = forward, Z = up; origin on the floor at frame center.
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

# --- parameters (mm) -------------------------------------------------------
FRAME = 360.0            # outer square size (inside the 381 mm cube)
CH_WEB = 38.0            # C-channel web width (~1.5")
CH_FLANGE = 19.0         # flange height (~0.75")
CH_WALL = 2.6
GRID = 12.7             # 0.5" hole grid
HOLE = 4.9
WHEEL_D = 82.5          # 3.25" omni
WHEEL_T = 25.0
MOTOR = (44.0, 44.0, 90.0)   # V5 Smart Motor envelope
DECK_THK = 6.0


def _box(dx, dy, dz, c=(0, 0, 0)):
    return cq.Workplane("XY").box(dx, dy, dz).translate(c)


def _channel(length):
    """A C-channel of given length running along X, opening +Z, centered on X."""
    web = _box(length, CH_WEB, CH_WALL, (0, 0, CH_WALL / 2))
    f = CH_WEB / 2 - CH_WALL / 2
    web = web.union(_box(length, CH_WALL, CH_FLANGE, (0, f, CH_FLANGE / 2)))
    web = web.union(_box(length, CH_WALL, CH_FLANGE, (0, -f, CH_FLANGE / 2)))
    # 0.5" hole grid down the web
    n = int((length - GRID) // GRID)
    xs = [(-(n - 1) / 2 + i) * GRID for i in range(n)]
    web = (web.faces(">Z").workplane(centerOption="CenterOfBoundBox")
           .pushPoints([(x, 0) for x in xs]).hole(HOLE))
    return web


def build():
    parts = []
    e = FRAME / 2 - CH_WEB / 2
    parts.append(("C-channel frame", _channel(FRAME).translate((0, e, 0)), "#8f9bad"))
    parts.append(("_f2", _channel(FRAME).translate((0, -e, 0)), "#8f9bad"))
    side = FRAME - 2 * CH_WEB
    for sx in (1, -1):
        r = _channel(side).rotate((0, 0, 0), (0, 0, 1), 90).translate((sx * e, 0, 0))
        parts.append(("_f3" if sx > 0 else "_f4", r, "#8f9bad"))

    # Corner drive modules: omni at 45°, V5 motor inboard.
    c = FRAME / 2 - 44
    first_w = first_m = True
    for (sx, sy) in [(1, 1), (1, -1), (-1, 1), (-1, -1)]:
        ang = 45 if sx * sy > 0 else -45
        w = (cq.Workplane("XY").circle(WHEEL_D / 2).extrude(WHEEL_T)
             .translate((0, 0, -WHEEL_T / 2)).rotate((0, 0, 0), (1, 0, 0), 90)
             .rotate((0, 0, 0), (0, 0, 1), ang).translate((sx * c, sy * c, WHEEL_D / 2)))
        parts.append(("Omni wheel (x4)" if first_w else "_w", w, "#23262b"))
        first_w = False
        # motor tucked inboard along the wheel axis
        mx, my = sx * (c - 60 * np.cos(np.radians(ang))), sy * (c - 60 * abs(np.sin(np.radians(ang))))
        m = (_box(*MOTOR).rotate((0, 0, 0), (0, 1, 0), 90)
             .rotate((0, 0, 0), (0, 0, 1), ang).translate((mx, my, WHEEL_D / 2)))
        parts.append(("V5 motor (x4)" if first_m else "_m", m, "#c24234"))
        first_m = False

    # Top deck.
    deck = (_box(FRAME - 20, FRAME - 20, DECK_THK, (0, 0, CH_FLANGE + DECK_THK / 2)))
    parts.append(("Top deck", deck, "#586170"))
    return parts


def _add(ax, solid, color, acc):
    verts, tris = solid.val().tessellate(0.4)
    V = np.array([[p.x, p.y, p.z] for p in verts]); F = np.array(tris)
    tri = V[F]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    nl = np.linalg.norm(n, axis=1, keepdims=True); nl[nl == 0] = 1; n = n / nl
    L = np.array([0.3, -0.5, 0.8]); L = L / np.linalg.norm(L)
    sh = 0.45 + 0.55 * np.clip(np.abs(n @ L), 0, 1)
    base = np.array([int(color[i:i + 2], 16) / 255 for i in (1, 3, 5)])
    ax.add_collection3d(Poly3DCollection(tri, facecolors=np.clip(sh[:, None] * base, 0, 1),
                                         edgecolors="none"))
    acc.append(V)


def render():
    parts = build()
    fig = plt.figure(figsize=(7, 6.4))
    ax = fig.add_subplot(111, projection="3d")
    acc = []
    legend = {}
    for (n, s, c) in parts:
        _add(ax, s, c, acc)
        if not n.startswith("_"):
            legend[n] = c
    P = np.vstack(acc)
    ctr = P.mean(axis=0); r = (P.max(axis=0) - P.min(axis=0)).max() / 2 or 1
    ax.set_xlim(ctr[0] - r, ctr[0] + r); ax.set_ylim(ctr[1] - r, ctr[1] + r); ax.set_zlim(0, 2 * r)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.view_init(elev=22, azim=-58)
    ax.set_axis_off()
    ax.set_title("Phase 2 — X-drive chassis", fontsize=12.5, pad=0)
    ax.legend(handles=[Patch(facecolor=c, edgecolor="none", label=l) for l, c in legend.items()],
              loc="upper left", fontsize=8.5, framealpha=0.9)
    fig.tight_layout()
    path = os.path.join(PREV, "chassis.png")
    fig.savefig(path, dpi=125, bbox_inches="tight")
    plt.close(fig)
    return path


if __name__ == "__main__":
    print(" ", render())
    print("Done.")
