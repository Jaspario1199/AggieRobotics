"""
Robot pose renders -- stow-in-cube / front-deploy / rear-work.

All kinematics come from robot.arm (the single source of truth); this module
only adds the electronics boxes, the 15" cube overlay, and tri-ball ghosts.
Gate G8 (stow-in-cube) reads robot.arm directly, so these renders can never
drift from what the gate verifies.  Run:  python -m robot.layout
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

from . import arm as A

HERE = os.path.dirname(__file__)
PREV = os.path.join(HERE, "previews")
os.makedirs(PREV, exist_ok=True)

BALL_TIP = 157.0


def _box(dx, dy, dz, c=(0, 0, 0)):
    return cq.Workplane("XY").box(dx, dy, dz).translate(c)


def _electronics():
    """Battery / brain / counterweight, low and rearward (pneumatics deleted -- R3 F15)."""
    return [
        ("Battery", _box(90, 90, 45, (-95, -120, A.DECK_Z + 25)), "#c24234"),
        ("V5 Brain", _box(120, 80, 40, (95, -120, A.DECK_Z + 23)), "#586170"),
        ("Steel counterweight", _box(60, 100, 40, (0, -140, A.DECK_Z + 23)), "#6b7280"),
    ]


def _triball(center):
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
    ax.add_collection3d(Poly3DCollection(
        tri, facecolors=np.clip(sh[:, None] * base, 0, 1),
        edgecolors="none", alpha=alpha))
    acc.append(V)


def _cube_edges(ax):
    h = A.CUBE / 2
    pts = [(-h, -h, 0), (h, -h, 0), (h, h, 0), (-h, h, 0),
           (-h, -h, A.CUBE), (h, -h, A.CUBE), (h, h, A.CUBE), (-h, h, A.CUBE)]
    E = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
         (0, 4), (1, 5), (2, 6), (3, 7)]
    ax.add_collection3d(Line3DCollection(
        [[pts[a], pts[b]] for (a, b) in E],
        colors="#d04040", linewidths=1.1, linestyles="--"))


def render_pose(title, phi, path, show_cube=False, ball=None, azim=-58):
    fig = plt.figure(figsize=(6.6, 6.2))
    ax = fig.add_subplot(111, projection="3d")
    acc = []
    legend = {}
    for (n, s, c) in A.context():
        _add(ax, s, c, 1.0, acc)
        if not n.startswith("_"):
            legend[n] = c
    for (n, s, c) in _electronics():
        _add(ax, s, c, 1.0, acc)
        legend[n] = c
    _add(ax, A.head_at(phi), "#3d6fb4", 0.9, acc)
    legend["Belt head (intake/hold/push + flywheel)"] = "#3d6fb4"
    if ball is not None:
        _add(ax, _triball(ball), "#f0c02a", 0.55, acc)
        legend["Tri-ball"] = "#f0c02a"
    if show_cube:
        _cube_edges(ax)

    r = A.CUBE / 2 + 130
    ax.set_xlim(-r, r); ax.set_ylim(-r + 40, r + 40); ax.set_zlim(0, 2 * r)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.view_init(elev=16, azim=azim)
    ax.set_axis_off()
    ax.set_title(title, fontsize=11.5, pad=0)
    handles = [Patch(facecolor=c, edgecolor="none", label=l)
               for l, c in legend.items()]
    if show_cube:
        handles.append(Patch(facecolor="none", edgecolor="#d04040",
                             label='15" start cube'))
    ax.legend(handles=handles, loc="upper left", fontsize=7, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    return path


def build_all():
    import math
    # front mouth: channel centre at the head's far end
    my = A.PIVOT[1] + A.HEAD_ALONG * math.cos(math.radians(A.FRONT))
    out = [
        render_pose("Stowed — legal start (gate G8: fits the 15\" cube)",
                    A.STOW, os.path.join(PREV, "pose_stowed.png"),
                    show_cube=True),
        render_pose("Deployed FRONT — floor intake / launch (mouth at ball height)",
                    A.FRONT, os.path.join(PREV, "pose_front.png"),
                    ball=(0, my + 60, BALL_TIP / 2)),
        render_pose("STOW rear-work — HOLD / FEED only (nip inverts: launches are front-side)",
                    A.STOW, os.path.join(PREV, "pose_rear.png"),
                    ball=(0, A.PIVOT[1] - A.HEAD_ALONG - 70, A.PIVOT[2]),
                    azim=-122),
    ]
    return out


if __name__ == "__main__":
    for p in build_all():
        print(" ", p)
    print("Done.")
