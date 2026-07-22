"""
Phase 5 — flywheel launcher + compression hood (parametric).

The dedicated launcher the belt intake feeds. A metal-mass flywheel stores the
energy the geared motors can't deliver in a single contact; a curved compression
hood holds the tri-ball against the wheel over a defined arc so it is spun up
consistently and released at a set angle. Run:  python -m robot.flywheel

World frame: X = wheel axis, +Y = forward (launch), Z = up.
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

FLY_D = 100.0            # flywheel outer diameter (metal-mass rim)
FLY_W = 45.0            # flywheel width
HEX_AF = 12.7           # 1/2" hex bore across-flats
BALL_TIP = 157.0
BALL_R = BALL_TIP / 2 * 0.86   # effective launch radius (body-ish)
HOOD_THK = 4.0
LAUNCH_DEG = 42.0       # release angle set by the hood


def _cyl(d, h, c=(0, 0, 0), axis="Z"):
    w = cq.Workplane("XY").circle(d / 2).extrude(h).translate((0, 0, -h / 2))
    if axis == "X":
        w = w.rotate((0, 0, 0), (0, 1, 0), 90)
    elif axis == "Y":
        w = w.rotate((0, 0, 0), (1, 0, 0), 90)
    return w.translate(c)


def _flywheel():
    """Heavy metal disc, hex bore, two traction grooves — spins about X."""
    R = FLY_D / 2
    w = _cyl(FLY_D, FLY_W, axis="X")
    # 1/2" hex bore along X
    ac = HEX_AF * 2 / np.sqrt(3)
    bore = (cq.Workplane("YZ").polygon(6, ac).extrude(FLY_W + 4)
            .translate((-(FLY_W + 4) / 2, 0, 0)))
    w = w.cut(bore)
    # traction grooves
    for xg in (-FLY_W / 4, FLY_W / 4):
        ring = (_cyl(FLY_D + 4, 3, (xg, 0, 0), axis="X")
                .cut(_cyl(FLY_D - 4, 4, (xg, 0, 0), axis="X")))
        w = w.cut(ring)
    return w


def _hood():
    """Curved compression plate hugging the ball's outer side over the launch arc."""
    # centre of the ball, tangent to the flywheel at ~45deg up-front
    d = FLY_D / 2 + BALL_R
    bc = np.array([0, d * np.cos(np.radians(45)), d * np.sin(np.radians(45))])
    r_in = BALL_R + 3
    r_out = r_in + HOOD_THK
    # tube centred on the ball, in the Y-Z plane, extruded along X
    W = BALL_R * 1.4
    tube = (cq.Workplane("YZ").workplane(offset=-W / 2).circle(r_out).circle(r_in)
            .extrude(W).translate((0, bc[1], bc[2])))
    # keep the outer arc facing away from the flywheel (the +radial, launch side)
    cutter = cq.Workplane("XY").box(600, 600, 600).translate((0, bc[1] - 300, bc[2]))
    hood = tube.cut(cutter)
    cut2 = cq.Workplane("XY").box(600, 600, 600).translate((0, bc[1], bc[2] - 300))
    hood = hood.cut(cut2)
    return hood, bc


def _ball(bc):
    r = BALL_TIP / 2 * 0.62
    off = BALL_TIP / 2 - r
    b = None
    for k in range(3):
        a = np.radians(90 + 120 * k)
        # lobes fan in the Y-Z plane (out-of-X)
        s = cq.Workplane("XY").sphere(r).translate(
            (bc[0], bc[1] + off * np.cos(a), bc[2] + off * np.sin(a)))
        b = s if b is None else b.union(s)
    return b


def _add(ax, solid, color, alpha, acc):
    verts, tris = solid.val().tessellate(0.4)
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
    hood, bc = _hood()
    parts = [("Flywheel (metal mass)", _flywheel(), "#aeb4bd", 1.0),
             ("Compression hood", hood, "#3d6fb4", 0.85),
             ("Tri-ball", _ball(bc), "#f0c02a", 0.85)]
    fig = plt.figure(figsize=(7, 6.4))
    ax = fig.add_subplot(111, projection="3d")
    acc = []
    for (n, s, c, a) in parts:
        _add(ax, s, c, a, acc)
    # launch arrow
    d = np.array([0, np.cos(np.radians(LAUNCH_DEG)), np.sin(np.radians(LAUNCH_DEG))])
    tail = bc + d * (BALL_R + 20)
    ax.quiver(*tail, *(d * 90), color="#c24234", linewidth=2.2, arrow_length_ratio=0.35)
    P = np.vstack(acc)
    ctr = P.mean(axis=0); r = (P.max(axis=0) - P.min(axis=0)).max() / 2 + 30
    ax.set_xlim(ctr[0] - r, ctr[0] + r); ax.set_ylim(ctr[1] - r, ctr[1] + r); ax.set_zlim(ctr[2] - r, ctr[2] + r)
    try:
        ax.set_box_aspect((1, 1, 1))
    except Exception:
        pass
    ax.view_init(elev=16, azim=-72)
    ax.set_axis_off()
    ax.set_title("Phase 5 — flywheel launcher + compression hood", fontsize=12, pad=0)
    handles = [Patch(facecolor=c, edgecolor="none", label=n) for (n, _, c, _) in parts]
    handles.append(Patch(facecolor="#c24234", edgecolor="none", label="launch (~42°)"))
    ax.legend(handles=handles, loc="upper left", fontsize=8.5, framealpha=0.9)
    fig.tight_layout()
    path = os.path.join(PREV, "flywheel.png")
    fig.savefig(path, dpi=125, bbox_inches="tight")
    plt.close(fig)
    return path


if __name__ == "__main__":
    print(" ", render())
    print("Done.")
