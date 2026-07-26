"""
Flywheel mount -- hangs the under-mouth launcher shaft below the bottom deck.

One per side of the flywheel notch (print TWO, the second MIRRORED). A base
plate bolts to the bottom deck's OUTER face on real grid holes (columns 25.4 /
38.1 -- free since the plow moved to +/-63.5); a drop arm carries the bearing
boss whose bore sits exactly on the flywheel axis (FLY_Y, FLY_Z). Snap a VEX
bearing flat over the bore; the 1/2" hex shaft carries the 3" flex wheel + the
fabricated mass disc between the two mounts.

Modelled in HEAD-ASSEMBLY coordinates (+X side). Gates: bolts on real grid
holes, bore on the flywheel axis, arm clears the wheel width (G9).

Print base-down; the drop arm is a straight wall (no supports). PETG, 4 walls.
"""

from __future__ import annotations

import cadquery as cq

from ..params import (
    VEX_GRID, VEX_HOLE, VEX_SHAFT_CLEAR, SIDE_INNER_HALF, PLATE_THK,
    FLY_Y, FLY_Z, FLY_W, BARREL_LEN,
)

X0 = 33.7                   # arm inner face: 2 mm outside the O72 mass disc
                            # (wheel +12.7 .. disc 13.7..29.7 .. arm 33.7)
ARM_T = 8.0
BASE_T = 4.0
EDGE = BARREL_LEN / 2 + 30.0
# Rows 3-4 x GRID: the plow plate covers world y 60..85 on the deck's inner
# face, so nuts at rows 38.1/50.8 land CLEAR of it (R3 A4).
BOLT_PTS = [(3 * VEX_GRID, 3 * VEX_GRID), (4 * VEX_GRID, 3 * VEX_GRID),
            (3 * VEX_GRID, 4 * VEX_GRID), (4 * VEX_GRID, 4 * VEX_GRID)]

Z_FACE = -(SIDE_INNER_HALF + PLATE_THK)     # bottom deck outer face (-91)


def make() -> cq.Workplane:
    # Base plate on the deck's outer face, spanning the bolt columns.
    # base inner edge at x=20: clear of the wheel disc (x <= 17.5)
    # inner edge at x=32: clear of the disc's swept plane (outer 29.7)
    base = (cq.Workplane("XY")
            .box(32.0, 2 * VEX_GRID + 20, BASE_T,
                 centered=(True, True, False))
            .translate((48.0, 3.5 * VEX_GRID, Z_FACE - BASE_T)))

    # Drop arm: straight wall from the base down to the bearing boss.
    arm = (cq.Workplane("YZ").workplane(offset=X0)
           .polyline([(EDGE, Z_FACE - BASE_T), (3 * VEX_GRID - 12, Z_FACE - BASE_T),
                      (FLY_Y - 16, FLY_Z - 8), (FLY_Y + 16, FLY_Z - 8)])
           .close().extrude(ARM_T))
    boss = (cq.Workplane("YZ").workplane(offset=X0)
            .center(FLY_Y, FLY_Z).circle(16.0).extrude(ARM_T))
    m = base.union(arm).union(boss)

    # Bearing bore on the flywheel axis.
    bore = (cq.Workplane("YZ").workplane(offset=X0 - 1)
            .center(FLY_Y, FLY_Z).circle(VEX_SHAFT_CLEAR / 2)
            .extrude(ARM_T + 2))
    m = m.cut(bore)

    # Bearing-flat bolt pair at +/-12.7 from the bore (R3 C4).
    for d in (VEX_GRID, -VEX_GRID):
        bh = (cq.Workplane("YZ").workplane(offset=X0 - 1)
              .center(FLY_Y + d, FLY_Z).circle(VEX_HOLE / 2).extrude(ARM_T + 2))
        m = m.cut(bh)

    # Bolts through base + deck (nylocks clear of the plow plate).
    for (px, py) in BOLT_PTS:
        hole = (cq.Workplane("XY").circle(VEX_HOLE / 2)
                .extrude(BASE_T + 2).translate((px, py, Z_FACE - BASE_T - 1)))
        m = m.cut(hole)

    return m


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "fly_mount"))
