"""
Pivot block -- chassis-side bearing block for the fold-pivot shaft.

One per side (print two, identical -- the part is symmetric). Sits on top of the
pivot tower (stock VEX C-channel, chassis side) and carries the round bearing
bore the 1/2" hex pivot shaft spins in (snap a VEX bearing flat over the bore
face; the block itself never takes the shaft directly). Two blocks = dual shear.

Local frame: base on z=0; bore axis along X at height BORE_H. The robot places
the towers so base_z + BORE_H equals the pivot height (gate G9e checks the
arithmetic; G9d checks the block stays outboard of the swinging hub boss).

Print base-down, no supports. PETG, 4+ walls, 50% infill: primary load path.
"""

from __future__ import annotations

import cadquery as cq

from ..params import VEX_GRID, VEX_HOLE, VEX_SHAFT_CLEAR

BASE_X, BASE_Y, BASE_T = 30.0, 50.0, 6.0
WALL_T = 20.0          # upright wall thickness (along the shaft axis)
BORE_H = 45.0          # bore centre height above the base bottom
TOP_R = 16.0           # rounded head around the bore


def make() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_X, BASE_Y, BASE_T,
                                  centered=(True, True, False))
    up = (cq.Workplane("XY").box(WALL_T, 34.0, BORE_H - BASE_T,
                                 centered=(True, True, False))
          .translate((0, 0, BASE_T)))
    head = (cq.Workplane("YZ").workplane(offset=-WALL_T / 2)
            .center(0, BORE_H).circle(TOP_R).extrude(WALL_T))
    blk = base.union(up).union(head)

    # Bearing bore (a VEX bearing flat snaps over this on the shaft side).
    bore = (cq.Workplane("YZ").workplane(offset=-WALL_T / 2 - 1)
            .center(0, BORE_H).circle(VEX_SHAFT_CLEAR / 2).extrude(WALL_T + 2))
    blk = blk.cut(bore)

    # 4 hold-down bolts on the 0.5" grid, through the base into the tower cap.
    pts = [(sx * VEX_GRID / 2, sy * VEX_GRID)
           for sx in (1, -1) for sy in (1, -1)]
    for (px, py) in pts:
        hole = (cq.Workplane("XY").circle(VEX_HOLE / 2)
                .extrude(BASE_T + 2).translate((px, py, -1)))
        blk = blk.cut(hole)

    return blk


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "pivot_block"))
