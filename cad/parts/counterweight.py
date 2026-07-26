"""
Rear counterweight -- FABRICATED STEEL 90 x 50 x 20 mm (~700 g), replacing the
deleted pneumatic reservoir's counterbalance role (R3 F15). Bolts low and
rearward on the chassis deck grid; gate G10's tip margin assumes it.
"""

from __future__ import annotations

import cadquery as cq

from ..params import VEX_GRID, VEX_HOLE


def make() -> cq.Workplane:
    w = cq.Workplane("XY").box(90.0, 50.0, 20.0, centered=(True, True, False))
    for sx in (1, -1):
        w = w.cut(cq.Workplane("XY").circle(VEX_HOLE / 2).extrude(22)
                  .translate((sx * 2 * VEX_GRID, 0, -1)))
    return w


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "counterweight"))
