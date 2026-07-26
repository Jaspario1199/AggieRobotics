"""
Distance-sensor mount -- the ball-present sensor the control table depends on
(red-team R3 F14: it existed only as a BOM row).

A wedge cradle on the head's TOP deck rear centre, aiming the V5 Distance
Sensor 20 deg down-forward through the rear opening into the ball channel.
Sensor straps to the 35 x 25 seat with a zip tie through the side slots; the
base bolts to real deck grid holes (cols +/-12.7, row -63.5). Print base-down,
no supports. PETG.
"""

from __future__ import annotations

import cadquery as cq

from ..params import VEX_GRID, VEX_HOLE, SIDE_INNER_HALF, PLATE_THK

Z0 = SIDE_INNER_HALF + PLATE_THK      # top deck top face (+91)


def make() -> cq.Workplane:
    base = (cq.Workplane("XY").box(40.0, 30.0, 4.0)
            .translate((0, -5 * VEX_GRID, Z0 + 2.0)))
    wedge = (cq.Workplane("YZ").workplane(offset=-17.5)
             .polyline([(-78, Z0 + 4), (-50, Z0 + 4), (-50, Z0 + 14)])
             .close().extrude(35.0))
    m = base.union(wedge)
    for sx in (1, -1):
        m = m.cut(cq.Workplane("XY").circle(VEX_HOLE / 2).extrude(6)
                  .translate((sx * VEX_GRID, -5 * VEX_GRID, Z0 - 1)))
        slot = (cq.Workplane("XY").box(4, 3.5, 20)
                .translate((sx * 16, -64, Z0 + 8)))
        m = m.cut(slot)
    return m


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "sensor_mount"))
