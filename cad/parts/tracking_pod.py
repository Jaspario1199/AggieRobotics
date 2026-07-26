"""
Tracking pod -- dead-wheel odometry fork (R3 F14: auton aiming had no
hardware). Holds a VEX 2.75" omni + rotation sensor on a 1/4"-bolt pivot;
rubber bands from the band post to the chassis preload the wheel onto the
tile. Top flange bolts to the chassis C-channel underside on the 0.5" grid.
Print two (X and Y axes). Print flange-down; fork legs rise vertically. PETG.
"""

from __future__ import annotations

import cadquery as cq

from ..params import VEX_GRID, VEX_HOLE, VEX_SHAFT_CLEAR

WHEEL_D = 69.85        # 2.75" omni
LEG_GAP = 30.0


def make() -> cq.Workplane:
    flange = cq.Workplane("XY").box(70.0, 30.0, 5.0, centered=(True, True, False))
    pod = flange
    for sx in (1, -1):
        leg = (cq.Workplane("XY")
               .box(6.0, 30.0, WHEEL_D / 2 + 18.0, centered=(True, True, False))
               .translate((sx * (LEG_GAP / 2 + 3), 0, -WHEEL_D / 2 - 18.0 + 0.01)))
        pod = pod.union(leg)
        bore = (cq.Workplane("YZ").workplane(offset=sx * (LEG_GAP / 2 + 7))
                .center(0, -WHEEL_D / 2 - 8.0).circle(VEX_SHAFT_CLEAR / 2)
                .extrude(-sx * 8))
        pod = pod.cut(bore)
    post = (cq.Workplane("XY").circle(4.0).extrude(12.0)
            .translate((0, 18.0, 2.0)))
    pod = pod.union(post)
    for (px, py) in [(2 * VEX_GRID, 0), (-2 * VEX_GRID, 0)]:
        pod = pod.cut(cq.Workplane("XY").circle(VEX_HOLE / 2).extrude(7)
                      .translate((px, py, -1)))
    return pod


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "tracking_pod"))
