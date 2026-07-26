"""
Flywheel drive plate -- mounts the launcher's V5 motor and gears it to the
flywheel shaft (the last "ghost motor" from red-team R3 F12).

An L-plate under the bottom deck: a flange bolts to the deck's outer face on
real grid holes (rows 0/-12.7 -- clear of the fly_mount bases at rows 3-4); a
vertical web hangs down carrying the motor pilot bore and the flywheel-shaft
pass bore at exactly GEAR_CD (12T:60T mesh). The motor body points INBOARD
(x 54..144) below the deck: verified clear of the deck (z < -91), belts
(z >= -60), plow blade (z >= -106), and the floor at FRONT pose (+8 mm).

Print flange-down (web rises vertically, no supports). PETG, 4+ walls.
"""

from __future__ import annotations

import math
import cadquery as cq

from ..params import (
    VEX_GRID, VEX_HOLE, VEX_SHAFT_CLEAR, SIDE_INNER_HALF, PLATE_THK,
    FLY_Y, FLY_Z, GEAR_CD,
)

WEB_X0 = 48.0          # web inner face (gear stack lives at 42.5..48)
WEB_T = 6.0
MOTOR_PILOT = 26.0
# Motor axis: below-behind the fly shaft so the body clears everything.
MOT_Y, MOT_Z = 46.0, -133.0
_cd = math.hypot(FLY_Y - MOT_Y, FLY_Z - MOT_Z)
assert abs(_cd - GEAR_CD) < 1.0, f"gear CD {_cd:.1f} != {GEAR_CD}"

Z_FACE = -(SIDE_INNER_HALF + PLATE_THK)


def make() -> cq.Workplane:
    flange = (cq.Workplane("XY")
              .box(51.0, 25.0, 4.0, centered=(False, False, False))
              .translate((15.0, -19.0, Z_FACE - 4.0)))
    web = (cq.Workplane("YZ").workplane(offset=WEB_X0)
           .polyline([(-15.0, Z_FACE - 4.0), (95.0, Z_FACE - 4.0),
                      (95.0, FLY_Z - 14.0), (62.0, MOT_Z - 20.0),
                      (28.0, MOT_Z - 20.0), (-15.0, Z_FACE - 30.0)])
           .close().extrude(WEB_T))
    m = flange.union(web)

    pilot = (cq.Workplane("YZ").workplane(offset=WEB_X0 - 1)
             .center(MOT_Y, MOT_Z).circle(MOTOR_PILOT / 2).extrude(WEB_T + 2))
    m = m.cut(pilot)
    for (cy, cz, d) in [(FLY_Y, FLY_Z, VEX_SHAFT_CLEAR),
                        (FLY_Y - VEX_GRID, FLY_Z, VEX_HOLE),
                        (FLY_Y + VEX_GRID, FLY_Z, VEX_HOLE)]:
        m = m.cut(cq.Workplane("YZ").workplane(offset=WEB_X0 - 1)
                  .center(cy, cz).circle(d / 2).extrude(WEB_T + 2))
    for (px, py) in [(2 * VEX_GRID, 0.0), (3 * VEX_GRID, 0.0),
                     (2 * VEX_GRID, -VEX_GRID), (3 * VEX_GRID, -VEX_GRID)]:
        m = m.cut(cq.Workplane("XY").circle(VEX_HOLE / 2).extrude(6)
                  .translate((px, py, Z_FACE - 5)))
    return m


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "fly_drive_plate"))
