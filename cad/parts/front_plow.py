"""
Front plow / ramp -- push blade, intake ramp, and bottom mouth guide in one part.

Mounted on the BOTTOM deck's forward edge, plate on the deck's INNER (channel)
face so the ramp surface flows smoothly onto the channel floor -- a ball rolling
up the blade crosses onto the plate top and into the channel with no step UP
(only a 4 mm drop off the plate's back edge, harmless). It is also the PUSH
tool: belts off, drive forward, the raked blade gets under the tri-ball.

  * MOUNT PLATE -- flat on the deck inner face; 4 vertical bolt holes on the
    deck's 0.5" grid (use low-profile screws inside the channel).
  * BLADE       -- raked down-forward at PLOW_DEG from the plate's forward-top
    edge; its top surface is continuous with the plate top.

The plate intrudes 4 mm into the mouth aperture at the bottom -- accounted for
in gate G5's aperture check (still >=6 mm clearance over the ball tip).

Print plate-down, blade up (the shallow rake needs no supports). PETG, 4 walls,
40%+ infill -- this part takes impact.
"""

from __future__ import annotations

import cadquery as cq

from ..params import (
    SIDE_INNER_HALF, PLOW_DEG, VEX_HOLE, VEX_GRID, BARREL_LEN,
)

SPAN = 2 * SIDE_INNER_HALF      # blade width (170)
PL_THK = 4.0                    # mount-plate thickness (= aperture intrusion)
PL_Y = 25.0                     # mount-plate reach back over the deck
BLADE_LEN = 72.0                # blade length along its face
BOLT_X = 3 * VEX_GRID           # bolt columns at +/-38.1

EDGE = BARREL_LEN / 2 + 30.0
BOLT_ROWS = [5 * VEX_GRID - EDGE, 6 * VEX_GRID - EDGE]


def make() -> cq.Workplane:
    # Mount plate on the deck inner face: y -PL_Y..0, z 0..PL_THK.
    plate = (cq.Workplane("XY")
             .box(SPAN, PL_Y, PL_THK, centered=(True, False, False))
             .translate((0, -PL_Y, 0)))

    # Blade: same thickness slab, hinged about the plate's forward-TOP edge
    # (y=0, z=PL_THK) and raked down-forward so its top surface continues from
    # the plate top. The rotation swings its rear-bottom corner back into the
    # plate, fusing the two solids.
    blade = (cq.Workplane("XY")
             .box(SPAN, BLADE_LEN, PL_THK, centered=(True, False, False)))
    blade = blade.rotate((-1, 0, PL_THK), (1, 0, PL_THK), -PLOW_DEG)
    plow = plate.union(blade)

    # Trim the sliver the top-edge hinge swings below the plate plane (it would
    # clash 15 mm^3 into the deck: gate G2 baseline finding).
    plow = plow.cut(cq.Workplane("XY").box(600, 600, 600)
                    .translate((0, -300, -300)))

    # 45-deg chamfer on the plate's channel-side edge so the ball rolls over a
    # ramp instead of a 4 mm step (gate G3 baseline finding).
    ch = (cq.Workplane("YZ").workplane(offset=-(SPAN / 2 + 1))
          .polyline([(-PL_Y, PL_THK), (-PL_Y + PL_THK, PL_THK), (-PL_Y, 0)])
          .close().extrude(SPAN + 2))
    plow = plow.cut(ch)

    # Vertical bolt holes on the deck grid.
    for hx in (BOLT_X, -BOLT_X):
        for hy in BOLT_ROWS:
            hole = (cq.Workplane("XY").circle(VEX_HOLE / 2)
                    .extrude(PL_THK + 2).translate((hx, hy, -1)))
            plow = plow.cut(hole)

    return plow


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "front_plow"))
