"""
Stow cradle -- the passive powered-off hold for inspection (R3 F6).

Two pillars on the chassis deck under the stowed head's flat bottom face
(z = 101 at STOW): the head simply RESTS in the V-tops (add adhesive foam
pads), gravity holds it in the 15" cube with everything off, and the pivot
motor lifts straight out to deploy -- no release actuator needed. Gate G15
verifies the seat height and that the fold sweep never clips the pillars.
Print two, base-down, no supports. PETG.
"""

from __future__ import annotations

import cadquery as cq

from ..params import VEX_GRID, VEX_HOLE

DECK_Z = 25.0          # chassis deck top (robot/arm.py)
SEAT_Z = 100.0         # head stow bottom 101 -> 1 mm for a foam pad
POS_Y = -120.0


def make() -> cq.Workplane:
    base = (cq.Workplane("XY").box(40.0, 40.0, 4.0)
            .translate((0, POS_Y, DECK_Z + 2.0)))
    post = (cq.Workplane("XY").box(24.0, 24.0, SEAT_Z - DECK_Z - 4.0)
            .translate((0, POS_Y, (DECK_Z + 4.0 + SEAT_Z) / 2)))
    vee = (cq.Workplane("YZ").workplane(offset=-13)
           .polyline([(POS_Y - 14, SEAT_Z + 8), (POS_Y, SEAT_Z - 2),
                      (POS_Y + 14, SEAT_Z + 8)])
           .close().extrude(26.0))
    m = base.union(post).union(vee.mirror("XZ", (0, POS_Y, 0)).union(vee)
                               .intersect(cq.Workplane("XY").box(26, 60, 30)
                                          .translate((0, POS_Y, SEAT_Z + 3))))
    for sy in (1, -1):
        m = m.cut(cq.Workplane("XY").circle(VEX_HOLE / 2).extrude(6)
                  .translate((0, POS_Y + sy * VEX_GRID, DECK_Z - 1)))
    return m


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "stow_cradle"))
