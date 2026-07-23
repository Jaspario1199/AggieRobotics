"""
Arm hub -- the real arm link: keys the belt head to the fold-pivot shaft.

One per side (print TWO, the left one MIRRORED). A vertical web plate rides just
outside the deck edge; a thick boss at its rear carries a keyed 1/2" HEX bore on
the pivot axis (the hub rotates WITH the shaft -- the shaft is driven by the
pivot motor and supported outboard in the pivot_block bearings, dual shear).
Two horizontal tabs bolt the hub to BOTH decks on real 0.5" grid holes, so the
hub + decks form a rigid triangle; the pair of hubs carries the whole head's
moment into the shaft.

Modelled in HEAD-ASSEMBLY coordinates (the +X / right side): pivot axis at
(y = PIV_Y, z = 0), deck half-width 113.5. Gates check: tab bolts land on real
deck grid holes (G9a), web clears the deck (G9b), tabs clear the motor plate
(G9c), boss clears the chassis-side pivot block (G9d).

Print WEB-FLAT (web on the bed): the boss stands as a cylinder and the tabs
rise as vertical walls -- NO supports needed (the earlier note claiming
otherwise was wrong; the tab bolt holes print sideways, fine at 4.9 mm).
PETG, 4+ walls, 50% infill: primary load path. If bench tests show tab flex,
the identical profile can be cut from VEX plate (legal fabricated metal).
"""

from __future__ import annotations

import cadquery as cq

from ..params import (
    VEX_GRID, VEX_HOLE, VEX_HEX_AF, VEX_HEX_CLEAR, SCREW_M3_TAP,
    SIDE_INNER_HALF, PLATE_THK,
)
from ..vexlib import hex_across_corners

PIV_Y = -133.0        # pivot axis y (the head's rear extreme plane)
DECK_HALF_W = 121.0   # deck half-width (accel_plate HY = YO + 41.5)
X0 = DECK_HALF_W + 1.0   # web inner face: 1 mm outboard of the deck edge
WEB_T = 6.0
BOSS_R = 16.0
BOSS_LEN = 14.0       # boss length along the shaft (from the web's inner face)
TAB_X0 = 108.0        # tabs reach inboard over the decks to here
TAB_T = 4.0
TAB_Y0, TAB_Y1 = -82.0, -54.0
BOLT_COL = 9 * VEX_GRID            # 114.3 -- REAL grid column (integer pitch)
BOLT_ROWS = [-6 * VEX_GRID, -5 * VEX_GRID]   # -76.2, -63.5

ZT = SIDE_INNER_HALF + PLATE_THK   # deck outer faces at +/-91


def make() -> cq.Workplane:
    # Web: trapezoid from the tab region back to the boss, in the YZ plane.
    web = (cq.Workplane("YZ").workplane(offset=X0)
           .polyline([(TAB_Y1, ZT + TAB_T + 2), (TAB_Y1, -(ZT + TAB_T + 2)),
                      (PIV_Y + 12, -14), (PIV_Y + 12, 14)])
           .close().extrude(WEB_T))

    # Boss on the pivot axis.
    boss = (cq.Workplane("YZ").workplane(offset=X0)
            .center(PIV_Y, 0).circle(BOSS_R).extrude(BOSS_LEN))
    hub = web.union(boss)

    # Tabs onto both decks' outer faces.
    for sz in (+1, -1):
        z0 = sz * ZT if sz > 0 else -(ZT + TAB_T)
        tab = (cq.Workplane("XY")
               .box(X0 + WEB_T - TAB_X0, TAB_Y1 - TAB_Y0, TAB_T,
                    centered=(False, False, False))
               .translate((TAB_X0, TAB_Y0, z0)))
        hub = hub.union(tab)

    # Keyed hex bore through boss + web.
    ac = hex_across_corners(VEX_HEX_AF + VEX_HEX_CLEAR)
    bore = (cq.Workplane("YZ").workplane(offset=X0 - 1)
            .center(PIV_Y, 0).polygon(6, ac).extrude(BOSS_LEN + 2))
    hub = hub.cut(bore)

    # Radial set-screw into the boss (locks hub-to-shaft phase).
    grub = (cq.Workplane("XY")
            .circle(SCREW_M3_TAP / 2).extrude(BOSS_R + 4)
            .translate((X0 + BOSS_LEN - 5, PIV_Y, 0)))
    hub = hub.cut(grub)

    # Tab bolt holes: one column, two rows, straight through BOTH tabs.
    for hy in BOLT_ROWS:
        hole = (cq.Workplane("XY").circle(VEX_HOLE / 2)
                .extrude(2 * (ZT + TAB_T + 2))
                .translate((BOLT_COL, hy, -(ZT + TAB_T + 2))))
        hub = hub.cut(hole)

    return hub


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "arm_hub"))
