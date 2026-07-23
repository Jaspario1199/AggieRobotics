"""
Accelerator plate -- the structural deck of the belt barrel (top and bottom).

With the belts on the LEFT and RIGHT, these two plates are the top and bottom
DECKS: each carries two vertical pulley shafts per side, contains the tri-ball
top/bottom, and the bottom deck is the floor the ball rides on down the barrel.
The front (muzzle) pulley bores are fixed; the rear (breach) pair are TENSION
SLOTS so you slide the idlers back to tension each belt. Everything else -- the
motors, throat lips, plow, drivetrain -- bolts to the 0.5" grid that fills it.

Solid grid floor (no big cut-outs) so the ball can't drop through; big corner
radii keep it clean. FLIP-SYMMETRIC: all features go straight through, so ONE
printed part is both the top and the bottom deck. Print FLAT, no supports.
Structural PLA/PETG, >=4 walls, 40%+ infill. Print two.
"""

from __future__ import annotations

import cadquery as cq

from ..params import (
    BELT_GAP, PULLEY_PITCH_DIA, BELT_THK, BARREL_LEN, PLATE_THK,
    VEX_GRID, VEX_HOLE, VEX_SHAFT_CLEAR,
)
from ..vexlib import grid_points

XO = BARREL_LEN / 2.0                                   # pulley offset along barrel
YO = BELT_GAP / 2.0 + PULLEY_PITCH_DIA / 2.0 + BELT_THK  # pulley offset across barrel
HX = XO + 30.0
# Cross-width margin is sized so a REAL grid column exists at 9 x 12.7 = 114.3
# outboard of the motor plates -- the arm-hub tabs bolt there (gate G9a).
HY = YO + 41.5
OUTLINE = [(-HX, -HY), (HX, -HY), (HX, HY), (-HX, HY)]


def make() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .polyline(OUTLINE).close().extrude(PLATE_THK)
        .edges("|Z").fillet(34)
    )

    # Front (muzzle) pulley bores -- fixed.
    plate = plate.faces(">Z").workplane().pushPoints([(XO, YO), (XO, -YO)]).hole(VEX_SHAFT_CLEAR)

    # Rear (breach) pulley bores -- FIXED (R3 A1: sliding tension slots broke
    # the motor gear mesh and floated the drive-shaft bearings; belt tension is
    # now set by the laced-strip belt's lace length).
    plate = (plate.faces(">Z").workplane()
             .pushPoints([(-XO, YO), (-XO, -YO)]).hole(VEX_SHAFT_CLEAR))

    # Bearing-flat bolt pairs at +/-12.7 mm along the barrel from EVERY bore
    # (R3 C4: VEX bearing flats need these two holes; the grid keepouts had
    # deleted every candidate).
    bpts = [(sx * XO + d, sy * YO)
            for sx in (1, -1) for sy in (1, -1) for d in (VEX_GRID, -VEX_GRID)]
    plate = plate.faces(">Z").workplane().pushPoints(bpts).hole(VEX_HOLE)

    # Flywheel notch: the under-mouth launcher wheel passes through the deck at
    # the muzzle centre (open to the forward edge). The part stays flip-
    # symmetric; on the top deck the notch sits under the throat-lip plate.
    plate = plate.cut(cq.Workplane("XY").box(55, 40, 40)
                      .translate((HX - 15, 0, 0)))

    # 0.5" VEX grid across the whole deck, clear of the bores.
    holes = grid_points(
        OUTLINE, VEX_GRID, margin=VEX_HOLE / 2 + 3.0,
        keepouts=[(XO, YO, 17.5), (XO, -YO, 17.5), (-XO, YO, 17.5), (-XO, -YO, 17.5)],
    )
    plate = plate.faces(">Z").workplane().pushPoints(holes).hole(VEX_HOLE)

    return plate


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "accel_plate"))
