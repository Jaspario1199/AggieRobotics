"""
Belt pulley -- carries a traction belt at one end of the barrel.

Four of these run the two conveyor belts (a drive + an idler per belt). Flanges
keep the belt tracking; the drum is the pitch surface the belt rides on. Runs on
a 1/2" VEX hex shaft; the drive pulleys are turned by the motors, the idler
pulleys free-spin (mount them in the side-plate tension slots to tension the belt).

Print BORE-UP (axis vertical) -- drum, flanges, and lightening windows all print
without supports. Chamfered edges keep the belt from catching. PLA/PETG, >=4
walls. Print four.
"""

from __future__ import annotations

import math
import cadquery as cq

from ..params import (
    PULLEY_PITCH_DIA, PULLEY_FLANGE_DIA, BELT_WIDTH, PULLEY_SPOKES,
    VEX_HEX_AF, VEX_HEX_CLEAR, SCREW_M3_TAP,
)
from ..vexlib import hex_across_corners

FLANGE_THK = 4.0
DRUM_EXTRA = 5.0       # drum length beyond the belt (2.5 mm axial float per side)
WIN_R = 14.8           # lightening-window ring radius
WIN_D = 9.5            # lightening-window diameter
# Total pulley length (drum + both flanges). Placement code imports this so the
# flanges land OUTBOARD of the ball's reach (gate G5 checks the z-separation).
TOTAL_LEN = BELT_WIDTH + DRUM_EXTRA + 2 * FLANGE_THK


def make() -> cq.Workplane:
    rp = PULLEY_PITCH_DIA / 2.0
    rf = PULLEY_FLANGE_DIA / 2.0
    total = TOTAL_LEN

    # Drum is CROWNED (+0.5 mm at centre, two lofted cones): a flat drum on a
    # vertical axis lets gravity park the belt on the bottom flange forever
    # (red-team R3 D5); the crown self-centres it.
    drum = BELT_WIDTH + DRUM_EXTRA
    p = (
        cq.Workplane("XY")
        .circle(rf).extrude(FLANGE_THK)
        .faces(">Z").workplane().circle(rp)
        .workplane(offset=drum / 2).circle(rp + 0.5).loft(combine=True)
        .faces(">Z").workplane().circle(rp + 0.5)
        .workplane(offset=drum / 2).circle(rp).loft(combine=True)
        .faces(">Z").workplane().circle(rf).extrude(FLANGE_THK)
    )

    # Lightening windows through the whole pulley.
    pts = [(WIN_R * math.cos(math.radians(a)), WIN_R * math.sin(math.radians(a)))
           for a in [360.0 * k / PULLEY_SPOKES + 36.0 for k in range(PULLEY_SPOKES)]]
    p = p.faces(">Z").workplane().pushPoints(pts).hole(WIN_D)

    # 1/2" hex bore.
    ac = hex_across_corners(VEX_HEX_AF + VEX_HEX_CLEAR)
    p = p.faces(">Z").workplane(centerOption="CenterOfBoundBox").polygon(6, ac).cutThruAll()

    # Ease the belt-entry edges.
    try:
        p = p.faces(">Z").chamfer(1.0)
        p = p.faces("<Z").chamfer(1.0)
    except Exception:
        pass

    # NO set screw (R3 D1: M3 in PETG under the belt strips + is unreachable).
    # Torque: hex bore. Axial retention: VEX shaft collars (BOM).
    return p


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "belt_pulley"))
