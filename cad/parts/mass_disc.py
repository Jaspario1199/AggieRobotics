"""
Flywheel mass disc -- FABRICATED STEEL (legal fabricated metal, not printed).

O72 x 16 mm, ~510 g: sized by gate-driven droop analysis (R3 F3, <6%/shot).
1/2" hex broach (or drill + file to hex) on the flywheel shaft beside the flex
wheel, inboard of the right fly_mount arm. Deburr; balance by spin test.
"""

from __future__ import annotations

import cadquery as cq

from ..params import VEX_HEX_AF, VEX_HEX_CLEAR
from ..vexlib import hex_across_corners

# Dia capped 2 mm BELOW the flex wheel's working surface so the ball always
# rides rubber, never steel; thickness recovers the inertia (droop ~15%/shot).
DIA, THK = 72.0, 16.0


def make() -> cq.Workplane:
    d = cq.Workplane("XY").circle(DIA / 2).extrude(THK)
    ac = hex_across_corners(VEX_HEX_AF + VEX_HEX_CLEAR)
    return d.cut(cq.Workplane("XY").polygon(6, ac).extrude(THK + 2)
                 .translate((0, 0, -1)))


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "mass_disc"))
