"""
Platen rail -- rigid backing behind each belt's inner run (red-team R3 D6).

Without it, grip normal force was set only by belt tension: the free 110 mm
TPU span just deflected away from the ball. This rail sits 0 mm behind the
inner run over the ball-contact band, so the squeeze reacts into structure.

H-shape: a low-friction contact BAR (y +/-40, z +/-25 -- the ball contact
patch) behind the belt, a central RISER (y +/-10, threading between the pulley
flanges which live at z 62..66.5), and FEET bolting to both decks' inner faces
on real grid holes. Print lying on the bar face; PETG; wax/PTFE-tape the bar.
Print TWO (second MIRRORED).
"""

from __future__ import annotations

import cadquery as cq

from ..params import (
    BELT_GAP, BELT_THK, VEX_GRID, VEX_HOLE, SIDE_INNER_HALF,
)

X0 = BELT_GAP / 2 + BELT_THK    # contact face: exactly the belt's outer side (57)
RAIL_T = 12.0
ZH = SIDE_INNER_HALF


def make() -> cq.Workplane:
    bar = (cq.Workplane("XY").box(RAIL_T, 60.0, 50.0)
           .translate((X0 + RAIL_T / 2, 0, 0)))
    riser = (cq.Workplane("XY").box(RAIL_T, 20.0, 2 * ZH)
             .translate((X0 + RAIL_T / 2, 0, 0)))
    rail = bar.union(riser)
    for sz in (1, -1):
        foot = (cq.Workplane("XY").box(25.0, 24.0, 4.0,
                                       centered=(False, True, False))
                .translate((55.0, 0, sz * ZH - (4.0 if sz > 0 else 0.0))))
        rail = rail.union(foot)
        hole = (cq.Workplane("XY").circle(VEX_HOLE / 2).extrude(6)
                .translate((5 * VEX_GRID, 0, sz * ZH - 5)))
        rail = rail.cut(hole)
    return rail


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "platen_rail"))
