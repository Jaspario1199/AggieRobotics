"""
Throat lip -- the flared TOP guide at the mouth of the barrel.

One printed part, mounted on the TOP deck's forward edge (the bottom of the
mouth is handled by the front_plow ramp). Three fused features:

  * MOUNT PLATE  -- lies flat on the deck's top face; 4 vertical bolt holes that
    land exactly on the deck's 0.5" grid rows, so screws drop straight through
    plate + deck into nylocks.
  * EDGE WALL    -- drops over the deck's forward edge, flush against it. Its
    lowest face is EXACTLY at the deck's inner (channel) plane, so the lip adds
    ZERO intrusion into the mouth aperture (gate G5 checks this).
  * FLARE ARC    -- quarter-arc tangent to the channel plane at the wall, rising
    up-forward. Funnels a tumbling tri-ball down into the channel; its inner
    surface never dips below the channel plane, so nothing snags.

Print plate-down; the flare overhangs toward vertical at its tip -- use a few
supports under the last ~15 mm of arc, or split-print. PETG, 4 walls.
"""

from __future__ import annotations

import cadquery as cq

from ..params import (
    SIDE_INNER_HALF, PLATE_THK, BARREL_LEN, VEX_HOLE, VEX_GRID,
)

W = 2 * SIDE_INNER_HALF - 2.0   # width across the mouth (168)
RI = 38.0                       # flare inner radius
RO = 41.0                       # flare outer radius (3 mm wall)
PL_THK = 4.0                    # mount-plate thickness
PL_Y = 25.0                     # mount-plate reach back over the deck
WALL_Y = 3.0                    # edge-wall thickness (forward of the deck edge)
BOLT_X = 3 * VEX_GRID           # bolt columns at +/-38.1 (clear of the bores)

# Deck forward edge (world +Y); local y=0 is that edge, local z=0 the deck top.
EDGE = BARREL_LEN / 2 + 30.0
BOLT_ROWS = [5 * VEX_GRID - EDGE, 6 * VEX_GRID - EDGE]   # grid rows on the deck


def make() -> cq.Workplane:
    # Mount plate on the deck top: y -PL_Y..0, z 0..PL_THK.
    plate = (cq.Workplane("XY")
             .box(W, PL_Y, PL_THK, centered=(True, False, False))
             .translate((0, -PL_Y, 0)))

    # Edge wall over the deck edge: y 0..WALL_Y, z -PLATE_THK..PL_THK.
    # Its bottom (-PLATE_THK) sits exactly at the deck's inner/channel plane.
    wall = (cq.Workplane("XY")
            .box(W, WALL_Y, PLATE_THK + PL_THK, centered=(True, False, False))
            .translate((0, 0, -PLATE_THK)))

    # Flare arc: ring centered (y=WALL_Y-1, z=RO-PLATE_THK) so the OUTER surface
    # is tangent to the channel plane (z=-PLATE_THK) -- zero intrusion. Keep the
    # quadrant y>=center, z<=center; full-size cut boxes (no sliver rings).
    cy, cz = WALL_Y - 1.0, RO - PLATE_THK
    flare = (cq.Workplane("YZ").workplane(offset=-W / 2)
             .center(cy, cz).circle(RO).circle(RI).extrude(W))
    flare = flare.cut(cq.Workplane("XY").box(600, 600, 600)
                      .translate((0, cy - 300, 0)))
    flare = flare.cut(cq.Workplane("XY").box(600, 600, 600)
                      .translate((0, 0, cz + 300)))

    lip = plate.union(wall).union(flare)

    # Vertical bolt holes on the deck grid (columns +/-BOLT_X, two rows).
    for hx in (BOLT_X, -BOLT_X):
        for hy in BOLT_ROWS:
            hole = (cq.Workplane("XY").circle(VEX_HOLE / 2)
                    .extrude(PL_THK + 2).translate((hx, hy, -1)))
            lip = lip.cut(hole)

    return lip


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "throat_lip"))
