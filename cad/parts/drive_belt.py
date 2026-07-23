"""
Drive belt -- LACED FLAT STRIP (printable), plus a loop proxy for the assembly.

Red-team R3-2 killed the closed-loop print: lying flat it needs a 110 mm TPU
bridge over a closed hole; standing up it stacks a 120 mm Z-seam weak line
through the ball nip. The manufacturable form is the standard tank-tread
answer: a FLAT TPU strip with finger-joint hinge ends, laced shut around the
pulleys IN PLACE with a 1.75 mm filament/music-wire pin. This also makes a
belt swap a 5-minute pit fix instead of a top-deck teardown (R3 C1), and belt
tension is set by lace-length choice (the deck bores are now all FIXED).

make()      -> ONE strip (print 2 per belt, 4 per robot, flat, TPU 95A).
make_loop() -> the assembled-loop proxy used by the assembly render and gates
               (inner radius rides the pulley CROWN, +0.5).

Print flat (tread up). No supports. TPU 95A, 0.3 mm layers, 4 walls.
"""

from __future__ import annotations

import math
import cadquery as cq

from ..params import PULLEY_PITCH_DIA, BELT_THK, BELT_WIDTH, BARREL_LEN

TREAD_N = 11          # transverse tread ribs per straight run (loop proxy)
TREAD_W = 1.4
TREAD_D = 1.1

# Strip: half the loop's neutral perimeter, laced at two joints.
LOOP_NEUTRAL = 2 * BARREL_LEN + math.pi * (PULLEY_PITCH_DIA / 2 + BELT_THK / 2) * 2
STRIP_LEN = LOOP_NEUTRAL / 2          # ~190 mm -- fits a 250 mm bed
N_FINGERS = 5                          # alternating finger joints
PIN_DIA = 2.0                          # 1.75 mm filament / music-wire lace pin


def make() -> cq.Workplane:
    W, L, T = BELT_WIDTH, STRIP_LEN, BELT_THK
    strip = cq.Workplane("XY").box(W, L, T, centered=(True, False, False))

    # Finger joints: cut alternating slots at each end (ends mate half-turned).
    fw = W / (2 * N_FINGERS)
    for end, phase in ((0.0, 0), (L, 1)):
        y0 = end - 8.0 if end else 0.0
        for k in range(2 * N_FINGERS):
            if k % 2 == phase:
                x0 = -W / 2 + k * fw
                strip = strip.cut(cq.Workplane("XY")
                                  .box(fw, 8.0, T + 2, centered=(False, False, False))
                                  .translate((x0, y0, -1)))

    # Lace-pin holes along each end, through the fingers.
    for end in (4.0, L - 4.0):
        pin = (cq.Workplane("YZ").workplane(offset=-(W / 2 + 1))
               .center(end, T / 2).circle(PIN_DIA / 2).extrude(W + 2))
        strip = strip.cut(pin)

    # Transverse tread grooves on the traction face.
    n = int(L // 15)
    for i in range(n):
        yy = 10.0 + i * 15.0
        strip = strip.cut(cq.Workplane("XY")
                          .box(W + 2, TREAD_W, TREAD_D * 2)
                          .translate((0, yy, T)))
    return strip


def make_loop() -> cq.Workplane:
    """Assembled-loop proxy: rides the crowned drum (pitch + 0.5)."""
    r_in = PULLEY_PITCH_DIA / 2.0 + 0.5
    r_out = r_in + BELT_THK
    L = BARREL_LEN
    outer = cq.Workplane("YZ").slot2D(L + 2 * r_out, 2 * r_out).extrude(BELT_WIDTH)
    inner = cq.Workplane("YZ").slot2D(L + 2 * r_in, 2 * r_in).extrude(BELT_WIDTH)
    belt = outer.cut(inner).translate((-BELT_WIDTH / 2.0, 0, 0))
    for run in (+1, -1):
        for i in range(TREAD_N):
            yy = -L / 2.0 + L * (i + 0.5) / TREAD_N
            belt = belt.cut(cq.Workplane("XY")
                            .box(BELT_WIDTH + 2, TREAD_W, TREAD_D * 2)
                            .translate((0, yy, run * r_out)))
    return belt


if __name__ == "__main__":
    from ..lib import export
    print(export(make(), "drive_belt"))
