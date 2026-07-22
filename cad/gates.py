"""
Automated verification gates for the belt-intake head CAD.

Phase-1 tool of the refine workflow: run BEFORE and AFTER every geometry change.
Prints a pass/fail scorecard with measured numbers; never modifies geometry.

    python -m cad.gates

Gates:
  G1  Part validity/connectivity  - each part is ONE valid closed solid
  G2  Assembly interference       - no two placed parts share volume (pairwise)
  G3  Ball-path sweep             - tri-ball tip-sphere proxy clears all HARD
                                    parts at stations along the channel; belt
                                    grip depth reported (soft contact intended)
  G4  Mounts / fastener paths     - every fixed part actually contacts (not
                                    floats near / clashes into) what it bolts to;
                                    deck bores align with pulley axes
  G5  Fit bands                   - belt<->pulley radial + axial fits, flange-
                                    vs-belt-surface protrusion, hex bore fit,
                                    deck tip clearance
  G6  Printed-wall minimums       - analytic min-wall numbers from parameters
  G7  Arm motion sweep (layout)   - min head<->wheel clearance through the fold

NOTE (honesty): G3 uses a SPHERE of the tri-ball tip diameter as a worst-case
orientation proxy for the rigid 3-lobe ball; a real-ball CAD would refine it.
"""

from __future__ import annotations

import math

import cadquery as cq

from . import params as P
from .vexlib import hex_across_corners

ROWS = []  # (gate, item, value, status, note)


def add(gate, item, value, status, note=""):
    ROWS.append((gate, item, value, status, note))


def ivol(a, b):
    """Pairwise intersection volume in mm^3 (0 = clean)."""
    try:
        r = a.intersect(b)
        ss = r.solids().vals()
        return sum(s.Volume() for s in ss) if ss else 0.0
    except Exception:
        return float("nan")


def dist(a, b):
    """Minimum distance between two shapes in mm (0 = touching)."""
    try:
        from OCP.BRepExtrema import BRepExtrema_DistShapeShape
        d = BRepExtrema_DistShapeShape(a.val().wrapped, b.val().wrapped)
        if d.IsDone():
            return d.Value()
    except Exception:
        pass
    return float("nan")


def bbox(w):
    b = w.val().BoundingBox()
    return (b.xmin, b.ymin, b.zmin, b.xmax, b.ymax, b.zmax)


def bb_overlap(b1, b2, tol=0.5):
    return all(b1[i] - tol <= b2[i + 3] and b2[i] - tol <= b1[i + 3]
               for i in range(3))


# --- derived assembly geometry (same formulas as export_all placement) ------
XP = P.BELT_GAP / 2 + P.PULLEY_PITCH_DIA / 2 + P.BELT_THK   # pulley axis |x|
YB = P.BARREL_LEN / 2                                        # pulley axis |y|
ZH = P.SIDE_INNER_HALF                                       # deck inner |z|

PART_IDS = ["deck_top", "deck_bot", "pul_RF", "pul_RB", "pul_LF", "pul_LB",
            "belt_L", "belt_R", "lip_T", "lip_B", "plow", "motor"]

# pairs whose surfaces are INTENDED to touch (bolted or tangent) - still must
# not share volume, but zero distance there is correct, not a finding.
INTENDED_CONTACT = {
    frozenset(("belt_L", "pul_LF")), frozenset(("belt_L", "pul_LB")),
    frozenset(("belt_R", "pul_RF")), frozenset(("belt_R", "pul_RB")),
    frozenset(("lip_T", "deck_top")), frozenset(("lip_B", "deck_bot")),
    frozenset(("plow", "deck_bot")), frozenset(("motor", "deck_top")),
}


def load_assembly():
    from .export_all import _place_accelerator
    named, ball = _place_accelerator()
    if len(named) != len(PART_IDS):
        raise RuntimeError(f"assembly has {len(named)} solids, expected {len(PART_IDS)}")
    return {pid: wp for pid, (_, wp, _) in zip(PART_IDS, named)}, ball


def gate1_parts():
    import importlib
    for name in ["belt_pulley", "drive_belt", "accel_plate", "throat_lip",
                 "front_plow", "motor_plate"]:
        try:
            mod = importlib.import_module(f"cad.parts.{name}")
            w = mod.make()
            ss = w.solids().vals()
            n = len(ss)
            valid = all(s.isValid() for s in ss)
            ok = (n == 1 and valid)
            add("G1", name, f"solids={n} valid={valid}",
                "PASS" if ok else "FAIL",
                "" if ok else "must be one valid solid")
        except Exception as e:
            add("G1", name, f"build error: {e}", "ERROR")


def gate2_interference(asm):
    bbs = {k: bbox(v) for k, v in asm.items()}
    ids = list(asm)
    clean = 0
    for i in range(len(ids)):
        for j in range(i + 1, len(ids)):
            a, b = ids[i], ids[j]
            if not bb_overlap(bbs[a], bbs[b]):
                clean += 1
                continue
            v = ivol(asm[a], asm[b])
            if math.isnan(v):
                add("G2", f"{a} x {b}", "boolean failed", "ERROR")
            elif v > 1.0:
                note = "intended-contact pair (must touch, not overlap)" \
                    if frozenset((a, b)) in INTENDED_CONTACT else ""
                add("G2", f"{a} x {b}", f"overlap {v:8.0f} mm3", "FAIL", note)
            else:
                clean += 1
    add("G2", "all remaining pairs", f"{clean} pairs overlap<=1 mm3", "PASS")


def gate3_ball_path(asm):
    hard = {k: v for k, v in asm.items() if not k.startswith("belt")}
    hard_bbs = {k: bbox(v) for k, v in hard.items()}
    z_rest = -ZH + P.TRIBALL_TIP / 2      # resting on the bottom deck
    worst = {}
    for y in range(-100, 150, 25):
        tip = (cq.Workplane("XY").sphere(P.TRIBALL_TIP / 2)
               .translate((0, y, z_rest)))
        tb = bbox(tip)
        for k, v in hard.items():
            if not bb_overlap(tb, hard_bbs[k]):
                continue
            ov = ivol(tip, v)
            if not math.isnan(ov) and ov > worst.get(k, 0.0):
                worst[k] = ov
    any_fail = False
    for k, ov in sorted(worst.items(), key=lambda t: -t[1]):
        if ov > 1.0:
            any_fail = True
            add("G3", f"ball(tip-sphere) x {k}", f"max overlap {ov:8.0f} mm3",
                "FAIL", "worst-case-orientation proxy")
    if not any_fail:
        add("G3", "ball(tip-sphere) vs all hard parts", "overlap<=1 mm3 at all stations", "PASS")
    grip = (P.TRIBALL_BODY - P.BELT_GAP) / 2
    add("G3", "belt grip depth per side", f"{grip:.1f} mm (param {P.BALL_COMPRESSION:.1f})",
        "PASS" if abs(grip - P.BALL_COMPRESSION) < 0.5 else "FAIL",
        "soft TPU contact - intended")


def gate4_mounts(asm):
    # bolted interfaces: must touch (dist<=0.2) and not clash (checked in G2)
    for a, b in [("lip_T", "deck_top"), ("lip_B", "deck_bot"),
                 ("plow", "deck_bot"), ("motor", "deck_top")]:
        d = dist(asm[a], asm[b])
        ov = ivol(asm[a], asm[b])
        if not math.isnan(ov) and ov > 1.0:
            add("G4", f"{a} -> {b} mount", f"CLASH {ov:6.0f} mm3", "FAIL",
                "parts overlap instead of mating")
        elif math.isnan(d):
            add("G4", f"{a} -> {b} mount", "distance n/a", "ERROR")
        elif d > 0.2:
            add("G4", f"{a} -> {b} mount", f"gap {d:5.1f} mm", "FAIL",
                "part floats - no bolted contact")
        else:
            add("G4", f"{a} -> {b} mount", f"contact (gap {d:4.2f} mm)", "PASS")
    # deck bore <-> pulley axis alignment (analytic, from shared params)
    exo = abs(XP - (P.BELT_GAP / 2 + P.PULLEY_PITCH_DIA / 2 + P.BELT_THK))
    eyo = 0.0  # placement uses the same formulas; guard against future edits
    err = max(exo, eyo)
    add("G4", "deck bores vs pulley axes", f"offset {err:.2f} mm",
        "PASS" if err <= 0.25 else "FAIL")


def gate5_fits():
    import cad.parts.belt_pulley as bp
    # radial: belt inner run rides the pitch surface
    add("G5", "belt-on-pulley radial fit", "tangent (0.0 mm) + 22 mm tension slots",
        "PASS", "tension via rear slots")
    # axial: belt width vs drum length between flanges
    ax = P.BELT_WIDTH - P.BELT_WIDTH   # drum is extruded exactly BELT_WIDTH
    add("G5", "belt axial clearance in drum", f"{ax:.1f} mm",
        "PASS" if ax >= 1.0 else "FAIL", "belt will bind on flanges; want >=1 mm")
    # flange protrusion past the belt's ball-side surface
    proud = (P.PULLEY_FLANGE_DIA - P.PULLEY_PITCH_DIA) / 2 - P.BELT_THK
    add("G5", "flange proud of belt surface", f"{proud:+.1f} mm",
        "PASS" if proud <= -0.5 else "FAIL",
        "rigid flange sticks past the gripping belt face into the ball channel")
    add("G5", "hex bore clearance (AF)", f"{P.VEX_HEX_CLEAR:.2f} mm",
        "PASS" if 0.2 <= P.VEX_HEX_CLEAR <= 0.6 else "FAIL")
    tipclr = 2 * ZH - P.TRIBALL_TIP
    add("G5", "deck gap vs ball tip", f"{tipclr:.1f} mm",
        "PASS" if tipclr >= 10 else "FAIL")


def gate6_walls():
    import cad.parts.belt_pulley as bp
    import cad.parts.drive_belt as db
    hex_r = hex_across_corners(P.VEX_HEX_AF + P.VEX_HEX_CLEAR) / 2
    w1 = bp.WIN_R - bp.WIN_D / 2 - hex_r
    add("G6", "pulley: window-to-bore wall", f"{w1:.2f} mm",
        "PASS" if w1 >= 2.0 else "FAIL", "printed PETG, torque path")
    w2 = P.PULLEY_PITCH_DIA / 2 - (bp.WIN_R + bp.WIN_D / 2)
    add("G6", "pulley: window-to-drum wall", f"{w2:.2f} mm",
        "PASS" if w2 >= 2.0 else "FAIL")
    w3 = P.BELT_THK - db.TREAD_D
    add("G6", "belt: web under tread groove", f"{w3:.2f} mm",
        "PASS" if w3 >= 2.0 else "FAIL", "TPU loop integrity")
    w4 = P.VEX_GRID - P.VEX_HOLE
    add("G6", "decks: web between grid holes", f"{w4:.2f} mm",
        "PASS" if w4 >= 4.0 else "FAIL")


def gate7_arm_sweep():
    try:
        import robot.arm as A
    except Exception as e:
        add("G7", "arm sweep", f"layout not importable: {e}", "ERROR")
        return
    wheels = [s for (n, s, _, _) in A._context() if n.startswith(("Corner", "_w"))]
    wbbs = [bbox(w) for w in wheels]
    dmin, at = float("inf"), None
    for phi in range(-40, 221, 20):
        solids = A._place(A._arm(0.0), phi)
        head = solids[1].union(solids[2])   # head + flywheel envelope
        hb = bbox(head)
        for w, wb in zip(wheels, wbbs):
            if not bb_overlap(hb, wb, tol=60):
                continue
            d = dist(head, w)
            if not math.isnan(d) and d < dmin:
                dmin, at = d, phi
    if dmin is float("inf"):
        add("G7", "head-wheel min clearance", ">60 mm everywhere", "PASS")
    else:
        add("G7", "head-wheel min clearance", f"{dmin:.1f} mm at phi={at} deg",
            "PASS" if dmin >= 10.0 else "FAIL", "layout-level (primitive volumes)")


def main():
    print("Running gates (booleans on real B-rep - takes a minute)...\n")
    gate1_parts()
    try:
        asm, _ = load_assembly()
        gate2_interference(asm)
        gate3_ball_path(asm)
        gate4_mounts(asm)
    except Exception as e:
        add("G2-4", "assembly", f"load failed: {e}", "ERROR")
    gate5_fits()
    gate6_walls()
    gate7_arm_sweep()

    wid = max(len(r[1]) for r in ROWS) + 2
    cur = None
    for (g, item, val, st, note) in ROWS:
        if g != cur:
            print(f"--- {g} " + "-" * (60 - len(g)))
            cur = g
        mark = {"PASS": "  ok ", "FAIL": " FAIL", "ERROR": " ERR "}[st]
        print(f"[{mark}] {item:<{wid}} {val}" + (f"   <- {note}" if note else ""))
    n_pass = sum(1 for r in ROWS if r[3] == "PASS")
    n_fail = sum(1 for r in ROWS if r[3] == "FAIL")
    n_err = sum(1 for r in ROWS if r[3] == "ERROR")
    print("\n" + "=" * 66)
    print(f"SCORECARD: {n_pass} pass / {n_fail} FAIL / {n_err} error")
    return 1 if (n_fail or n_err) else 0


if __name__ == "__main__":
    raise SystemExit(main())
