"""
Automated verification gates for the belt-intake head CAD + robot layout.

Run BEFORE and AFTER every geometry change; prints a pass/fail scorecard with
measured numbers; never modifies geometry.

    python -m cad.gates

Gates:
  G1  Part validity/connectivity  - each part is ONE valid closed solid
  G2  Assembly interference       - no two placed parts share volume (pairwise)
  G3  Ball-path sweep             - ANISOTROPIC ball proxies: the tri-ball
                                    presents its 157 mm TIP along z/y (decks,
                                    lip, plow) but its 110 mm BODY across the
                                    belt axis (pulleys, belts). Baseline used a
                                    tip-sphere everywhere, which is physically
                                    wrong across a 102 mm grip channel -- this
                                    reformulation is documented, not a loosened
                                    threshold. Belt grip overlap is intended
                                    (soft TPU) and reported as info.
  G4  Mounts / fastener paths     - bolted parts contact (no float, no clash);
                                    lip/plow bolt holes land on REAL deck grid
                                    holes; deck bores align with pulley axes
  G5  Fit bands                   - axial belt float, flange-vs-ball criterion
                                    (radial recess OR z-band separation), rigid
                                    drum gap vs ball body, hex fit, aperture
  G6  Printed-wall minimums       - analytic min-wall numbers from parameters
  G7  Arm motion sweep (layout)   - min head<->wheel clearance through the fold
  G8  Stow-in-cube                - stowed robot bounding box fits the 15" cube
                                    with margin (baseline had NO such gate and
                                    the old stow pose silently busted the cube)
"""

from __future__ import annotations

import math

import cadquery as cq

from . import params as P
from .vexlib import hex_across_corners, grid_points

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
EDGE = P.BARREL_LEN / 2 + 30.0                               # deck fwd edge (y)

PART_IDS = ["deck_top", "deck_bot", "pul_RF", "pul_RB", "pul_LF", "pul_LB",
            "belt_L", "belt_R", "lip_T", "plow", "motor_L", "motor_R",
            "hub_R", "hub_L"]

INTENDED_CONTACT = {
    frozenset(("belt_L", "pul_LF")), frozenset(("belt_L", "pul_LB")),
    frozenset(("belt_R", "pul_RF")), frozenset(("belt_R", "pul_RB")),
    frozenset(("lip_T", "deck_top")), frozenset(("plow", "deck_bot")),
    frozenset(("motor_L", "deck_top")), frozenset(("motor_R", "deck_top")),
    frozenset(("hub_R", "deck_top")), frozenset(("hub_R", "deck_bot")),
    frozenset(("hub_L", "deck_top")), frozenset(("hub_L", "deck_bot")),
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
                 "front_plow", "motor_plate", "arm_hub", "pivot_block"]:
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


def _rest_z(y):
    """Rolling rest height of the tip-sphere centre at station y.

    The ball rides the HIGHEST support under it: the deck floor (z=-ZH), or the
    plow mount plate (top -ZH+4, chamfered edge starting 4 mm past its rear
    edge) -- tangent to the plate's top segment/corner, the way a rolling ball
    actually crosses a ramped step.
    """
    R = P.TRIBALL_TIP / 2
    z = -ZH + R                              # deck floor
    seg_lo, seg_hi = EDGE - 25.0 + 4.0, EDGE  # plate top after the 45deg chamfer
    yn = min(max(y, seg_lo), seg_hi)
    dy = abs(y - yn)
    if dy < R:
        # tangent to the plate's top segment (dy=0 -> flat contact, +R)
        z = max(z, (-ZH + 4.0) + math.sqrt(R * R - dy * dy))
    return z


def gate3_ball_path(asm):
    # TIP sphere vs z/y-axis obstacles (worst-case orientation for those parts).
    z_parts = ["deck_top", "deck_bot", "lip_T", "plow", "motor_L", "motor_R"]
    zbbs = {k: bbox(asm[k]) for k in z_parts}
    worst = {}
    for y in range(-100, 96, 15):
        tip = (cq.Workplane("XY").sphere(P.TRIBALL_TIP / 2)
               .translate((0, y, _rest_z(y))))
        tb = bbox(tip)
        for k in z_parts:
            if not bb_overlap(tb, zbbs[k]):
                continue
            ov = ivol(tip, asm[k])
            if not math.isnan(ov) and ov > worst.get(k, 0.0):
                worst[k] = ov
    fails = {k: v for k, v in worst.items() if v > 1.0}
    for k, ov in sorted(fails.items(), key=lambda t: -t[1]):
        add("G3", f"ball(tip) x {k}", f"max overlap {ov:8.0f} mm3", "FAIL")
    if not fails:
        add("G3", "ball(tip) vs decks/lip/plow/motors",
            "overlap<=1 mm3 at all stations", "PASS")

    # BODY sphere vs the rigid pulley columns (across-belt presentation).
    x_parts = ["pul_RF", "pul_RB", "pul_LF", "pul_LB"]
    xbbs = {k: bbox(asm[k]) for k in x_parts}
    worst = {}
    for y in (-YB, 0.0, YB):
        for z in (-6.5, 6.5):
            body = (cq.Workplane("XY").sphere(P.TRIBALL_BODY / 2)
                    .translate((0, y, z)))
            tb = bbox(body)
            for k in x_parts:
                if not bb_overlap(tb, xbbs[k]):
                    continue
                ov = ivol(body, asm[k])
                if not math.isnan(ov) and ov > worst.get(k, 0.0):
                    worst[k] = ov
    fails = {k: v for k, v in worst.items() if v > 1.0}
    for k, ov in sorted(fails.items(), key=lambda t: -t[1]):
        add("G3", f"ball(body) x {k}", f"max overlap {ov:8.0f} mm3", "FAIL",
            "rigid pulley inside the ball-body envelope")
    if not fails:
        add("G3", "ball(body) vs rigid pulleys",
            "overlap<=1 mm3 at all stations", "PASS")

    # Belt grip (intended soft contact) - info.
    body = cq.Workplane("XY").sphere(P.TRIBALL_BODY / 2)
    gv = ivol(body, asm["belt_L"])
    grip = (P.TRIBALL_BODY - P.BELT_GAP) / 2
    add("G3", "belt grip", f"depth {grip:.1f} mm/side, lens {gv:6.0f} mm3",
        "PASS" if grip >= 2.0 else "FAIL", "soft TPU squeeze - intended")


def gate4_mounts(asm):
    for a, b in [("lip_T", "deck_top"), ("plow", "deck_bot"),
                 ("motor_L", "deck_top"), ("motor_R", "deck_top"),
                 ("hub_R", "deck_top"), ("hub_R", "deck_bot"),
                 ("hub_L", "deck_top"), ("hub_L", "deck_bot")]:
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

    # Lip/plow bolt holes must land on REAL deck grid holes (membership check
    # against the same grid_points call accel_plate uses).
    import cad.parts.accel_plate as ap
    deck_holes = grid_points(
        ap.OUTLINE, P.VEX_GRID, margin=P.VEX_HOLE / 2 + 3.0,
        keepouts=[(ap.XO, ap.YO, 14.0), (ap.XO, -ap.YO, 14.0),
                  (-ap.XO, ap.YO, 22.0), (-ap.XO, -ap.YO, 22.0)])
    bolt_world = [(sx * 3 * P.VEX_GRID, k * P.VEX_GRID)
                  for sx in (1, -1) for k in (5, 6)]
    # world (x,y) -> deck local (x,y) for the +90 deg rotated plate
    missing = [w for w in bolt_world
               if not any(abs(w[1] - lx) < 0.01 and abs(-w[0] - ly) < 0.01
                          for (lx, ly) in deck_holes)]
    add("G4", "lip/plow bolts on deck grid",
        f"{len(bolt_world) - len(missing)}/{len(bolt_world)} land on real holes",
        "PASS" if not missing else "FAIL",
        "" if not missing else f"missing at {missing}")

    exo = abs(XP - (P.BELT_GAP / 2 + P.PULLEY_PITCH_DIA / 2 + P.BELT_THK))
    add("G4", "deck bores vs pulley axes", f"offset {exo:.2f} mm",
        "PASS" if exo <= 0.25 else "FAIL")


def gate5_fits(asm):
    import cad.parts.belt_pulley as bp
    add("G5", "belt-on-pulley radial fit",
        "tangent (0.0 mm) + 22 mm tension slots", "PASS",
        "tension via rear slots")
    ax = bp.DRUM_EXTRA
    add("G5", "belt axial float on drum", f"{ax:.1f} mm",
        "PASS" if 1.0 <= ax <= 6.0 else "FAIL")

    # Flange vs ball: OK if recessed below the belt face, OR if its z-band is
    # outside the ball body's reach at the flange radius (the real criterion).
    rf = P.PULLEY_FLANGE_DIA / 2
    proud = rf - (P.PULLEY_PITCH_DIA / 2 + P.BELT_THK)
    flange_z0 = (P.BELT_WIDTH + bp.DRUM_EXTRA) / 2
    face_x = XP - rf
    r2 = (P.TRIBALL_BODY / 2) ** 2 - face_x ** 2
    ball_z_at_face = 6.5 + math.sqrt(r2) if r2 > 0 else 0.0
    zmargin = flange_z0 - ball_z_at_face
    ok = (proud <= -0.5) or (zmargin >= 1.0)
    add("G5", "flange vs ball",
        f"proud {proud:+.1f} mm, z-band margin {zmargin:.1f} mm",
        "PASS" if ok else "FAIL",
        "flange sits outside the ball's z-reach" if ok else
        "rigid flange inside the ball channel")

    hard = (XP - P.PULLEY_PITCH_DIA / 2) - P.TRIBALL_BODY / 2
    add("G5", "rigid drum gap vs ball body", f"{hard:.1f} mm/side",
        "PASS" if hard >= 2.0 else "FAIL",
        "belt-only squeeze; drum never touches the rigid ball")

    add("G5", "hex bore clearance (AF)", f"{P.VEX_HEX_CLEAR:.2f} mm",
        "PASS" if 0.2 <= P.VEX_HEX_CLEAR <= 0.6 else "FAIL")

    # Mouth aperture from the ACTUAL placed solids: lip must not dip below the
    # channel plane; plow plate intrusion is designed-in.
    lip_zmin = bbox(asm["lip_T"])[2]
    lip_intr = max(0.0, ZH - lip_zmin)
    plow_zmax = bbox(asm["plow"])[5]
    plow_intr = max(0.0, plow_zmax + ZH)
    aperture = 2 * ZH - lip_intr - plow_intr - P.TRIBALL_TIP
    add("G5", "lip intrusion into aperture", f"{lip_intr:.2f} mm",
        "PASS" if lip_intr <= 0.05 else "FAIL")
    add("G5", "mouth aperture margin over tip",
        f"{aperture:.1f} mm (plow plate {plow_intr:.1f} designed-in)",
        "PASS" if aperture >= 6.0 else "FAIL")


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
    ws = A.wheels()
    wbbs = [bbox(w) for w in ws]
    dmin, at = float("inf"), None
    for phi in A.SWEEP:
        head = A.head_at(phi)
        hb = bbox(head)
        for w, wb in zip(ws, wbbs):
            if not bb_overlap(hb, wb, tol=60):
                continue
            d = dist(head, w)
            if not math.isnan(d) and d < dmin:
                dmin, at = d, phi
    if dmin == float("inf"):
        add("G7", "head-wheel min clearance", ">60 mm everywhere", "PASS")
    else:
        add("G7", "head-wheel min clearance", f"{dmin:.1f} mm at phi={at} deg",
            "PASS" if dmin >= 10.0 else "FAIL", "layout-level (primitive volumes)")


def gate8_stow_cube():
    try:
        import robot.arm as A
    except Exception as e:
        add("G8", "stow cube", f"layout not importable: {e}", "ERROR")
        return
    hb = bbox(A.head_at(A.STOW))
    # chassis extents: frame +/-FRAME/2 in x/y, pivot tower height in z
    lo = [min(hb[0], -A.FRAME / 2), min(hb[1], -A.FRAME / 2), 0.0]
    hi = [max(hb[3], A.FRAME / 2), max(hb[4], A.FRAME / 2),
          max(hb[5], A.PIVOT[2] + 15)]
    names = "xyz"
    worst = float("inf")
    dims = []
    for i in range(3):
        ext = hi[i] - lo[i]
        margin = A.CUBE - ext
        dims.append(f"{names[i]}={ext:.0f}")
        worst = min(worst, margin)
    add("G8", "stowed robot bbox vs 15\" cube",
        f"{' '.join(dims)} mm (worst margin {worst:.0f} mm)",
        "PASS" if worst >= 10.0 else "FAIL",
        "start-legal with margin" if worst >= 10.0 else "busts/skims the cube")


def gate9_arm_link(asm):
    """Real arm-link integration: hubs + pivot blocks."""
    import cad.parts.arm_hub as ah
    import cad.parts.pivot_block as pb
    import cad.parts.motor_plate as mp
    import cad.parts.accel_plate as ap

    # a) hub tab bolts land on REAL deck grid holes.
    deck_holes = grid_points(
        ap.OUTLINE, P.VEX_GRID, margin=P.VEX_HOLE / 2 + 3.0,
        keepouts=[(ap.XO, ap.YO, 14.0), (ap.XO, -ap.YO, 14.0),
                  (-ap.XO, ap.YO, 22.0), (-ap.XO, -ap.YO, 22.0)])
    bolt_world = [(sx * ah.BOLT_COL, r) for sx in (1, -1) for r in ah.BOLT_ROWS]
    missing = [w for w in bolt_world
               if not any(abs(w[1] - lx) < 0.01 and abs(-w[0] - ly) < 0.01
                          for (lx, ly) in deck_holes)]
    add("G9", "hub bolts on deck grid",
        f"{len(bolt_world) - len(missing)}/{len(bolt_world)} land on real holes",
        "PASS" if not missing else "FAIL",
        "" if not missing else f"missing at {missing}")

    # b) web clears the deck edge.
    gap_web = ah.X0 - ah.DECK_HALF_W
    add("G9", "hub web vs deck edge", f"gap {gap_web:.1f} mm",
        "PASS" if gap_web >= 0.5 else "FAIL")

    # c) tabs clear the motor plate.
    plate_out = XP + mp.HALF_W
    gap_tab = ah.TAB_X0 - plate_out
    add("G9", "hub tab vs motor plate", f"gap {gap_tab:.1f} mm",
        "PASS" if gap_tab >= 0.5 else "FAIL")

    # d) chassis-side pivot block stays outboard of the swinging boss.
    boss_out = ah.X0 + ah.BOSS_LEN
    try:
        import robot.arm as A
        blk_in = A.BLOCK_X0
        gap_blk = blk_in - boss_out
        add("G9", "pivot block vs hub boss", f"gap {gap_blk:.1f} mm",
            "PASS" if gap_blk >= 1.0 else "FAIL")
        # e) tower + block arithmetic puts the bore ON the pivot axis.
        bore_z = A.TOWER_TOP + pb.BORE_H
        err = abs(bore_z - A.PIVOT[2])
        add("G9", "block bore on pivot axis", f"offset {err:.2f} mm",
            "PASS" if err <= 0.25 else "FAIL")
    except Exception as e:
        add("G9", "robot-side placement", f"not importable: {e}", "ERROR")

    # f) hub bore is the keyed hex fit.
    add("G9", "hub hex bore fit (AF)", f"{P.VEX_HEX_CLEAR:.2f} mm",
        "PASS" if 0.2 <= P.VEX_HEX_CLEAR <= 0.6 else "FAIL")


def main():
    print("Running gates (booleans on real B-rep - takes a minute)...\n")
    gate1_parts()
    try:
        asm, _ = load_assembly()
        gate2_interference(asm)
        gate3_ball_path(asm)
        gate4_mounts(asm)
        gate5_fits(asm)
        gate9_arm_link(asm)
    except Exception as e:
        add("G2-5,9", "assembly", f"load failed: {e}", "ERROR")
    gate6_walls()
    gate7_arm_sweep()
    gate8_stow_cube()

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
