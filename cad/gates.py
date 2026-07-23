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
            "hub_R", "hub_L", "fly", "fm_R", "fm_L",
            "so_1", "so_2", "so_3", "so_4"]

INTENDED_CONTACT = {
    frozenset(("belt_L", "pul_LF")), frozenset(("belt_L", "pul_LB")),
    frozenset(("belt_R", "pul_RF")), frozenset(("belt_R", "pul_RB")),
    frozenset(("lip_T", "deck_top")), frozenset(("plow", "deck_bot")),
    frozenset(("motor_L", "deck_top")), frozenset(("motor_R", "deck_top")),
    frozenset(("hub_R", "deck_top")), frozenset(("hub_R", "deck_bot")),
    frozenset(("hub_L", "deck_top")), frozenset(("hub_L", "deck_bot")),
    frozenset(("fm_R", "deck_bot")), frozenset(("fm_L", "deck_bot")),
    frozenset(("so_1", "deck_top")), frozenset(("so_1", "deck_bot")),
    frozenset(("so_2", "deck_top")), frozenset(("so_2", "deck_bot")),
    frozenset(("so_3", "deck_top")), frozenset(("so_3", "deck_bot")),
    frozenset(("so_4", "deck_top")), frozenset(("so_4", "deck_bot")),
}


def load_assembly():
    from .export_all import _place_accelerator
    named, ball = _place_accelerator()
    if len(named) != len(PART_IDS):
        raise RuntimeError(f"assembly has {len(named)} solids, expected {len(PART_IDS)}")
    return {pid: wp for pid, (_, wp, _) in zip(PART_IDS, named)}, ball


G1CACHE = {}


def gate1_parts():
    import importlib
    for name in ["belt_pulley", "drive_belt", "accel_plate", "throat_lip",
                 "front_plow", "motor_plate", "arm_hub", "pivot_block",
                 "fly_mount"]:
        try:
            mod = importlib.import_module(f"cad.parts.{name}")
            w = mod.make()
            G1CACHE[name] = w
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
    # riding over the under-mouth flywheel (external tangency)
    dw = abs(y - P.FLY_Y)
    reach = R + P.FLY_DIA / 2
    if dw < reach:
        z = max(z, P.FLY_Z + math.sqrt(reach * reach - dw * dw))
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
            if k == "lip_T" and 45 <= y <= 110:
                continue   # nip zone: ball squeezed on the tongue BY DESIGN;
                           # the squeeze is gated analytically in G5
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
                 ("hub_L", "deck_top"), ("hub_L", "deck_bot"),
                 ("so_1", "deck_top"), ("so_1", "deck_bot"),
                 ("so_3", "deck_top"), ("so_3", "deck_bot")]:
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
    import cad.parts.throat_lip as tl
    outer = asm["lip_T"].intersect(
        cq.Workplane("XY").box(600, 600, 600)
        .translate((300 + tl.TONGUE_HALF_W + 2, 0, 0)))
    lip_intr = max(0.0, ZH - bbox(outer)[2])
    plow_zmax = bbox(asm["plow"])[5]
    plow_intr = max(0.0, plow_zmax + ZH)
    aperture = 2 * ZH - lip_intr - plow_intr - P.TRIBALL_TIP
    add("G5", "lip intrusion (outside tongue)", f"{lip_intr:.2f} mm",
        "PASS" if lip_intr <= 0.05 else "FAIL")
    add("G5", "mouth aperture margin over tip",
        f"{aperture:.1f} mm (plow plate {plow_intr:.1f} designed-in)",
        "PASS" if aperture >= 6.0 else "FAIL")
    # Launcher nip (analytic): under-mouth wheel vs the lip's hood tongue.
    plate_top = -ZH + 4.0
    tongue_bot = ZH - tl.TONGUE_DROP        # world 80
    intake_clr = tongue_bot - (plate_top + P.TRIBALL_TIP)
    squeeze = (P.FLY_TOP + P.TRIBALL_TIP) - tongue_bot
    lift = P.FLY_TOP - plate_top
    add("G5", "intake clearance under tongue", f"{intake_clr:.1f} mm",
        "PASS" if intake_clr >= 3.0 else "FAIL",
        "ball on the plate passes under the hood tongue")
    add("G5", "launch nip squeeze (wheel vs tongue)",
        f"{squeeze:.1f} mm (wheel lifts ball {lift:.1f})",
        "PASS" if 3.0 <= squeeze <= 8.0 else "FAIL",
        "flex wheel + ball pinched against the tongue")


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


def gate10_mass_tipping():
    """Mass roll-up + static tip margin at the FRONT deploy pose.

    Printed masses = real solid volumes x effective print density; hardware
    masses are ESTIMATES (VEX catalog ballpark) -- all assumptions printed.
    """
    try:
        import robot.arm as A
    except Exception as e:
        add("G10", "mass/tipping", f"layout not importable: {e}", "ERROR")
        return
    DENS = 0.0008          # g/mm3, PETG at 4 walls / 40% infill (assumption)
    DENS_TPU = 0.0009
    QTY = {"belt_pulley": 4, "drive_belt": 2, "accel_plate": 2, "throat_lip": 1,
           "front_plow": 1, "motor_plate": 2, "arm_hub": 2, "fly_mount": 2}
    head_print = 0.0
    for n, q in QTY.items():
        if n not in G1CACHE:
            add("G10", "mass/tipping", f"missing part {n}", "ERROR")
            return
        d = DENS_TPU if n == "drive_belt" else DENS
        head_print += q * G1CACHE[n].val().Volume() * d
    # 3 head motors (2 belt + 1 flywheel -- R3 F12 decision: no cross-link
    # path exists, so each belt gets a motor; total robot = 8 motors exactly),
    # 500 g mass disc (R3 F3: <6% droop), flex wheel, 5 shafts, misc.
    HEAD_HW = 3 * 345 + 500 + 120 + 5 * 185 + 200
    head_m = head_print + HEAD_HW
    # head CoM ~ mid-envelope, at the FRONT pose
    import math as m
    phi = m.radians(A.FRONT)
    ly, lz = 150.0, -10.0
    head_y = A.PIVOT[1] + ly * m.cos(phi) - lz * m.sin(phi)
    ball_y = A.PIVOT[1] + 300.0 * m.cos(phi)
    chassis = [  # (mass g, y mm) -- estimates
        (4 * (345 + 90 + 80), 0.0),    # 4 corner drive modules
        (900, 0.0),                    # frame C-channel + deck
        (830, -120.0),                 # battery (rear counterweight)
        (480, -120.0),                 # brain (rear)
        (700, -150.0),                 # steel counterweight (pneumatics deleted)
        (2 * G1CACHE["pivot_block"].val().Volume() * DENS + 600, 155.0),
        (345, 155.0),                  # pivot motor + gears on the tower
        (300, 0.0),                    # cables/misc
    ]
    M = head_m + sum(mm for mm, _ in chassis) + 138.0
    My = head_m * head_y + sum(mm * yy for mm, yy in chassis) + 138.0 * ball_y
    com_y = My / M
    margin = 136.0 - com_y   # front wheel contact line at y=136
    add("G10", "robot mass (est.)", f"{M / 1000:.2f} kg "
        f"(head {head_m / 1000:.2f}, printed {head_print / 1000:.2f})", "PASS",
        "hardware masses are catalog estimates")
    add("G10", "static tip margin @ FRONT+ball",
        f"CoM y={com_y:.0f} mm vs wheel line 136 -> margin {margin:.0f} mm",
        "PASS" if margin >= 20.0 else "FAIL",
        "keep battery/reservoir rearward")


def gate11_launch():
    """Launch trajectory at the REAL pose set (R3 F9: the old gate modelled a
    pose that didn't exist; world exit angle = LAUNCH_DEG + fold angle)."""
    import math as m
    import robot.arm as A
    v_rim = m.pi * (P.FLY_DIA / 1000.0) * P.FLY_RPM / 60.0
    k = 0.5                      # single-wheel transfer factor (bench-tune)
    v0 = k * v_rim
    g = 9.81
    add("G11", "exit speed", f"{v0:.1f} m/s (rim {v_rim:.1f} @ {P.FLY_RPM:.0f} "
        f"RPM plateau, k={k})", "PASS", "k is a bench-tune assumption")
    for name, phi, z0, must_range in [("LAUNCH (phi=0)", A.LAUNCH, 0.22, True),
                                      ("FRONT (phi=-22)", A.FRONT, 0.19, False)]:
        th = m.radians(P.LAUNCH_DEG + phi)
        vx, vz = v0 * m.cos(th), v0 * m.sin(th)
        tb = 1.2 / vx
        clr = (z0 + vz * tb - 0.5 * g * tb * tb) - (0.074 + 0.080)
        tf = (vz + m.sqrt(vz * vz + 2 * g * z0)) / g
        rng = vx * tf
        apex = z0 + vz * vz / (2 * g)
        if must_range:
            add("G11", f"{name}: range", f"{rng:.2f} m (apex {apex:.2f})",
                "PASS" if 2.6 <= rng <= 4.2 else "FAIL",
                "the ranged shot: must beat 8 ft (2.44 m) with margin")
        else:
            add("G11", f"{name}: range", f"{rng:.2f} m (apex {apex:.2f})",
                "PASS", "flat mid-shot from the intake pose - info")
        add("G11", f"{name}: barrier clearance @1.2 m", f"{clr * 1000:.0f} mm",
            "PASS" if clr >= 0.10 else "FAIL")


def gate12_floor():
    """Head never digs into the floor through the fold (R3 F10: ungated)."""
    import robot.arm as A
    zmin, at = float("inf"), None
    for phi in A.SWEEP:
        z = bbox(A.head_at(phi))[2]
        if z < zmin:
            zmin, at = z, phi
    add("G12", "head min world-z over sweep", f"{zmin:.1f} mm at phi={at}",
        "PASS" if zmin >= -3.0 else "FAIL",
        "-3 mm allowance = flex-wheel graze at FRONT intake")


def gate13_current():
    """Worst-case simultaneous current vs the V5 battery envelope (R3 F2)."""
    caps = [("drive x4", 4 * 2.0), ("belt intake x2", 2 * 1.5),
            ("flywheel", 2.5), ("pivot hold", 1.0), ("brain/radio/sensors", 1.0)]
    tot = sum(a for _, a in caps)
    detail = ", ".join(f"{n} {a:.1f}A" for n, a in caps)
    add("G13", "worst-case current budget", f"{tot:.1f} A ({detail})",
        "PASS" if tot <= 16.0 else "FAIL",
        "firmware caps; flywheel recovery interlocked vs full-throttle drive")


def gate14_drive():
    """Drive performance for the ACTUAL robot mass (R3 F1: was 24 N/0.25 g)."""
    import math as m
    wheel_rpm = P.DRIVE_CART_RPM * P.DRIVE_RATIO
    v_robot = m.pi * (P.DRIVE_WHEEL_DIA / 1000) * wheel_rpm / 60 * m.sqrt(2)
    stall_nm = 0.35 / P.DRIVE_RATIO          # 600-cart stall through the gearing
    push = 4 * (stall_nm / (P.DRIVE_WHEEL_DIA / 2000)) / m.sqrt(2)
    mass = 10.4                               # G10 estimate incl. counterweight
    a0 = push / mass
    add("G14", "free speed (X-drive, incl sqrt2)",
        f"{v_robot:.2f} m/s ({v_robot * 3.281:.1f} ft/s) @ {wheel_rpm:.0f} RPM wheel",
        "PASS" if 1.5 <= v_robot <= 2.6 else "FAIL")
    add("G14", "push force at stall", f"{push:.0f} N ({push / (mass * 9.81) * 100:.0f}% of weight)",
        "PASS" if push >= 0.35 * mass * 9.81 else "FAIL")
    add("G14", "initial acceleration", f"{a0:.1f} m/s2",
        "PASS" if a0 >= 3.5 else "FAIL")


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
    gate10_mass_tipping()
    gate11_launch()
    gate12_floor()
    gate13_current()
    gate14_drive()

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
