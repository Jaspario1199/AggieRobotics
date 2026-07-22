# VEXU Over Under Robot — Design Document (working draft)

> **Process note.** No CAD until this document has been critiqued and iterated to
> "no holes." This is the single source of truth for *what every part is, why it
> exists, how it interacts with everything else, and how it is fastened.* Sections
> tagged **[RULES — pending]** are being grounded against the official VEXU Over
> Under manual (research in progress) and will be filled with cited numbers before
> we lock v1.

---

## 0. Locked decisions (from the user)

| # | Decision | Choice | Why it matters downstream |
|---|----------|--------|---------------------------|
| D1 | Drivetrain | **X-drive** (4 omni @ 45°) | Sets corner geometry, motor count (4), center-of-mass, and how the arm mass loads the base. |
| D2 | Starting envelope | **~15 in cube** (VEXU small-robot class; exact limit pending) | Everything must nest inside this at match start; the arm expands *after* start. |
| D3 | Actuation | **Hybrid** — motors for drive + belts, pneumatics for arm deploy/latch | Trades motor budget for air; sets solenoid/reservoir count. |
| D4 | Arm | **Linear slide + pivot** (telescoping extend, then fold to the back) | Two clean DOF; defines the shoulder joint, the slide, and the folded/deployed poses. |

---

## 1. Purpose & the core idea

Rebuild (and improve) a VEXU Over Under robot whose entire manipulator is **one
belt accelerator** (the "belt railgun" in this repo) mounted on a **fold-out
arm**, on a **holonomic X-drive** base.

The one manipulator does four things (see the mechanism README):

- **PULL** (intake) — belts run inward, draw the tri-ball into the barrel.
- **HOLD** — belts stop, ball pinched between them (ball control while driving).
- **PUSH** — belts off, drive forward, the plow shoves the ball.
- **LAUNCH** — belts spin up, ball fired out at belt surface speed.

**The dual-position trick.** The belt barrel is open at both ends and the belts
are reversible, so a *single* working mouth can both intake and launch. The arm
gives that mouth two useful home positions:

- **Deployed (extended forward):** the arm slides + swings the belt head out the
  **front** of the robot, mouth facing forward — intake off the field, or launch
  forward.
- **Folded (resting on the back):** the arm folds ~180° so the belt head lies on
  the rear deck with its mouth now facing **rearward** — intake/launch off the
  *back* of the robot without deploying the arm.

So the robot can work off either end depending on what the match wants, and the
**folded pose is also the legal start configuration** (compact, inside the cube);
**deploying = the post-start expansion** the rules allow.

---

## 2. Requirements

### 2.1 Rules/legality (VEXU / VURC 2023–24 Over Under)
Grounded against the VRC Over Under Game Manual v4.0 + VEX U appendix and the
VURC Q&A. Numbers are high-confidence unless flagged **(verify)**; the five
flagged items in §11-H9 must be checked against the manual PDF before the build.

- **R1 Two robots/team, two size classes.** Start cubes are **24×24×24 in** and
  **15×15×15 in**. *We are building the **15″ robot.*** It must start fully inside
  the 15″ cube and pass inspection there.
- **R2 Expansion after start (corrected — good news).** The **15″ robot is governed
  by standard SG2/SG3 → it may expand horizontally to 36 in**; only the *start* is
  the 15″ cube. The **24 in** cap (VUG2) applies to the team's *other* (24″) robot,
  not this one. → The arm's ~20.5″ deployed footprint is legal with large margin;
  reach is not the binding constraint (packaging/tipping are).
- **R3 Motors: no count cap in VEXU** (VRC's 88 W cap is removed). BUT VEXos
  drops the per-motor current limit once **>8 motors** are connected. → **Hold the
  total to ≤ 8 V5 motors** so each keeps full current (matters for launch power).
- **R4 Pneumatics (corrected):** **VEX pneumatics only**, **≤2 reservoirs**,
  **100 psi max**, VEX solenoids. (Not "less restricted" — the 2-reservoir limit
  binds.) **Air budget matters:** a 200 mL reservoir ≈ 15–25 double-acting actuations
  → don't cycle the slide every intake or it runs dry; compute the budget or move the
  slide to the spare motor.
- **R5 Parts (corrected to VEX U appendix, not VRC):** **3D-printed + custom-fabricated
  parts from the raw-material list are legal and unlimited** — **polyurethane is on the
  list, so TPU belts are legal as a fabricated part** (not "because COTS belts are
  banned"). **COTS bearings + linear slides/bearings are legal** (ruled "fasteners,"
  Q&A #1208). **COTS gears are NOT legal** (VUR4 — must be VEX or fabricated). **Only
  V5 Smart Motors** may actuate (no other motors/servos). **1 V5 Brain + 1 radio** per
  robot; **1 extra battery allowed for sensors/processing only** (not motors/solenoids).
  → Keep gears + motors + wheels VEX; the linear slide *may* be COTS but keeping it VEX
  is the safe call; print structure/pulleys/plates + TPU belts freely.
- **R6 No robot weight limit.** (Still keep mass sane for tipping/agility.)
- **R7 Two-robot team.** VEX U fields **two robots (24″ + 15″)**; the standard split is
  **big robot elevates, small robot scores.** *This* is the 15″ scorer → deferring hang
  (H11) is justified **iff** the 24″ partner elevates. Elevation scores **20/15/10/5**
  (confirmed); measured under the robot's lowest point (bar height: verify Appendix A).

**Field/scoring targets for the launcher (designer-usable):**
- Field **12 ft × 12 ft**; central **Barrier** ≈ **2.9 in** tall PVC — you launch
  *over* it. Each half ~72 in deep.
- **Two netted goals**, lowest opening ≈ **5.78 in** above the tiles; a triball
  scores when **≥2 of its 3 points are inside** = **5 pts** (offensive zone = 2).
- Launching over the barrier is **legal and mainstream**. Typical cross-barrier
  shot ≈ **4–8 ft** horizontal clearing a ~3 in barrier → **modest launch energy**
  (this shrinks the belt head we need). Never launch elements *out of* the field.

### 2.2 Functional
- F1 Intake a tri-ball off the floor (front, deployed).
- F2 Intake a tri-ball off the floor (rear, folded) — the "resting on the back" mode.
- F3 Hold a ball securely while driving holonomically (no ejection on hard turns).
- F4 Launch a ball across the field into the far goal (repeatable near/far).
- F5 Push a ball through a contested zone (belts off, plow engaged).
- F6 Deploy/retract the arm reliably and hold either pose under drive loads.
- F7 Drive omnidirectionally (translate + rotate simultaneously).

### 2.3 Mechanical / quality
- M1 Fit the start envelope with margin (target ≥ 0.5 in on each axis).
- M2 Center of mass low and centered enough that the deployed arm doesn't tip it.
- M3 Every load path resolved into metal fasteners into metal or heat-set inserts;
  no printed part carries a thread that sees repeated high load.
- M4 Field-serviceable: swap a belt, motor, or the whole arm head without a teardown.
- M5 Wire/air routing that survives the arm's full range of motion (no pinch/stretch).

---

## 3. System architecture

Six subsystems. Each has its own spec below; §7 is the interaction matrix and §8
the fastening scheme.

```
                         ┌─────────────────────────────┐
                         │  S6 Power & Electronics       │
                         │  (V5 brain, battery, radio)   │
                         └──────────────┬────────────────┘
             commands / power           │
        ┌───────────────┬───────────────┼───────────────┬──────────────┐
        ▼               ▼               ▼               ▼              ▼
  ┌───────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐ ┌────────────┐
  │ S2 X-drive │  │ S3 Belt    │  │ S4 Arm     │  │ S5 Pneu-   │ │ S1 Chassis │
  │ (4 motors) │  │ head       │  │ deploy     │  │ matics     │ │ / structure│
  │            │  │ (2 motors) │  │ (slide+piv)│  │ (air)      │ │ (frame)    │
  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘ └─────┬──────┘
        │ reacts thrust  │ carries ball  │ moves head    │ actuates     │ carries
        └────────────────┴───────┬───────┴──────┬────────┘   slide/latch │ everything
                                  ▼              ▼                        ▼
                          the tri-ball    the field                 the cube envelope
```

- **S1 Chassis / structure** — the frame everything bolts to; two decks.
- **S2 X-drive** — 4 omni wheels @45°, 4 V5 motors; holonomic base.
- **S3 Belt head** — the belt accelerator (this repo's parts), 2 belt motors.
- **S4 Arm deploy** — shoulder pivot + linear slide that carry S3 between poses.
- **S5 Pneumatics** — reservoir(s), solenoids, cylinders for slide/latch.
- **S6 Power & electronics** — V5 brain, battery, radio, wiring, air valves.

---

## 4. Packaging & coordinate strategy

**World frame (robot):** +X = right, +Y = forward, +Z = up. Origin at the base
center on the floor plane.

**Two decks:**
- **Lower deck (drive deck):** X-drive motors + wheels, V5 battery, brain, and the
  air reservoir — the heavy stuff, low, for a low CoM.
- **Upper deck (arm deck):** the shoulder pivot, the linear slide rails, and the
  folded belt head resting surface. The belt head sits *here* when folded.

**Start (folded) config → fits the cube.** Belt head folded flat on the upper
deck, mouth facing rear, arm slide retracted, pivot latched. This is the ≤15″
start box.

**Deployed config → the legal expansion.** Slide extends forward, pivot swings the
head down/out to intake height at the front. The head clears the front drive
wheels because the X-drive corners are chamfered inward (§S2).

**The central packaging tension (must be solved before CAD):** the belt head is
inherently ~tri-ball-sized (the barrel wraps a ~7″ ball). A head that big folded
on a 15″ deck plus a 15″ X-drive base plus electronics is tight. Candidate
resolutions, to be decided in §11:
- (a) **Compact the head** to a *single* belt-gap barrel just long enough to grip
  the ball over ~1 wrap (shorten `BARREL_LEN`, drop to the minimum pulley count),
  accepting lower launch energy for size.
- (b) **Cantilever the head off the back** in the folded pose so it overhangs the
  rear bumper *within* the expansion allowance rather than the start cube — i.e.
  the head only has to be inside the cube if we call folded the "start," so we may
  instead start with the head nested lower and deploy both directions.
- (c) **Split decks vertically** — head folds into the volume *above* the drive
  motors, using full cube height (Z) rather than deck area (XY).

---

## 5. Subsystem specs

### S1 — Chassis / structure
**Purpose:** rigid frame that (a) locates the 4 X-drive corners, (b) carries the
two decks, (c) provides the shoulder-pivot mounts and slide-rail mounts, (d) takes
launch reaction and drive thrust into the wheels without racking.

**Build:** VEX aluminum C-channel perimeter (square), corner gussets, standoffs
between decks. Custom 3D-printed brackets only where they *locate* parts, not where
they carry the primary launch load (that goes through C-channel + steel screws).

**Key interfaces:**
- 4× corner motor/wheel modules (S2) bolt to the lower-deck corners at 45°.
- Shoulder-pivot bearing blocks (S4) bolt to the upper deck, rear edge.
- Slide rails (S4) bolt along the upper deck.
- Battery/brain/reservoir trays (S6/S5) strap to the lower deck.

**Fastening:** C-channel joined with 8-32 screws + nylocks through the 0.5″ grid;
printed brackets use heat-set inserts *only* for light locating loads; primary
load joints are metal-to-metal.

### S2 — X-drive (holonomic)
**Purpose:** omnidirectional translation + rotation; also *reacts the launch
recoil* and any push loads.

**Build:** 4× VEX 4″ omni wheels at the four corners, each wheel's roller axis at
45° to the frame, each driven by 1× V5 Smart Motor through a spur pair (ratio TBD
for a speed/torque target; start ~ direct or 1:1.6 for speed). Wheels ride on ½″
hex axles in bearing flats.

**Why X-drive (recap):** compact center (leaves the middle free for the arm mass
and battery), true holonomic, and the 45° wheels keep the corners — where the
front wheels would otherwise block the deployed head — clear.

**Motors:** 4 (one per corner). **[RULES — pending]** confirm this fits the budget
alongside S3's belt motors and S4.

**Interactions:** must *hold position* during launch (recoil pushes the base
backward); driver or code applies a brief brace (all wheels resist) at launch, or
we launch only while translating to cancel recoil. Flagged as a control item (§6).

### S3 — Belt head (the manipulator)
**Purpose:** the four-mode manipulator (pull/hold/push/launch). This is the belt
accelerator already modeled in this repo (`cad/parts/*`): 4 `belt_pulley`, 2
`drive_belt`, 2 `accel_plate` decks, 2 `throat_lip`, 1 `front_plow`.

**Tri-ball reality (corrects the mechanism's current sizing):** the ball is a
**three-lobed shape ≈ 6.18 in tip-to-tip**, body ≈ 4–4.5 in across, ≈ **138 g**,
hollow plastic — *not* a 178 mm sphere. Two consequences:
- The **throat opening** must admit the ~6.18 in tip dimension in any orientation
  (the ball is irregular, so intake orientation matters), while the **belts grip
  the ~4–4.5 in body**. So: wide throat, narrower belt gap. `TRIBALL_DIA` in
  `params.py` becomes the *tip* size for the throat; a new `BALL_BODY` (~110 mm)
  sets `BELT_GAP`. → **[H10]**.
- Everything shrinks vs. the current model — good for §4 packaging (H1).

**Adaptation for the arm:**
- `BARREL_LEN`, `BELT_WIDTH`, and pulley Ø are the levers to size the head to the
  corrected ball and the folded envelope (§4, §11).
- **Belts are 3D-printed TPU** (legal custom part per R5) — not a COTS belt. Tread
  wraps the ~4–4.5 in body for grip.
- **Reversible + dual-end:** belts driven by 2 V5 motors (one per side) for
  independent spin control and launch power; the barrel's single mouth intakes or
  launches by belt direction/speed, and the fold reorients that mouth front↔rear.
- **Launch energy is modest** (R2 target ~4–8 ft over a ~3 in barrier, 138 g ball),
  so the head need not be large — belt surface speed + a good exit angle dominate.
- Mounts to the arm's moving carriage (S4) via the belt-head decks' grid holes.

**Motors:** 2 (belt L, belt R). Could be reduced to 1 with a cross-linkage if the
motor budget is tight — noted as a trade in §11.

**Interactions:** hands the ball to "the field" on launch; receives it from the
floor on intake; its mass is the biggest thing the arm (S4) has to move and the
base (S2) has to keep from tipping.

### S4 — Arm deploy (linear slide + pivot)
**Purpose:** carry the belt head (S3) between **folded** (rear deck, mouth rear)
and **deployed** (front, mouth forward, at intake height), and hold either pose
under load.

**Two DOF:**
1. **Shoulder pivot** — a hinge at the rear-top of the upper deck. Swings the arm
   ~180° from lying-on-the-back to pointing-forward-down. **Actuation candidate:**
   a V5 motor through a high-reduction gearbox (holds position, precise angle) —
   *or* pneumatic via a linkage if we can hit the throw (decision in §11; motor
   is the current lean because 180° under a heavy head wants controlled, holdable
   motion).
2. **Linear slide** — a telescoping rail on the arm that extends the head forward
   for reach when deployed. **Actuation:** pneumatic cylinder (fast, self-holds at
   both stroke ends), matching D3 (pneumatics for deploy).

**Latch:** a pneumatic (or passive over-center) latch holds the **folded** pose for
the start and during hard driving so the head can't flop; released to deploy.

**Interactions:** the pivot axis, slide stroke, and folded rest surface are the
geometry that S1 must provide and that S3's size must fit; the air lines and the 2
belt-motor cables must survive the full travel (S5/S6 routing).

### S5 — Pneumatics
**Purpose:** actuate the linear slide (extend/retract) and the fold latch; possibly
a launch-hood/angle flip. **[RULES — pending]** confirm reservoir/cylinder count.

**Build (candidate):** 1–2 air reservoirs on the lower deck, a regulator, and
2–3 double-acting cylinders (slide, latch, optional hood) driven by V5 pneumatic
solenoid valves off the brain's 3-wire ports.

**Interactions:** valves commanded by S6; cylinders act on S4; reservoir mass sits
low in S1 for CoM.

### S6 — Power & electronics
**Purpose:** power and command everything.
**Build:** 1× V5 Brain, 1× V5 Battery, 1× V5 Radio + controller, motor cables,
3-wire leads to pneumatic solenoids, and (optional) sensors: an optical/distance
sensor at the mouth (ball present?), an IMU for field-oriented holonomic drive, and
motor encoders (built into V5) for belt-speed = launch-speed closed loop.
**Interactions:** every subsystem; wire routing across the arm hinge is a named
risk (M5). **[RULES — pending]** total motor count must fit the VEXU limit.

---

## 6. Control strategy (per mode)

| Mode | Drive (S2) | Belts (S3) | Arm (S4) | Pneu (S5) | Notes |
|------|-----------|-----------|----------|-----------|-------|
| Drive/seek | holonomic | off or HOLD | folded or deployed | latched | field-oriented via IMU |
| Intake front | creep fwd | PULL (inward, slow) | deployed, mouth fwd | slide out | ball sensor confirms capture → HOLD |
| Intake rear | creep back | PULL | folded, mouth rear | latched | no deploy needed |
| Hold + travel | holonomic | HOLD (stall/creep) | either | — | belts lightly grip; no launch energy |
| Push | drive into ball | off | deployed low | slide out | plow engaged |
| Launch | brace/translate to cancel recoil | spin up to target belt speed | pose set for angle | hood set | encoder closes speed loop for range |

**Recoil:** launching a ~60 g ball at speed imparts a recoil impulse; the X-drive
holds/opposes it (all four wheels brace), or we launch while creeping to cancel —
a control-level item, but it validates choosing a holonomic base that can hold
station.

---

## 7. Interaction matrix (who touches whom, and how)

| ↓ acts on → | S1 Chassis | S2 Drive | S3 Belt head | S4 Arm | S5 Pneu | S6 Elec |
|-------------|-----------|----------|--------------|--------|---------|---------|
| **S1 Chassis** | — | locates 4 corners; takes thrust | provides folded rest surface | provides pivot + rail mounts | holds reservoir tray | holds brain/battery trays |
| **S2 Drive** | thrust/recoil into frame | — | keeps base under the ball | keeps base stable while arm moves | — | draws power/PWM |
| **S3 Belt head** | mass on upper deck | ball control aids pushing | — | is the payload the arm carries | — | 2 motor cables |
| **S4 Arm** | loads pivot/rails | shifts CoM when deployed | positions + orients the mouth | — | driven by slide cylinder + latch | motor cable + encoder |
| **S5 Pneu** | reservoir mass | — | — | extends slide, throws latch | — | solenoids on 3-wire ports |
| **S6 Elec** | tray loads | commands drive | commands belts + reads ball sensor | commands pivot + reads angle | commands valves | — |

*(Each non-empty cell is a real interface that must have a defined mount, load
path, and — where it moves — a routing plan. Cells that move relative to each other
across the hinge: S3↔S6 cables and S4↔S5 air lines — flagged M5.)*

---

## 8. Fastening scheme (global rules)

1. **Primary structure (frame, decks, motor mounts, shoulder blocks):** VEX metal
   C-channel/plate, joined with **8-32 screws + nylock nuts** through the 0.5″ grid.
   No printed part in a primary load path.
2. **Shafts:** ½″ hex in VEX bearing flats; wheels/pulleys/gears retained with
   **shaft collars + set screws**; hex captures torque.
3. **Printed parts (belt head decks, pulleys, throat lips, plow, brackets):**
   bolt to metal with 8-32 clearance holes + nuts; where a screw must thread into
   plastic, use a **heat-set brass insert**, never a bare tapped hole under load.
4. **Belt-head to arm carriage:** the two accel-plate decks bolt to the carriage
   through their existing 0.5″ grid, straddling the pulley bores for a stiff joint.
5. **Pivot joint:** hardened ½″ or ¾″ shaft in dual bearing blocks (both sides),
   never a single-shear printed pivot.
6. **Air/electrical:** service loops at the hinge; strain-relief clamps on both the
   fixed and moving sides so nothing pulls on a connector.

---

## 9. Mass / CG budget (rough, to refine)

| Group | Est. mass | Location | CoM effect |
|-------|-----------|----------|-----------|
| X-drive (motors+wheels+frame) | heavy | low, corners | anchors CoM low/centered |
| Battery + brain + reservoir | heavy | low deck, centered | low CoM |
| Belt head + 2 motors | heavy | upper/rear (folded) or front (deployed) | **the swing risk** |
| Arm slide + pivot gearbox | medium | upper rear | rear-biases CoM |

**The tipping check (must pass in §10 iteration):** with the arm fully deployed
forward + a ball at the mouth, the CoM must stay inside the front wheelbase with
margin. Levers: keep battery/reservoir low and rearward to counterbalance; limit
slide extension; widen the effective front track via the 45° wheel geometry.

---

## 10. BOM (skeleton — quantities/motors finalize after [RULES])

| Subsys | Item | Qty | Purpose |
|--------|------|-----|---------|
| S1 | VEX aluminum C-channel (various) | TBD | frame perimeter + decks |
| S1 | Corner gussets / plates | TBD | rack-resist the frame |
| S1 | Standoffs (deck spacers) | TBD | separate the two decks |
| S2 | VEX 4″ omni wheel | 4 | X-drive |
| S2 | V5 Smart Motor (11W) | 4 | drive |
| S2 | ½″ hex axle + bearing flats + collars | 4 sets | wheel mounts |
| S2 | Spur gear pair | 4 | drive ratio |
| S3 | `belt_pulley` (printed) | 4 | belts ride on these |
| S3 | `drive_belt` (**3D-printed TPU** — legal custom part) | 2 | grip + accelerate the ball |
| S3 | `accel_plate` deck (printed) | 2 | barrel walls / pulley mounts |
| S3 | `throat_lip` (printed) | 2 | flared mouth |
| S3 | `front_plow` (printed) | 1 | push mode + cross-brace |
| S3 | V5 Smart Motor | 2 | belt drive (or 1 + cross-link) |
| S3 | ½″ hex shafts + bearings + collars | 4 | pulley shafts |
| S4 | Linear slide rail + carriage | 1 | extend the head |
| S4 | Pneumatic cylinder (slide) | 1 | extend/retract |
| S4 | Shoulder pivot shaft + bearing blocks | 1 set | fold DOF |
| S4 | V5 Smart Motor + gearbox (pivot) | 1 | fold actuation (candidate) |
| S4 | Latch (pneumatic or over-center) | 1 | hold folded |
| S5 | Air reservoir | 1–2 | pneumatic supply |
| S5 | Solenoid valve | 2–3 | slide/latch/hood |
| S5 | Regulator + tubing + fittings | 1 set | plumbing |
| S6 | V5 Brain | 1 | controller |
| S6 | V5 Battery | 1 | power |
| S6 | V5 Radio + Controller | 1 | comms |
| S6 | Optical/distance sensor | 1 | ball-present at mouth |
| S6 | IMU (or V5 built-in) | 1 | field-oriented drive |

**Motor tally:** 4 drive + 2 belt + 1 pivot = **7 V5 motors** (slide + latch are
pneumatic). VEXU has no motor cap, but we deliberately stay **≤8** so VEXos keeps
each motor at full current (launch power). 1 spare port; use it for an assist
roller, a 2nd pivot motor, or leave it. All motors + wheels + gears + bearings +
shafts are **VEX** (R5); pulleys/plates/lips/plow are **printed**, belts **TPU**.

---

## 11. Open holes to close *before* CAD (the iteration list)

These are the "holes" we critique and drive to zero. Each must get a decided
answer written back into the sections above.

- **H1 Size reconciliation** *(the #1 hole; eased by H10)* — does the belt head fit
  folded in the 15″ cube with the X-drive + electronics? With the corrected smaller
  ball the head shrinks a lot; still must pick the packaging (§4 a/b/c) and prove
  the folded stack ≤ 15″ on all axes with ≥0.5″ margin, and the deployed footprint
  ≤ the R2 horizontal cap.
- **H2 Fold actuation** — motor vs pneumatic-linkage for the ~180° pivot; confirm it
  holds the head folded and deployed under drive/recoil. (Motor is the lean.)
- **H3 Motor budget** — *mostly resolved:* VEXU has no cap but we hold **≤8** to keep
  full current. Plan = 4 drive + 2 belt + 1 pivot = **7** (1 spare). Decide 1-vs-2
  belt motors only if we want the 8th for an assist roller or 2nd pivot motor.
- **H4 Recoil management** — quantify launch recoil (≈ m·Δv of a 138 g ball) vs base
  holding force; confirm the X-drive holds station, or launch-while-creeping.
- **H5 Tipping** — CoM/wheelbase check deployed + loaded (§9).
- **H6 Dual-end geometry** — verify the *same* mouth presents correctly both folded
  (rear) and deployed (front); define the two intake heights (front floor pickup;
  rear pickup height when folded). Throat must clear the 6.18″ tip in any orientation.
- **H7 Wire/air over the hinge** — define the service-loop routing (M5).
- **H8 Launch performance** — *targets now known:* clear a ~2.9″ barrier into a goal
  whose opening is ~5.78″ up, ~4–8 ft away, 138 g ball. Solve exit speed → belt
  surface speed = `(RPM/60)·π·PULLEY_PITCH_DIA` → motor RPM + gear-up; set a launch
  angle (hood). Modest energy → confirms a small head is enough.
- **H9 Rules legality** — *filled (§2.1).* Residual items to verify against the
  manual PDF: (1) 15″-robot horizontal expansion cap; (2) elevation-bar height +
  point values; (3) exact VUR rule IDs; (4) triball weight/secondary dims; (5) VEXU
  reservoir-count nuance / 2nd brain-battery allowance.
- **H10 Ball re-size** — update `params.py`: `TRIBALL_DIA` → ~6.18″ *tip* (throat),
  add `BALL_BODY` ~110 mm → `BELT_GAP`; re-derive head size. Feeds H1/H6/H8.
- **H11 Endgame hang (optional / stretch)** — elevation scores 20/15/10/5. Could the
  folding arm double as a hook to hang on the Elevation Bar? Big value if cheap;
  out of core scope for v1 but worth a hook geometry check before we finalize the arm.

---

## 13. Resolved design — closing H1–H11 (v0.3)

Concrete decisions + numbers that supersede the placeholders above. Units mixed
(VEX is imperial; mechanism is metric) but each value notes both where it matters.
Masses/dims marked ≈ are engineering estimates to confirm in CAD.

### 13.0 Tri-ball working numbers (H10 — closed)
- Tip-to-tip **157 mm (6.18″)** → sets throat + tip-clearance.
- Body across **≈110 mm (≈4.3″)** → sets the belt grip *(community-approx; verify)*.
- Mass **138 g**, **rigid hollow shell** → grip by **compliant TPU belts** (belts
  deform, ball doesn't). `params.py`: `TRIBALL_DIA`=157 (throat), new `BALL_BODY`=110.

### 13.1 Chassis (S1) — closed
- Footprint **368 × 368 mm (14.5″)** inside the 15″ cube (≈0.5″/side margin, M1).
- Drive deck height **≈100 mm (4″)**; upper/arm deck above it.
- VEX aluminum C-channel perimeter + corner gussets; steel 8-32 through the 0.5″ grid.

### 13.2 X-drive (S2) — closed
- 4× VEX **3.25″ omni** at 45° corners, **½″ hex** in bearing flats, **shaft-collar**
  retained. Each on **1× V5 motor, 600 RPM (blue) cartridge, ~1:1** → **≈8 ft/s** free.
- Holonomic control + **field-oriented via a V5 Inertial (IMU)**.
- **Recoil (H4 — closed):** launch impulse = m·v = 0.138 kg × 6 m/s ≈ **0.83 kg·m/s**;
  over a ~0.06 s contact ≈ **~14 N** peak. Wheel friction (robot ≈5 kg → ~25 N/wheel
  static) dwarfs it → base holds station; **no special recoil measure needed**, and
  code adds a 4-wheel brace at launch for repeatability.

### 13.3 Belt head (S3) — compact, closed (H1)
- `BELT_GAP` **≈100 mm** (grips the ≈110 mm body via TPU deform).
- Tip clearance (deck spacing across the other axis) **≈170 mm** so the 157 mm tips
  clear as the ball tumbles in.
- `BARREL_LEN` **≈90 mm**, `BELT_WIDTH` **≈90 mm**, `PULLEY_PITCH_DIA` **50 mm**.
- Throat lips flare the mouth to **≈160 mm** to swallow the tip in any orientation;
  the throat **cams the ball into a repeatable lobe-leading pose** for consistent
  launch (H6 detail; confirm against real ball CAD).
- **Head envelope ≈ 180 × 165 × 150 mm (≈7 × 6.5 × 6″)** — about half the robot.
- **2× V5 belt motors** geared **≈3.75:1** (600 RPM → ~2250 RPM at the 50 mm pulley).
- **Launch (H8 — closed):** belt surface speed v=(RPM/60)·π·D = (2250/60)·π·0.05 ≈
  **5.9 m/s**. Projectile range R=v²·sin2θ/g: at 45°, v=4.9 m/s→**2.4 m (8 ft)**;
  v=3.5→1.25 m (near). So **~3.5–6 m/s covers the whole ~4–8 ft window**; clears the
  74 mm barrier and reaches the ~147 mm goal opening. Speed set by encoder-closed
  belt RPM; a **launch hood sets ~40–45°**. Modest energy → the compact head suffices.

### 13.4 Arm (S4) — closed (H1/H2/H6)
- **Fold plane is vertical (fore-aft).** The shoulder pivot swings the head between
  **front-floor** (deployed intake/launch) and **rear working** (folded intake/launch
  off the back), passing over the top. ~**180°** throw.
- **Pivot = V5 motor + ≈7:1 reduction** (H2 — closed). Hold torque worst case
  (head ≈1.5 kg at ≈0.2 m, horizontal) = m·g·r ≈ **2.9 N·m**; motor+7:1 gives
  ample margin and 180° in ~1.5 s; **active braking holds deployed**, and a
  **pneumatic latch** holds folded (start + hard driving).
- **Slide = VEX linear rail + pneumatic cylinder, stroke ≈100 mm (4″)** for front reach.
- **Deployed footprint:** base 14.5″ + reach ≈6″ ≈ **20.5″ < 24″** expansion target (R2).
- **Poses / intake heights (H6 — closed):**
  - *Start/folded:* head at rear, mouth angled **down-rear ~30–45°** to work off the
    floor behind; slide retracted; latched. Stack height ≈ **280 mm (11″) < 15″** (M1).
  - *Deployed-front:* mouth at floor (**0–50 mm**), slide out.

### 13.5 Pneumatics (S5) — closed
- **1 air reservoir**, regulator at **~60–80 psi** (100 psi max, R4), on the low deck.
- **2 double-acting cylinders:** slide (≈100 mm stroke) + fold latch — each on a V5
  **3-wire solenoid**. (Reserve capacity for an optional hood cylinder.)

### 13.6 Power/electronics (S6) — closed
- **7 V5 motors** (4 drive, 2 belt, 1 pivot) — ≤8 for full current (R3). **1 spare port.**
- V5 Brain, Battery, Radio + Controller; **Inertial sensor** (field-oriented drive);
  **1 distance/optical sensor** at the mouth (ball-present → auto HOLD); motor encoders
  close the belt-speed loop for launch range.
- **Routing (H7 — closed):** the 2 belt-motor cables + pivot encoder + the slide air
  line cross the shoulder **coaxially with the pivot axis** with a service loop,
  strain-relieved on both fixed and moving sides (M5). Reservoir/valves on the low deck.

### 13.7 Tipping (H5 — closed)
- Deployed + ball: head ≈1.5 kg at ≈0.25 m ahead of pivot → CoM shift ≈ 1.5·0.25/5.0
  ≈ **75 mm** forward; front half-wheelbase ≈ **184 mm** → **stable with ~2.4× margin**.
  Keep **battery + reservoir low and rear**; cap slide extension. Confirm with CAD masses.

### 13.8 Endgame hang (H11 — decision)
- **Deferred to a v2 stretch,** *but* we **reserve the 8th motor port and add a hook
  boss** to the head so the arm can later hook the Elevation Bar (20 pts) without a
  redesign. Not on the v1 critical path.

### 13.9 Motor & pneumatic budget (final)
| Motor | Use | Cartridge / ratio |
|-------|-----|-------------------|
| ×4 | X-drive | 600 RPM, ~1:1 |
| ×2 | belts | 600 RPM, ~3.75:1 up |
| ×1 | arm pivot | ~7:1 down |
| (spare) | reserved (hang / assist) | — |

Pneumatics: 1 reservoir, 2 cylinders (slide, latch), 2 solenoids.

**All eleven holes H1–H11 now have a written resolution.** Remaining before CAD:
one adversarial critique pass to catch anything missed (§11 says "iterate to no
holes"), plus confirming the five flagged rule numbers (H9) against the manual PDF.

---

## 15. v1.0 — post-critique resolved design (supersedes §13)

Every §14 finding folded in. Launch-independent holes are closed with corrected
numbers; the two things that genuinely need a bench prototype are called out (§15.9)
rather than fake-closed. **Working launch baseline = flywheel-fed-by-belt** (my
recommendation; reversible — only §15.4 changes if we go catapult).

### 15.0 What survived, what changed
- **Belt keeps 3 of its 4 jobs — intake / hold / push / ball-control — unchanged.**
  It is no longer the launcher (both red-teams: wrong tool for a rigid irregular ball).
- **Launch is now a dedicated flywheel** the belt feeds. This is the only architectural
  change; everything else is a numbers/geometry fix.

### 15.1 Fold kinematics — fixed topology (closes C-B1/B4)
The rear-pivot + 100 mm slide was impossible. New scheme: **center-top pivot +
telescoping slide.**
- **Pivot at the robot center**, z ≈ 110 mm (top of drive deck), in **dual** bearing
  blocks (no single-shear). Driven by the pivot motor.
- **Short arm** (pivot → head carriage) + **slide** provides reach. The head folds into
  the **center-to-rear** deck span (≈190 mm — exactly the head length), mouth rearward,
  for the legal start. Deployed, the arm swings forward-down and the **slide extends** to
  put the mouth at the **front floor** and over the front edge (barrier reach).
- **Swing clearance:** head radius ≈190 mm about a center pivot → peak height ≈300 mm
  < 381 mm cube, and stays within ±190 mm horizontally. **Slide must retract before
  folding** (interlock, C-M12); the swept volume through 0–180° must clear the wheels/
  electronics.
- **Residual (legit CAD item):** exact link lengths + the 0/30/…/180° pose drawings are
  a **CAD swept-volume study** — the topology closes; the millimeters get set on the drawing.

### 15.2 Head — corrected envelope, mass, load path (closes C-B3/M2/M6)
- **Envelope ≈ 190 (L) × 210 (W) × 182 (H) mm**, mass **≈ 2.0 kg** (2× V5 ≈ 0.46 kg alone).
- **Metal structural spine** (VEX C-channel/plate) carries belt tension, the carriage
  joint, and the flywheel/pulley bearing loads; **printed plates become shrouds/guides,
  not primary structure** (obeys §8.1). Pulleys get **VEX/metal hubs or heat-set inserts**
  — no plastic hex bore under launch/intake torque reversal.

### 15.3 Belt intake (role unchanged, re-geared)
- Grips the ≈110 mm body; throat flares to swallow the 157 mm tip; **`BELT_WIDTH` → ~120 mm**
  (≥ body, fixes C-M14). Since the belt no longer launches, gear it for **intake speed
  (~1:1)**, not 3.75:1 — lower stress, no seam-ripple launch dependence (defuses C-M3).
- **1 belt motor** (cross-linked L/R) suffices for intake/hold → frees a port. A throat
  detent/hard stop holds the ball so belts needn't stall-hold (C-m5).

### 15.4 Flywheel launcher (new — closes C2-B1)
- **Dedicated flywheel at the muzzle** with **real metal flywheel mass** (VEX/fabricated) =
  the energy store the belt lacked. Driven by **1 V5 motor geared UP** to a rim speed of
  ~5–7 m/s; **encoder holds RPM**, flywheel inertia carries the shot (no torque-starve).
- **Belt feeds the ball into the flywheel nip; a compression hood** holds it against the
  wheel over a defined arc → more consistent than a single-tangent kick, and sets the
  **launch angle (~40–45°)**.
- **Rate ~1 shot/s** (re-spin); **mid-range** (~4–8 ft, over the 2.9″ barrier, into the
  net) — the far goal (>8 ft) is a stretch, accepted. **Trajectory = launch-from-height,
  and must clear/stay under the midfield Elevation Bar structure** (C2-B2) — a real
  CAD/sim check with field geometry, not the flat-ground formula.

### 15.5 Motor & pneumatic budget (final, ≤8)
| Motor | Use | Cartridge / ratio |
|-------|-----|-------------------|
| ×4 | X-drive | 600 RPM ~1:1 |
| ×1 | belt intake (cross-linked) | ~1:1 |
| ×1 | flywheel launch | up to ~5–7 m/s rim |
| ×1 | arm pivot | **100 RPM cartridge** ×~7 → ~14.7 N·m ≫ the ~5.6 N·m hold need (fixes C2-m2) |
| (spare) | 2nd flywheel or 2nd belt | — |
= **7 motors, 1 spare.** Pneumatics: **VEX, ≤2 reservoirs, 100 psi**; **compute the air
budget** — slide gets a mechanical hard-stop; **fold latch is passive over-center** (works
powered-off for inspection). If air budget is tight, motor the slide with the spare port.

### 15.6 Localization + controls (closes C2-M5)
- **2–3 dead-wheel tracking pods** for X/Y odometry (omni encoders slip) + IMU heading →
  field position for auton/skills aiming; a **goal-relative align routine** before launch.
- Ball-present distance sensor → auto-HOLD; launch only when not flooring the drive (peak-
  current, C2-m7). Recoil is a non-issue (~14 N ≪ ~49 N total friction).

### 15.7 Two-robot strategy (closes C2-M4 / R7)
This is the **15″ scorer**; the team's **24″ robot elevates**. Hang deferred here is now
*justified*, not hand-waved. (If we later want this bot to also hang, that's a v2 with the
spare port + a hook — not on the v1 path.)

### 15.8 Margins to re-verify in CAD (with real masses)
Start footprint incl. wheels/fasteners → target frame **≤360 mm (≈0.4″/side margin**, fixes
m1/C-M7); tipping with the ~2 kg head deployed (battery + reservoir low & rear); front-wheel-
gap clearance for the ~210 mm head mid-fold (C-M5).

### 15.9 The two things a prototype must close *before/at* CAD (honest)
Both red-teams agree these can't be closed on paper:
1. **Belt capture of the real rigid tri-ball** — does the throat actually orient/admit a
   157 mm irregular ball without jamming? Bench-test with the real (or printed) ball.
2. **Flywheel launch consistency** with the real ball — measure exit-speed/angle scatter.
→ Recommend a quick physical bench test (or a printed-ball proxy) to de-risk before
committing full CAD; or proceed to CAD with the compression-hood + detent mitigations
designed in and validate on the first print.

---

## 14. Critique round 1 — mechanical red-team (REOPENS "closed" holes)

An adversarial mechanical review re-did the §13 arithmetic and found several
closures don't hold. §13's "all holes closed" is **retracted** pending redesign.

**Blockers**
- **C-B1 Fold kinematics don't close.** Rear pivot + 100 mm slide can't reach the
  front floor *and* fold in-cube: pivot→mouth ≈ 518 mm horizontal, arm ≈ 430–530 mm;
  folding it rearward busts the 381 mm cube by ~250 mm or points it up. → **reopen H1/H6;**
  must publish real pivot (x,z), link lengths, slide stroke, and draw 0–180° poses.
  Likely needs a *central* pivot, longer slide, or a 2nd link.
- **C-B2 Rigid 3-lobe ball breaks belt-launch repeatability.** Belt-speed=exit-speed
  is sphere logic; lobe-apex contact → variable effective radius → non-repeatable
  speed/spin; 157 mm tip won't enter a 100 mm gap (jam risk); TPU must deform ~5 mm
  (> belt thickness). → **architecture question:** prototype on the real ball, or split
  intake (compliant flex-wheel) from launch (flywheel/catapult).
- **C-B3 Head undersized.** Real envelope ≈ **190×210×182 mm** (tip-clearance 170 +2 decks
  = 182; belt+pulley flanges → ~210 wide), not 180×165×150. Every downstream packaging
  number inherits this. → **reopen H1.**
- **C-B4 Rear floor intake can't be both in-cube and reach the floor.** Latched folded
  pose can't put the mouth at the rear floor without ~120 mm rear overhang (busts cube).
  → decide: rear intake is a post-start *deploy* (counts vs expansion cap), or drop it.

**Major**
- **C-M1** 90 mm `BARREL_LEN` < ball → no real acceleration distance (H1 fix broke H8).
- **C-M2** Printed PETG decks + plastic-hex pulleys are in the launch/cantilever load
  path — violates §8.1/M3. Need a metal structural spine + metal-hub pulleys.
- **C-M3** Printed TPU belt @2250 RPM: seam speed-ripple + fatigue = worst launch
  consistency, and R5 forces it. Consider **VEX flex wheels / tank tread** (VEX = legal)
  instead of a belt.
- **C-M4** Single linear slide = compliant cantilever holding ~2 kg → aim sag/creep.
  Need dual rails or a trussed pivot-only arm + a hard stop at the launch angle.
- **C-M5** Head (~210 wide) may not clear the gap between front drive modules mid-fold.
- **C-M6** Head mass ~1.8–2.0 kg (2× V5 = 460 g alone) → pivot hold ≈ 4.9 N·m, tipping
  ~1.8× not 2.4×.
- **C-M7** Cube margin ~0.25″/side excludes omni wheels + fasteners → may exceed 15″.

**Minor:** deployed pose needs a mechanical hard-stop not motor-hold; start latch must
be passive (inspection is powered-off); dynamic tipping unanalyzed; hood-vs-tilt launch
angle contradiction; double-articulation cabling fatigue; slide-retract-before-fold
interlock; deployed 20.5″ rides on the unverified 24″ cap; belt width 90 < body 110.

**Top 3 to fix:** (1) close fold kinematics with real geometry; (2) prototype belt grip
on the real rigid tri-ball before CAD — seriously weigh flex-wheel-intake + flywheel;
(3) reopen H1 with correct head dims + mass + wheel clearance.

### Critique round 2 — rules / controls / launch-physics red-team

**Blockers**
- **C2-B1 No energy store for launch.** 2× V5 @600 RPM geared 3.75:1 up → ~0.19 N·m at
  the pair of 50 mm pulleys, but the shot needs ~0.29 N·m — **motor stall torque is below
  the shot demand.** The energy must come from flywheel inertia the doc never sized;
  ~2 kg flywheel needed for <10% droop (impossible on a 15″ bot), ~0.4 kg → ~30% speed
  droop → range scatter; re-spin ≈ 0.7–1 s → ~1 shot/s. **The belt cannot be a
  repeatable launcher without becoming a flywheel shooter with real (metal) flywheel mass.**
- **C2-B2 Trajectory used flat-ground physics + ignored the field.** `R=v²sin2θ/g` assumes
  equal launch/land height; real shot is launch-from-height into a raised net. At 45°,
  5 m/s → **apex ≈ 24″** — and midfield has the **Elevation Bar structure** ("Over
  **Under**"); the arc must clear/stay under it. Belt-surface 5.9 vs range-calc 3.5–4.9 m/s
  implies unstated 15–40% slip. **Far goal (>8 ft) is unreachable** at this speed →
  mid-range only. Redo trajectory from height into real goal geometry + bar no-fly zone.

**Major**
- **C2-M1** Irregular ball → variable grip radius/release → exit **speed *and* angle**
  scatter at constant RPM; encoder loop can't fix geometry. (Confirms C-B2.)
- **C2-M2/M3 Rules re-framed** (now in R2/R4/R5/R7): 15″ expands to 36″; VEX pneumatics
  ≤2 reservoirs; TPU belts legal (fabricated); COTS bearings/slides legal, COTS gears not;
  only V5 motors; two-robot game. Air budget can run dry — compute or motor the slide.
- **C2-M4** Two-robot strategy undocumented → "defer hang" only valid if the 24″ partner
  elevates (now R7).
- **C2-M5** **No localization** → auton/skills aiming unsupported (IMU = heading only,
  omni odometry slips). Add dead-wheel tracking pods or a goal-relative align routine.

**Minor (verified):** start margin 0.25″/side not 0.5″ (m1); **pivot needs the 100 RPM
cartridge** — deployed r≈0.35 m, m≈1.6 kg → ~5.6 N·m, a 600-RPM×7 gives only ~2.45 N·m
(m2); head mass 1.8–2.2 kg (m3); TPU-belt durability (m4); ports/battery **fine** (m7);
recoil conclusion holds, number was loose (m6). **R3 "≤8 motors" confirmed correct.**

### Synthesis (the decision this forces)
Both red-teams independently conclude: **the belt is an excellent intake / hold / push /
ball-control mechanism but a poor *precision launcher* for a rigid irregular tri-ball.**
The elegant "one mechanism does all four" has to bend at LAUNCH. Everything else is a
fixable numbers/geometry problem (redo the fold kinematics with real link lengths; recompute
the head envelope ≈190×210×182 mm / ~2 kg; add a metal structural spine + VEX-hub pulleys;
add tracking wheels; assign hang to the 24″ partner). **The one fork that needs the user:
how to launch** — see the recommendation in the reply. Once that's set, the fold geometry +
numbers get redone into an airtight v1.0, *then* CAD.

## 16. Gate-driven refinement (v1.1) — verify, don't assert

The CAD was put under 8 automated verification gates (`cad/gates.py`: pairwise
interference, anisotropic ball-path sweep, mount/fastener contact + real
grid-hole membership, fit bands, printed-wall minimums, arm motion sweep,
stow-in-cube). **Baseline: 15 pass / 24 FAIL. Final: 29 pass / 0 FAIL** —
regression-run after every change (`docs/gates_baseline.txt` → `docs/gates_after.txt`).

**Geometry fixed (all verified by re-run, numbers in the logs):**
- **Throat lip rebuilt** — the old part had full-ring slivers from undersized cut
  boxes and side-plate-era flanges burying 518 mm³ into the decks and clipping
  pulleys/belts. Now: grid-bolted top plate + edge wall + flare with **0.00 mm
  aperture intrusion**; all clashes → 0.
- **Bottom lip DELETED, plow re-designed as the bottom ramp** — mount plate on
  the bottom deck's inner face (45° chamfered edge: ball rolls a ramp, not a
  4 mm step), raked blade to the floor. One part = bottom guide + push blade.
- **Rigid-ball squeeze corrected** — `BALL_COMPRESSION 6→4`, `BELT_THK 4→6`
  (constraint: compression ≤ belt−1). Rigid drum now clears the ball body by
  **2.0 mm/side** (was −2: metal-on-ball interference).
- **Pulley** — drum +5 mm (belt axial float 5.0 mm, was 0 = binding), flanges
  moved **outboard of the ball's z-reach** (38 mm margin), lightening windows
  re-sized (window-to-bore wall 1.97→2.52 mm).
- **Motor plates** — seated on the deck (was floating 10 mm), and the second
  plate added; lip/plow bolt holes verified to land on **real** deck grid holes.

**Layout corrected (robot/arm.py is now the single kinematic source):**
- The head (~227 mm wide) is WIDER than the wheel gap (~196 mm) → it can never
  pass between wheels; every swing must clear them vertically. Pivot moved to
  (y=155, z=221), carriage at the head's rear face.
- **Slide REMOVED** (decision, rationale recorded): a straight arm from a high
  pivot provably cannot reach the REAR floor over the rear wheels (head bottom
  ≥ 92 mm at y=−136 forces pitch ≥ −0.7°, i.e. horizontal). Rear floor intake =
  **yaw the holonomic base 180°** (~0.3 s); the stow pose still works the rear
  directly (mouth rearward at ~200 mm: launch / hold / feed). Frees a pneumatic
  circuit, stiffens the arm (red-team C-M4), and front floor intake is reached
  by the pivot alone (mouth at 80 mm = ball centre height at φ=−25°).
- Gates: head-wheel clearance 0.0 → **13.3 mm**; stow-in-cube (NEW gate G8 — the
  old stow pose silently poked the flywheel out of the cube): worst margin **21 mm**.

**Carve-outs / still-open (documented, not hidden):**
1. Bench items unchanged (§15.9): tip-orientation capture at the throat, and
   launch scatter — cannot be closed on paper.
2. TPU belt loop seam durability at speed — bench test.
3. Lip flare overhang needs supports or a split print.
4. G7 (13.3 mm) and G8 (21 mm) margins are adequate but not generous — re-verify
   when real arm-link/carriage CAD replaces the layout primitives.
5. Front-deploy tipping ≈1.5–2× static margin by estimate — verify with CAD
   masses; keep battery/reservoir low and rear.
6. `HEAD_ALONG = 333 mm` includes a ±20 mm flywheel-protrusion estimate — pin it
   down when the launcher hood CAD is integrated into the head assembly.

## 12. Change log
- **v1.2** — real arm-link CAD: `arm_hub` ×2 (hex-keyed web + boss, bolts to both
  decks) and `pivot_block` ×2 (chassis-side bearings), new gate **G9** (6 checks).
  G9 caught a phantom half-pitch bolt column → deck widened so the REAL 114.3 mm
  grid column exists; motor plates slimmed to ±20; pivot z 221→223 for sweep
  clearance. Full suite **41 pass / 0 FAIL** (was 39/2 mid-loop).
- **v1.1** — gate-driven refinement (§16): 8 automated gates, baseline 15/24-fail
  → final 29/0. Lip rebuilt, plow=ramp (bottom lip deleted), rigid-ball squeeze
  corrected, pulley/flange/motor-plate fixes, pivot re-solved, slide removed
  (rear-floor proof), stow-in-cube now gated.

## 12. Change log
- **v1.0** — §15 resolves the post-critique design. Launch → dedicated **flywheel** fed
  by the belt (belt keeps intake/hold/push). Fold fixed to a **center pivot + slide**
  (topology closes; exact link lengths = a CAD swept-volume study). Head corrected
  (≈190×210×182 mm, ~2 kg, metal spine). 7 motors (belt cross-linked, pivot on 100 RPM),
  ≤2 VEX reservoirs, tracking-wheel odometry, hang → 24″ partner. Two bench-prototype
  items (§15.9) flagged as the only things that can't close on paper. Ready for CAD after
  those two are de-risked.
- **v0.4** — two adversarial red-teams (§14) retracted §13's "all closed." Belt is a
  poor precision launcher for the rigid tri-ball (no energy store, orientation scatter);
  fold kinematics don't close as written; head under-sized. Rules re-cited to the VEX U
  appendix (15″→36″ expansion, VEX pneumatics ≤2 reservoirs, TPU legal, VEX gears, two-
  robot game). Blocking decision = launch architecture (to user); rest is a numbers/geometry
  redo → then v1.0 → then CAD.
- **v0.3** — §13 resolves all eleven holes with concrete numbers: compact head
  (≈180×165×150 mm), 3.25″ omni X-drive @600 RPM, belts geared 3.75:1 → ~5.9 m/s
  exit (covers 4–8 ft), motor-driven ~180° fold + pneumatic slide/latch, recoil
  ~14 N (base holds), tipping ~2.4× margin, 7 motors, routing over the pivot,
  hang deferred but provisioned. Next: adversarial critique pass, then CAD.
- **v0.2** — integrated VEXU rules research: §2.1 grounded (15″ robot, no motor cap
  → hold ≤8, COTS pneumatics 100 psi, printed/fabricated parts legal but non-VEX
  COTS mechanicals not → TPU belts; field/scoring targets). Corrected the tri-ball
  to ~6.18″ three-lobed / 138 g (H10), which eases packaging (H1). Motor plan
  fixed at 7. Next: work the §11 holes, starting **H1/H10 (size)** and **H8
  (launch numbers)**, iterate to no holes, *then* CAD.
- v0.1 — foundation (rules-independent architecture, kinematics, interactions,
  fastening, BOM skeleton).
