# Bill of Materials — VEXU Over Under 15" scorer

Grouped by subsystem. **Purchased parts are VEX products, pneumatics, or fasteners
only** (VEXU rule VUR — non-VEX COTS mechanicals are illegal); **custom parts are
3D-printed / fabricated** (legal, unlimited). VEX SKUs are given by product name —
**confirm the exact part number in the current VEX catalog before ordering** (SKUs
drift between catalog revisions).

Motor budget: **8 of 8** V5 Smart Motors (at the ≤8 full-current limit exactly).
Pneumatics: **DELETED** (nothing left to actuate — R3 F15); the fold latch is a
passive mechanical over-center (tracked part).

## S1 — Chassis / structure  (purchased: VEX; some custom plate)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| Aluminum C-channel (1×2×1×25-hole) | ~4 | square frame perimeter | VEX |
| Aluminum plate / gusset stock | a few | corner gussets, decks | VEX |
| Standoffs + couplers | ~8 | separate the drive & arm decks | VEX |
| 8-32 screws (0.375–0.75") + nylock nuts | ~120 | all metal joints on the 0.5" grid | VEX / fastener |
| Heat-set brass inserts (8-32) | ~20 | hub-tab deck joints + service points | fastener |
| `platen_rail` (PETG, 2nd MIRRORED) | 2 | rigid backing behind each belt inner run — grip force reacts into structure (R3 D6) | **printed** |
| `counterweight` steel 90×50×20 (~700 g) | 1 | rear ballast (STEP provided) | **fabricated** |

## S2 — X-drive  (purchased: VEX)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| 3.25" Omni-Directional Wheel | 4 | holonomic drive @ 45° corners | VEX |
| V5 Smart Motor (11 W) | 4 | drive | VEX |
| ½" high-strength shaft | 4 | wheel axles | VEX |
| High-strength bearing flat | 8 | axle supports | VEX |
| Shaft collar | 8 | axle retention | VEX |
| High-strength spur gears **36T:60T** | 8 | 360 RPM wheel — R3 F1 re-gear (was an effective 12 ft/s / 24 N push) | VEX |

## S3 — Belt intake head  (purchased: VEX shafts/gears/bearings; custom: printed)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| `belt_pulley` (metal-hub or heat-set) | 4 | belts ride on these | **printed** + VEX hub |
| `drive_belt` **laced TPU strips** (2 per belt) | 4 | printable flat, laced shut in place with a 1.75 mm filament pin — 5-min belt swaps, no seam through the nip (R3 B2/C1) | **printed (TPU)** |
| Lace pins (1.75 mm filament / music wire) | 4 | close the belt loops; length sets tension (bores now all FIXED) | fastener |
| `accel_plate` deck (PETG) | 2 | pulley mounts / ball channel | **printed** |
| `throat_lip` (PETG) | 1 | flared TOP mouth guide, bolts to the deck grid | **printed** |
| `front_plow` (PETG) | 1 | bottom ramp + push blade + bottom mouth guide | **printed** |
| Muzzle standoffs (deck-to-deck) | 4 | close the launch-nip pry path at the mouth corners (R3 A3) | VEX |
| Flat steel strips along deck outer faces | 2 | the "metal spine" — belt-tension load path (hardware, not modeled) | VEX |
| V5 Smart Motor (600 cart, 1.5 A cap) | 2 | one per belt — no cross-link path existed (R3 A2); jam auto-reverse in firmware | VEX |
| ½" hex shaft + bearing flats + shaft collars | 4 sets | pulley shafts (bearing-flat hole pairs now modeled at every bore — R3 C4; collars replace the deleted set screws — R3 D1) | VEX |

## S4 — Fold arm  (purchased: VEX + pneumatics; custom: brackets)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| ½" high-strength hex pivot shaft | 1 | shoulder axis (hubs keyed to it) | VEX |
| `arm_hub` (PETG, print 2nd MIRRORED) | 2 | keys the head to the pivot shaft; bolts to both decks on the grid | **printed** |
| `pivot_block` (PETG) | 2 | chassis-side bearing blocks on the towers (dual shear, VEX bearing flats) | **printed** |
| V5 Smart Motor + gearbox (~7:1, **100 RPM cart.**) | 1 | fold actuation — re-sized for the REAL 4.1 kg head: hold ≈7 N·m of 14.7 capacity; hard-stop carries the deployed pose | VEX |
| Shaft collars (pivot shaft) | 4 | axial retention of hubs (set screws deleted — R3 D1) | VEX |
| V5 Rotation Sensor (pivot shaft) | 1 | direct angle: motor-encoder-through-7:1 backlash is ±2–3° at the head | VEX |
| `stow_cradle` (PETG) | 2 | passive powered-off stow hold — head rests in foam-padded V-tops (gate G15) | **printed** |
| Hard-stop **pin + horn** | — | integrated INTO arm_hub + pivot_block: deployed pose is a mechanical seat, motor at ~0 A (gate G15) | — |

## S5 — (deleted) Pneumatics
Removed in v1.4 (R3 F15): the slide is gone and the fold latch went passive, so
nothing pneumatic actuates anything. The 700 g steel counterweight (rear, low)
replaces the reservoir's counterbalance role — fabricate from plate stock.

## S6 — Power / electronics / sensing  (purchased: VEX)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| V5 Robot Brain | 1 | controller (1 per robot) | VEX |
| V5 Robot Battery | 1 | power | VEX |
| V5 Controller + Radio | 1 | driver comms | VEX |
| V5 Inertial Sensor | 1 | field-oriented heading | VEX |
| V5 Distance / Optical Sensor | 1 | ball-present → auto-HOLD | VEX |
| `tracking_pod` (PETG) + 2.75" omni + rotation sensor | 2 | dead-wheel odometry forks (auton aiming) | **printed** + VEX |
| `sensor_mount` (PETG) | 1 | aims the distance sensor into the channel | **printed** |
| V5 Smart Cables | ~10 | motor/sensor wiring | VEX |

## S7 — Launcher  (purchased: VEX; custom: hood)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| 3" flex wheel (**1" wide**, soft) + ½" hex shaft | 1 | launcher wheel — its compliance IS the nip give | VEX |
| `mass_disc` steel **Ø72 × 16 mm, ~510 g** | 1 | flywheel store beside the wheel INSIDE the deck notch, 2 mm below the wheel surface (gate G15) | **fabricated** (STEP provided) |
| `fly_mount` (PETG, print 2nd MIRRORED) | 2 | hangs the wheel under the mouth; VEX bearing flats | **printed** |
| V5 Smart Motor (600 cart) | 1 | flywheel drive on `fly_drive_plate` | VEX |
| `fly_drive_plate` (PETG) | 1 | motor mount + 12T:60T mesh at exact GEAR_CD, under the bottom deck | **printed** |
| High-strength gears 12T:60T | 2 | 600 cart → ~2700 RPM plateau at the wheel (3000 is the no-load asymptote — R3 F3) | VEX |
| *(hood)* — the `throat_lip` TONGUE is the hood | — | ball squeezed wheel↔tongue; no extra part, no extra DOF | — |

---

**Launcher is reversible**: spinning inward it is an intake-assist roller;
stopped it retains; reversed at speed it fires — **front-side only** (R3-6: at
STOW the head inverts and the nip fires downward; rear work = HOLD/FEED, all
launches happen at the LAUNCH/FRONT poses after a ~0.3 s yaw).

### Motor tally (8 of 8)
4 drive · 2 belt intake · 1 flywheel · 1 arm pivot → **8** (the spare went to
the second belt motor when the cross-link proved to have no path — R3 A2).
Firmware caps per gate G13: drive 4×2.0 A, belts 2×1.5 A, flywheel 2.5 A
(interlocked vs full-throttle drive), pivot 1.0 A → 15.5 A worst case.

### Team note
This is the **15" scorer**; the team's **24" robot handles elevation** (endgame
20/15/10/5). See `docs/DESIGN.md` §15 for the full design rationale.
