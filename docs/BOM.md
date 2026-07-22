# Bill of Materials — VEXU Over Under 15" scorer

Grouped by subsystem. **Purchased parts are VEX products, pneumatics, or fasteners
only** (VEXU rule VUR — non-VEX COTS mechanicals are illegal); **custom parts are
3D-printed / fabricated** (legal, unlimited). VEX SKUs are given by product name —
**confirm the exact part number in the current VEX catalog before ordering** (SKUs
drift between catalog revisions).

Motor budget: **7 of 8** V5 Smart Motors (staying ≤8 keeps every motor at full
2.5 A current — VEXos throttles past 8). Pneumatics: **≤2 VEX reservoirs, 100 psi**.

## S1 — Chassis / structure  (purchased: VEX; some custom plate)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| Aluminum C-channel (1×2×1×25-hole) | ~4 | square frame perimeter | VEX |
| Aluminum plate / gusset stock | a few | corner gussets, decks | VEX |
| Standoffs + couplers | ~8 | separate the drive & arm decks | VEX |
| 8-32 screws (0.375–0.75") + nylock nuts | ~120 | all metal joints on the 0.5" grid | VEX / fastener |
| Heat-set brass inserts (8-32) | ~20 | screws that thread into printed parts | fastener |

## S2 — X-drive  (purchased: VEX)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| 3.25" Omni-Directional Wheel | 4 | holonomic drive @ 45° corners | VEX |
| V5 Smart Motor (11 W) | 4 | drive | VEX |
| ½" high-strength shaft | 4 | wheel axles | VEX |
| High-strength bearing flat | 8 | axle supports | VEX |
| Shaft collar | 8 | axle retention | VEX |
| High-strength spur gears (drive ratio) | 8 | ~1:1 speed | VEX |

## S3 — Belt intake head  (purchased: VEX shafts/gears/bearings; custom: printed)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| `belt_pulley` (metal-hub or heat-set) | 4 | belts ride on these | **printed** + VEX hub |
| `drive_belt` (**TPU**, 6 mm) | 2 | grip the ~4.3" ball body (squeeze lives in the belt) | **printed (TPU)** |
| `accel_plate` deck (PETG) | 2 | pulley mounts / ball channel | **printed** |
| `throat_lip` (PETG) | 1 | flared TOP mouth guide, bolts to the deck grid | **printed** |
| `front_plow` (PETG) | 1 | bottom ramp + push blade + bottom mouth guide | **printed** |
| **Metal spine** (C-channel/plate) | 1 | primary load path behind the printed shells | VEX |
| V5 Smart Motor | 1 | belt intake (L/R cross-linked) | VEX |
| ½" hex shaft + bearings + collars | 4 sets | pulley shafts | VEX |

## S4 — Fold arm  (purchased: VEX + pneumatics; custom: brackets)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| Pivot shaft (½" HS or ¾") + bearing blocks | 1 set | shoulder pivot (dual-shear) | VEX |
| V5 Smart Motor + gearbox (~7:1, **100 RPM cart.**) | 1 | fold actuation, holds ~5.6 N·m | VEX |
| Over-center latch (passive) | 1 | holds folded pose powered-off (inspection) | printed/VEX |
| Deployed hard-stop | 1 | fixes launch angle, off motor hold | printed/VEX |

## S5 — Pneumatics  (purchased: VEX pneumatics)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| Air reservoir | 1 (≤2) | supply | VEX |
| Solenoid valve (single/double-acting) | 2 | slide + latch | VEX |
| Regulator + tubing + fittings | 1 set | 100 psi max plumbing | VEX |

## S6 — Power / electronics / sensing  (purchased: VEX)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| V5 Robot Brain | 1 | controller (1 per robot) | VEX |
| V5 Robot Battery | 1 | power | VEX |
| V5 Controller + Radio | 1 | driver comms | VEX |
| V5 Inertial Sensor | 1 | field-oriented heading | VEX |
| V5 Distance / Optical Sensor | 1 | ball-present → auto-HOLD | VEX |
| Tracking omni + rotation/encoder | 2–3 | dead-wheel odometry (auton aiming) | VEX |
| V5 Smart Cables | ~10 | motor/sensor wiring | VEX |

## S7 — Launcher  (purchased: VEX; custom: hood)
| Item | Qty | Purpose | Source |
|------|----:|---------|--------|
| Flywheel (metal-mass) + ½" hex | 1 | energy store for a repeatable shot | VEX / fabricated |
| V5 Smart Motor (geared up) | 1 | spin the flywheel | VEX |
| `compression_hood` (PETG) | 1 | holds ball on the wheel, sets ~42° | **printed** |
| High-strength gears (up-ratio) | 2 | flywheel speed | VEX |

---

### Motor tally (7)
4 drive · 1 belt intake · 1 flywheel · 1 arm pivot  → **7** (1 spare port reserved
for a 2nd flywheel or an endgame hook). The fold latch is **pneumatic**; the
telescoping slide was REMOVED in the gate-driven refinement (see DESIGN.md §16).

### Team note
This is the **15" scorer**; the team's **24" robot handles elevation** (endgame
20/15/10/5). See `docs/DESIGN.md` §15 for the full design rationale.
