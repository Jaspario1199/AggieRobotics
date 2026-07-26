# Build Guide — VEXU Over Under 15" Scorer

Every custom part has STEP (`cad/step/`) + STL (`cad/stl/`); print counts and
materials are in `docs/BOM.md`. Verify any change with `python -m cad.gates`
(77 checks) before printing. Tools: 3/32" + 5/64" hex drivers, 11/32" nut
driver/wrench, soldering iron (heat-set inserts), lighter (TPU lace ends).

## Print / fabricate first
| Part | Qty | Material / notes |
|------|----:|------------------|
| accel_plate | 2 | PETG 40%; print rotated ~40° on the bed, raft; check flatness < 0.5 mm |
| belt_pulley | 4 | PETG; bore-up |
| drive_belt strips | 4 (2/belt) + spares | TPU 95A flat; tread up |
| throat_lip | 1 | PETG; plate-down (ribs are the supports) |
| front_plow | 1 | PETG 50%; blade-edge-up with supports (impact part — consider a spare) |
| arm_hub | 2 (2nd MIRRORED) | PETG 50%, 6 walls; web-flat, supports under boss+tabs; metal fallback if tabs flex |
| pivot_block, fly_mount (2nd mirrored), fly_drive_plate, platen_rail (2nd mirrored), sensor_mount, stow_cradle ×2, tracking_pod ×2 | — | PETG; base-down |
| mass_disc Ø72×16 (~510 g), counterweight 90×50×20 (~700 g) | 1 + 1 | FABRICATED steel; deburr, spin-balance the disc |

## Head assembly (on the bench, upside down then flipped)
1. **Bottom deck**: bolt bearing flats over all 4 pulley bores (the ±12.7 mm
   hole pairs); bolt the **plow** plate to the inner face (±63.5 columns,
   countersunk heads) and the two **fly_mounts** + **fly_drive_plate** to the
   outer face (bearing flats on their bores).
2. **Flywheel shaft**: slide in ½" hex carrying **3" flex wheel (1" wide) +
   mass disc** (disc on the +x side, 2 mm clear of everything — gate-verified);
   collars outboard. Add the 60T gear on the motor, 12T on the shaft; bolt the
   flywheel **V5 motor (600)** to the drive plate pilot.
3. **Pulleys + belts**: stand the 4 crowned pulleys on their hex shafts in the
   bottom-deck bores; collar below. Wrap each **belt** (2 laced strips) around
   its pulley pair, lace shut with a filament pin, melt-mushroom the ends.
   **Platen rails** bolt between the decks behind each belt's inner run.
4. **Top deck** on: shafts through the bores (bearing flats outside), 4 muzzle
   **standoffs** at the ±114.3 columns, **throat_lip** on top at the muzzle
   (tongue centred over the wheel), **motor plates + 2 belt motors** (600,
   36T:36T or direct) at the rear, **sensor_mount** behind the lip.
5. **Arm hubs**: bolt tabs to both decks at the ±114.3 rear columns (heat-set
   inserts in the decks recommended); hex-key both hubs to the **pivot shaft**;
   collars at the bosses. Zip the 4 head cables to the hub **cable boss**, leave
   a 160 mm service loop across the pivot.

## Chassis + integration
6. **X-drive**: VEX C-channel square frame (360 mm), 4 corner modules — 3.25"
   omni, 36T:60T gear-down, V5 motor (600) each; wheels at 45°.
7. **Towers**: two C-channel towers at y=155, capped by the **pivot_blocks**
   (bore at z=219 exactly — gate G15); slide the head's pivot shaft through the
   block bearings; collars. The hub **pin** must land on the block **horn** at
   the horizontal (LAUNCH) pose — that's the hard stop.
8. **Pivot motor** (100 RPM cart, ~7:1 to the shaft) on a tower + **rotation
   sensor** on the shaft's free end.
9. **Deck furniture**: battery + brain at y=-120, steel **counterweight** at the
   rear, **stow_cradles** at (±60, -120) with adhesive foam pads — the folded
   head rests on them powered-off (inspection). **Tracking pods** under the
   frame (one X, one Y), rubber-banded down. Radio high on a tower, clear of
   metal.

## Firmware must-haves (gate G13/G14 assumptions)
Current caps: drive 4×2.0 A, belts 2×1.5 A, flywheel 2.5 A, pivot 1.0 A.
Belt jam auto-reverse (250 ms stall → 300 ms reverse). Flywheel recovery
inhibited during >50% drive. Brake-slew limit when the arm is below 30°.
Launch sequence: yaw-align → pivot to hard-stop → spin to commanded RPM →
belts feed. Rear work at STOW is HOLD/FEED only.

## Bench checklist before first match (the paper-unprovables)
Throat capture rate with a real tri-ball (all orientations); launch k-factor
(commanded RPM vs measured exit speed → recalibrate G11); TPU lace fatigue
(500 cycles); pivot backlash at the head; sizing cube WITH cables zip-tied.
