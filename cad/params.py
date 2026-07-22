"""
Parameters for the VEX tri-ball belt accelerator (millimetres).

Change values here and re-run `python -m cad.export_all` to regenerate every
STEP/STL/preview. Everything is parametric, so the mechanism adapts to your
hardware and game element -- especially TRIBALL_DIA and the belt/pulley sizing.
"""

# --- Fasteners -------------------------------------------------------------
SCREW_M3_TAP = 2.8        # mm, M3 self-tap / grub-screw hole

# ==========================================================================
# Belt accelerator ("belt railgun")
# ==========================================================================
# The tri-ball runs down a barrel gripped between two motor-driven conveyor
# belts on its LEFT and RIGHT (the "rails"), with top + bottom DECKS containing
# it and forming the floor it rides on. Like a railgun it accelerates the ball
# ALONG the barrel -- but the rails are traction belts, and belt SURFACE SPEED
# sets the exit speed. One mechanism, four behaviours:
#   PULL  -- belts run inward, slow:  draw the ball into the barrel.
#   HOLD  -- belts stopped:           ball pinched between them (compression grip).
#   LAUNCH-- belts run outward, fast: ball spun up to belt speed, fired out the muzzle.
#   PUSH  -- belts off:               drive forward, the front plow shoves the ball.
# It targets the VEX system: 1/2" high-strength HEX shafts, the 0.5" (12.7 mm)
# hole grid, and #8-32 hardware, so it bolts to VEX C-channel and V5 motors.
#
# DESIGN BASIS (first-principles sizing -- edit these and re-export):
#   Exit speed (no slip):   v_exit ~= v_belt = (RPM / 60) * pi * PULLEY_PITCH_DIA
#   Grip / no-slip:         mu * N  >=  m_ball * a     (N grows with BALL_COMPRESSION)
#   Channel gap:            BELT_GAP = TRIBALL_DIA - 2 * BALL_COMPRESSION
#   Barrel length:          BARREL_LEN = accel distance for the ball to reach v_belt
# The side belts squeeze the ball (BELT_GAP < TRIBALL_DIA) for grip; the decks
# sit a hair wider than it (2 * SIDE_INNER_HALF > TRIBALL_DIA) so it can't pop
# out top or bottom.

# --- VEX hardware ----------------------------------------------------------
VEX_GRID = 12.7           # mm, 0.5" structure hole-grid pitch
VEX_HOLE = 4.9            # mm, structure hole (0.181" clearance for a #8-32 screw)
VEX_HEX_AF = 12.70        # mm, 1/2" high-strength hex shaft, across-flats
VEX_HEX_CLEAR = 0.35      # mm, added to a hex bore across-flats (slip fit on shaft)
VEX_SHAFT_CLEAR = 16.0    # mm, round hole for a hex shaft to pass / a bearing to seat
GEAR_CD = 3 * VEX_GRID    # mm, motor<->pulley gear centre distance (VEX-legal, 1.5")

# --- Tri-ball game element (VEX Over Under) --------------------------------
# CORRECTED to the real element (was a 178 mm sphere placeholder): the tri-ball is
# a rigid, hollow, 3-lobed shell ~6.18" tip-to-tip with a ~4.3" body. The belts
# grip the BODY; the throat/decks must clear the TIP. See docs/DESIGN.md §13/§15.
TRIBALL_TIP = 157.0       # mm, tip-to-tip (6.18") -> sets throat + deck clearance
TRIBALL_BODY = 110.0      # mm, body across (~4.3") -> sets the belt grip
TRIBALL_DIA = TRIBALL_TIP  # back-compat alias (renderers use TRIBALL_DIA/2)

# --- Belt INTAKE head geometry (robot role: intake / hold / push; launch = flywheel)
PLATE_THK = 6.0           # mm, deck (side-plate) thickness
# Rigid-ball constraint: only the TPU belt can deform, so the squeeze must fit
# inside the belt: BALL_COMPRESSION <= BELT_THK - 1, which also guarantees the
# rigid drum behind the belt clears the ball body by >= 2 mm per side.
BALL_COMPRESSION = 4.0    # mm, TPU belt deform onto the RIGID body (belt yields, not ball)
BELT_GAP = TRIBALL_BODY - 2 * BALL_COMPRESSION  # mm, inner-run separation (102)
BELT_WIDTH = 120.0        # mm, belt width >= body so it grips the whole 110 mm band
BELT_THK = 6.0            # mm, TPU belt thickness incl. tread (> compression + 1)
BARREL_LEN = 110.0        # mm, pulley centre-to-centre (compact intake engagement)
PULLEY_PITCH_DIA = 45.0   # mm, traction diameter the belt rides on
PULLEY_FLANGE_DIA = 55.0  # mm, belt-retaining flange OD
PULLEY_SPOKES = 5         # lightening windows in the pulley
# Decks sit wider than the TIP so the 3-lobe ball can't pop out as it tumbles in.
SIDE_INNER_HALF = 85.0    # mm, half the deck-to-deck gap (2*85 = 170 > 157 tip)
PLOW_DEG = 20.0           # deg, front plow-blade rake (down-and-forward)
