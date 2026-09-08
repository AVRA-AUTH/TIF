// ================= CONFIG: all tunable constants, no logic =================
// Single source of truth for every number this app tunes against — world
// scale, boat physics, thruster hardware, start poses, world geometry
// (island/marina/jetty/lake), and guidance gains. Split out of the original
// app.js (where these were scattered across ~1400 lines, each declared right
// before its first use) so every tunable lives in one place. Loaded FIRST
// (see index.html) — everything else in this app depends on these values,
// and several (island/marina meshes, boat spawn poses) use them immediately
// at their own top-level, not just inside functions.

// ================= WORLD LAYOUT SCALE ================= //
// Applied to every map POSITION (island, marina/piers/jetties/berths, boat
// spawn, pathfinding grid bounds, coastal city, LAKE_RADIUS) at the user's
// explicit request to shrink the water body — overriding earlier caution
// (see HANDOFF.md) about compressing the tightly-packed marina. Lowered
// again from the earlier 0.5 to 0.4 per a later explicit request to cut
// travel time further: every position below scales with this constant, so
// the whole layout (spawn -> marina -> island) shrinks together and stays
// internally consistent.
// Per explicit user choice, SIZES (pier length/width, jetty dimensions,
// buoy radius, boat hull, etc.) are NOT scaled, only positions — the piers/
// jetties are now more tightly packed and may visually overlap; that
// tradeoff was accepted knowingly, not an oversight.
const WORLD_SCALE = 0.4;

// ================= MODE 1 (LEVEL DESIGNER) TUNABLES ================= //
// Caps on player-placed entities — keeps a public exhibit session from
// silently degrading (the A* pathfinder grid re-runs every 250ms during
// live nav and gets slower the more obstacles it has to route around) or
// packing the small lake so densely there's no clear water left to design
// with. Enforced once, in input.js's placeMode1EntityAt(), so it applies
// identically whether an entity was placed by mouse click or a gamepad
// button. Deliberately excludes the ~30 baked-in moored boats at the marina
// (isParkedShip) and the single goal marker — those aren't player-placed
// obstacles this cap is about.
const MAX_BUOYS = 7;
const MAX_MOVING_BOATS = 4;

// ================= MODE 2 (JOYSTICK DRIVE) TUNABLES ================= //
// FWD/REV updated from real hardware: a single Blue Robotics T200 per side
// at this project's real ~16V operating voltage (AVRA/Matlab/ASV_BMS/
// init_Pwm.m + Init_BMS.m) gives 51.5N/-40.2N per thruster — see
// exhibition_water.sdf's max_thrust_cmd/min_thrust_cmd derivation comment.
// Solving this boat's own drag model for that combined force gives these
// equilibrium speeds (was 6.0/-1.5, both back-derived software targets with
// no real hardware basis).
const MAX_LINEAR_FWD = 2.49;     // top forward speed (m/s) — real T200 @16V
const MAX_LINEAR_REV = -2.19;    // top reverse speed (m/s) — real T200 @16V

// Hull surge-drag coefficients MAX_LINEAR_FWD/REV above were solved from
// (xU*v + xUU*v^2 = thrust) — mirrors exhibition_water.sdf's <xU>/<xUU>
// exactly. Re-exposed here so dof-panel.js's live thrust readout can run the
// same equation in reverse (thrust from measured speed) instead of a second,
// driftable copy of the number.
const HULL_DRAG_LINEAR = 1.05;     // xU
const HULL_DRAG_QUADRATIC = 16.24; // xUU

// T200-at-16V current draw at full commanded thrust, from the same
// datasheet sweep (voltages=[10,12,14,16], 16V row) MAX_LINEAR_FWD/REV were
// sourced from — full-throttle rows only (1900us/1100us PWM): 23.83A @ full
// forward, 24.30A @ full reverse. Used by dof-panel.js's live thrust readout
// to estimate current/power at partial throttle via linear interpolation
// from 0A — NOT the datasheet's own curve, which is markedly non-linear and
// noisy near the neutral dead-band (efficiency readings there spike past
// 100 g/W, an artifact of near-zero current, not a real operating point).
// A straight two-point interpolation between idle and full-throttle is a
// deliberate simplification: close enough for a ballpark current/power
// readout, but NOT accurate enough to also derive an efficiency/"% battery
// wasted" figure from — that would need the real mid-throttle curve, which
// this reference doesn't reliably give us. So the readout stops at current/
// power and does not claim an efficiency number.
const T200_MAX_CURRENT_FWD_A = 23.83; // @ full fwd PWM, 16V
const T200_MAX_CURRENT_REV_A = 24.30; // @ full rev PWM, 16V
const BATTERY_VOLTAGE_V = 16.0;
const MAX_ANGULAR = 1.2;         // top turn rate (rad/s)
const THRUST_RAMP_RATE = 6.0;    // units/sec: how fast velocity reaches its target while a thruster is held open
const WATER_FRICTION_RATE = 1.0; // units/sec: how fast velocity decays toward zero once released

// Differential-thrust -> yaw-rate model for Mode 2's manual twin-thruster
// local simulation (see input.js's twin-thruster branch) — lets W/A/R/D and
// the gamepad's Twin Thruster stick mode derive an equivalent targetAngular
// the same way the combined-drive scheme already has one, instead of relying
// on real Gazebo torque/damping to shape the turn (see state.js's
// localSimEnabled for why that reliance can be a problem). Values mirror the
// real boat's own documented yaw dynamics elsewhere in this codebase: a
// 0.38m moment arm between the two thrusters' lines of action, and nR=12
// (nRR=0, i.e. linear-only) yaw damping — e.g. max differential thrust
// 51.5-(-40.2)=91.7N gives 0.38*91.7/12 ≈ 2.9 rad/s at full opposite-thrust,
// matching the real physics ceiling this was cross-checked against.
const HULL_YAW_MOMENT_ARM = 0.38; // meters
const HULL_YAW_DAMPING = 12.0;    // nR

// ================= MODE 2 "BUOY RUN" CHALLENGE COURSE ================= //
// A fixed slalom course laid out east of the island (MODE2_START = (38, 0)),
// well clear of both the island's keep-out ring (33m) and the marina (which
// sits far off in negative-x/negative-y territory) — see mode2-game.js for
// the win/fail logic that reads these. Kept as flat ROS-frame coordinates
// (not WORLD_SCALE-relative like the marina/island) since this course has no
// pre-scale legacy reference to stay proportional to; it was laid out
// directly against the actual (post-scale) lake, which is what matters for
// "does this course fit."
const MODE2_TIME_LIMIT_MS = 100000; // 100s countdown once START CHALLENGE is pressed — a bit more room than the original 90s to match the denser 7-buoy/7-boat course below
const MODE2_GOAL = { x: 118.0, y: 0.0 };
const MODE2_GOAL_RADIUS = 5.0; // finish-line tolerance (m)
// Static hazard buoys, alternating above/below the direct start->goal line
// every 10m so reaching the goal requires an actual weave, not a straight
// cruise. 7 buoys, evenly spaced x:[46,106].
const MODE2_COURSE_BUOYS = [
    { x: 46, y: 10 }, { x: 56, y: -10 }, { x: 66, y: 10 }, { x: 76, y: -10 },
    { x: 86, y: 10 }, { x: 96, y: -10 }, { x: 106, y: 10 }
];
// Patrol-boat spawn points — reuses the existing dynamic-entity diamond
// patrol (updateDynamicEntities(), navigation.js), so each roams a 24m
// (Manhattan-radius) diamond loop around its spawn, crossing the channel
// unpredictably. 4 boats, spread evenly across the buoy field and alternating
// speed tier (medium/fast/medium/fast) so both are encountered throughout the
// run, not clustered in one section. Chosen (via a small offline search — see
// the min-distance-to-diamond-edge formula in that script) so no patrol
// diamond ever passes within the same function's own 2.5m buoy-crash radius
// of any MODE2_COURSE_BUOYS entry — otherwise a patrol boat could silently
// sink into (and permanently remove) a course buoy mid-run, making the course
// inconsistent between attempts.
// Real m/s now (navigation.js's updateDynamicEntities() moves patrol boats
// on real elapsed time, same as everything else) — calibrated directly
// against the player's own real top speed (MAX_LINEAR_FWD = 2.49 m/s), not
// arbitrary numbers. The original values here (1.0/1.7) were only ever
// meaningful under the OLD framerate-dependent movement (implicitly ~3x at a
// 60fps baseline — see navigation.js's dt-based rewrite) and worked out to
// ~3.0/~5.1 m/s: already faster than the player's real max at "medium," and
// over double it at "fast" — boats that outran the player outright and felt
// instant next to the player's own real, ramped/dragged physics, not just
// quick. These replacements are deliberately UNDER MAX_LINEAR_FWD so a
// player at full throttle can still out-position either tier.
const MODE2_PATROL_SPEED_MEDIUM = 1.4;  // ~56% of MAX_LINEAR_FWD
const MODE2_PATROL_SPEED_FAST = 2.3;    // ~92% of MAX_LINEAR_FWD — noticeably quicker, still bounded by this boat's own slow heavy-ship turn rate (navigation.js) so it stays dodgeable
const MODE2_COURSE_BOATS = [
    { x: 46, y: -6, speed: MODE2_PATROL_SPEED_MEDIUM },
    { x: 66, y: -6, speed: MODE2_PATROL_SPEED_FAST },
    { x: 86, y: -6, speed: MODE2_PATROL_SPEED_MEDIUM },
    { x: 106, y: -6, speed: MODE2_PATROL_SPEED_FAST }
];
// Danger zones (new concept, no Mode 1 equivalent — see mode2-game.js):
// sit further off-line than the buoys so a wide swing to dodge one risks
// clipping the other instead of being a free, no-cost detour.
const MODE2_DANGER_ZONES = [
    { x: 70, y: 26, radius: 9 }, { x: 96, y: -27, radius: 9 }
];
// Numbered intermediate checkpoints, must be reached IN ORDER before the
// goal counts as a finish (mode2-game.js's checkMode2GameState()) — sat at
// the buoy-gap midpoints between two consecutive weave buoys (so they're
// already on the natural line), but #2 and #3 are also deliberately nudged
// toward a danger zone's own edge (just outside it) so hitting them requires
// genuinely hugging the hazard, not just threading the buoys.
const MODE2_CHECKPOINT_RADIUS = 4.0;
const MODE2_CHECKPOINTS = [
    { number: 1, x: 51, y: 0 },   // gap between buoys 1 & 2
    { number: 2, x: 71, y: 16 },  // gap between buoys 3 & 4, ~1m outside danger zone 1's edge
    { number: 3, x: 92, y: -17 }, // gap between buoys 5 & 6, ~2m outside danger zone 2's edge
];
// Course boundary — leaving this rectangle instantly fails the run ("left
// the safe channel"), same idea as a real river/regatta course marked by
// channel buoys. Generous margin around the buoys/danger zones/patrol loops
// above (all comfortably within x:[38,120], y:[-28,28]) so a wide dodge
// never accidentally fails the run on its own; xMin stays clear of the
// island's own 33m keep-out (already enforced separately by the boat's
// normal collision handling) and xMax/yMin/yMax stay inside LAKE_RADIUS (140).
const MODE2_COURSE_BOUNDS = { xMin: 25, xMax: 132, yMin: -40, yMax: 40 };
// 2D tactical map zoom for Mode 2 (render-2d.js's getCurrentViewParams()),
// same idea as Mode 3's marina zoom — centered on MODE2_COURSE_BOUNDS so the
// whole course fills the map canvas instead of being a small strip inside
// the full 140m-radius lake view Mode 1 uses.
const MODE2_MAP_VIEW = { scale: 4.0, offsetX: (MODE2_COURSE_BOUNDS.xMin + MODE2_COURSE_BOUNDS.xMax) / 2, offsetY: 0.0 };

// Mode 1/3 docking-leg speeds, expressed as ratios of MAX_LINEAR_FWD instead
// of flat numbers, so they scale automatically if the top speed is ever
// retuned again. Ratios preserve the original tuning (1.6/-1.3/1.3 m/s and
// a 0.3 m/s^2 brake) against the OLD MAX_LINEAR_FWD=6.0 this boat used
// before the real-T200 retune.
const DOCK_APPROACH_SPEED_RATIO = 1.6 / 6.0;        // ~0.267
const DOCK_REVERSE_SWING_SPEED_RATIO = -1.3 / 6.0;  // ~-0.217
const DOCK_CREEP_SPEED_RATIO = 1.3 / 6.0;           // ~0.217
const DOCK_BRAKE_DECEL = 0.3 * (MAX_LINEAR_FWD / 6.0); // ~0.1245 m/s^2

// Hard speed ceiling for the whole of Mode 3 (autonomous docking) — user-
// requested, independent of MAX_LINEAR_FWD/REV. Only the 'transit' leg
// (which otherwise cruises at the full MAX_LINEAR_FWD) is actually affected
// today; approach/creep/reverse_swing are already well under this.
const DOCK_MAX_SPEED = 1.03;

// Stuck-recovery maneuver duration (navigation.js's driveRecovery()) — a
// committed reverse-and-turn held for this long (real seconds, not frames),
// not a single-frame reactive nudge. A one-frame nudge (kill/cap speed for
// one tick, re-evaluate the next) wasn't enough real distance to clear a
// genuine wedge: as soon as hull contact briefly cleared, the normal
// plan-following logic immediately steered straight back into it — a tight
// "twitch and reapproach" loop instead of actually getting clear. 3 seconds
// of committed reverse at ~0.9 m/s covers ~2.7m, enough to back the hull
// meaningfully away from whatever it was touching before trying again.
const RECOVERY_DURATION_MS = 3000;
const RECOVERY_REVERSE_SPEED = -0.9;
// Follow-on "creep forward on the new heading" phase, once the turn phase
// reports aligned and clear — real separation from whatever the boat was
// stuck on before handing back to the live replan (see
// navigation.js's driveRecoveryForward()), rather than resuming
// plan-following the instant it was JUST barely clear, right next to
// whatever it turned away from.
const RECOVERY_FORWARD_DURATION_MS = 2000;
const RECOVERY_FORWARD_SPEED = 1.2;
// How long a stuck-spot breadcrumb (state.js's recoveryBreadcrumbs) keeps
// forcing the live replan to route around it — long enough to survive many
// replan cycles (250ms each) so the boat doesn't just drift back the moment
// it expires, short enough that a spot which genuinely was just temporarily
// blocked (e.g. a dynamic boat that has since moved on) isn't avoided
// forever.
const RECOVERY_BREADCRUMB_DURATION_MS = 25000;

// Angular ramp rate for ordinary autonomous transit steering (navigation.js),
// used instead of THRUST_RAMP_RATE (6.0 — reaches MAX_ANGULAR in ~0.2s,
// tuned for Mode 2's snappy manual joystick feel). Reusing that same fast
// ramp for autonomous steering meant every 250ms live replan that picked a
// meaningfully different heading turned into a near-instant hard turn — read
// as a sudden, "dramatic" swing rather than a smooth arc into the new
// heading. 1.5 reaches MAX_ANGULAR in ~0.8s instead — still responsive
// enough to react to a fresh obstacle, just eased into rather than snapped
// into. Deliberately NOT used for align/approach/creep/reverse_swing (those
// keep the fast THRUST_RAMP_RATE) or the stuck-recovery maneuver — both are
// low-frequency, hand-tuned, or urgent state transitions that want to commit
// decisively, not the routine "replan tweaks the transit heading a bit"
// case this exists for.
const AUTONOMOUS_ANGULAR_RAMP_RATE = 1.5;

// Mode 3's "logical place for docking": the open-water fairway mouth just
// outside the marina's dock structure that every docking run funnels
// through before entering a specific berth. Also where Reset parks the
// boat, so a fresh session already starts here.
const DOCK_ENTRANCE_X = -220.0 * WORLD_SCALE;
const DOCK_ENTRANCE_Y = -95.0 * WORLD_SCALE;

// Each mode's own starting pose — switching to a mode (even switching BACK to
// one you were already in) always snaps the boat here, via resetBoatToPose()
// in lifecycle.js, so a session never carries over state from whatever you
// were doing a moment ago.
const MODE1_START = {
    // Mode 1 (Level Designer & Auto Nav): open water just off the Coastal
    // City, which is drawn at ROS (230, 230) — that point itself is dry land
    // well outside LAKE_RADIUS (140) and the pathfinding grid, so this is the
    // nearest navigable water along the same bearing (radius 110, 45°),
    // ~30m in from the shoreline. Yaw faces back out toward open water/the
    // lake center (bearing 225°), away from the coast.
    x: 77.8, y: 77.8, yaw: -2.356
};
const MODE2_START = {
    // Mode 2 (Joystick Drive): open water just outside the island's keep-out
    // ring (ISLAND_KEEP_OUT = ISLAND_RADIUS(25) + 8 = 33), due east of it
    // with a small safety margin so it doesn't spawn already touching the
    // friction boundary. Yaw faces away from the island.
    x: 38.0, y: 0.0, yaw: 0.0
};
const MODE3_START = {
    // Mode 3 (Autonomous Docking): unchanged — the marina's Harbor Fairway
    // Entrance, same as Reset.
    x: DOCK_ENTRANCE_X, y: DOCK_ENTRANCE_Y, yaw: -1.57
};

// Mirrors exhibition_water.sdf's max_thrust_cmd/min_thrust_cmd — independent
// thruster control (Mode 2 W/A/R/D) targets each thruster's real physical
// limit directly, not a JS-side demand ceiling. Real T200-at-16V values
// (5.25/4.1 kgf).
const THRUSTER_MAX_FWD_N = 51.5;
const THRUSTER_MIN_REV_N = -40.2;

// Mode 2's Motor Power gauges (input.js's updateMode2MotorGauges()) — the
// real ceiling those bars normalize against. Deliberately 3x the real
// per-thruster max, not the max itself: FLASH mode (input.js's
// setFlashBoost()) genuinely commands up to 3x THRUSTER_MAX_FWD_N, so a
// gauge that capped at "100% = THRUSTER_MAX_FWD_N" would flatline at the
// top for the entire top third of what's actually reachable — exactly the
// most exciting part to actually see move. A normal (non-FLASH) full
// throttle now reads as ~33%, FLASH as up to 100%.
const MODE2_MOTOR_GAUGE_MAX_N = THRUSTER_MAX_FWD_N * 3;

// Real turn-rate ceiling for Mode 2's LOCAL SIM cruise scheme (arrow keys /
// gamepad Cruise stick) — derived the same way the twin-thruster branch's
// own targetAngular already is (max differential thrust, same
// HULL_YAW_MOMENT_ARM/HULL_YAW_DAMPING model above), so both control
// schemes share one realistic turning feel instead of the cruise scheme
// having its own, different ceiling. The cruise scheme's own targetAngular
// (input.js) asks for as much as ±15.0 rad/s — a deliberate, still-active
// "let real physics decide" experiment (see that assignment's own comment)
// that's harmless under real Gazebo physics (the real boat's own torque/
// damping caps the ACTUAL rotation regardless of how large the demand is)
// but not under local sim, which has nothing else to cap it — boatPos.yaw
// integrates directly from currentAngular there, so an uncapped demand read
// as "turning way too fast, unrealistic" once local sim shipped.
// Real-physics mode is intentionally left alone (still sends the full
// ±15.0 demand, unaffected by this) since Gazebo's own physics already
// makes it a no-op there; this constant only clamps local sim's integration.
const MODE2_CRUISE_MAX_ANGULAR_PHYSICAL = HULL_YAW_MOMENT_ARM * (THRUSTER_MAX_FWD_N - THRUSTER_MIN_REV_N) / HULL_YAW_DAMPING; // ≈2.9 rad/s

// User feedback (2026-09-09): even clamped at the fully-physical ceiling
// above, the cruise scheme (arrow keys / gamepad Cruise stick) still felt
// overly sensitive under local sim — "turns too much". The real boat softens
// a turn command through actual hull inertia/torque ramp-up before it shows
// up as rotation; local sim has none of that, it just integrates
// currentAngular directly, so the same peak rate reads as far twitchier with
// nothing smoothing it out. Scaled down from the physical ceiling (rather
// than replacing it outright) so the derivation above stays meaningful and
// this can be dialed back toward 1.0 later if it turns out to be over-tuned.
const MODE2_LOCAL_SIM_TURN_SENSITIVITY = 0.5;
const MODE2_CRUISE_MAX_ANGULAR = MODE2_CRUISE_MAX_ANGULAR_PHYSICAL * MODE2_LOCAL_SIM_TURN_SENSITIVITY;

// How fast Mode 2's LOCAL SIM turning RAMPS UP toward a held turn command —
// its own, slower rate than THRUST_RAMP_RATE (6.0, tuned for the linear
// throttle's snappy feel and reused as approachVelocity()'s default). Same
// reasoning as the sensitivity cap right above: real hull inertia already
// softens a turn's rise on the real boat, local sim has to do that softening
// itself instead of integrating straight to the capped rate in ~0.2s.
const MODE2_LOCAL_SIM_ANGULAR_RAMP_RATE = 3.0;

// How fast Mode 2's LOCAL SIM turning stops once the turn key/stick is
// released — deliberately its own, faster rate than WATER_FRICTION_RATE
// (1.0, tuned for linear coast-down), since a real hull's yaw damping
// settles a turn much quicker than translational drift decays, and per
// explicit request ("turning should stop faster"). Passed as
// approachVelocity()'s overrideRate on release only; ramp-UP into a turn
// uses MODE2_LOCAL_SIM_ANGULAR_RAMP_RATE just above instead.
const MODE2_ANGULAR_STOP_RATE = 5.0;

// Pre-Defined Open Marina Docking Berths (Side Parking & Slip Parking)
const availableBerths = [
    { id: 0, name: 'Berth #1: Main Spine Pier (Side Parking)', ros_x: -215.0 * WORLD_SCALE, ros_y: -171.5 * WORLD_SCALE, fairway_x: -215.0 * WORLD_SCALE, type: 'parallel' },
    { id: 1, name: 'Berth #2: Jetty 2 Left Pier Face (Side Parking)', ros_x: -198.2 * WORLD_SCALE, ros_y: -142.2 * WORLD_SCALE, fairway_x: -220.0 * WORLD_SCALE, type: 'parallel' },
    { id: 2, name: 'Berth #3: Jetty 2 Right Slip (Bow-In Docking)', ros_x: -189.0 * WORLD_SCALE, ros_y: -142.2 * WORLD_SCALE, fairway_x: -172.0 * WORLD_SCALE, type: 'slip' },
    { id: 3, name: 'Berth #4: Jetty 3 Left Slip (Bow-In Docking)', ros_x: -139.0 * WORLD_SCALE, ros_y: -142.0 * WORLD_SCALE, fairway_x: -158.0 * WORLD_SCALE, type: 'slip' }
];

// Island keep-out — see boundaries.js's maxSpeedNearIsland()/isMovingIntoObstacle()
// for the full "why" (graduated braking + hard collision design history).
const ISLAND_X = 0.0;
const ISLAND_Y = 0.0;
const ISLAND_RADIUS = 25.0;
const ISLAND_KEEP_OUT = ISLAND_RADIUS + 8.0; // 33.0m keep-out buffer (Guaranteed clearance, NO island crashes!)
// Gentle braking-zone deceleration reused by the island/shore/marina graduated
// speed caps — see boundaries.js's maxSpeedNearIsland() for the full derivation.
const ISLAND_BRAKE_DECEL = 0.5; // m/s^2 — ~6.2m braking zone at MAX_LINEAR_FWD (2.49 m/s)

// Circular Lake water body — see scene-environment.js for the 3D mesh and
// pathfinding.js's isPointBlocked() for the shoreline keep-out that uses
// this. Uses a bigger pre-scale reference (350, was 300) than the marina/
// island layout so the lake grows slightly relative to the marina footprint
// — the marina's farthest pier corner sits at ~126.6m from origin at this
// WORLD_SCALE, so this leaves it a real margin inside the shore instead of
// poking through it (confirmed by hand: old 300 reference put that same
// corner just OUTSIDE the shoreline).
const LAKE_RADIUS = 350.0 * WORLD_SCALE;

// Canonical marina dock geometry — single source of truth for the pier + 3
// finger jetties, read by the 3D mesh (scene-marina.js), the 2D map draw
// (render-2d.js), the hard hull collision (boundaries.js), the pathfinder
// (pathfinding.js), and the Mode 3 dynamic-docking click handler (docking.js).
// These used to be four/five separately hand-copied literal numbers that
// drifted out of sync — the 3D mesh and 2D map each scaled position by
// WORLD_SCALE but left their SIZE literals unscaled, rendering a pier/jetty
// far bigger than what the collision/pathfinding system (which scaled both
// consistently) actually protected — "the ship flows on top of the wood."
const MARINA_PIER_X_MIN = -265.0 * WORLD_SCALE;
const MARINA_PIER_X_MAX = -125.0 * WORLD_SCALE;
const MARINA_PIER_Y = -173.0 * WORLD_SCALE;    // north face, toward open water
const MARINA_PIER_THICKNESS = 4.0;             // unscaled — a real structural thickness, not a layout span (same reasoning as JETTY_HALF_WIDTH below)
const JETTY_X_LIST = [-245.0, -195.0, -145.0].map(x => x * WORLD_SCALE);
const JETTY_HALF_WIDTH = 2.0;                  // unscaled — matches the existing hull-collision margin
const JETTY_Y_NEAR = -115.0 * WORLD_SCALE;     // open-water mouth
const JETTY_Y_FAR = -175.0 * WORLD_SCALE;      // pier-connected end (slightly past MARINA_PIER_Y so there's no gap)
// Derived geometry helpers, reused by both the 3D mesh (scene-marina.js) and
// the 2D map draw (render-2d.js) so they can never disagree with each other.
const pierLength = MARINA_PIER_X_MAX - MARINA_PIER_X_MIN;
const pierCenterX = (MARINA_PIER_X_MIN + MARINA_PIER_X_MAX) / 2;
const pierCenterY = MARINA_PIER_Y - MARINA_PIER_THICKNESS / 2; // ROS y (south of the north face)
const jettyWidth = JETTY_HALF_WIDTH * 2;
const jettyLength = JETTY_Y_NEAR - JETTY_Y_FAR;
const jettyCenterY = (JETTY_Y_NEAR + JETTY_Y_FAR) / 2; // ROS y

// Pathfinder keep-out margins — see pathfinding.js's isPointBlocked()/isInsideMarinaStructure().
const SAFETY_RADIUS = 5.5; // Restrictive imaginary keep-out radius for buoys/boats (meters)
// Smaller keepout for the ~30 baked-in moored boats specifically (see
// pathfinding.js's isPointBlocked()) — they sit only ~2.4m off their jetty's
// own centerline by design (packed marina), so the generic 5.5m SAFETY_RADIUS
// (sized for a buoy the player can drop anywhere) stacked on top of the
// jetty's own 5.5m keepout and pinched the lane between two jetties down
// to ~4m. This stays a real margin over their actual ~2.0m hull-contact
// radius (boundaries.js) without adding to what the jetty's own keepout
// already covers.
const MOORED_SHIP_SAFETY_RADIUS = 2.5;
const SHORE_MARGIN = 4.0;  // Keep-out band inside LAKE_RADIUS, off-limits to route planning (beach/greenery)

// Extra pathfinder-only clearance added around the spine pier's exposed
// north face and east/west ends (on top of its real MARINA_PIER_* footprint)
// — unlike every other obstacle (buoys/boats get SAFETY_RADIUS, the shore
// gets SHORE_MARGIN), the pier itself had ZERO planning buffer, so a live
// replan could route the boat right along its face with no standoff at all;
// real hull-footprint collision then caught up a moment later, which read
// as "the boat gets stuck against the marina." Kept under 3.7m so it can't
// swallow any docking corridor/align point that funnels close to the pier.
// The finger jetties already have their own 5.5m half-width buffer (vs.
// their real 2.0m) and are left as-is; widening that further would break
// the docking corridors that deliberately funnel close to a jetty mouth.
const MARINA_PIER_PATH_MARGIN = 3.0;

// ================= ILOS GUIDANCE TUNABLES (see guidance.js) ================= //
// Starting points, scaled down from the user's own MATLAB ILOS_wrapper
// reference values (Delta=8, sigma=0.05, R_switch=2.0 at u_d0=1.0 m/s) for
// this sim's WORLD_SCALE-compressed world and this boat's real thrust-derived
// top speed (MAX_LINEAR_FWD=2.49 m/s, not 1.0) — see guidance.js's
// ilosGuidance() for the full port notes. sigma is a dimensionless ratio,
// left as-is.
const ILOS_DELTA = 8.0 * WORLD_SCALE;     // lookahead distance (m) — smaller Delta = more aggressive heading correction
const ILOS_SIGMA = 0.05;                  // anti-windup term on the integral (dimensionless)
const ILOS_R_SWITCH = 2.0 * WORLD_SCALE;  // switch to the next segment once within this of the current one's end (m)

// DWA-style local correction tunables (see guidance.js's localCorrectedYaw()).
// Widened from the original [0, -0.2, 0.2, -0.4, 0.4] (max ~23deg either
// side) — that fan was too narrow to find a way around a real corner (e.g.
// a jetty/pier corner clipped mid-transit), so once the preferred heading
// was blocked it just fell back to that SAME blocked heading every frame.
// Still tried smallest-deviation-first, just with more/wider rungs (up to
// ~92deg) so a genuine "go around it" heading is actually in the search.
const DWA_HEADING_OFFSETS = [0, -0.2, 0.2, -0.4, 0.4, -0.7, 0.7, -1.0, 1.0, -1.3, 1.3, -1.6, 1.6]; // rad, preferred heading tried first
const DWA_LOOKAHEAD_SEC = 1.5;
