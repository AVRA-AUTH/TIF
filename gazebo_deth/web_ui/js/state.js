// ================= SHARED MUTABLE STATE =================
// Every `let` that more than one part of this app reads or writes. Loaded
// right after config.js (needs MODE1_START). Classic scripts sharing one
// global scope — every other file just reads/writes these directly, exactly
// as it did when they all lived in one app.js.

// State (activeAppMode defaults to 1 below, so start where Mode 1 does —
// otherwise a fresh page load put the boat at the marina entrance instead,
// only matching Mode 1's actual spawn point once you switched modes and back)
let boatPos = { x: MODE1_START.x, y: MODE1_START.y, yaw: MODE1_START.yaw, speed: 0 };
// Odom updates arriving before this timestamp are dropped — see resetBoatToPose() in lifecycle.js.
let ignoreOdomUntil = 0;
let currentMode = 'static';
let activeAppMode = 1;

let activeBerthIdx = 1; // Default to Berth #2 Side Parking
let entities = [];
// Static marina scenery detection targets (moored boats), fed by
// scene-marina.js's createMooredBoat() and read by detection-overlay.js.
// Separate from `entities`/`threeEntities`: those track things the player
// can place or that move, while the marina is fixed scenery built once.
const staticDetections = [];
let plannedPath = [];
let isNavigating = false;
let pathIndex = 0;
// ILOS guidance state (see guidance.js's ilosGuidance()) — explicit since JS
// has no equivalent of the MATLAB source's `persistent`. Reset at every
// "start a fresh run" site (btn-run, startDynamicDocking, resetBoatToPose);
// k alone (not y_int) also resets on Mode 1's periodic live replan — see
// navigation.js's runNavigationStep() for why.
let ilosState = { k: 1, y_int: 0 };
let obsCounter = 1;
let currentGoal = null;
let lastRecalcTime = 0;
let threePathLine = null;
// Mode 3's live-replan target — Mode 1's own replan re-targets `currentGoal`,
// but docking's real transit destination is the align point computed once in
// startDynamicDocking (docking.js), so this holds what a docking replan
// needs: {alignX, alignY, chosen}. Set there, cleared whenever a docking run
// ends (goal reached, Reset, mode switch).
let dockingTarget = null;
// Mode 1 ship/buoy collision popup (navigation.js) — latched so the alert()
// fires once per contact instead of every frame the hull stays touching.
let shipCollisionAlertShown = false;
// Mode 1 player-hull sink state (navigation.js sets these on a crash,
// main.js's boat-transform step reads them to ease in the tilt/submerge
// look). playerCrashTime anchors the animation's elapsed-time easing.
let playerBoatCrashed = false;
let playerCrashTime = 0;

let isAutoDocking = false;
let dockHoldStartTime = null; // timestamp when the boat first settled at the final berth waypoint
let dockingBerthName = '';

// Stuck-recovery state (navigation.js's driveRecovery()) — a committed
// multi-second reverse-and-turn maneuver, not a per-frame reactive nudge.
// recoveryUntil is a Date.now()-style timestamp (0 = not currently
// recovering); recoveryYaw is the escape heading picked once at entry
// (boundaries.js's findRecoveryYaw()) and held for the whole maneuver
// instead of being recomputed — and potentially flip-flopping — every frame.
let recoveryUntil = 0;
let recoveryYaw = 0;
// Follow-on "creep forward on the new heading" phase (navigation.js's
// driveRecoveryForward()) once the turn phase above reports aligned and
// clear — gets real separation from whatever the boat was stuck on before
// handing back to the live replan, instead of doing so the instant it was
// JUST barely clear.
let recoveryForwardUntil = 0;
// Breadcrumbs of recent stuck spots ({x, y, until}) — see navigation.js's
// activeRecoveryBreadcrumbObstacles(). Backing off on its own doesn't help
// when the live replan's target sits behind the same choke point: without
// this, the very next 250ms replan just re-finds the same route back
// through the same spot the boat only just cleared, producing an endless
// back-off/re-approach loop rather than genuine progress. Fed into the live
// replan's obstacle list as synthetic obstacles for a while so it's forced
// to actually route around a spot that already proved to be a dead end,
// instead of having no memory of it at all.
let recoveryBreadcrumbs = [];

// Mode 2 manual-drive state (thruster loop, input.js).
let currentLinear = 0.0;
let currentAngular = 0.0;
const heldAxes = new Set(); // 'fwd' | 'rev' | 'left' | 'right' — arrow keys only
const heldThrusterKeys = new Set(); // 'leftFwd' | 'leftRev' | 'rightFwd' | 'rightRev'
let lastThrusterTime = performance.now();
let lastPublishedLinear = 0.0;
let lastPublishedAngular = 0.0;
let lastPublishedLeftThrust = 0.0;
let lastPublishedRightThrust = 0.0;
let manualThrustOverrideActive = false; // mirrors what was last sent on manualOverrideTopic
let flashBoostActive = false; // mirrors what was last sent on flashBoostTopic (Mode 2's FLASH preset)

// Mode 2 PS4/gamepad state (input.js's thrusterLoop()). false = Cruise (left
// stick drives the combined-drive scheme, same as arrow keys); true = Twin
// Thruster (left/right stick each drive one hull's thruster directly, same
// scheme as W/A/R/D) — toggled by the Circle (◯) button.
let gamepadThrusterMode = false;

// Generic button-press edge-detector state, keyed by standard Gamepad API
// button index, shared across every mode's gamepad handling (input.js's
// gamepadButtonJustPressed()) — a physical button's press/release is a
// controller-level fact, not a per-mode one, so one shared map avoids each
// mode needing its own prev-state flag for the same button.
let gamepadButtonPrev = {};

// Mode 1 PS4/gamepad cursor (input.js's thrusterLoop()) — a virtual mouse
// position in ROS-frame meters, moved by the left stick and drawn as a
// crosshair on the 2D map (render-2d.js's renderGamepadCursor2D()).
// Square/Triangle/Circle place a buoy/moving-boat/goal at this position,
// same as clicking the map does at the real mouse position. Starts at Mode
// 1's own spawn point (a sensible first cursor spot near the boat) the first
// time a gamepad is used; `active` gates both drawing it and the cap on
// clamping it into the play area (see thrusterLoop).
// Offset +15m on x from MODE1_START rather than sitting exactly on it — the
// boat spawns AT MODE1_START, so a cursor starting dead-on it would reject
// a first, eager placement press as "too close to your boat" before the
// player has even moved the stick once.
let gamepadCursor = { x: MODE1_START.x + 15, y: MODE1_START.y, active: false };

// Mode 1's gamepad "which way does the stick move the cursor" toggle (R3) —
// 'map' or 'camera'. The shared cursor itself is always drawn on BOTH
// panels (render-2d.js's renderGamepadCursor2D(), detection-overlay.js's
// renderGamepadCursorDetection()) — this only changes how the LEFT STICK's
// direction gets interpreted (thrusterLoop, input.js): 'map' moves it
// map-aligned (stick-right = map-east), 'camera' moves it relative to
// whichever way the boat's camera currently faces (stick-right = camera's
// own screen-right), which is what actually feels intuitive while watching
// that view instead of the top-down map. index.html's view-panel gets a
// glowing yellow border to match (input.js's updateGamepadPerspectiveUI())
// so it's clear which interpretation is currently active.
let gamepadPerspective = 'map';
