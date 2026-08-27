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

let isAutoDocking = false;
let dockingBerthName = '';

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
