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
const MAX_ANGULAR = 1.2;         // top turn rate (rad/s)
const THRUST_RAMP_RATE = 6.0;    // units/sec: how fast velocity reaches its target while a thruster is held open
const WATER_FRICTION_RATE = 1.0; // units/sec: how fast velocity decays toward zero once released

// Mode 1/3 docking-leg speeds, expressed as ratios of MAX_LINEAR_FWD instead
// of flat numbers, so they scale automatically if the top speed is ever
// retuned again. Ratios preserve the original tuning (1.6/-1.3/1.3 m/s and
// a 0.3 m/s^2 brake) against the OLD MAX_LINEAR_FWD=6.0 this boat used
// before the real-T200 retune.
const DOCK_APPROACH_SPEED_RATIO = 1.6 / 6.0;        // ~0.267
const DOCK_REVERSE_SWING_SPEED_RATIO = -1.3 / 6.0;  // ~-0.217
const DOCK_CREEP_SPEED_RATIO = 1.3 / 6.0;           // ~0.217
const DOCK_BRAKE_DECEL = 0.3 * (MAX_LINEAR_FWD / 6.0); // ~0.1245 m/s^2

// Mode 3's "logical place for docking": the open-water fairway mouth just
// outside the marina's dock structure that every docking run funnels
// through before entering a specific berth. Also where Reset parks the
// boat, so a fresh session already starts here.
const DOCK_ENTRANCE_X = -220.0 * WORLD_SCALE;
const DOCK_ENTRANCE_Y = -95.0 * WORLD_SCALE;

// Each mode's own starting pose — switching to a mode (even switching BACK to
// one you were already in) always snaps the boat here, via resetBoatToPose()
// in the mode-switch handlers below, so a session never carries over state
// from whatever you were doing a moment ago.
//
// Coordinates are hardcoded numerically (not via the ISLAND_*/city constants
// below) since those aren't defined until later in the file and these are
// evaluated immediately, top-to-bottom, at load.
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
    // new friction boundary. Yaw faces away from the island.
    x: 38.0, y: 0.0, yaw: 0.0
};
const MODE3_START = {
    // Mode 3 (Autonomous Docking): unchanged — the marina's Harbor Fairway
    // Entrance, same as Reset.
    x: DOCK_ENTRANCE_X, y: DOCK_ENTRANCE_Y, yaw: -1.57
};

// Connect to ROS via roslibjs
const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

const statusEl = document.getElementById('status');

ros.on('connection', function () {
    statusEl.textContent = 'Connected to ROS';
    statusEl.className = 'connected';
});

ros.on('error', function () {
    statusEl.textContent = 'Connection Error';
    statusEl.className = 'disconnected';
});

ros.on('close', function () {
    statusEl.textContent = 'Disconnected';
    statusEl.className = 'disconnected';
});

// ROS Topics
const spawnTopic = new ROSLIB.Topic({ ros: ros, name: '/exhibition/spawn_obstacle', messageType: 'std_msgs/String' });
const cmdVelTopic = new ROSLIB.Topic({ ros: ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' });
const odomTopic = new ROSLIB.Topic({ ros: ros, name: '/odom', messageType: 'nav_msgs/Odometry' });
const goalTopic = new ROSLIB.Topic({ ros: ros, name: '/goal_pose', messageType: 'geometry_msgs/PoseStamped' });
// Raw per-thruster topics — same ones cmd_vel_thrust_mixer.py publishes to,
// but written directly from the browser for independent thruster control
// (W/A/R/D below), bypassing /cmd_vel and the mixer entirely.
const leftThrustTopic = new ROSLIB.Topic({ ros: ros, name: '/asv_boat/thrusters/left/thrust', messageType: 'std_msgs/Float64' });
const rightThrustTopic = new ROSLIB.Topic({ ros: ros, name: '/asv_boat/thrusters/right/thrust', messageType: 'std_msgs/Float64' });
// Mirrors exhibition_water.sdf's max_thrust_cmd/min_thrust_cmd — independent
// thruster control targets each thruster's real physical limit directly,
// not a JS-side demand ceiling. Real T200-at-16V values (5.25/4.1 kgf).
const THRUSTER_MAX_FWD_N = 51.5;
const THRUSTER_MIN_REV_N = -40.2;

// ============== DOF Panel (Mode 2 only — real /odom, not the JS sim) ============== //
// Scoped to surge and yaw rate specifically — heave/pitch are the interesting
// DOFs for the buoyancy bug hunted down in HANDOFF.md's Session update, but
// surge/yaw are what matter for actually driving the boat, which is this
// panel's purpose. Heave/pitch stay diagnosable via a raw /odom capture if
// that investigation ever needs to resume.
const DOF_HISTORY_LEN = 150; // ~3s of history at the odom plugin's ~50Hz rate
const dofPanel = document.getElementById('dof-panel');
const dofHist = { surge: [], yawRate: [] };
const dofCanvas = {
    surge: document.getElementById('dof-surge-chart'),
    yawRate: document.getElementById('dof-yawrate-chart'),
};
const dofValEl = {
    surge: document.getElementById('dof-surge-val'),
    yawRate: document.getElementById('dof-yawrate-val'),
};

function pushDof(key, value) {
    const arr = dofHist[key];
    arr.push(value);
    if (arr.length > DOF_HISTORY_LEN) arr.shift();
}

// Simple auto-scaled sparkline with a zero reference line — good enough to see
// oscillation, bias, and decay at a glance without pulling in a charting lib.
function drawSparkline(canvas, hist) {
    if (!canvas || hist.length < 2) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    let min = Math.min(...hist, 0);
    let max = Math.max(...hist, 0);
    const range = (max - min) || 1;
    const pad = range * 0.15;
    min -= pad;
    max += pad;

    const zy = h - ((0 - min) / (max - min)) * h;
    ctx.strokeStyle = '#333344';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, zy);
    ctx.lineTo(w, zy);
    ctx.stroke();

    ctx.strokeStyle = '#00ffcc';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    hist.forEach((v, i) => {
        const x = (i / (hist.length - 1)) * w;
        const y = h - ((v - min) / (max - min)) * h;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
}

function updateDofPanel(surge, yawRate) {
    pushDof('surge', surge);
    pushDof('yawRate', yawRate);

    dofValEl.surge.textContent = surge.toFixed(2);
    dofValEl.yawRate.textContent = yawRate.toFixed(2);

    drawSparkline(dofCanvas.surge, dofHist.surge);
    drawSparkline(dofCanvas.yawRate, dofHist.yawRate);
}

// State (activeAppMode defaults to 1 below, so start where Mode 1 does —
// otherwise a fresh page load put the boat at the marina entrance instead,
// only matching Mode 1's actual spawn point once you switched modes and back)
let boatPos = { x: MODE1_START.x, y: MODE1_START.y, yaw: MODE1_START.yaw, speed: 0 };
// Odom updates arriving before this timestamp are dropped — see resetBoatToPose().
let ignoreOdomUntil = 0;
let currentMode = 'static';
let activeAppMode = 1;

// Pre-Defined Open Marina Docking Berths (Side Parking & Slip Parking)
const availableBerths = [
    { id: 0, name: 'Berth #1: Main Spine Pier (Side Parking)', ros_x: -215.0 * WORLD_SCALE, ros_y: -171.5 * WORLD_SCALE, fairway_x: -215.0 * WORLD_SCALE, type: 'parallel' },
    { id: 1, name: 'Berth #2: Jetty 2 Left Pier Face (Side Parking)', ros_x: -198.2 * WORLD_SCALE, ros_y: -142.2 * WORLD_SCALE, fairway_x: -220.0 * WORLD_SCALE, type: 'parallel' },
    { id: 2, name: 'Berth #3: Jetty 2 Right Slip (Bow-In Docking)', ros_x: -189.0 * WORLD_SCALE, ros_y: -142.2 * WORLD_SCALE, fairway_x: -172.0 * WORLD_SCALE, type: 'slip' },
    { id: 3, name: 'Berth #4: Jetty 3 Left Slip (Bow-In Docking)', ros_x: -139.0 * WORLD_SCALE, ros_y: -142.0 * WORLD_SCALE, fairway_x: -158.0 * WORLD_SCALE, type: 'slip' }
];
let activeBerthIdx = 1; // Default to Berth #2 Side Parking
let entities = [];
let plannedPath = [];
let isNavigating = false;
let pathIndex = 0;
let obsCounter = 1;
let currentGoal = null;
let lastRecalcTime = 0;
let threePathLine = null;

function update3DPathLine() {
    if (threePathLine) {
        scene.remove(threePathLine);
        if (threePathLine.geometry) threePathLine.geometry.dispose();
        threePathLine = null;
    }

    if (plannedPath && plannedPath.length > 1) {
        // Filter out duplicate points (like in 'align' states) that break computeLineDistances!
        const uniquePoints = [];
        plannedPath.forEach(p => {
            if (uniquePoints.length === 0) {
                uniquePoints.push(new THREE.Vector3(p.x, 0.4, -p.y));
            } else {
                const lastP = uniquePoints[uniquePoints.length - 1];
                if (Math.abs(lastP.x - p.x) > 0.01 || Math.abs(lastP.z - (-p.y)) > 0.01) {
                    uniquePoints.push(new THREE.Vector3(p.x, 0.4, -p.y));
                }
            }
        });

        if (uniquePoints.length > 1) {
            const lineGeo = new THREE.BufferGeometry().setFromPoints(uniquePoints);
            const lineMat = new THREE.LineDashedMaterial({
                color: 0x00ffcc,
                linewidth: 3,
                scale: 1,
                dashSize: 1,
                gapSize: 0.5
            });
            threePathLine = new THREE.Line(lineGeo, lineMat);
            threePathLine.computeLineDistances();
            scene.add(threePathLine);
        }
    }
}

// Smooth speed limit approaching the island — unlike the hard block below,
// this shrinks the achievable speed continuously as the hull nears
// ISLAND_KEEP_OUT, converging to 0 exactly at the boundary via real
// stopping-distance kinematics (v = sqrt(2 * decel * clearance)). Direction
// agnostic on purpose — it's a distance-only cap, applied to whichever of
// forward/reverse the caller is about to command, so a boat can sail into
// the island stern-first just as easily as bow-first (backing toward it
// while facing away) unless BOTH directions are throttled the same way.
// Three things were tried and rejected before this: a hard cut right at the
// boundary let real momentum coast the hull past it before drag caught up
// ("lets the boat go into the island"), forcing full opposite thrust on
// contact fixed that but fought a held key every single frame, reading as
// the boat vibrating in place ("kind of stuck"), and capping forward only
// left reverse with the original momentum problem ("sail in backwards").
// This is the middle ground — out at open sea (clearance large) this
// returns a huge number and has zero effect, so cruising is completely
// unaffected; only inside the last few meters of the island's keep-out ring
// does it start throttling the demand down, smoothly, like water getting
// thick with mud rather than a switch flipping.
const ISLAND_BRAKE_DECEL = 0.5; // m/s^2 — gentle; ~6.2m braking zone at MAX_LINEAR_FWD (2.49 m/s)
function maxSpeedNearIsland() {
    const distToIsland = Math.hypot(ISLAND_X - boatPos.x, ISLAND_Y - boatPos.y);
    const clearance = distToIsland - ISLAND_KEEP_OUT;
    if (clearance <= 0) return 0;
    return Math.sqrt(2 * ISLAND_BRAKE_DECEL * clearance);
}

// Same graduated-braking idea, applied to the shoreline (isExitingLake below
// only ever gave the shore a hard, zero-standoff cutoff — the "lets the boat
// go into the island" bug, just for the beach instead of the island, since it
// was never given the same fix). Distance is measured from the world origin
// since the lake is centered there, same as the island — the "clearance" is
// just measured the other way round (room before crossing OUT past
// LAKE_RADIUS, not room before crossing IN past ISLAND_KEEP_OUT).
function maxSpeedNearShore() {
    const distFromCenter = Math.hypot(boatPos.x, boatPos.y);
    const clearance = (LAKE_RADIUS - 0.5) - distFromCenter;
    if (clearance <= 0) return 0;
    return Math.sqrt(2 * ISLAND_BRAKE_DECEL * clearance);
}

// Direction-aware companion to maxSpeedNearShore(), same reasoning as
// isHeadingTowardIsland() above: only cap whichever direction is actually
// increasing distance from center (heading toward/through the shore),
// never the direction that's heading back toward open water, so braking to
// a stop at the shore can never also block getting away from it.
function isHeadingTowardShore(direction) {
    const dist = Math.hypot(boatPos.x, boatPos.y);
    if (dist < 0.01) return false; // at the world center — nowhere near the shore either way
    const moveX = direction * Math.cos(boatPos.yaw), moveY = direction * Math.sin(boatPos.yaw);
    return (moveX * boatPos.x + moveY * boatPos.y) > 0; // positive dot with the outward radial vector = heading out
}

// Is commanding `direction` (+1 forward, -1 reverse) actually heading INTO
// the island (shrinking distance to it), or away/tangential? maxSpeedNearIsland()
// on its own is a pure distance cap with no direction awareness — applied
// to both forward and reverse unconditionally, it caps whichever way you're
// currently facing equally, including the escape direction. Right at the
// boundary (clearance ~0, cap ~0) that meant BOTH directions got clamped to
// 0 at once — genuinely stuck, unable to leave. This lets each call site
// only cap the direction that's actually closing the distance, leaving the
// other direction completely free to always be able to back away.
function isHeadingTowardIsland(direction) {
    const dx = boatPos.x - ISLAND_X, dy = boatPos.y - ISLAND_Y;
    const dist = Math.hypot(dx, dy);
    if (dist < 0.01) return true; // degenerate (on top of the island center) — treat as "in"
    const moveX = direction * Math.cos(boatPos.yaw), moveY = direction * Math.sin(boatPos.yaw);
    return (moveX * dx + moveY * dy) < 0; // negative dot with the outward radial vector = closing in
}

// Marina (pier + 3 finger jetties) equivalent of maxSpeedNearIsland() /
// maxSpeedNearShore() — same graduated-braking idea, but the marina isn't a
// single circle, so this measures point-to-rectangle distance to the pier
// deck and each jetty instead of point-to-point. Reads MARINA_PIER_*/
// JETTY_* (defined further down this file, next to the 3D mesh that also
// reads them — safe, since this is only ever called after the whole script
// has run once). Returns 0 (already touching/inside) or a real clearance in
// meters; 0 clearance anywhere is fine, same "min over every candidate
// obstacle" shape as isTouchingObstacleAt.
function distanceToMarinaStructure(x, y) {
    let minDist = Infinity;
    const pierDx = x < MARINA_PIER_X_MIN ? MARINA_PIER_X_MIN - x : Math.max(0, x - MARINA_PIER_X_MAX);
    const pierDy = Math.max(0, y - MARINA_PIER_Y);
    minDist = Math.min(minDist, Math.hypot(pierDx, pierDy));
    for (const jx of JETTY_X_LIST) {
        const dx = Math.max(0, Math.abs(x - jx) - JETTY_HALF_WIDTH);
        const dy = y > JETTY_Y_NEAR ? y - JETTY_Y_NEAR : Math.max(0, JETTY_Y_FAR - y);
        minDist = Math.min(minDist, Math.hypot(dx, dy));
    }
    return minDist;
}
function maxSpeedNearMarina() {
    const d = distanceToMarinaStructure(boatPos.x, boatPos.y);
    return d <= 0 ? 0 : Math.sqrt(2 * ISLAND_BRAKE_DECEL * d); // reuses the same gentle decel as the island/shore
}
// Direction-aware companion, same purpose as isHeadingTowardIsland()/
// isHeadingTowardShore() above — but since the marina isn't a single point,
// this compares distanceToMarinaStructure() before and after a small step
// instead of a radial dot product: closing in on ANY part of the structure
// (pier or any jetty) counts, whichever one is nearest.
function isHeadingTowardMarina(direction) {
    const d0 = distanceToMarinaStructure(boatPos.x, boatPos.y);
    if (d0 > 15) return false; // cheap short-circuit far from the marina
    const stepX = boatPos.x + direction * Math.cos(boatPos.yaw) * 0.5;
    const stepY = boatPos.y + direction * Math.sin(boatPos.yaw) * 0.5;
    return distanceToMarinaStructure(stepX, stepY) < d0;
}

// Combines all three graduated boundaries (island keep-out, shoreline, and
// the marina dock structure) into one speed ceiling for `direction`: only
// the boundary(ies) that direction is actually closing in on contribute a
// cap, so the escape direction away from any of them is always left at
// rawMax. Returns a magnitude (always >= 0); the caller applies the sign.
// Mode 2 (manual driving) only — Mode 1/3's autonomous docking keeps its
// own hand-tuned approach/creep speeds and never calls this, so this can't
// fight the docking state machine.
function boundaryCappedSpeed(direction, rawMax) {
    let cap = rawMax;
    if (isHeadingTowardIsland(direction)) cap = Math.min(cap, maxSpeedNearIsland());
    if (isHeadingTowardShore(direction)) cap = Math.min(cap, maxSpeedNearShore());
    if (isHeadingTowardMarina(direction)) cap = Math.min(cap, maxSpeedNearMarina());
    return cap;
}

// Shoreline boundary check — direction-aware, unlike isTouchingObstacle()
// below. A plain "blocked outside LAKE_RADIUS" flag (like the island's)
// would also have to block reverse to stop the boat backing out stern-first,
// but that risks wedging it dead at the edge with BOTH directions blocked
// (e.g. right after a forward touch auto-backs it off in reverse). Instead
// this predicts whether one step in the given direction (+1 forward, -1
// reverse) would push the hull past the shore, so whichever direction is
// actually headed back toward open water always stays free.
function isExitingLake(direction) {
    const dist = Math.hypot(boatPos.x, boatPos.y);
    if (dist < LAKE_RADIUS - 2.0) return false; // well clear of shore, skip the trig
    const stepX = boatPos.x + direction * Math.cos(boatPos.yaw) * 0.5;
    const stepY = boatPos.y + direction * Math.sin(boatPos.yaw) * 0.5;
    return Math.hypot(stepX, stepY) > LAKE_RADIUS - 0.5;
}

// Solid-boundary contact check — island, pier walls, moored vessels, buoys/
// dynamic obstacles — evaluated at an arbitrary (x, y, yaw), not just the
// boat's current pose, so isMovingIntoObstacle() below can also evaluate it
// at a predicted next position. Called fresh from every control path
// (Mode 2's two drive schemes, Mode 1/3's auto-nav) right before publishing,
// rather than cached, since draw() and thrusterLoop() are independent rAF
// loops with no guaranteed ordering.
function isTouchingObstacleAt(x, y, yaw) {
    // ISLAND_KEEP_OUT (33m), not ISLAND_RADIUS+0.5 (25.5m): the rendered
    // island mesh — grass cylinder plus its sand-beach ring
    // (islandSandGeo = CylinderGeometry(26.5, 30, ...)) — visually extends
    // out to 30m, well past the old 25.5m boundary. That gap let the boat
    // visibly drive across the sand before any collision ever triggered.
    // ISLAND_KEEP_OUT already exists as the pathfinder's "guaranteed
    // clearance" radius and comfortably clears the visible island (33 > 30),
    // so the hard hull-collision boundary now matches what Mode 1/3 routes
    // already stay clear of, instead of being tighter than the visuals.
    const distToIsland = Math.hypot(ISLAND_X - x, ISLAND_Y - y);
    if (distToIsland < ISLAND_KEEP_OUT) return true;

    const hullXExtent = Math.abs(Math.cos(yaw)) * 2.8 + Math.abs(Math.sin(yaw)) * 0.8;
    const hullYExtent = Math.abs(Math.sin(yaw)) * 2.8 + Math.abs(Math.cos(yaw)) * 0.8;

    // Main Spine Pier — canonical geometry (see MARINA_PIER_* consts, defined
    // near the 3D mesh that also reads them, further down this file).
    if (y - hullYExtent <= MARINA_PIER_Y && x + hullXExtent >= MARINA_PIER_X_MIN && x - hullXExtent <= MARINA_PIER_X_MAX) {
        return true;
    }

    // Finger Jetties — canonical geometry (JETTY_X_LIST/JETTY_HALF_WIDTH/JETTY_Y_NEAR/JETTY_Y_FAR).
    for (const jx of JETTY_X_LIST) {
        const overlapsX = (x + hullXExtent >= jx - JETTY_HALF_WIDTH) && (x - hullXExtent <= jx + JETTY_HALF_WIDTH);
        const overlapsY = (y + hullYExtent >= JETTY_Y_FAR) && (y - hullYExtent <= JETTY_Y_NEAR);
        if (overlapsX && overlapsY) return true;
    }

    // Moored vessels along the jetties
    for (let b = -168 * WORLD_SCALE; b <= -128 * WORLD_SCALE; b += 6.5 * WORLD_SCALE) {
        if (Math.abs(b - (-147.5 * WORLD_SCALE)) > 3.0) {
            const parkedLocs = [
                { x: -250.5 * WORLD_SCALE, y: b }, { x: -239.5 * WORLD_SCALE, y: b },
                { x: -200.5 * WORLD_SCALE, y: b }, { x: -189.5 * WORLD_SCALE, y: b },
                { x: -150.5 * WORLD_SCALE, y: b }, { x: -139.5 * WORLD_SCALE, y: b }
            ];
            for (const pl of parkedLocs) {
                if (Math.hypot(pl.x - x, pl.y - y) < 2.0) return true;
            }
        }
    }

    // Buoys & dynamic obstacles placed in Mode 1
    for (const ent of entities) {
        if (ent.type === 'static' || ent.type === 'dynamic') {
            if (Math.hypot(ent.ros_x - x, ent.ros_y - y) < 1.7) return true;
        }
    }

    return false;
}

function isTouchingObstacle() {
    return isTouchingObstacleAt(boatPos.x, boatPos.y, boatPos.yaw);
}

// Direction-aware obstacle check — same step-prediction idea as
// isExitingLake() above, applied to the island/pier/jetty/moored-boat/buoy
// geometry instead of the shoreline. Only blocks the direction that would
// actually drive the hull deeper into what it's touching, so a forward
// collision can still always be escaped by reversing AND (new) a reverse
// collision can always be escaped by going forward — neither direction is
// trusted as a blanket "always free" escape hatch anymore, each is only
// left free when it demonstrably moves the hull out of contact.
//
// Safety case a plain "block whichever direction is still touching after
// its step" rule would miss: sliding along a flat wall (e.g. pinned
// sideways against the spine pier) can leave BOTH a forward and a reverse
// step still reading as "touching," since the touch check is an
// axis-aligned box and a small step parallel to the wall doesn't cross out
// of it either way. Blocking both in that case would strand the boat
// needing a manual Reset — worse than the bug this is fixing — so when
// both directions look blocked, treat it as ambiguous and block neither.
function isMovingIntoObstacle(direction) {
    if (!isTouchingObstacle()) return false;
    const stepX = boatPos.x + direction * Math.cos(boatPos.yaw) * 0.5;
    const stepY = boatPos.y + direction * Math.sin(boatPos.yaw) * 0.5;
    const oppStepX = boatPos.x - direction * Math.cos(boatPos.yaw) * 0.5;
    const oppStepY = boatPos.y - direction * Math.sin(boatPos.yaw) * 0.5;
    const thisBlocked = isTouchingObstacleAt(stepX, stepY, boatPos.yaw);
    const oppBlocked = isTouchingObstacleAt(oppStepX, oppStepY, boatPos.yaw);
    if (thisBlocked && oppBlocked) return false;
    return thisBlocked;
}

// Listen to boat Odometry — the sole source of boatPos, unconditionally, in
// every mode. This used to be gated to "only while Mode 2 or actively
// navigating" as a minor perf shortcut, but that let boatPos sit frozen on a
// stale/cosmetic value (e.g. right after a reset) while the REAL simulated
// boat was somewhere else entirely (still settling at its old spot, or
// mid-collision) — invisible until the next nav run suddenly synced to the
// true position out from under it. Collision detection below now also runs
// in every mode, so it needs boatPos to be live at all times, not just
// during Mode 2 or an active nav/dock run.
odomTopic.subscribe((msg) => {
    // Drop odom updates for a short window right after resetBoatToPose() —
    // it snaps boatPos locally AND fires an async 'set_pose' to the Gazebo
    // backend, but that physics teleport takes real time to land. Without
    // this guard, an /odom message reporting the boat's OLD (pre-reset)
    // pose could arrive first and clobber the reset right back, before the
    // teleport actually completes — a race that only sometimes loses,
    // which is why it looked like "the right spot, then the wrong one" on
    // different clicks instead of failing consistently.
    if (Date.now() < ignoreOdomUntil) return;

    // twist is in the child_frame (base_link, body frame) per this odometry
    // plugin's config (robot_base_frame: base_link) — linear.x is signed
    // surge speed (forward positive, reverse negative) directly, not a
    // magnitude. Using sqrt(x^2+y^2) here used to erase the sign, making
    // reverse motion indistinguishable from forward on the telemetry panel.
    boatPos.x = msg.pose.pose.position.x;
    boatPos.y = msg.pose.pose.position.y;
    boatPos.speed = msg.twist.twist.linear.x;
    const q = msg.pose.pose.orientation;
    boatPos.yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

    // DOF panel: yaw rate is twist.angular.z (body frame, already signed).
    // Updates whenever boatPos itself is being synced (all modes now),
    // matching the panel's visibility (Part B).
    const yawRate = msg.twist.twist.angular.z;
    updateDofPanel(boatPos.speed, yawRate);
});

// Canvas Setup. SCALE is chosen so the navigable lake circle (LAKE_RADIUS,
// defined below) fills most of the 550x500 map canvas instead of being a
// small circle lost in a sea of green land — per explicit user request.
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const SCALE = 1.35;

function getCurrentViewParams() {
    if (activeAppMode === 3) {
        // Zoomed in on Marina
        return { scale: 3.5, offsetX: -195.0 * WORLD_SCALE, offsetY: -145.0 * WORLD_SCALE }; // ROS center of marina
    }
    // Global view
    return { scale: SCALE, offsetX: 0.0, offsetY: 0.0 };
}

function rosToCanvas(rx, ry) {
    const view = getCurrentViewParams();
    const cx = canvas.width / 2 + (rx - view.offsetX) * view.scale;
    const cy = canvas.height / 2 - (ry - view.offsetY) * view.scale;
    return { x: cx, y: cy };
}

function canvasToRos(cx, cy) {
    const view = getCurrentViewParams();
    const rx = (cx - canvas.width / 2) / view.scale + view.offsetX;
    const ry = -(cy - canvas.height / 2) / view.scale + view.offsetY;
    return { rx, ry };
}

// ================= THREE.JS 3D FPV SETUP ================= //
const container3d = document.getElementById('threejs-container');
const scene = new THREE.Scene();

// 1. Realistic Atmosphere & Sky
scene.background = new THREE.Color(0x7ec0ee); // Lake Sky Blue
scene.fog = new THREE.FogExp2(0x7ec0ee, 0.001);

const camera = new THREE.PerspectiveCamera(68, 550 / 500, 0.1, 3000);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(550, 500);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
container3d.appendChild(renderer.domElement);

// Lighting System
scene.add(new THREE.AmbientLight(0xffffff, 1.1));
const sunLight = new THREE.DirectionalLight(0xfffaed, 1.3);
sunLight.position.set(300, 500, 200);
sunLight.castShadow = true;
sunLight.shadow.camera.left = -400;
sunLight.shadow.camera.right = 400;
sunLight.shadow.camera.top = 400;
sunLight.shadow.camera.bottom = -400;
scene.add(sunLight);

// 1B. Glowing 3D Sun & Sky Corona
const sunGeo = new THREE.SphereGeometry(22, 32, 32);
const sunMat = new THREE.MeshBasicMaterial({ color: 0xfff5cc });
const sunMesh = new THREE.Mesh(sunGeo, sunMat);
sunMesh.position.set(300, 450, -200);
scene.add(sunMesh);

const haloGeo = new THREE.SphereGeometry(35, 32, 32);
const haloMat = new THREE.MeshBasicMaterial({ color: 0xffea9f, transparent: true, opacity: 0.35 });
const haloMesh = new THREE.Mesh(haloGeo, haloMat);
haloMesh.position.set(300, 450, -200);
scene.add(haloMesh);

// 1C. Volumetric Fluffy Clouds
function createCloud(x, y, z, scale = 1.0) {
    const cloudGroup = new THREE.Group();
    const cloudMat = new THREE.MeshLambertMaterial({ color: 0xffffff, transparent: true, opacity: 0.85 });

    const puffPositions = [
        { x: 0, y: 0, z: 0, r: 12 },
        { x: 10, y: 3, z: 2, r: 10 },
        { x: -10, y: 2, z: -2, r: 11 },
        { x: 18, y: -2, z: 1, r: 8 },
        { x: -16, y: -1, z: 3, r: 9 },
        { x: 4, y: 6, z: -1, r: 9 }
    ];

    puffPositions.forEach(p => {
        const puff = new THREE.Mesh(new THREE.SphereGeometry(p.r * scale, 16, 16), cloudMat);
        puff.position.set(p.x * scale, p.y * scale, p.z * scale);
        cloudGroup.add(puff);
    });

    cloudGroup.position.set(x, y, z);
    return cloudGroup;
}

const clouds = [];
const cloudCoords = [
    { x: -200, y: 140, z: -300, s: 1.5 },
    { x: 100, y: 160, z: -400, s: 1.8 },
    { x: 300, y: 150, z: -150, s: 1.4 },
    { x: -350, y: 170, z: 100, s: 1.6 },
    { x: 50, y: 180, z: 250, s: 1.7 }
];
cloudCoords.forEach(c => {
    const cloud = createCloud(c.x, c.y, c.z, c.s);
    scene.add(cloud);
    clouds.push(cloud);
});

// 1C-2. Low-Lying Coastal Horizon Clouds (Hugging distant shoreline hills & mountains)
function createHorizonCloud(x, y, z, scale = 1.0) {
    const horizonGroup = new THREE.Group();
    const mistMat = new THREE.MeshLambertMaterial({
        color: 0xf0f4f8,
        transparent: true,
        opacity: 0.65
    });

    // Elongated horizontal stratus cloud bank
    const stratusPuffs = [
        { x: 0, y: 0, z: 0, rx: 35, ry: 10, rz: 15 },
        { x: 25, y: 3, z: 5, rx: 28, ry: 9, rz: 12 },
        { x: -25, y: -2, z: -4, rx: 30, ry: 8, rz: 14 },
        { x: 50, y: 1, z: 2, rx: 22, ry: 7, rz: 10 },
        { x: -50, y: 2, z: -3, rx: 24, ry: 8, rz: 11 }
    ];

    stratusPuffs.forEach(p => {
        const pGeo = new THREE.SphereGeometry(1, 16, 16);
        pGeo.scale(p.rx * scale, p.ry * scale, p.rz * scale);
        const puff = new THREE.Mesh(pGeo, mistMat);
        puff.position.set(p.x * scale, p.y * scale, p.z * scale);
        horizonGroup.add(puff);
    });

    horizonGroup.position.set(x, y, z);
    return horizonGroup;
}

// Generate 360-degree ring of low horizon clouds clinging to outer land mass (r ~ 450-600m, height 25-45m)
for (let angle = 0; angle < Math.PI * 2; angle += 0.45) {
    const dist = 450 + (Math.sin(angle * 4) * 50) + (Math.random() * 60);
    const hx = Math.cos(angle) * dist;
    const hz = Math.sin(angle) * dist;
    const hy = 25 + Math.random() * 20; // Low right above land horizon!

    const hCloud = createHorizonCloud(hx, hy, hz, 1.2 + Math.random() * 0.8);
    scene.add(hCloud);
    clouds.push(hCloud);
}

// 1D. Flying Seagull Flock with Wing-Flap Animation
function createSeagull() {
    const birdGroup = new THREE.Group();
    const wingMat = new THREE.MeshStandardMaterial({ color: 0x222222, side: THREE.DoubleSide });

    const leftWing = new THREE.Mesh(new THREE.PlaneGeometry(1.4, 0.5), wingMat);
    leftWing.position.set(-0.7, 0, 0);
    leftWing.rotation.y = -0.2;
    leftWing.name = 'leftWing';

    const rightWing = new THREE.Mesh(new THREE.PlaneGeometry(1.4, 0.5), wingMat);
    rightWing.position.set(0.7, 0, 0);
    rightWing.rotation.y = 0.2;
    rightWing.name = 'rightWing';

    birdGroup.add(leftWing, rightWing);
    return birdGroup;
}

const flock = [];
for (let b = 0; b < 9; b++) {
    const bird = createSeagull();
    bird.userData = {
        radius: 80 + Math.random() * 90,
        height: 50 + Math.random() * 35,
        speed: 0.008 + Math.random() * 0.005,
        angle: Math.random() * Math.PI * 2,
        phase: Math.random() * Math.PI * 2
    };
    scene.add(bird);
    flock.push(bird);
}

// 2. Circular Lake Water Body. Uses a bigger pre-scale reference (350, was
// 300) than the marina/island layout so the lake grows slightly relative to
// the marina footprint below — the marina's farthest pier corner sits at
// ~126.6m from origin at this WORLD_SCALE, so this leaves it a real margin
// inside the shore instead of poking through it (confirmed by hand: old
// 300 reference put that same corner just OUTSIDE the shoreline).
const LAKE_RADIUS = 350.0 * WORLD_SCALE;
const waterGeo = new THREE.RingGeometry(0.001, LAKE_RADIUS, 128, 32); // High-density mesh for fluid wave motion

const waterMat = new THREE.MeshPhongMaterial({
    color: 0x003d73,       // Deep ocean blue with rich base tone
    emissive: 0x001020,    // Deep water shadow
    specular: 0x88d4ff,    // Bright ocean water specular reflection
    shininess: 80,         // High gloss for wave facet highlights
    transparent: true,
    opacity: 0.92,
    flatShading: true,     // Explicitly render wave facets so wave shapes are crisp & visible!
    side: THREE.DoubleSide
});
const waterMesh = new THREE.Mesh(waterGeo, waterMat);
waterMesh.rotation.x = -Math.PI / 2;
waterMesh.receiveShadow = true;
scene.add(waterMesh);

// 3. Surrounding Shoreline Land Mass & Forest (Trees strictly on land!)
// Outer Sandy Beach Ring, tied directly to LAKE_RADIUS (previously a fixed
// 295-330 left over from before WORLD_SCALE existed — that stranded the
// beach/land ring far outside the actual water body, leaving a bare gap
// between the lake edge and the shore with nothing rendered in it).
const beachGeo = new THREE.RingGeometry(LAKE_RADIUS - 5, LAKE_RADIUS + 15, 64);
const beachMat = new THREE.MeshLambertMaterial({ color: 0xd2b48c, side: THREE.DoubleSide });
const beachMesh = new THREE.Mesh(beachGeo, beachMat);
beachMesh.rotation.x = -Math.PI / 2;
beachMesh.position.y = -0.05;
scene.add(beachMesh);

// Surrounding Green Forest Hills Land Mass — starts right where the beach
// ends, also tied to LAKE_RADIUS for the same reason.
const landGeo = new THREE.RingGeometry(LAKE_RADIUS + 13, LAKE_RADIUS * 4, 64);
const landMat = new THREE.MeshLambertMaterial({ color: 0x3e5c26, side: THREE.DoubleSide });
const landMesh = new THREE.Mesh(landGeo, landMat);
landMesh.rotation.x = -Math.PI / 2;
landMesh.position.y = -0.1;
scene.add(landMesh);

// Helper function to build 3D Pine Trees
function createTree(x, z, scale = 1.0) {
    const treeGroup = new THREE.Group();
    // Trunk
    const trunkGeo = new THREE.CylinderGeometry(0.3 * scale, 0.5 * scale, 3.5 * scale, 8);
    const trunkMat = new THREE.MeshLambertMaterial({ color: 0x5c4033 });
    const trunkMesh = new THREE.Mesh(trunkGeo, trunkMat);
    trunkMesh.position.y = (3.5 * scale) / 2;
    treeGroup.add(trunkMesh);

    // Pine Foliage Cones
    const foliageMat = new THREE.MeshLambertMaterial({ color: 0x1e4d2b });
    const c1 = new THREE.Mesh(new THREE.ConeGeometry(2.2 * scale, 4 * scale, 8), foliageMat);
    c1.position.y = 3.5 * scale;
    const c2 = new THREE.Mesh(new THREE.ConeGeometry(1.7 * scale, 3 * scale, 8), foliageMat);
    c2.position.y = 5.0 * scale;
    treeGroup.add(c1, c2);

    treeGroup.position.set(x, 0, z);
    return treeGroup;
}

// Generate Trees STRICTLY on outer land terrain, starting right at the land
// ring's inner edge (tied to LAKE_RADIUS, same reasoning as beachGeo/landGeo
// above — a fixed radius here would leave trees stranded far from the
// now-smaller lake).
for (let angle = 0; angle < Math.PI * 2; angle += 0.05) {
    const r = (LAKE_RADIUS + 15) + Math.sin(angle * 6) * 9 + Math.random() * 37;
    const tx = Math.cos(angle) * r;
    const tz = Math.sin(angle) * r;
    scene.add(createTree(tx, tz, 1.2 + Math.random() * 1.0));
}

// Core Land Obstacle: Scenic Central Island in Ocean Bay. Placed dead center
// of the lake (was offset toward the marina corner) per explicit request
// that the island sit in the middle rather than off to one side.
const ISLAND_X = 0.0;
const ISLAND_Y = 0.0;
const ISLAND_RADIUS = 25.0;
const ISLAND_KEEP_OUT = ISLAND_RADIUS + 8.0; // 33.0m keep-out buffer (Guaranteed clearance, NO island crashes!)

const islandGroup = new THREE.Group();
const islandTerrainGeo = new THREE.CylinderGeometry(22, 26, 2, 32);
const islandTerrainMat = new THREE.MeshLambertMaterial({ color: 0x3a6024 }); // Grass
const islandTerrain = new THREE.Mesh(islandTerrainGeo, islandTerrainMat);
islandTerrain.position.y = 0.4;
islandGroup.add(islandTerrain);

// Island Sand Border
const islandSandGeo = new THREE.CylinderGeometry(26.5, 30, 1.2, 32);
const islandSandMat = new THREE.MeshLambertMaterial({ color: 0xc2b280 });
const islandSand = new THREE.Mesh(islandSandGeo, islandSandMat);
islandSand.position.y = 0.0;
islandGroup.add(islandSand);

islandGroup.add(createTree(4, 5, 1.2));
islandGroup.add(createTree(-6, -4, 1.1));
islandGroup.add(createTree(2, -8, 1.3));
islandGroup.position.set(ISLAND_X, 0, -ISLAND_Y);
scene.add(islandGroup);

// Canonical marina dock geometry — single source of truth for the pier +
// 3 finger jetties, read by the 3D mesh, the 2D map draw, the hard hull
// collision (isTouchingObstacleAt), the pathfinder (isInsideMarinaStructure),
// and the Mode 3 dynamic-docking click handler. These used to be four/five
// separately hand-copied literal numbers that drifted out of sync — the 3D
// mesh and 2D map each scaled position by WORLD_SCALE but left their SIZE
// literals unscaled, rendering a pier/jetty far bigger than what the
// collision/pathfinding system (which scaled both consistently) actually
// protected — "the ship flows on top of the wood." Fixing that here, once.
const MARINA_PIER_X_MIN = -265.0 * WORLD_SCALE;
const MARINA_PIER_X_MAX = -125.0 * WORLD_SCALE;
const MARINA_PIER_Y = -173.0 * WORLD_SCALE;    // north face, toward open water
const MARINA_PIER_THICKNESS = 4.0;             // unscaled — a real structural thickness, not a layout span (same reasoning as JETTY_HALF_WIDTH below)
const JETTY_X_LIST = [-245.0, -195.0, -145.0].map(x => x * WORLD_SCALE);
const JETTY_HALF_WIDTH = 2.0;                  // unscaled — matches the existing hull-collision margin
const JETTY_Y_NEAR = -115.0 * WORLD_SCALE;     // open-water mouth
const JETTY_Y_FAR = -175.0 * WORLD_SCALE;      // pier-connected end (slightly past MARINA_PIER_Y so there's no gap)

// 3B. Ultra-Packed Real Marina Grid (30+ Vessels Packed Side-by-Side with 40cm Ultra-Small Tolerances)
const marinaGroup = new THREE.Group();

// Main Horizontal Floating Spine Pier — size AND position both derived from
// the canonical MARINA_PIER_* constants above, so this mesh can no longer
// drift out of sync with the collision/pathfinding geometry the way the old
// hardcoded BoxGeometry(130, 0.8, 6) did (that 130/6 were never scaled by
// WORLD_SCALE while the position was, rendering a pier far bigger than what
// actually blocked the boat).
const pierMat = new THREE.MeshLambertMaterial({ color: 0x5d4037, roughness: 0.8 });
const pierLength = MARINA_PIER_X_MAX - MARINA_PIER_X_MIN;
const pierCenterX = (MARINA_PIER_X_MIN + MARINA_PIER_X_MAX) / 2;
const pierCenterY = MARINA_PIER_Y - MARINA_PIER_THICKNESS / 2; // ROS y (south of the north face)
const mainPier = new THREE.Mesh(new THREE.BoxGeometry(pierLength, 0.8, MARINA_PIER_THICKNESS), pierMat);
mainPier.position.set(pierCenterX, 0.4, -pierCenterY); // Three.js z = -ROS y
marinaGroup.add(mainPier);

// 3 Vertical Finger Jetties extending perpendicularly up into water — same
// derive-from-constants fix as the pier above (was BoxGeometry(4, 0.8, 62),
// a real 62m-long jetty when the collision system only protected 24m).
const jettyWidth = JETTY_HALF_WIDTH * 2;
const jettyLength = JETTY_Y_NEAR - JETTY_Y_FAR;
const jettyCenterY = (JETTY_Y_NEAR + JETTY_Y_FAR) / 2; // ROS y
JETTY_X_LIST.forEach(jx => {
    const jetty = new THREE.Mesh(new THREE.BoxGeometry(jettyWidth, 0.8, jettyLength), pierMat);
    jetty.position.set(jx, 0.4, -jettyCenterY);
    marinaGroup.add(jetty);

    // Mooring Pylons along each finger jetty, spanning its corrected length
    for (let pz = -JETTY_Y_NEAR; pz <= -JETTY_Y_FAR; pz += 6.5 * WORLD_SCALE) {
        const pylonL = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.3, 4.5, 8), new THREE.MeshLambertMaterial({ color: 0x3e2723 }));
        pylonL.position.set(jx - 1.8, 0.9, pz);
        const pylonR = pylonL.clone();
        pylonR.position.x = jx + 1.8;
        marinaGroup.add(pylonL, pylonR);
    }
});

// Helper to spawn realistic moored yachts, sailboats, & speedboats along berths
function createMooredBoat(x, z, hullColor, boatType = 'yacht') {
    const bGroup = new THREE.Group();
    const isSailboat = boatType === 'sailboat';
    const isSmall = boatType === 'speedboat';

    const bLen = isSmall ? 3.0 : 3.6;
    const bWid = isSmall ? 1.25 : 1.45;

    const hull = new THREE.Mesh(new THREE.BoxGeometry(bLen, 0.65, bWid), new THREE.MeshStandardMaterial({ color: hullColor, roughness: 0.2 }));
    hull.position.y = 0.2;
    bGroup.add(hull);

    if (isSailboat) {
        // Tall Sailboat Mast
        const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.07, 7.5, 8), new THREE.MeshStandardMaterial({ color: 0xdddddd }));
        mast.position.set(0, 3.8, 0);
        bGroup.add(mast);
    } else {
        // Cabin Superstructure
        const cabin = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.7, 1.05), new THREE.MeshStandardMaterial({ color: 0xffffff }));
        cabin.position.set(-0.2, 0.7, 0);
        bGroup.add(cabin);
    }

    bGroup.position.set(x, 0.2, z);
    bGroup.rotation.y = 0; // Parked horizontally into berth

    // Register physical collision obstacle (isParkedShip avoids yellow buoy rings)
    entities.push({ id: 'moored_' + x + '_' + z, type: 'static', isParkedShip: true, ros_x: x, ros_y: -z });
    return bGroup;
}

// Populate Jetty 1 (Left Jetty at x: -245.0) - Densely Packed Side-by-Side (z: 168 to 120)
const zList = [168, 161.5, 155, 148.5, 142, 135.5, 129, 122.5].map(z => z * WORLD_SCALE);
const colorsList = [0x1d3557, 0x2a9d8f, 0xe63946, 0x457b9d, 0x0f4c5c, 0x3d5a80, 0x9b5de5, 0xf15bb5];
const typesList = ['yacht', 'sailboat', 'speedboat', 'yacht', 'sailboat', 'speedboat', 'yacht', 'sailboat'];

// Populate Jetty 1 (Left Jetty at x: -245.0) - Open Berth #1 at z: 142.0.
// Thinned to every other slot (idx % 2 === 0), on top of the existing named
// berth gap, per explicit request for more open choices — the marina used
// to leave only the 4 named availableBerths open with every other slot
// packed solid.
zList.forEach((z, idx) => {
    if (idx % 2 !== 0) return;
    if (Math.abs(z - 142.0 * WORLD_SCALE) > 3.0) {
        marinaGroup.add(createMooredBoat(-251 * WORLD_SCALE, z, colorsList[idx % colorsList.length], typesList[idx % typesList.length]));
    }
    marinaGroup.add(createMooredBoat(-239 * WORLD_SCALE, z, colorsList[(idx + 2) % colorsList.length], typesList[(idx + 1) % typesList.length]));
});

// Populate Jetty 2 (Middle Jetty at x: -195.0) - Open Berth #2 at z: 142.2 (Tight 40cm Gap) and Open Berth #3 at z: 142.2 (Right Side)
// Thinned to every other hand-placed slot, same reasoning as Jetty 1 above.
marinaGroup.add(createMooredBoat(-201 * WORLD_SCALE, 168 * WORLD_SCALE, 0xf8f9fa, 'yacht'));
marinaGroup.add(createMooredBoat(-201 * WORLD_SCALE, 155 * WORLD_SCALE, 0x0f4c5c, 'speedboat'));

// ===> OPEN BERTH #2: X: -201.0, Y: -142.2 (z = 142.2) <=== (ROS coords, pre-WORLD_SCALE)

marinaGroup.add(createMooredBoat(-201 * WORLD_SCALE, 135.9 * WORLD_SCALE, 0x1d3557, 'yacht')); // BOTTOM BOUNDARY OF BERTH #2
marinaGroup.add(createMooredBoat(-201 * WORLD_SCALE, 122.9 * WORLD_SCALE, 0x457b9d, 'speedboat'));

// Right side berths of Jetty 2 - Open Berth #3 at z: 142.2 — thinned to every other slot
zList.forEach((z, idx) => {
    if (idx % 2 !== 0) return;
    if (Math.abs(z - 142.2 * WORLD_SCALE) > 3.0) {
        marinaGroup.add(createMooredBoat(-189 * WORLD_SCALE, z, colorsList[(idx + 3) % colorsList.length], typesList[idx % typesList.length]));
    }
});

// Populate Jetty 3 (Right Jetty at x: -145.0) - Open Berth #4 at z: 142.0 — thinned to every other slot
zList.forEach((z, idx) => {
    if (idx % 2 !== 0) return;
    if (Math.abs(z - 142.0 * WORLD_SCALE) > 3.0) {
        marinaGroup.add(createMooredBoat(-139 * WORLD_SCALE, z, colorsList[(idx + 1) % colorsList.length], typesList[(idx + 2) % typesList.length]));
    }
});

// Dynamic 3D Target Beacon Light for clicked berth
const bGeo = new THREE.TorusGeometry(1.6, 0.2, 16, 100);
const bMat = new THREE.MeshBasicMaterial({ color: 0x00ffcc, transparent: true, opacity: 0.8 });
const dynamic3DBerthMesh = new THREE.Mesh(bGeo, bMat);
dynamic3DBerthMesh.rotation.x = Math.PI / 2;
dynamic3DBerthMesh.position.set(-220, -10, 0); // Hide offscreen initially
const bLight = new THREE.PointLight(0x00ffcc, 2, 20);
bLight.position.set(0, 2, 0);
dynamic3DBerthMesh.add(bLight);
marinaGroup.add(dynamic3DBerthMesh);

// Harbor Master Control Office & Lighthouse Beacon on Spine Pier
const officeMesh = new THREE.Mesh(
    new THREE.BoxGeometry(10, 6, 8),
    new THREE.MeshStandardMaterial({ color: 0xf1faee, roughness: 0.3 })
);
officeMesh.position.set(-260 * WORLD_SCALE, 3.8, 175 * WORLD_SCALE);

const officeRoof = new THREE.Mesh(
    new THREE.ConeGeometry(7, 3, 4),
    new THREE.MeshStandardMaterial({ color: 0xe63946 })
);
officeRoof.rotation.y = Math.PI / 4;
officeRoof.position.set(-260 * WORLD_SCALE, 8.3, 175 * WORLD_SCALE);

const beaconLight = new THREE.PointLight(0x00ffff, 2.5, 30);
beaconLight.position.set(-260 * WORLD_SCALE, 9.0, 175 * WORLD_SCALE);
marinaGroup.add(officeMesh, officeRoof, beaconLight);

// Coastal Town Villas & City Buildings, nestled just past the beach on the
// NE shore (r ~= 165-200m from origin). Previously fixed at r ~= 305-330m —
// a leftover from before WORLD_SCALE existed, left behind when the lake
// shrank so the "coastal" city ended up nowhere near the actual coast.
// Repositioned (not just rescaled) to sit right at the new, smaller
// shoreline instead, on the opposite side of the lake from the marina.
const bldgColors = [0xfaf0e6, 0xdfc09f, 0xe8d8c8, 0xd7ccc8, 0xc05a46, 0xefebe9];
const bldgPositions = [
    { x: 120, z: -115, w: 16, h: 20, d: 14, colorIdx: 0 },
    { x: 140, z: -123, w: 20, h: 26, d: 18, colorIdx: 1 },
    { x: 112, z: -133, w: 15, h: 18, d: 14, colorIdx: 2 },
    { x: 150, z: -111, w: 18, h: 22, d: 16, colorIdx: 3 },
    { x: 124, z: -143, w: 16, h: 20, d: 14, colorIdx: 4 },
    { x: 160, z: -119, w: 22, h: 28, d: 20, colorIdx: 5 },
    { x: 142, z: -139, w: 18, h: 24, d: 16, colorIdx: 0 }
];

bldgPositions.forEach(b => {
    // Villa Wall
    const bldgMat = new THREE.MeshStandardMaterial({ color: bldgColors[b.colorIdx], roughness: 0.5 });
    const bldgMesh = new THREE.Mesh(new THREE.BoxGeometry(b.w, b.h, b.d), bldgMat);
    bldgMesh.position.set(b.x, b.h / 2 - 0.1, b.z);
    bldgMesh.castShadow = true;
    marinaGroup.add(bldgMesh);

    // Terracotta Pitched Roof
    const roofMat = new THREE.MeshStandardMaterial({ color: 0x8d3c1b, roughness: 0.4 }); // Terracotta tile
    const roofGeo = new THREE.ConeGeometry(Math.max(b.w, b.d) * 0.75, 3.5, 4);
    const roofMesh = new THREE.Mesh(roofGeo, roofMat);
    roofMesh.rotation.y = Math.PI / 4;
    roofMesh.position.set(b.x, b.h + 1.7, b.z);
    marinaGroup.add(roofMesh);
});

scene.add(marinaGroup);

// 4. High-Detail ASV Vessel Model (Main Boat) — twin-hull catamaran styled
// after the real WAM-V: black cylindrical pontoons with an orange deck/trim
// (colors taken from the actual WAM-V texture), linked by an open frame
// rather than a solid slab, matching its real look instead of a plain box hull.
const boatGroup = new THREE.Group();
const HULL_Z_OFFSET = 0.55; // distance from centerline to each hull's centerline
const HULL_RADIUS = 0.28;

const hullGeo = new THREE.CylinderGeometry(HULL_RADIUS, HULL_RADIUS, 3.0, 16);
const hullMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.4, metalness: 0.3 });
const bowGeo = new THREE.ConeGeometry(HULL_RADIUS, 1.0, 16);
const accentMat = new THREE.MeshStandardMaterial({ color: 0xff7a1a, roughness: 0.4 });
const engineGeo = new THREE.BoxGeometry(0.5, 0.8, 0.35);
const engineMat = new THREE.MeshStandardMaterial({ color: 0x111111, metalness: 0.8 });

// Port + Starboard Pontoon Hulls (each with a tapered bow and stern engine)
[HULL_Z_OFFSET, -HULL_Z_OFFSET].forEach(z => {
    const hullMesh = new THREE.Mesh(hullGeo, hullMat);
    hullMesh.rotation.z = Math.PI / 2;
    hullMesh.position.set(-0.1, 0.2, z);
    hullMesh.castShadow = true;
    boatGroup.add(hullMesh);

    const bowMesh = new THREE.Mesh(bowGeo, hullMat);
    bowMesh.rotation.z = -Math.PI / 2;
    bowMesh.position.set(1.9, 0.2, z);
    boatGroup.add(bowMesh);

    const engMesh = new THREE.Mesh(engineGeo, engineMat);
    engMesh.position.set(-1.8, 0.2, z);
    boatGroup.add(engMesh);
});

// Open Frame Bridging the Two Hulls (crossbeams, not a solid deck)
const beamGeo = new THREE.CylinderGeometry(0.05, 0.05, HULL_Z_OFFSET * 2 + 0.2, 8);
[1.0, -1.0].forEach(x => {
    const beamMesh = new THREE.Mesh(beamGeo, accentMat);
    beamMesh.rotation.x = Math.PI / 2;
    beamMesh.position.set(x, 0.55, 0);
    boatGroup.add(beamMesh);
});

// Flat Equipment Deck on top of the frame
const deckGeo = new THREE.BoxGeometry(1.85, 0.08, HULL_Z_OFFSET * 2 - 0.1);
const deckMesh = new THREE.Mesh(deckGeo, accentMat);
deckMesh.position.set(0, 0.62, 0);
boatGroup.add(deckMesh);

// Electronics/Sensor Pod
const podGeo = new THREE.BoxGeometry(1.0, 0.5, 0.9);
const podMat = new THREE.MeshStandardMaterial({ color: 0xdddddd, roughness: 0.3 });
const podMesh = new THREE.Mesh(podGeo, podMat);
podMesh.position.set(-0.2, 0.95, 0);
boatGroup.add(podMesh);

// Navigation Arch & Lidar Mast
const archGeo = new THREE.CylinderGeometry(0.04, 0.04, 1.2);
const archMat = new THREE.MeshStandardMaterial({ color: 0x888888, metalness: 0.9 });
const mast1 = new THREE.Mesh(archGeo, archMat); mast1.position.set(-0.8, 1.4, 0.5);
const mast2 = new THREE.Mesh(archGeo, archMat); mast2.position.set(-0.8, 1.4, -0.5);
const topBar = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.1, 1.2), archMat);
topBar.position.set(-0.8, 2.0, 0);
boatGroup.add(mast1, mast2, topBar);

// Rotating Radar Dome
let radarMesh;
const radarGeo = new THREE.CylinderGeometry(0.4, 0.4, 0.15, 16);
const radarMat = new THREE.MeshStandardMaterial({ color: 0xffffff });
radarMesh = new THREE.Mesh(radarGeo, radarMat);
radarMesh.position.set(-0.8, 2.15, 0);
boatGroup.add(radarMesh);

scene.add(boatGroup);

// 3D Entity Tracker Map (High-Detail Obstacles)
const threeEntities = new Map();

function sync3DEntities() {
    entities.forEach(ent => {
        if (ent.isParkedShip) return; // Skip rendering yellow buoy rings for parked quay ships!
        if (!threeEntities.has(ent.id)) {
            let mesh;
            if (ent.type === 'static') {
                // High-detail Nautical Buoy
                const buoyGroup = new THREE.Group();
                const buoyGeo = new THREE.CylinderGeometry(0.55, 0.4, 2.2, 16);
                const buoyMat = new THREE.MeshLambertMaterial({ color: 0xff8c00 }); // Nautical Orange
                const buoyMesh = new THREE.Mesh(buoyGeo, buoyMat);
                buoyGroup.add(buoyMesh);

                // Dotted Safety Keep-Out Ring in 3D
                const ringGeo = new THREE.RingGeometry(5.8, 6.0, 32);
                const ringMat = new THREE.MeshBasicMaterial({ color: 0xffa500, side: THREE.DoubleSide, transparent: true, opacity: 0.6 });
                const ringMesh = new THREE.Mesh(ringGeo, ringMat);
                ringMesh.rotation.x = Math.PI / 2;
                ringMesh.position.y = -0.4;
                buoyGroup.add(ringMesh);

                // Blinking Beacon Light
                const light = new THREE.PointLight(0xff0000, 1.2, 12);
                light.position.set(0, 1.4, 0);
                buoyGroup.add(light);

                mesh = buoyGroup;
            } else if (ent.type === 'dynamic') {
                // Ultra-Realistic Dynamic Patrol Vessel Model
                const obsBoatGroup = new THREE.Group();

                // 1. Hydrodynamic Sleek V-Hull (Deep Navy / Marine Teal)
                const hullMat = new THREE.MeshStandardMaterial({ color: 0x0f4c5c, roughness: 0.15, metalness: 0.3 });
                const hullMesh = new THREE.Mesh(new THREE.BoxGeometry(3.8, 0.7, 1.5), hullMat);
                hullMesh.position.set(0, 0.2, 0);
                obsBoatGroup.add(hullMesh);

                // 2. Red Anti-Fouling Waterline Stripe
                const stripeMat = new THREE.MeshStandardMaterial({ color: 0xc1121f, roughness: 0.3 });
                const stripeMesh = new THREE.Mesh(new THREE.BoxGeometry(3.82, 0.22, 1.52), stripeMat);
                stripeMesh.position.set(0, 0.05, 0);
                obsBoatGroup.add(stripeMesh);

                // 3. Tapered Pointed Bow Nose
                const bowGeo = new THREE.ConeGeometry(0.75, 1.3, 4);
                const bowMesh = new THREE.Mesh(bowGeo, hullMat);
                bowMesh.rotation.x = Math.PI / 2;
                bowMesh.rotation.z = -Math.PI / 2;
                bowMesh.position.set(2.5, 0.2, 0);
                obsBoatGroup.add(bowMesh);

                // 4. White Superstructure Cabin Bridge & Tinted Glass
                const cabinBase = new THREE.Mesh(
                    new THREE.BoxGeometry(1.6, 0.85, 1.2),
                    new THREE.MeshStandardMaterial({ color: 0xf8f9fa, roughness: 0.1 })
                );
                cabinBase.position.set(-0.2, 0.825, 0);
                obsBoatGroup.add(cabinBase);

                // Tinted Glass Windshield
                const glassMat = new THREE.MeshStandardMaterial({ color: 0x1d2d44, metalness: 0.9, roughness: 0.05, transparent: true, opacity: 0.85 });
                const glassMesh = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.5, 1.15), glassMat);
                glassMesh.position.set(0.2, 0.98, 0);
                obsBoatGroup.add(glassMesh);

                // 5. Radar Arch & Rotating Radar Antenna Bar
                const archMat = new THREE.MeshStandardMaterial({ color: 0xd1d5db, metalness: 0.8, roughness: 0.2 });
                const mastLeft = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.9), archMat);
                mastLeft.position.set(-0.8, 1.45, 0.45);
                const mastRight = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.9), archMat);
                mastRight.position.set(-0.8, 1.45, -0.45);
                const archTop = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.08, 1.0), archMat);
                archTop.position.set(-0.8, 1.9, 0);
                obsBoatGroup.add(mastLeft, mastRight, archTop);

                // Rotating Radar Scanner
                const radarBar = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.08, 0.15), new THREE.MeshStandardMaterial({ color: 0xffffff }));
                radarBar.position.set(-0.8, 2.0, 0);
                radarBar.name = 'radarBar';
                obsBoatGroup.add(radarBar);

                // 6. Stainless Steel Bow Guard Railings
                const railMat = new THREE.MeshStandardMaterial({ color: 0xe5e7eb, metalness: 0.9, roughness: 0.1 });
                const railLeft = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 1.8), railMat);
                railLeft.rotation.z = Math.PI / 2;
                railLeft.position.set(1.4, 0.7, 0.7);
                const railRight = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 1.8), railMat);
                railRight.rotation.z = Math.PI / 2;
                railRight.position.set(1.4, 0.7, -0.7);
                obsBoatGroup.add(railLeft, railRight);

                // 7. Orange LifeBuoy Rescue Ring on Cabin Side
                const buoyRingGeo = new THREE.TorusGeometry(0.18, 0.05, 8, 16);
                const buoyRingMat = new THREE.MeshLambertMaterial({ color: 0xff5722 });
                const buoyRing = new THREE.Mesh(buoyRingGeo, buoyRingMat);
                buoyRing.position.set(-0.2, 0.9, 0.62);
                buoyRing.rotation.y = Math.PI / 2;
                obsBoatGroup.add(buoyRing);

                // 8. Navigation Lights (Port/Red on left, Starboard/Green on right)
                const portLight = new THREE.Mesh(
                    new THREE.SphereGeometry(0.08, 8, 8),
                    new THREE.MeshBasicMaterial({ color: 0xff0000 })
                );
                portLight.position.set(0.5, 1.05, 0.6);
                const stbdLight = new THREE.Mesh(
                    new THREE.SphereGeometry(0.08, 8, 8),
                    new THREE.MeshBasicMaterial({ color: 0x00ff00 })
                );
                stbdLight.position.set(0.5, 1.05, -0.6);
                obsBoatGroup.add(portLight, stbdLight);

                // 9. Dual Black Outboard Engines at Stern
                const engineMat = new THREE.MeshStandardMaterial({ color: 0x111111, metalness: 0.8 });
                const eng1 = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.85, 0.35), engineMat);
                eng1.position.set(-2.1, 0.2, 0.4);
                const eng2 = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.85, 0.35), engineMat);
                eng2.position.set(-2.1, 0.2, -0.4);
                obsBoatGroup.add(eng1, eng2);

                // 10. Dynamic Trailing White Water Propeller Wake
                const wakeGeo = new THREE.PlaneGeometry(3.5, 1.8);
                const wakeMat = new THREE.MeshBasicMaterial({
                    color: 0xffffff,
                    transparent: true,
                    opacity: 0.45,
                    side: THREE.DoubleSide
                });
                const wakeMesh = new THREE.Mesh(wakeGeo, wakeMat);
                wakeMesh.rotation.x = -Math.PI / 2;
                wakeMesh.position.set(-3.6, -0.38, 0);
                obsBoatGroup.add(wakeMesh);

                mesh = obsBoatGroup;
            } else if (ent.type === 'goal') {
                const goalGeo = new THREE.TorusGeometry(1.5, 0.2, 16, 100);
                const goalMat = new THREE.MeshBasicMaterial({ color: 0x00ff00, transparent: true, opacity: 0.8 });
                mesh = new THREE.Mesh(goalGeo, goalMat);
                mesh.rotation.x = Math.PI / 2;
                const glow = new THREE.PointLight(0x00ff00, 2, 20);
                glow.position.set(0, 2, 0);
                mesh.add(glow);
            }
            mesh.position.set(ent.ros_x, 0.5, -ent.ros_y);
            scene.add(mesh);
            threeEntities.set(ent.id, mesh);
        }
    });
}

// ================= UNIFIED A* + STRING-PULLING PATHFINDER ================= //
const SAFETY_RADIUS = 5.5; // Restrictive imaginary keep-out radius for buoys/boats (meters)
const SHORE_MARGIN = 4.0;  // Keep-out band inside LAKE_RADIUS, off-limits to route planning (beach/greenery)

// Real marina dock structure (main spine pier + 3 finger jetties). This is
// the single source of truth for "is this point solid dock" — used by BOTH
// the direct-line fast path AND the grid search below, so the two can never
// disagree about whether a given spot is water or structure (they used to:
// one checked this exact rectangular shape, the other blocked a plain circle
// of a different, unit-mismatched size around the marina's center).
function isInsideMarinaStructure(x, y) {
    // Main Spine Pier Wall — canonical geometry (MARINA_PIER_* consts).
    if (y <= MARINA_PIER_Y && x >= MARINA_PIER_X_MIN && x <= MARINA_PIER_X_MAX) return true;
    // 3 Finger Jetties (2.5m clearance around each jetty centerline, y = -115 to -175)
    // 5.5m half-width, not the jetty's own 2.0m — path planning treats the
    // boat as a point, but isTouchingObstacle()'s real hull-collision check
    // below adds the boat's actual footprint (up to ~2.9m half-extent when
    // diagonal to the jetty, e.g. mid-turn) on top of the jetty's physical
    // width. A planned route that only kept a point 2.5m off the centerline
    // left the real hull enough to clip the jetty while turning even though
    // the plan itself was "clear" — this matches SAFETY_RADIUS (used for
    // every other obstacle) instead of understating it.
    for (const jx of JETTY_X_LIST) {
        if (Math.abs(x - jx) < 5.5 && y <= JETTY_Y_NEAR && y >= JETTY_Y_FAR) return true;
    }
    return false;
}

// Single obstacle predicate for the whole pathfinder: outside the navigable
// lake (shoreline/greenery), the island, the marina structure, or within
// SAFETY_RADIUS of any live obstacle (buoys, moving boats, and the ~30
// baked-in moored boats, all passed in via `obstacles`).
function isPointBlocked(x, y, obstacles) {
    if (Math.hypot(x, y) > LAKE_RADIUS - SHORE_MARGIN) return true;
    if (Math.hypot(x - ISLAND_X, y - ISLAND_Y) < ISLAND_KEEP_OUT) return true;
    if (isInsideMarinaStructure(x, y)) return true;
    for (const obs of obstacles) {
        if (Math.hypot(x - obs.ros_x, y - obs.ros_y) < SAFETY_RADIUS) return true;
    }
    return false;
}

// Segment-blocked check, sampled at sub-grid resolution against the SAME
// predicate the grid search uses — this is what keeps the "is a straight
// line clear?" fast path and the A* grid in agreement.
function isSegmentBlocked(p1, p2, obstacles) {
    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    const len = Math.hypot(dx, dy);
    if (len === 0) return isPointBlocked(p1.x, p1.y, obstacles);
    const steps = Math.max(1, Math.ceil(len / 1.0)); // sample every ~1m
    for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        if (isPointBlocked(p1.x + t * dx, p1.y + t * dy, obstacles)) return true;
    }
    return false;
}

// Minimal binary min-heap for A*'s open set — the old BFS used a plain
// array with .shift() as its queue, which is O(n) per pop (O(n^2) overall
// on the ~140x140 cell grid) and was slow enough, re-run every 250ms during
// Mode 1's live replanning, to visibly stutter the frame loop.
class MinHeap {
    constructor() { this.items = []; }
    get size() { return this.items.length; }
    push(item, priority) {
        this.items.push({ item, priority });
        let i = this.items.length - 1;
        while (i > 0) {
            const parent = (i - 1) >> 1;
            if (this.items[parent].priority <= this.items[i].priority) break;
            [this.items[parent], this.items[i]] = [this.items[i], this.items[parent]];
            i = parent;
        }
    }
    pop() {
        const top = this.items[0];
        const last = this.items.pop();
        if (this.items.length > 0) {
            this.items[0] = last;
            let i = 0;
            while (true) {
                const l = 2 * i + 1, r = 2 * i + 2;
                let smallest = i;
                if (l < this.items.length && this.items[l].priority < this.items[smallest].priority) smallest = l;
                if (r < this.items.length && this.items[r].priority < this.items[smallest].priority) smallest = r;
                if (smallest === i) break;
                [this.items[smallest], this.items[i]] = [this.items[i], this.items[smallest]];
                i = smallest;
            }
        }
        return top ? top.item : undefined;
    }
}

function findOptimalPath(start, goal, obstacles) {
    const validObs = obstacles.filter(o => o.type !== 'goal');

    // Direct line check first (instant if unblocked)
    if (!isSegmentBlocked(start, goal, validObs)) {
        return [{ x: start.x, y: start.y }, { x: goal.x, y: goal.y }];
    }

    // 1. Setup Grid A* Parameters (Massive 600m Ocean Bay Pathfinder: 560m x 560m)
    const STEP = 4.0; // 4.0 meter grid cells for fast vector search
    const MIN_X = -280 * WORLD_SCALE, MAX_X = 280 * WORLD_SCALE, MIN_Y = -280 * WORLD_SCALE, MAX_Y = 280 * WORLD_SCALE;

    function toKey(gx, gy) { return `${gx},${gy}`; }
    function toWorld(gx, gy) { return { x: gx * STEP, y: gy * STEP }; }

    const startGx = Math.round(start.x / STEP);
    const startGy = Math.round(start.y / STEP);
    const goalGx = Math.round(goal.x / STEP);
    const goalGy = Math.round(goal.y / STEP);

    function heuristic(gx, gy) { return Math.hypot(gx - goalGx, gy - goalGy) * STEP; }

    // Blocked-cell lookups are memoized per search — isPointBlocked gets
    // called for every neighbor candidate, and neighbors are revisited often.
    const blockedCache = new Map();
    function cellBlocked(gx, gy) {
        const key = toKey(gx, gy);
        let cached = blockedCache.get(key);
        if (cached === undefined) {
            const w = toWorld(gx, gy);
            cached = isPointBlocked(w.x, w.y, validObs);
            blockedCache.set(key, cached);
        }
        return cached;
    }

    const neighbors = [
        { dx: 1, dy: 0, cost: STEP }, { dx: -1, dy: 0, cost: STEP },
        { dx: 0, dy: 1, cost: STEP }, { dx: 0, dy: -1, cost: STEP },
        { dx: 1, dy: 1, cost: STEP * Math.SQRT2 }, { dx: -1, dy: 1, cost: STEP * Math.SQRT2 },
        { dx: 1, dy: -1, cost: STEP * Math.SQRT2 }, { dx: -1, dy: -1, cost: STEP * Math.SQRT2 }
    ];

    // 2. A* Search (start cell is always allowed to leave from, even if it's
    // technically inside a keep-out — e.g. hugging close to an obstacle
    // already — matching the old BFS's explicit `blocked.delete(start)`.)
    const startKey = toKey(startGx, startGy);
    const gScore = new Map([[startKey, 0]]);
    const cameFrom = new Map();
    const closed = new Set();
    const open = new MinHeap();
    open.push({ gx: startGx, gy: startGy }, heuristic(startGx, startGy));

    let foundGoalKey = null;
    let bestKey = startKey;
    let bestH = heuristic(startGx, startGy);

    while (open.size > 0) {
        const curr = open.pop();
        const currKey = toKey(curr.gx, curr.gy);
        if (closed.has(currKey)) continue;
        closed.add(currKey);

        const h = heuristic(curr.gx, curr.gy);
        if (h < bestH) { bestH = h; bestKey = currKey; }

        if (curr.gx === goalGx && curr.gy === goalGy) {
            foundGoalKey = currKey;
            break;
        }

        for (const n of neighbors) {
            const ngx = curr.gx + n.dx;
            const ngy = curr.gy + n.dy;
            const nWorld = toWorld(ngx, ngy);
            if (nWorld.x < MIN_X || nWorld.x > MAX_X || nWorld.y < MIN_Y || nWorld.y > MAX_Y) continue;

            const nKey = toKey(ngx, ngy);
            if (closed.has(nKey) || cellBlocked(ngx, ngy)) continue;

            const tentativeG = gScore.get(currKey) + n.cost;
            if (!gScore.has(nKey) || tentativeG < gScore.get(nKey)) {
                gScore.set(nKey, tentativeG);
                cameFrom.set(nKey, currKey);
                open.push({ gx: ngx, gy: ngy }, tentativeG + heuristic(ngx, ngy));
            }
        }
    }

    // Fallback: goal unreachable (e.g. it's blocked) — route to the closest
    // node actually reached instead.
    if (!foundGoalKey) foundGoalKey = bestKey;

    // Reconstruct raw grid path
    let rawPath = [];
    let curr = foundGoalKey;
    while (curr) {
        const [gx, gy] = curr.split(',').map(Number);
        rawPath.unshift(toWorld(gx, gy));
        curr = cameFrom.get(curr);
    }
    if (rawPath.length === 0) {
        rawPath = [{ x: start.x, y: start.y }, { x: goal.x, y: goal.y }];
    }
    rawPath[0] = { x: start.x, y: start.y };

    // 3. String Pulling (Shortcut Optimization for smooth vector turns)
    const smoothPath = [rawPath[0]];
    let currIdx = 0;
    while (currIdx < rawPath.length - 1) {
        let farthest = currIdx + 1;
        for (let nextIdx = rawPath.length - 1; nextIdx > currIdx + 1; nextIdx--) {
            if (!isSegmentBlocked(rawPath[currIdx], rawPath[nextIdx], validObs)) {
                farthest = nextIdx;
                break;
            }
        }
        smoothPath.push(rawPath[farthest]);
        currIdx = farthest;
    }

    return smoothPath;
}

// Render & Motion Loop
let lastNavTime = performance.now();
function draw() {
    const navNow = performance.now();
    const navDt = Math.min((navNow - lastNavTime) / 1000, 0.1); // clamp so a stalled tab doesn't jump
    lastNavTime = navNow;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Current view's scale (global 1.35, or the 3.5x marina zoom used in Mode 3 —
    // see getCurrentViewParams). Every radius/spacing drawn below must use THIS,
    // not the fixed global SCALE, or it renders at the wrong size/place relative
    // to positions (which rosToCanvas already draws using this same view scale) —
    // that mismatch was why the marina looked stranded out in the forest in Mode 3.
    const mapView = getCurrentViewParams();
    const mapScale = mapView.scale;

    // 1. Render Outer Land Mass Background (Forest Green)
    ctx.fillStyle = '#1b3b18';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    const centerP = rosToCanvas(0, 0);

    // 2. Render Sandy Beach Shore Ring (Radius: 315m to match 3D world)
    ctx.beginPath();
    ctx.arc(centerP.x, centerP.y, (LAKE_RADIUS + 15.0) * mapScale, 0, 2 * Math.PI);
    ctx.fillStyle = '#d2b48c';
    ctx.fill();

    // 3. Render Circular Blue Lake Water Body (Clean 2D Ocean Fill)
    const waterGrad = ctx.createRadialGradient(centerP.x, centerP.y, 0, centerP.x, centerP.y, LAKE_RADIUS * mapScale);
    waterGrad.addColorStop(0, '#005f9e');
    waterGrad.addColorStop(1, '#002952');

    ctx.beginPath();
    ctx.arc(centerP.x, centerP.y, LAKE_RADIUS * mapScale, 0, 2 * Math.PI);
    ctx.fillStyle = waterGrad;
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#001a33';
    ctx.stroke();

    // 4. Draw Tactical Grid Overlay
    ctx.strokeStyle = 'rgba(255,255,255,0.08)';
    ctx.lineWidth = 1;
    for (let i = 0; i < canvas.width; i += mapScale * 10) {
        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, canvas.height); ctx.stroke();
    }
    for (let i = 0; i < canvas.height; i += mapScale * 10) {
        ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(canvas.width, i); ctx.stroke();
    }

    // Active Navigation Kinematic & ROS Steer Loop (Receding Horizon Live Pathfinder)
    if (isNavigating && plannedPath.length > 0) {
        const now = Date.now();
        // Receding Horizon Sensor Scan: Recalculate path live every 250ms based on local 25m sensor horizon.
        // Mode 3 docking runs its own fixed waypoint sequence (transit/align/creep) — this must NOT
        // reroute it, or a currentGoal left over from a prior Mode 1 run silently replaces the docking
        // path with an A* route back to that old goal, and the boat never reaches the berth.
        if (!isAutoDocking && now - lastRecalcTime > 250 && currentGoal) {
            lastRecalcTime = now;
            const sensedObstacles = entities.filter(ent => {
                if (ent.type === 'goal' || ent.isCrashed) return false;
                const dist = Math.hypot(ent.ros_x - boatPos.x, ent.ros_y - boatPos.y);
                return dist <= 25.0; // 25-meter sensor horizon
            });

            const newPath = findOptimalPath(
                { x: boatPos.x, y: boatPos.y },
                { x: currentGoal.ros_x, y: currentGoal.ros_y },
                sensedObstacles
            );
            if (newPath && newPath.length > 1) {
                plannedPath = newPath;
                pathIndex = 1; // Point directly to upcoming waypoint to prevent waypoint 0 snap
                update3DPathLine();
            }
        }

        if (pathIndex < plannedPath.length) {
            const target = plannedPath[pathIndex];
            const dx = target.x - boatPos.x;
            const dy = target.y - boatPos.y;
            const dist = Math.hypot(dx, dy);

            const targetYaw = (target.mode === 'align' && target.targetYaw !== undefined) ? target.targetYaw : Math.atan2(dy, dx);
            let yawDiff = targetYaw - boatPos.yaw;
            while (yawDiff > Math.PI) yawDiff -= 2 * Math.PI;
            while (yawDiff < -Math.PI) yawDiff += 2 * Math.PI;

            // State Machine Progression Logic
            let advancePath = false;
            if (target.mode === 'align') {
                if (Math.abs(yawDiff) < 0.05) advancePath = true; // Advance only when heading is locked!
            } else if (target.mode === 'creep' || target.mode === 'reverse_swing') {
                if (dist < 0.3) advancePath = true; // High precision finish line
            } else if (target.mode === 'approach') {
                if (dist < 0.5) advancePath = true; // Wait to reach near the pier
            } else if (isAutoDocking) {
                // 1.2m, not the standard-transit 1.8m (docking is tighter quarters)
                // but not 0.4m either — that was tight enough that a multi-leg A*
                // route through the marina (several short, sharp-angled corners)
                // could get stuck circling trying to nail an intermediate corner
                // exactly, rather than just cutting the turn and moving on.
                if (dist < 1.2) advancePath = true;
            } else {
                if (dist < 1.8) advancePath = true; // Standard transit tolerance
            }

            if (advancePath) {
                pathIndex++;
            } else {
                // Unified movement model: this drives Mode 1's auto-nav and Mode 3's
                // docking with the SAME momentum/braking physics Mode 2 uses for manual
                // driving (currentLinear/currentAngular ramped via approachVelocity,
                // capped by MAX_LINEAR_FWD/MAX_LINEAR_REV/MAX_ANGULAR), instead of the
                // old separate model that set speed directly every frame with no
                // momentum at all (instantly matching a fixed per-state speed, then
                // snapping to the next state's speed with no transition).
                let cruiseSpeed = MAX_LINEAR_FWD;
                let angularGain = 1.0; // rad/s of turn commanded per rad of heading error
                let pivotOnly = false;  // true = zero forward speed, pure rotation (align)
                // Deceleration used to compute AND perform the final glide to a stop.
                // WATER_FRICTION_RATE (1.0) is tuned for Mode 2's up-to-6.0 m/s manual
                // driving — reused as-is here, a slow ~1.3 m/s docking leg only gets
                // ~0.85m / ~1.3s of braking before reaching zero, which reads as a
                // sudden stop rather than a gradual one. The close-quarters docking
                // legs (approach/creep/reverse_swing) use a gentler, dedicated rate
                // instead, giving a multi-second, clearly visible glide to a stop —
                // still the same ramp-toward-a-target mechanism, just paced for a
                // careful docking maneuver instead of open-water cruising.
                let brakeDecel = WATER_FRICTION_RATE;

                // DOCKING STATE MACHINE
                if (isAutoDocking) {
                    if (target.mode === 'transit') {
                        cruiseSpeed = MAX_LINEAR_FWD;
                        document.getElementById('tele-status').textContent = '⚓ STATE 1: Transit to Staging Area...';
                        document.getElementById('tele-status').style.color = '#00ffcc';
                    } else if (target.mode === 'align') {
                        // STATE 2: PIVOT ALIGNMENT (Zero forward speed, pivot in place!)
                        pivotOnly = true;
                        angularGain = 3.0; // decisive pivot, saturates to MAX_ANGULAR quickly
                        document.getElementById('tele-status').textContent = '🔵 STATE 2: Pivoting to Alignment...';
                        document.getElementById('tele-status').style.color = '#ffaa00';
                    } else if (target.mode === 'approach') {
                        // Angled Approach, braking into the berth. Was a
                        // flat 1.6/-0.3 (tuned as a fraction of the old
                        // MAX_LINEAR_FWD=6.0 baseline) — now expressed as
                        // that same ratio of the current (real,
                        // thrust-derived) MAX_LINEAR_FWD, so this scales
                        // automatically if the top speed is ever retuned
                        // again instead of drifting out of proportion.
                        cruiseSpeed = MAX_LINEAR_FWD * DOCK_APPROACH_SPEED_RATIO;
                        brakeDecel = DOCK_BRAKE_DECEL;
                        document.getElementById('tele-status').textContent = '🟢 STATE 3: Angled Approach...';
                        document.getElementById('tele-status').style.color = '#00ff00';
                    } else if (target.mode === 'reverse_swing') {
                        // Reverse Swing (Reverse speed + hard rudder, braking into position)
                        cruiseSpeed = MAX_LINEAR_FWD * DOCK_REVERSE_SWING_SPEED_RATIO;
                        angularGain = 2.0; // hard pivot while reversing
                        brakeDecel = DOCK_BRAKE_DECEL;
                        document.getElementById('tele-status').textContent = '🟣 STATE 4: Reversing & Swinging Stern...';
                        document.getElementById('tele-status').style.color = '#cc00ff';
                    } else if (target.mode === 'creep') {
                        // STATE 3: CREEP INSERTION, braking to a stop at the slot
                        cruiseSpeed = MAX_LINEAR_FWD * DOCK_CREEP_SPEED_RATIO;
                        brakeDecel = DOCK_BRAKE_DECEL;
                        document.getElementById('tele-status').textContent = '🟢 STATE 3: Creep Insertion...';
                        document.getElementById('tele-status').style.color = '#00ff00';
                    }
                }

                // Heading lock: hold position and turn first when badly
                // misaligned, rather than cruising into the turn. A softer
                // "just reduce speed" version of this (never below 45%) was
                // tried and verified against the real running sim (see the
                // headless rosbridge repro used to debug this) to still let
                // the boat drift meaningfully off the intended line during
                // the ~1+ second a large turn takes — because the target
                // bearing is recomputed from the boat's OWN moving position
                // every tick, that drift compounds (a classic pursuit-curve
                // divergence) rather than damping out, and was enough for
                // the boat to clip real obstacle geometry (e.g. a marina
                // jetty) that the PLANNED straight segment never crossed.
                // Gating hard on heading error — the same "pivot first"
                // principle 'align' already uses — keeps the executed path
                // close enough to the planned one that this can't happen.
                const HEADING_LOCK = 0.5; // ~29 degrees
                if (!pivotOnly && Math.abs(yawDiff) > HEADING_LOCK) {
                    cruiseSpeed = 0;
                } else if (!pivotOnly) {
                    cruiseSpeed *= Math.max(0.4, Math.cos(yawDiff));
                }

                // Slow down ahead of a sharp UPCOMING turn, not just react to the
                // CURRENT heading error above. An A*-planned route can string
                // together several short, sharp-angled legs (e.g. threading
                // between marina jetties) — approaching one at full cruise speed
                // leaves only brakeDist ~= v^2/(2*brakeDecel) of room to slow down
                // in, which a short leg doesn't have. Looking one waypoint ahead
                // and easing cruiseSpeed down before the corner (not just at it)
                // gives brakeDist time to shrink to match.
                const nextTarget = plannedPath[pathIndex + 1];
                if (nextTarget && !pivotOnly) {
                    const legDx = target.x - boatPos.x, legDy = target.y - boatPos.y;
                    const nextDx = nextTarget.x - target.x, nextDy = nextTarget.y - target.y;
                    const legLen = Math.hypot(legDx, legDy), nextLen = Math.hypot(nextDx, nextDy);
                    if (legLen > 0.01 && nextLen > 0.01) {
                        const cosTurn = (legDx * nextDx + legDy * nextDy) / (legLen * nextLen); // 1 = straight, -1 = reversal
                        const turnAheadFactor = Math.max(0.3, (cosTurn + 1) / 2);
                        cruiseSpeed *= turnAheadFactor;
                    }
                }

                // --- Linear: cruise, then glide to a stop exactly at the waypoint ---
                if (pivotOnly) {
                    currentLinear = approachVelocity(currentLinear, 0.0, navDt);
                } else {
                    const brakeDist = (currentLinear * currentLinear) / (2 * brakeDecel);
                    if (dist > brakeDist) {
                        // Plenty of room — cruise (accelerating toward it via THRUST_RAMP_RATE
                        // if not already there)
                        currentLinear = approachVelocity(currentLinear, cruiseSpeed, navDt);
                    } else {
                        // Inside the braking window: decelerate at brakeDecel directly,
                        // rather than through approachVelocity's hardcoded
                        // WATER_FRICTION_RATE — the rate used to decide WHEN to start
                        // braking has to match the rate actually applied, or a gentler
                        // brakeDecel here would never actually produce the longer glide
                        // the brakeDist above was computed for.
                        const step = brakeDecel * navDt;
                        if (Math.abs(currentLinear) <= step) {
                            currentLinear = 0;
                        } else {
                            currentLinear -= Math.sign(currentLinear) * step;
                        }
                        // Safety net: if there's somehow still less room than this needs
                        // (e.g. the path got recalculated mid-leg), brake harder with
                        // reverse thrust instead of overshooting.
                        const requiredDecel = (currentLinear * currentLinear) / (2 * Math.max(dist, 0.05));
                        if (requiredDecel > brakeDecel * 2.0 && Math.abs(currentLinear) > 0.05) {
                            const reverseTarget = -Math.sign(currentLinear) * Math.min(Math.abs(MAX_LINEAR_REV), 1.0);
                            currentLinear = approachVelocity(currentLinear, reverseTarget, navDt);
                        }
                    }
                }
                currentLinear = Math.max(MAX_LINEAR_REV, Math.min(MAX_LINEAR_FWD, currentLinear));

                // Hull is already touching a solid boundary (island/pier wall/moored
                // vessel/buoy/shoreline) — kill any further push in whichever
                // direction is actually driving deeper into it, like running
                // aground into thick mud. Unlike Mode 2 (where a human is holding
                // the stick and can just reverse), there's no one to un-stick an
                // autonomous run — so a forward block backs off a little instead
                // of freezing at zero, so a graze against real hull geometry
                // doesn't strand the mission needing a manual Reset.
                if (currentLinear > 0 && (isMovingIntoObstacle(1) || isExitingLake(1))) {
                    currentLinear = Math.max(MAX_LINEAR_REV, -0.6);
                } else if (currentLinear < 0 && (isMovingIntoObstacle(-1) || isExitingLake(-1))) {
                    currentLinear = 0;
                }

                // --- Angular: proportional heading control, capped at MAX_ANGULAR ---
                const targetAngular = Math.sign(yawDiff) * Math.min(MAX_ANGULAR, Math.abs(yawDiff) * angularGain);
                currentAngular = approachVelocity(currentAngular, targetAngular, navDt);

                // boatPos itself is NOT integrated here — it comes solely
                // from the /odom subscription (same as Mode 2), so what's
                // rendered/used for the next frame's dist/yawDiff is
                // Gazebo's real physics output, not a client-side guess.
                // currentLinear/currentAngular above only shape the
                // outgoing /cmd_vel demand.

                // Hydrodynamic Roll Banking: Lean boat hull realistically into turns in 3D
                if (boatGroup) {
                    boatGroup.rotation.z = -currentAngular * 0.15;
                }

                // Send cmd_vel to ROS
                const twist = new ROSLIB.Message({
                    linear: { x: currentLinear, y: 0.0, z: 0.0 },
                    angular: { x: 0.0, y: 0.0, z: currentAngular }
                });
                cmdVelTopic.publish(twist);
            }
        } else {
            isNavigating = false;
            currentLinear = 0;
            currentAngular = 0;
            boatPos.speed = 0;

            // Publish zero velocity to ROS to freeze boat motors
            const stopTwist = new ROSLIB.Message({
                linear: { x: 0.0, y: 0.0, z: 0.0 },
                angular: { x: 0.0, y: 0.0, z: 0.0 }
            });
            cmdVelTopic.publish(stopTwist);

            if (isAutoDocking) {
                isAutoDocking = false;
                document.getElementById('tele-status').textContent = `🎉 DOCKED SAFELY: ${dockingBerthName}!`;
                document.getElementById('tele-status').style.color = '#28a745';
            } else {
                document.getElementById('tele-status').textContent = '🎉 Goal Reached!';
                document.getElementById('tele-status').style.color = '#28a745';
            }
        }
        document.getElementById('tele-x').textContent = boatPos.x.toFixed(2);
        document.getElementById('tele-y').textContent = boatPos.y.toFixed(2);
        document.getElementById('tele-speed').textContent = boatPos.speed.toFixed(2);
    }

    // Draw Planned Path Trajectory
    if (plannedPath.length > 1) {
        ctx.beginPath();
        ctx.setLineDash([6, 6]);
        ctx.strokeStyle = '#00ffcc';
        ctx.lineWidth = 3;
        const startP = rosToCanvas(plannedPath[0].x, plannedPath[0].y);
        ctx.moveTo(startP.x, startP.y);

        for (let i = 1; i < plannedPath.length; i++) {
            const p = rosToCanvas(plannedPath[i].x, plannedPath[i].y);
            ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
    }

    // Update Dynamic Moving Boats Realistic Heavy Ship Kinematics & Buoy Collision
    entities.forEach(entity => {
        if (entity.type === 'dynamic') {
            const mesh = threeEntities.get(entity.id);

            // Handle Crashed Dynamic Boats (Sunk in buoy collision)
            if (entity.isCrashed) {
                if (mesh) {
                    mesh.rotation.z = 0.8; // Tilt sideways
                    mesh.position.y = -0.3; // Submerge
                }
                return;
            }

            // Check if dynamic boat hits any static buoy
            entities.forEach(other => {
                if (other.type === 'static' && !entity.isCrashed) {
                    const distToBuoy = Math.hypot(other.ros_x - entity.ros_x, other.ros_y - entity.ros_y);
                    if (distToBuoy < 2.5) {
                        entity.isCrashed = true; // Dynamic boat crashes & sinks into buoy!
                    }
                }
            });

            if (entity.isCrashed) return;

            // Initialize realistic heavy ship patrol waypoints & heading if missing
            if (!entity.patrolPoints) {
                const r = 24.0;
                entity.patrolPoints = [
                    { x: entity.ros_x + r, y: entity.ros_y },
                    { x: entity.ros_x, y: entity.ros_y + r },
                    { x: entity.ros_x - r, y: entity.ros_y },
                    { x: entity.ros_x, y: entity.ros_y - r }
                ];
                entity.patrolIdx = 0;
                entity.heading = Math.random() * Math.PI * 2;
                entity.speed = 1.0; // Heavy ship cruising speed
            }

            // Target current patrol waypoint in loop
            const target = entity.patrolPoints[entity.patrolIdx];
            const dx = target.x - entity.ros_x;
            const dy = target.y - entity.ros_y;
            const dist = Math.hypot(dx, dy);

            if (dist < 5.0) {
                entity.patrolIdx = (entity.patrolIdx + 1) % entity.patrolPoints.length;
            }

            // Gradual heavy rudder turning towards target
            const targetHeading = Math.atan2(dy, dx);
            let headingDiff = targetHeading - entity.heading;

            while (headingDiff > Math.PI) headingDiff -= 2 * Math.PI;
            while (headingDiff < -Math.PI) headingDiff += 2 * Math.PI;

            entity.heading += headingDiff * 0.015; // Slow, wide ship turning radius

            // Advance boat position forward along its hull orientation
            entity.ros_x += Math.cos(entity.heading) * entity.speed * 0.05;
            entity.ros_y += Math.sin(entity.heading) * entity.speed * 0.05;

            // Sync 3D Mesh Position, Yaw, Pitch, Roll & Water Heave Physics in Three.js
            if (mesh) {
                const nowTime = Date.now() * 0.0025;
                const heave = Math.sin(nowTime * 1.5 + entity.ros_x * 0.1) * 0.06;
                const pitch = Math.sin(nowTime + entity.ros_y * 0.1) * 0.04;
                const roll = Math.cos(nowTime * 1.2 + entity.ros_x * 0.1) * 0.05;

                mesh.position.set(entity.ros_x, 0.4 + heave, -entity.ros_y);
                mesh.rotation.set(pitch, entity.heading, roll);

                // Spin radar antenna scanner
                const rBar = mesh.getObjectByName('radarBar');
                if (rBar) rBar.rotation.y += 0.08;
            }
        }
    });

    // Draw 2D Marina using proper scaling for zoomed views
    ctx.save();
    const s = mapScale;

    // Main Horizontal Pier Spine — drawn from the same canonical MARINA_PIER_*
    // constants (and the pierLength derived from them) the 3D mesh and the
    // collision/pathfinding checks use, instead of separate hardcoded
    // width/height literals that used to draw this at the pre-WORLD_SCALE
    // size (140/58 real meters) regardless of how compressed the rest of
    // the map is — the actual bug behind "the map doesn't match the camera."
    ctx.fillStyle = '#5d4037';
    const spineTL = rosToCanvas(MARINA_PIER_X_MIN, MARINA_PIER_Y); // top-left = min X, max Y (larger ROS y draws higher on screen)
    ctx.fillRect(spineTL.x, spineTL.y, pierLength * s, MARINA_PIER_THICKNESS * s);

    // 3 Vertical Finger Jetties extending up into water — same fix.
    JETTY_X_LIST.forEach(jx => {
        const jettyTL = rosToCanvas(jx - JETTY_HALF_WIDTH, JETTY_Y_NEAR); // top-left = min X, max Y
        ctx.fillRect(jettyTL.x, jettyTL.y, jettyWidth * s, jettyLength * s);
    });

    // Draw Moored Ships Along All Jetties — reads the REAL entities array
    // (isParkedShip, populated by createMooredBoat() above) instead of a
    // separately hardcoded loop over synthetic positions. That old loop
    // always drew every slot regardless of what was actually spawned in the
    // 3D scene, so thinning the moored fleet there (fewer boats, more open
    // choices) would otherwise have left the 2D map showing boats that no
    // longer exist — the same "map doesn't match the camera" bug, just for
    // moored boats instead of the pier/jetty structure.
    const bColors2D = ['#1d3557', '#2a9d8f', '#e63946', '#457b9d', '#0f4c5c', '#3d5a80', '#9b5de5', '#f15bb5'];
    entities.forEach(ent => {
        if (!ent.isParkedShip) return;
        const p = rosToCanvas(ent.ros_x, ent.ros_y);
        ctx.fillStyle = bColors2D[Math.abs(Math.floor(ent.ros_x * 10 + ent.ros_y)) % bColors2D.length];
        ctx.fillRect(p.x - (3.5 * s) / 2, p.y - (6 * s) / 2, 3.5 * s, 6 * s); // width 3.5, height 6, centered on the boat's real position
    });

    // Draw Dynamic Target Berth Marker on 2D Map if active
    if (activeAppMode === 3 && plannedPath && plannedPath.length > 0) {
        const lastP = plannedPath[plannedPath.length - 1];
        const bp = rosToCanvas(lastP.x, lastP.y);

        ctx.save();
        ctx.beginPath();
        ctx.setLineDash([3, 3]);
        ctx.arc(bp.x, bp.y, 14, 0, 2 * Math.PI);
        ctx.strokeStyle = '#00ffcc';
        ctx.lineWidth = 2.5;
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.font = 'bold 11px sans-serif';
        ctx.fillStyle = '#00ffcc';
        ctx.textAlign = 'center';
        ctx.fillText('⚓ Selected Berth', bp.x, bp.y - 16);
        ctx.restore();
    }

    // Label Text
    ctx.font = 'bold 10px sans-serif';
    ctx.fillStyle = '#00ffcc';
    const marinaBase = rosToCanvas(-195 * WORLD_SCALE, -175 * WORLD_SCALE);
    ctx.fillText('⚡ 30+ PACKED MARINA GRID (<40cm TOLERANCE)', marinaBase.x - 65, marinaBase.y + 12);
    ctx.restore();

    // City Building Blocks (Strictly on Upper-Right Sandy Coast)
    const cityCanvas = rosToCanvas(230, 230);
    ctx.fillStyle = '#37474f';
    ctx.fillRect(cityCanvas.x - 28, cityCanvas.y - 14, 62, 28);
    ctx.fillStyle = '#eceff1';
    ctx.font = 'bold 10px sans-serif';
    ctx.fillText('🏙️ Coastal City', cityCanvas.x - 24, cityCanvas.y + 3);
    ctx.restore();

    // Draw 2D Core Island Land Mass (Obstacle at x: 20m, y: 15m, radius: 12m)
    const islandCanvas = rosToCanvas(ISLAND_X, ISLAND_Y);
    ctx.save();

    // Sand Border
    ctx.beginPath();
    ctx.arc(islandCanvas.x, islandCanvas.y, (ISLAND_RADIUS + 1.5) * mapScale, 0, 2 * Math.PI);
    ctx.fillStyle = '#d2b48c';
    ctx.fill();
    // Grass Island Top
    ctx.beginPath();
    ctx.arc(islandCanvas.x, islandCanvas.y, ISLAND_RADIUS * mapScale, 0, 2 * Math.PI);
    ctx.fillStyle = '#2e7d32';
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#1b5e20';
    ctx.stroke();
    // Island Label & Trees
    ctx.font = '14px sans-serif';
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';
    ctx.fillText('🌲 Island 🌲', islandCanvas.x, islandCanvas.y + 5);
    ctx.restore();

    // Draw Entities & Imaginary Dotted Safety Keep-Out Circles
    entities.forEach(entity => {
        const p = rosToCanvas(entity.ros_x, entity.ros_y);

        // Draw Visual Restrictive Imaginary Dotted Safety Circle
        if (entity.type === 'static' || entity.type === 'dynamic') {
            ctx.save();
            ctx.beginPath();
            ctx.setLineDash([4, 4]);
            ctx.arc(p.x, p.y, SAFETY_RADIUS * mapScale, 0, 2 * Math.PI);
            ctx.strokeStyle = entity.type === 'dynamic' ? 'rgba(23, 162, 184, 0.7)' : 'rgba(255, 193, 7, 0.7)';
            ctx.lineWidth = 1.5;
            ctx.stroke();
            ctx.fillStyle = entity.type === 'dynamic' ? 'rgba(23, 162, 184, 0.1)' : 'rgba(255, 193, 7, 0.1)';
            ctx.fill();
            ctx.restore();
        }

        ctx.beginPath();
        if (entity.type === 'static') {
            ctx.arc(p.x, p.y, 8, 0, 2 * Math.PI);
            ctx.fillStyle = '#ffc107'; // Yellow buoy
        } else if (entity.type === 'dynamic') {
            ctx.save();
            ctx.translate(p.x, p.y);
            ctx.rotate(-entity.heading);
            ctx.rect(-10, -6, 20, 12);
            ctx.fillStyle = '#17a2b8'; // Blue boat
            ctx.fill();
            ctx.lineWidth = 2;
            ctx.strokeStyle = '#ffffff';
            ctx.stroke();
            ctx.restore();
            return;
        } else if (entity.type === 'goal') {
            ctx.arc(p.x, p.y, 12, 0, 2 * Math.PI);
            ctx.fillStyle = '#28a745'; // Green goal
        }
        ctx.fill();
        ctx.lineWidth = 2;
        ctx.strokeStyle = '#ffffff';
        ctx.stroke();
    });

    // Draw ASV Boat (2D Map)
    const boatCanvas = rosToCanvas(boatPos.x, boatPos.y);
    ctx.save();
    ctx.translate(boatCanvas.x, boatCanvas.y);
    ctx.rotate(-boatPos.yaw);

    ctx.beginPath();
    ctx.moveTo(14, 0);
    ctx.lineTo(-10, -7);
    ctx.lineTo(-10, 7);
    ctx.closePath();
    ctx.fillStyle = '#dc3545';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();

    // Sync 3D Scene & FPV Camera
    sync3DEntities();

    // Water stays flat — matches the actual sim, which has no wave field
    // (vrx::Surface here is flat-fluid-level buoyancy only, no vrx::Wavefield
    // plugin is instantiated; see exhibition_water.sdf).
    const time = Date.now() * 0.0015;

    // Drift Clouds Slowly Across Sky
    clouds.forEach(cloud => {
        cloud.position.x += 0.05;
        if (cloud.position.x > 600) cloud.position.x = -600;
    });

    // Animate Flying Seagull Flock (Circling Sky & Flapping Wings)
    flock.forEach(bird => {
        bird.userData.angle += bird.userData.speed;
        const bx = Math.cos(bird.userData.angle) * bird.userData.radius;
        const bz = Math.sin(bird.userData.angle) * bird.userData.radius;
        const by = bird.userData.height + Math.sin(bird.userData.angle * 2) * 4;

        bird.position.set(bx, by, bz);
        bird.rotation.y = -bird.userData.angle + Math.PI / 2; // Facing flight vector

        // Flap Wings
        const flap = Math.sin(Date.now() * 0.012 + bird.userData.phase) * 0.45;
        const lw = bird.getObjectByName('leftWing');
        const rw = bird.getObjectByName('rightWing');
        if (lw) lw.rotation.z = flap;
        if (rw) rw.rotation.z = -flap;
    });

    // Simulate Buoy Blinking
    threeEntities.forEach((mesh, id) => {
        if (id.startsWith('obs_')) {
            const light = mesh.children.find(c => c.isPointLight);
            if (light) {
                light.intensity = Math.sin(time * 2) > 0 ? 1 : 0;
            }
        }
    });

    // 3D Boat Group Transform
    if (radarMesh) radarMesh.rotation.y += 0.05; // Spin radar scanner

    boatGroup.position.set(boatPos.x, 0.4, -boatPos.y);
    boatGroup.rotation.y = boatPos.yaw;
    boatGroup.rotation.z = 0;

    const targetCamPos = new THREE.Vector3(-8, 4, 0);
    targetCamPos.applyAxisAngle(new THREE.Vector3(0, 1, 0), boatPos.yaw);
    targetCamPos.add(new THREE.Vector3(boatPos.x, 0, -boatPos.y));

    camera.position.lerp(targetCamPos, 0.12); // Smooth 3D FPV camera tracking lerp
    camera.lookAt(boatPos.x, 0.8, -boatPos.y);

    renderer.render(scene, camera);

    requestAnimationFrame(draw);
}
draw();

// UI Interactions
document.querySelectorAll('.tool-btn').forEach(btn => {
    btn.addEventListener('click', (e) => {
        document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
        e.currentTarget.classList.add('active');
        currentMode = e.currentTarget.dataset.type;
    });
});

function selectBerth(idx) {
    activeBerthIdx = idx;
    const bSel = document.getElementById('berth-selector');
    if (bSel) bSel.value = idx.toString();

    // Update 3D Beacon Mesh Lights
    berth3DMeshes.forEach((mesh, i) => {
        const isAct = i === activeBerthIdx;
        mesh.material.color.setHex(isAct ? 0x00ffcc : 0xffc107);
        const l = mesh.children.find(c => c.isPointLight);
        if (l) l.color.setHex(isAct ? 0x00ffcc : 0xffc107);
    });
}

canvas.addEventListener('click', (e) => {
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    let { rx, ry } = canvasToRos(cx, cy);

    // In Mode 3, click near any pier edge to calculate dynamic autonomous parking
    if (activeAppMode === 3) {
        let bestBerth = null;
        let minDist = 3.5; // 3.5m snap distance

        const dockTypeSel = document.getElementById('dock-type-selector');
        const isParallel = dockTypeSel ? (dockTypeSel.value === 'parallel') : true;

        // 1. Check Spine Pier (Horizontal, y = -175.0) — canonical MARINA_PIER_* bounds
        if (rx >= MARINA_PIER_X_MIN && rx <= MARINA_PIER_X_MAX) {
            if (Math.abs(ry - (-175.0 * WORLD_SCALE)) < minDist) {
                minDist = Math.abs(ry - (-175.0 * WORLD_SCALE));
                bestBerth = {
                    name: 'Dynamic Docking (Main Spine Pier)',
                    ros_x: rx,
                    ros_y: isParallel ? -171.2 * WORLD_SCALE : -168.8 * WORLD_SCALE,
                    type: isParallel ? 'parallel' : 'slip',
                    parkedYaw: isParallel ? 0.0 : -1.57, 
                    corridor_x: isParallel ? rx - 15.0 : rx,
                    corridor_y: -145.0 * WORLD_SCALE,
                    staging_x: isParallel ? rx - 15.0 : rx,
                    staging_y: isParallel ? -171.2 * WORLD_SCALE : -145.0 * WORLD_SCALE
                };
            }
        }

        // 2. Check Finger Jetties (Vertical, x = -245, -195, -145) — canonical JETTY_* bounds
        JETTY_X_LIST.forEach((jx, index) => {
            if (ry <= JETTY_Y_NEAR && ry >= JETTY_Y_FAR) {
                // Left Face (wall at jx - 2.0)
                if (Math.abs(rx - (jx - 2.0)) < minDist) {
                    minDist = Math.abs(rx - (jx - 2.0));
                    const targetX = isParallel ? jx - 3.6 : jx - 5.5;
                    const stagingX = isParallel ? jx - 5.5 : targetX;
                    const openCorridorX = (index === 0) ? -265.0 * WORLD_SCALE : (index === 1 ? -220.0 * WORLD_SCALE : -170.0 * WORLD_SCALE);
                    bestBerth = {
                        name: `Dynamic Docking (Jetty ${index+1} Left)`,
                        ros_x: targetX,
                        ros_y: ry,
                        corridor_x: openCorridorX,
                        corridor_y: -105.0 * WORLD_SCALE,
                        staging_x: stagingX,
                        staging_y: ry,
                        type: isParallel ? 'parallel' : 'slip',
                        parkedYaw: isParallel ? -1.57 : 0.0
                    };
                }
                // Right Face (wall at jx + 2.0)
                if (Math.abs(rx - (jx + 2.0)) < minDist) {
                    minDist = Math.abs(rx - (jx + 2.0));
                    const targetX = isParallel ? jx + 3.6 : jx + 5.5;
                    const stagingX = isParallel ? jx + 5.5 : targetX;
                    const openCorridorX = (index === 2) ? -125.0 * WORLD_SCALE : (index === 0 ? -220.0 * WORLD_SCALE : -170.0 * WORLD_SCALE);
                    bestBerth = {
                        name: `Dynamic Docking (Jetty ${index+1} Right)`,
                        ros_x: targetX,
                        ros_y: ry,
                        corridor_x: openCorridorX,
                        corridor_y: -105.0 * WORLD_SCALE,
                        staging_x: stagingX,
                        staging_y: ry,
                        type: isParallel ? 'parallel' : 'slip',
                        parkedYaw: isParallel ? -1.57 : 3.14
                    };
                }
            }
        });

        if (!bestBerth) {
            alert("No valid pier edge detected! Please click closer to a rigid wooden dock.");
            return;
        }

        // Check if selected spot overlaps a parked vessel on finger jetties
        if (bestBerth.ros_y <= -128.0 * WORLD_SCALE && bestBerth.ros_y >= -168.0 * WORLD_SCALE && Math.abs(bestBerth.ros_y - (-147.5 * WORLD_SCALE)) > 3.5) {
            if (bestBerth.name.includes('Jetty')) {
                alert("⛔ Space Occupied! A parked vessel is currently docked at this location. Please select an open spot.");
                return;
            }
        }

        // 3. Perception / Collision Check for obstacles
        let blocked = false;
        entities.forEach(ent => {
            if (ent.type === 'static' || ent.type === 'dynamic') {
                const dist = Math.hypot(ent.ros_x - bestBerth.ros_x, ent.ros_y - bestBerth.ros_y);
                if (dist < 5.5) blocked = true;
            }
        });

        if (blocked) {
            alert("🚫 OBSTACLE DETECTED! There is another vessel parked there. Choose an empty space.");
            return;
        }

        // 4. Start Dynamic Docking Sequence!
        startDynamicDocking(bestBerth);
        return;
    }

    if (activeAppMode !== 1 || !currentMode) return;

    // Enforce Lake Open-Water Constraints (Keep goals & buoys inside water body).
    // 330 keeps the same ~5% margin inside LAKE_RADIUS's own 350 pre-scale
    // reference that the old 285/300 pair had.
    const distFromOrigin = Math.hypot(rx, ry);
    if (distFromOrigin > 330.0 * WORLD_SCALE) {
        rx = (rx / distFromOrigin) * (330.0 * WORLD_SCALE);
        ry = (ry / distFromOrigin) * (330.0 * WORLD_SCALE);
    }

    // Keep clear of Central Island (r >= 33.0m from island center)
    const distFromIsland = Math.hypot(rx - ISLAND_X, ry - ISLAND_Y);
    if (distFromIsland < 33.0) {
        const angle = Math.atan2(ry - ISLAND_Y, rx - ISLAND_X);
        rx = ISLAND_X + Math.cos(angle) * 33.0;
        ry = ISLAND_Y + Math.sin(angle) * 33.0;
    }

    if (currentMode === 'goal') {
        entities = entities.filter(ent => ent.type !== 'goal');
        currentGoal = { type: 'goal', ros_x: rx, ros_y: ry, id: 'goal_node' };
        entities.push(currentGoal);
    } else {
        const obsId = `obs_${obsCounter++}`;
        const ent = {
            type: currentMode,
            ros_x: rx,
            ros_y: ry,
            id: obsId,
            vx: currentMode === 'dynamic' ? (Math.random() > 0.5 ? 0.14 : -0.14) : 0,
            vy: currentMode === 'dynamic' ? (Math.random() > 0.5 ? 0.08 : -0.08) : 0
        };
        entities.push(ent);

        const msg = new ROSLIB.Message({
            data: JSON.stringify({ type: currentMode, x: rx, y: ry, id: obsId })
        });
        spawnTopic.publish(msg);
    }
});

// ▶️ RUN ASV DEMO Button
document.getElementById('btn-run').addEventListener('click', () => {
    if (!currentGoal) {
        alert("Please set a Target Goal (🏁) first before running!");
        return;
    }

    document.getElementById('tele-status').textContent = '▶️ Navigating...';
    document.getElementById('tele-status').style.color = '#00ffcc';

    // Calculate A* Collision-Free Path around all buoys
    plannedPath = findOptimalPath(
        { x: boatPos.x, y: boatPos.y },
        { x: currentGoal.ros_x, y: currentGoal.ros_y },
        entities
    );

    // Start Smooth Execution
    isNavigating = true;
    pathIndex = 0;

    // Send Goal Pose to ROS
    const goalMsg = new ROSLIB.Message({
        header: { frame_id: 'map' },
        pose: {
            position: { x: currentGoal.ros_x, y: currentGoal.ros_y, z: 0.0 },
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        }
    });
    goalTopic.publish(goalMsg);
});

// Stops any nav/dock in progress and snaps the boat to `pose`, both locally
// (so the map/3D view update instantly) and on the REAL Gazebo boat via the
// backend's 'set_pose' handler. odom now drives boatPos unconditionally in
// every mode (see odomTopic.subscribe), so without ignoreOdomUntil below,
// boatPos would get overwritten straight back to wherever the boat
// physically still is by the next /odom tick — the backend's teleport is
// async and takes real time to land. Used both by Reset and by every
// mode-switch, so entering a mode never carries over position or
// navigation state from what came before.
function resetBoatToPose(pose) {
    isNavigating = false;
    isAutoDocking = false;
    plannedPath = [];
    currentGoal = null;
    pathIndex = 0;
    currentLinear = 0.0;
    currentAngular = 0.0;

    boatPos = { x: pose.x, y: pose.y, yaw: pose.yaw, speed: 0 };
    // Give the backend's async 'set_pose' teleport time to actually land in
    // Gazebo before trusting /odom again — otherwise a stale reading from
    // the boat's pre-reset position can win the race and clobber this.
    ignoreOdomUntil = Date.now() + 1000;
    if (boatGroup) {
        boatGroup.position.set(pose.x, 0.4, -pose.y);
        boatGroup.rotation.set(0, pose.yaw, 0);
    }
    update3DPathLine(); // clears any leftover path line from a prior nav/dock run
    if (dynamic3DBerthMesh) dynamic3DBerthMesh.position.set(-220, -10, 0); // hide stale berth marker

    cmdVelTopic.publish(new ROSLIB.Message({
        linear: { x: 0.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
    }));

    spawnTopic.publish(new ROSLIB.Message({
        data: JSON.stringify({ type: 'set_pose', x: pose.x, y: pose.y, yaw: pose.yaw })
    }));
}

// Clears everything placed in Mode 1 (buoys, moving boats, the goal marker)
// while keeping the ~30 permanent moored boats (isParkedShip) intact — those
// are baked-in marina scenery set up once at page load, not part of a
// design, and never get re-added if wiped. Used by Reset AND by every mode
// switch below, so a design from Mode 1 never silently carries over into
// Mode 2/3 (or into a fresh Mode 1 session) — same "always start clean"
// principle resetBoatToPose already applies to the boat's position.
function clearMode1Design() {
    threeEntities.forEach(mesh => scene.remove(mesh));
    threeEntities.clear();
    entities = entities.filter(ent => ent.isParkedShip);
    currentGoal = null;
}

document.getElementById('btn-reset').addEventListener('click', () => {
    // 1. Halt nav/docking, snap boat back to Mode 1's own start pose (this
    // button only exists in mode1-tools — it used to hardcode the marina
    // entrance instead, which only matches Mode 1's actual spawn point by
    // coincidence when MODE1_START happens to equal it).
    resetBoatToPose(MODE1_START);

    // 2. Send Reset signal to Gazebo Backend Spawner to delete physical obstacle models
    const resetMsg = new ROSLIB.Message({
        data: JSON.stringify({ type: 'reset' })
    });
    spawnTopic.publish(resetMsg);

    // 3. Remove all 3D meshes from Three.js scene & clear 2D entity lists
    clearMode1Design();

    // 4. Reset UI Telemetry Display
    document.getElementById('tele-status').textContent = 'Design Phase';
    document.getElementById('tele-status').style.color = '#ffc107';
    document.getElementById('tele-x').textContent = '0.00';
    document.getElementById('tele-y').textContent = '0.00';
    document.getElementById('tele-speed').textContent = '0.00';
});

// Mode Switcher
const mode1Btn = document.getElementById('mode1-btn');
const mode2Btn = document.getElementById('mode2-btn');
const mode3Btn = document.getElementById('mode3-btn');
const mode1Tools = document.getElementById('mode1-tools');
const mode2Tools = document.getElementById('mode2-tools');
const mode3Tools = document.getElementById('mode3-tools');

if (mode1Btn) {
    mode1Btn.addEventListener('click', () => {
        if (activeAppMode === 2) stopThrusters();
        activeAppMode = 1;
        resetBoatToPose(MODE1_START);
        clearMode1Design();
        mode1Btn.classList.add('active');
        if (mode2Btn) mode2Btn.classList.remove('active');
        if (mode3Btn) mode3Btn.classList.remove('active');
        if (mode1Tools) mode1Tools.style.display = 'block';
        if (mode2Tools) mode2Tools.style.display = 'none';
        if (mode3Tools) mode3Tools.style.display = 'none';
        if (dofPanel) dofPanel.style.display = 'block';
    });
}

if (mode2Btn) {
    mode2Btn.addEventListener('click', () => {
        if (activeAppMode === 2) stopThrusters();
        activeAppMode = 2;
        resetBoatToPose(MODE2_START);
        clearMode1Design();
        mode2Btn.classList.add('active');
        if (mode1Btn) mode1Btn.classList.remove('active');
        if (mode3Btn) mode3Btn.classList.remove('active');
        if (mode2Tools) mode2Tools.style.display = 'block';
        if (mode1Tools) mode1Tools.style.display = 'none';
        if (mode3Tools) mode3Tools.style.display = 'none';
        if (dofPanel) dofPanel.style.display = 'block';
    });
}

if (mode3Btn) {
    mode3Btn.addEventListener('click', () => {
        if (activeAppMode === 2) stopThrusters();
        activeAppMode = 3;
        resetBoatToPose(MODE3_START);
        clearMode1Design();
        mode3Btn.classList.add('active');
        if (mode1Btn) mode1Btn.classList.remove('active');
        if (mode2Btn) mode2Btn.classList.remove('active');
        if (mode3Tools) mode3Tools.style.display = 'block';
        if (mode1Tools) mode1Tools.style.display = 'none';
        if (mode2Tools) mode2Tools.style.display = 'none';
        if (dofPanel) dofPanel.style.display = 'block';
    });
}

// Mode 3: Autonomous Docking Action Trigger
let isAutoDocking = false;
let dockingBerthName = '';

function startDynamicDocking(chosen) {
    isNavigating = false;
    isAutoDocking = true;
    dockingBerthName = chosen.name;

    if (dynamic3DBerthMesh) {
        dynamic3DBerthMesh.position.set(chosen.ros_x, 0.4, -chosen.ros_y);
    }

    const isParallelMode = (chosen.type === 'parallel');

    const statusEl = document.getElementById('tele-status');
    if (statusEl) {
        statusEl.textContent = isParallelMode
            ? `⚓ Side-Docking into ${chosen.name}...`
            : `⚓ Bow-In Docking into ${chosen.name}...`;
        statusEl.style.color = '#00ffcc';
    }

    // Figure out where the close-quarters "align" pivot should happen — a
    // pre-berth point with enough wall/jetty clearance for the pivot's full
    // swept hull length (not just its parked width; see the per-case notes
    // below) — then A* the ENTIRE journey there in one shot: island, user
    // buoys/moving boats, the moored fleet, AND the marina's own dock
    // structure (pier deck + finger jetties) are all real geometry to this
    // search now, not a single blob the boat could only route around, so it
    // actually threads the fairway to reach the berth instead of assuming a
    // fixed, obstacle-blind relay of waypoints was clear. Only the final
    // align + creep into the slot stays hand-tuned — that close-quarters
    // maneuvering (pivot swept-length clearance, wall margins, insertion
    // braking) needs sub-meter precision an A* grid isn't meant to give.
    let alignX, alignY;
    if (chosen.name.includes('Spine Pier')) {
        if (isParallelMode) {
            // Pivoting sweeps the hull's footprint out to its full length
            // (up to ~2.8m of clearance at some angle during the turn), not
            // just its parked width — aligning right at the berth's own
            // ros_y (only ~1.8m off the wall) let the hull clip the wall
            // mid-turn even though the boat's center never got close.
            // Align a safe distance further out, then creep the last stretch
            // into the tight slot already parallel to the wall.
            const ALIGN_WALL_MARGIN = 3.0;
            alignX = chosen.corridor_x;
            alignY = chosen.ros_y + ALIGN_WALL_MARGIN;
        } else {
            alignX = chosen.ros_x;
            alignY = -145.0 * WORLD_SCALE;
        }
    } else {
        // Finger Jetties: align out in the open channel at corridor_x/berth
        // Y-level, not after already creeping into the slot — this used to
        // creep in first and pivot last, meaning side-docking's ~90° turn
        // (parkedYaw is perpendicular to the approach heading for
        // parallel/side berths) happened stationary, wedged against the
        // jetty wall/neighboring moored boat, sweeping the hull's full
        // ~2.8m length into them almost every time.
        alignX = chosen.corridor_x;
        alignY = chosen.ros_y;
    }

    const transit = findOptimalPath(
        { x: boatPos.x, y: boatPos.y },
        { x: alignX, y: alignY },
        entities
    );

    const dockWaypoints = [
        ...transit.map(p => ({ x: p.x, y: p.y, mode: 'transit' })),
        { x: alignX, y: alignY, mode: 'align', targetYaw: chosen.parkedYaw },
        { x: chosen.ros_x, y: chosen.ros_y, mode: 'creep' }
    ];

    plannedPath = dockWaypoints;
    pathIndex = 1;
    update3DPathLine();
    isNavigating = true;
}

const resetDockBtn = document.getElementById('btn-reset-dock');
if (resetDockBtn) {
    resetDockBtn.addEventListener('click', () => {
        document.getElementById('btn-reset').click();
    });
}

// Mode 2 Controls: continuous thruster + water-friction velocity model.
// A held control ramps quickly toward its fixed end velocity (thrusters
// "open"); releasing it lets velocity decay gradually toward zero at the
// water-friction rate instead of snapping to zero immediately.
function sendCmdVel(linear, angular) {
    const twist = new ROSLIB.Message({
        linear: { x: linear, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angular }
    });
    cmdVelTopic.publish(twist);
}

function publishThrust(topic, value) {
    topic.publish(new ROSLIB.Message({ data: value }));
}

let currentLinear = 0.0;
let currentAngular = 0.0;
const heldAxes = new Set(); // 'fwd' | 'rev' | 'left' | 'right' — arrow keys only

// Arrow keys drive the combined boat (unchanged). W/A/R/D independently
// command the left/right thrusters directly — see keyToThruster below.
function keyToAxis(key) {
    switch (key) {
        case 'arrowup': return 'fwd';
        case 'arrowdown': return 'rev';
        case 'arrowleft': return 'left';
        case 'arrowright': return 'right';
        default: return null;
    }
}

const heldThrusterKeys = new Set(); // 'leftFwd' | 'leftRev' | 'rightFwd' | 'rightRev'

// W = left thruster forward, A = left thruster reverse, R = right thruster
// forward, D = right thruster reverse. Bypasses cmd_vel_thrust_mixer.py
// entirely (see thrusterLoop) so this and the arrow-key combined drive never
// both try to command the same two thrust topics in the same frame.
//
// Keyed off e.code (the physical key location, e.g. 'KeyW') rather than
// e.key (the character that key produces) — e.key depends on the OS's active
// keyboard layout, so on a non-US layout (e.g. Greek) the physical W/A/R/D
// keys stop producing 'w'/'a'/'r'/'d' at all and these controls silently do
// nothing. e.code is layout-independent, so the physical keys always work.
function keyToThruster(code) {
    switch (code) {
        case 'KeyW': return 'leftFwd';
        case 'KeyA': return 'leftRev';
        case 'KeyR': return 'rightFwd';
        case 'KeyD': return 'rightRev';
        default: return null;
    }
}

function approachVelocity(current, target, dt) {
    const rate = target !== 0 ? THRUST_RAMP_RATE : WATER_FRICTION_RATE;
    const maxStep = rate * dt;
    const diff = target - current;
    if (Math.abs(diff) <= maxStep) return target;
    return current + Math.sign(diff) * maxStep;
}

function stopThrusters() {
    heldAxes.clear();
    heldThrusterKeys.clear();
    currentLinear = 0.0;
    currentAngular = 0.0;
    sendCmdVel(0.0, 0.0);
    publishThrust(leftThrustTopic, 0.0);
    publishThrust(rightThrustTopic, 0.0);
    lastPublishedLeftThrust = 0.0;
    lastPublishedRightThrust = 0.0;
}

// On-screen D-pad: press-and-hold, matching keyboard behavior
function bindThruster(el, axis) {
    if (!el) return;
    el.addEventListener('mousedown', () => heldAxes.add(axis));
    el.addEventListener('touchstart', (e) => { e.preventDefault(); heldAxes.add(axis); });
    ['mouseup', 'mouseleave', 'touchend', 'touchcancel'].forEach(evt =>
        el.addEventListener(evt, () => heldAxes.delete(axis))
    );
}
bindThruster(document.getElementById('btn-up'), 'fwd');
bindThruster(document.getElementById('btn-down'), 'rev');
bindThruster(document.getElementById('btn-left'), 'left');
bindThruster(document.getElementById('btn-right'), 'right');

const btnStop = document.getElementById('btn-stop');
if (btnStop) {
    btnStop.addEventListener('click', stopThrusters);
}

window.addEventListener('keydown', (e) => {
    if (activeAppMode !== 2) return;
    if (e.key === ' ') { stopThrusters(); return; }
    const axis = keyToAxis(e.key.toLowerCase());
    if (axis) heldAxes.add(axis);
    const thrusterKey = keyToThruster(e.code);
    if (thrusterKey) heldThrusterKeys.add(thrusterKey);
});

window.addEventListener('keyup', (e) => {
    if (activeAppMode !== 2) return;
    const axis = keyToAxis(e.key.toLowerCase());
    if (axis) heldAxes.delete(axis);
    const thrusterKey = keyToThruster(e.code);
    if (thrusterKey) heldThrusterKeys.delete(thrusterKey);
});

// Safety net: if the window/tab loses focus while a key is physically held
// (alt-tab, clicking into dev tools, an OS dialog), the browser never fires
// its keyup, so that thruster would otherwise stay "on" forever with no way
// to release it from the keyboard.
window.addEventListener('blur', stopThrusters);

let lastThrusterTime = performance.now();
let lastPublishedLinear = 0.0;
let lastPublishedAngular = 0.0;
let lastPublishedLeftThrust = 0.0;
let lastPublishedRightThrust = 0.0;

function thrusterLoop() {
    const now = performance.now();
    const dt = Math.min((now - lastThrusterTime) / 1000, 0.1); // clamp so a stalled tab doesn't jump velocity
    lastThrusterTime = now;

    if (activeAppMode === 2 && heldThrusterKeys.size > 0) {
        // Independent per-thruster control takes priority over the combined
        // arrow-key drive whenever any W/A/R/D is held, so the two schemes
        // never both publish to the same thrust topics in the same frame.
        // Instant on/off, no ramp — real Gazebo physics does all the
        // shaping, same principle as the combined drive's instant-cutoff
        // fix below. Both directions are scaled by the same graduated
        // braking curve the combined drive uses (island keep-out AND
        // shoreline, via boundaryCappedSpeed) — 1.0 (no effect) out at open
        // sea, shrinking smoothly toward 0 only inside a boundary's braking
        // zone, and ONLY for whichever direction is actually heading toward
        // that boundary — the escape direction is always left at full
        // strength, so getting braked to a stop never also blocks getting
        // away.
        const fwdThrustScale = Math.max(0, Math.min(1, boundaryCappedSpeed(1, MAX_LINEAR_FWD) / MAX_LINEAR_FWD));
        const revThrustScale = Math.max(0, Math.min(1, boundaryCappedSpeed(-1, Math.abs(MAX_LINEAR_REV)) / Math.abs(MAX_LINEAR_REV)));
        let leftThrust = heldThrusterKeys.has('leftFwd') ? THRUSTER_MAX_FWD_N * fwdThrustScale
            : heldThrusterKeys.has('leftRev') ? THRUSTER_MIN_REV_N * revThrustScale : 0.0;
        let rightThrust = heldThrusterKeys.has('rightFwd') ? THRUSTER_MAX_FWD_N * fwdThrustScale
            : heldThrusterKeys.has('rightRev') ? THRUSTER_MIN_REV_N * revThrustScale : 0.0;

        // Hull already touching a solid boundary — kill thrust on whichever
        // thruster(s) are pushing deeper into it (like running aground into
        // thick mud), in whichever direction (forward OR reverse) that
        // actually is; the other direction always stays free to back off.
        // (Not a forced opposite-thrust push: fighting a still-held key that
        // way re-triggers every frame as the ramp pulls back toward it,
        // which reads as the boat vibrating in place at the boundary rather
        // than cleanly stopping. The widened ISLAND_KEEP_OUT boundary below
        // is what actually stops the boat short of the visible island —
        // this only needs to hold the line, not shove back.)
        const blockedFwd = (leftThrust > 0 || rightThrust > 0) && (isMovingIntoObstacle(1) || isExitingLake(1));
        if (blockedFwd) {
            leftThrust = Math.min(leftThrust, 0);
            rightThrust = Math.min(rightThrust, 0);
        }
        const blockedRev = (leftThrust < 0 || rightThrust < 0) && (isMovingIntoObstacle(-1) || isExitingLake(-1));
        if (blockedRev) {
            leftThrust = Math.max(leftThrust, 0);
            rightThrust = Math.max(rightThrust, 0);
        }
        const blocked = blockedFwd || blockedRev;

        publishThrust(leftThrustTopic, leftThrust);
        publishThrust(rightThrustTopic, rightThrust);
        lastPublishedLeftThrust = leftThrust;
        lastPublishedRightThrust = rightThrust;

        const teleStatusEl = document.getElementById('tele-status');
        if (teleStatusEl) {
            if (blocked) {
                teleStatusEl.textContent = '🚧 Hull Contact — Reverse to Clear';
                teleStatusEl.style.color = '#ff8800';
            } else {
                teleStatusEl.textContent = '🎛️ Independent Thruster Control';
                teleStatusEl.style.color = '#ff66cc';
            }
        }
        document.getElementById('tele-x').textContent = boatPos.x.toFixed(2);
        document.getElementById('tele-y').textContent = boatPos.y.toFixed(2);
        document.getElementById('tele-speed').textContent = boatPos.speed.toFixed(2);
    } else if (activeAppMode === 2) {
        // Just left independent-thruster control (or never entered it this
        // session) — send one final zero so the two raw thrust topics don't
        // stay pinned at their last commanded value. The Thruster plugin
        // holds the last command indefinitely; there's no watchdog timeout.
        if (lastPublishedLeftThrust !== 0.0 || lastPublishedRightThrust !== 0.0) {
            publishThrust(leftThrustTopic, 0.0);
            publishThrust(rightThrustTopic, 0.0);
            lastPublishedLeftThrust = 0.0;
            lastPublishedRightThrust = 0.0;
        }

        // FWD/REV demand ceilings are now real hardware-derived equilibrium
        // speeds (see MAX_LINEAR_FWD/MAX_LINEAR_REV above), not arbitrary
        // software targets — no reason left to uncap these for experiments,
        // real physics and the JS ceiling should now roughly agree. Whichever
        // direction is actually heading toward a boundary (island keep-out OR
        // shoreline, via boundaryCappedSpeed) gets graduated braking — a no-op
        // out at open sea, it only bites within a boundary's braking zone. The
        // OTHER direction (the escape route) is left uncapped, so braking to a
        // stop near either boundary never also blocks getting away from it.
        const targetLinear = heldAxes.has('fwd') ? boundaryCappedSpeed(1, MAX_LINEAR_FWD)
            : heldAxes.has('rev') ? -boundaryCappedSpeed(-1, Math.abs(MAX_LINEAR_REV))
                : 0.0;
        // EXPERIMENT (still active): turn demand uncapped (was MAX_ANGULAR =
        // 1.2) — same "let real physics decide" test already run on forward
        // speed. Once demand exceeds the thruster caps, one side saturates
        // at max_thrust_cmd=51.5N and the other at min_thrust_cmd=-40.2N
        // (exhibition_water.sdf, now real T200-at-16V values — see that
        // file's derivation comment), giving a max differential torque of
        // 0.38*(51.5-(-40.2))=34.8 N*m; balanced against this boat's yaw
        // damping (nR=12, nRR=0), the real physics ceiling works out to
        // roughly 34.8/12 ≈ 2.9 rad/s — this target (15.0) is comfortably
        // past that so the real spin rate, not this number, is what
        // determines the outcome. Revert to MAX_ANGULAR once you've seen
        // the result.
        const targetAngular = heldAxes.has('left') ? 15.0 : heldAxes.has('right') ? -15.0 : 0.0;

        // Ramp UP toward a held throttle position (eases the lever open over
        // THRUST_RAMP_RATE), but cut instantly to 0 on release instead of
        // fading via WATER_FRICTION_RATE. Real Gazebo drag
        // (vrx::SimpleHydrodynamics) now provides the actual coast-down —
        // ramping the outgoing demand down too kept commanding real, decaying
        // thrust for several seconds after release, double-applying
        // deceleration on top of real drag and masking its true
        // fast-then-slow (quadratic-then-linear) shape.
        currentLinear = targetLinear !== 0.0 ? approachVelocity(currentLinear, targetLinear, dt) : 0.0;
        currentAngular = targetAngular !== 0.0 ? approachVelocity(currentAngular, targetAngular, dt) : 0.0;

        // Hull already touching a solid boundary — kill push in whichever
        // direction (forward or reverse) is actually driving deeper into it
        // (like running aground into thick mud); the other direction always
        // stays free so the boat can back off instead of getting stuck.
        // (Not a forced opposite-thrust push: fighting a still-held key that
        // way re-triggers every frame as the ramp pulls back toward it,
        // which read as the boat vibrating in place at the boundary rather
        // than stopping cleanly. The widened ISLAND_KEEP_OUT boundary is
        // what actually keeps the boat clear of the visible island — this
        // only needs to hold the line, not shove back.)
        const blocked = (currentLinear > 0 && (isMovingIntoObstacle(1) || isExitingLake(1)))
            || (currentLinear < 0 && (isMovingIntoObstacle(-1) || isExitingLake(-1)));
        if (blocked) currentLinear = 0;

        // boatPos itself is NOT integrated here — it comes solely from the
        // /odom subscription above, so what's rendered is Gazebo's real
        // physics output, not a client-side guess. currentLinear/currentAngular
        // below only shape the outgoing /cmd_vel ramp (how fast a held key
        // opens the throttle), like a joystick position, not the boat's
        // actual resulting motion.

        // Publish while moving/decaying; once settled at zero, send one
        // final stop and go quiet rather than flooding the bridge forever.
        const stillish = currentLinear === 0.0 && currentAngular === 0.0;
        const wasPublishingMotion = lastPublishedLinear !== 0.0 || lastPublishedAngular !== 0.0;
        if (!stillish || wasPublishingMotion) {
            sendCmdVel(currentLinear, currentAngular);
            lastPublishedLinear = currentLinear;
            lastPublishedAngular = currentAngular;
        }

        // Telemetry panel: this is the only place driving it in Mode 2 (the
        // status-machine block further down only runs during Mode 1/3
        // auto-nav), so without this it just sits frozen on stale values.
        const teleStatusEl = document.getElementById('tele-status');
        if (teleStatusEl) {
            if (blocked) {
                teleStatusEl.textContent = '🚧 Hull Contact — Reverse to Clear';
                teleStatusEl.style.color = '#ff8800';
            } else if (stillish) {
                teleStatusEl.textContent = '⚓ Idle (Manual Mode)';
                teleStatusEl.style.color = '#ffc107';
            } else {
                teleStatusEl.textContent = '🕹️ Manual Drive';
                teleStatusEl.style.color = '#00ffcc';
            }
        }
        document.getElementById('tele-x').textContent = boatPos.x.toFixed(2);
        document.getElementById('tele-y').textContent = boatPos.y.toFixed(2);
        document.getElementById('tele-speed').textContent = boatPos.speed.toFixed(2);
    }

    requestAnimationFrame(thrusterLoop);
}
requestAnimationFrame(thrusterLoop);
