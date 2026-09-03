// ================= UI INPUT WIRING =================
// Tool buttons, the map-click handler (obstacle/goal placement in Mode 1,
// berth selection in Mode 3), run/reset buttons, mode-switch buttons, and
// Mode 2's keyboard + on-screen thruster controls. Depends on everything
// else (config.js, state.js, ros.js, boundaries.js, pathfinding.js,
// render-2d.js, navigation.js, docking.js, lifecycle.js, utils.js) — this
// is UI glue, not core logic, so it's fine for it to depend on all of it.
// Loads before main.js (main.js only needs draw() to start after the DOM
// listeners below are attached, and attaching a listener doesn't require
// the render loop to already be running).

let hoveredBerth = null;
// 0.4 slow, 0.6 medium, 1.0 fast — fractions of real max thrust (see
// config.js's MAX_LINEAR_FWD/REV and THRUSTER_MAX_FWD_N/MIN_REV_N). Fast
// stops at 1.0, not higher: the previous 1.6 asked the combined drive for
// 3.98 m/s, a target this boat's real T200s can't reach (they saturate at
// the same ~2.49 m/s "medium" already hits — see /cmd_vel_thrust_mixer.py's
// KP_LINEAR clamp), so "fast" had zero effect beyond medium.
let boatSpeedMultiplier = 0.6;

// ================= CUSTOM POPUP (replaces window.alert()/confirm()) =================
// A blocking window.alert()/confirm() can't be acted on by a gamepad at all
// — no gamepad input reaches a native browser dialog, and it pauses ALL page
// JS (including thrusterLoop's requestAnimationFrame loop below) while open,
// so a gamepad's own polling loop can't even run to detect a button press.
// This non-blocking overlay replaces every alert()/confirm() call in the app
// (this file, docking.js, navigation.js) so a connected controller can act
// on it with Cross (✕ = OK) / Circle (◯ = Cancel) — wired up in thrusterLoop
// below — exactly like clicking the on-screen buttons or pressing
// Enter/Escape.
const customAlertOverlay = document.getElementById('custom-alert-overlay');
const customAlertMessageEl = document.getElementById('custom-alert-message');
const customAlertIconEl = document.getElementById('custom-alert-icon');
const customAlertCancelBtn = document.getElementById('custom-alert-cancel');
let customPopupOnConfirm = null; // set only for a showConfirm() — null means "plain alert, nothing to run on OK"

function isCustomAlertVisible() {
    return !!customAlertOverlay && customAlertOverlay.style.display !== 'none';
}

// A single OK button, no real choice — same as window.alert(). `icon` is a
// single emoji shown as the popup's big badge — pass one fitting the
// message's actual cause (🌲 island, ⛔ occupied, ...) instead of the
// generic default, and leave any emoji already inline in `message` itself
// out of it (the badge is doing that job now, so it isn't duplicated).
function showAlert(message, icon = '⚠️') {
    showPopup(message, null, icon);
}

// OK/Cancel — same as window.confirm(), but onConfirm only runs if OK/Cross
// is chosen (there's no "wantsReset" boolean to check afterward the way
// confirm()'s return value worked, since this can't block for one — callers
// pass what should happen on confirm directly).
function showConfirm(message, onConfirm, icon = '⚠️') {
    showPopup(message, onConfirm, icon);
}

function showPopup(message, onConfirm, icon) {
    if (!customAlertOverlay || !customAlertMessageEl) { // defensive fallback, should never hit
        if (onConfirm) { if (confirm(message)) onConfirm(); } else alert(message);
        return;
    }
    customAlertMessageEl.textContent = message;
    if (customAlertIconEl) customAlertIconEl.textContent = icon;
    if (customAlertCancelBtn) customAlertCancelBtn.style.display = onConfirm ? 'inline-flex' : 'none';
    customPopupOnConfirm = onConfirm;
    customAlertOverlay.style.display = 'flex';
}

function hideCustomAlert() {
    if (customAlertOverlay) customAlertOverlay.style.display = 'none';
}

// OK / Cross — runs the pending confirm callback, if this was a showConfirm().
function confirmCustomPopup() {
    const cb = customPopupOnConfirm;
    hideCustomAlert();
    if (cb) cb();
}

// Cancel / Circle / Escape — always just closes, callback never runs.
function cancelCustomPopup() {
    hideCustomAlert();
}

document.getElementById('custom-alert-ok')?.addEventListener('click', confirmCustomPopup);
customAlertCancelBtn?.addEventListener('click', cancelCustomPopup);
customAlertOverlay?.addEventListener('click', (e) => {
    if (e.target === customAlertOverlay) cancelCustomPopup(); // backdrop click, not the box itself
});
window.addEventListener('keydown', (e) => {
    if (!isCustomAlertVisible()) return;
    if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); confirmCustomPopup(); }
    else if (e.key === 'Escape') { e.preventDefault(); cancelCustomPopup(); }
});

canvas.addEventListener('mousemove', (e) => {
    if (activeAppMode !== 3) { hoveredBerth = null; return; }
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const { rx, ry } = canvasToRos(cx, cy);
    const dockTypeSel = document.getElementById('dock-type-selector');
    const isParallel = dockTypeSel ? (dockTypeSel.value === 'parallel') : true;
    // {berth, status} now, not just a dockable-or-null berth — see
    // probeBerthHoverStatus() (docking.js) for why: it lets the hover
    // preview (render-2d.js) show occupied/blocked spots in red/orange
    // instead of going silent on them.
    hoveredBerth = probeBerthHoverStatus(rx, ry, isParallel);
});


// Only sends flashBoostTopic on an actual change (mirrors the
// manualThrustOverrideActive/turnReserveTopic pattern in ros.js/lifecycle.js
// — publish on transition, not every call) — cmd_vel_thrust_mixer.py swaps
// its real 51.5N/-40.2N ceiling for the unrealistic 154.5N/-120.6N one while
// this is true (see exhibition_water.sdf's matching Thruster plugin
// headroom). Only the combined-drive (cmd_vel) scheme needs this; W/A/R/D's
// raw thrust-topic path bypasses the mixer and gets the same headroom
// straight from the SDF regardless of this flag.
function setFlashBoost(active) {
    if (flashBoostActive === active) return;
    flashBoostActive = active;
    flashBoostTopic.publish(new ROSLIB.Message({ data: active }));
}

// Ordered slowest -> fastest so the gamepad D-Pad (thrusterLoop) can cycle
// through them by index — the mouse buttons below all reuse this same list
// rather than duplicating the multiplier/id pairs.
const SPEED_PRESETS = [
    { id: 'speed-slow', mult: 0.4 },
    { id: 'speed-medium', mult: 0.6 },
    { id: 'speed-fast', mult: 1.0 },
    { id: 'speed-flash', mult: 3.0 }, // intentionally unrealistic — see setFlashBoost()
];

SPEED_PRESETS.forEach(preset => {
    document.getElementById(preset.id)?.addEventListener('click', () => {
        boatSpeedMultiplier = preset.mult;
        document.querySelectorAll('#mode2-tools .dbtn').forEach(b => b.classList.remove('active'));
        document.getElementById(preset.id).classList.add('active');
        setFlashBoost(preset.id === 'speed-flash');
    });
});

// Gamepad D-Pad Left/Right (thrusterLoop) — steps one preset at a time
// rather than wrapping, so Left at Slow (or Right at Flash) is a no-op
// instead of jumping straight from one extreme to the other.
function cycleSpeedPreset(direction) {
    const currentIndex = SPEED_PRESETS.findIndex(p => p.mult === boatSpeedMultiplier);
    const nextIndex = Math.max(0, Math.min(SPEED_PRESETS.length - 1, (currentIndex === -1 ? 1 : currentIndex) + direction));
    document.getElementById(SPEED_PRESETS[nextIndex].id)?.click();
}

// UI Interactions
document.querySelectorAll('.tool-btn').forEach(btn => {
    btn.addEventListener('click', (e) => {
        document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
        e.currentTarget.classList.add('active');
        currentMode = e.currentTarget.dataset.type;
    });
});

// Shared by both placement surfaces (2D tactical map below, and the 3D FPV
// view further down) — clamps a raw ROS-frame click point to Mode 1's
// open-water/island constraints, then either sets the goal or spawns a new
// buoy/dynamic-boat entity (both locally and on the real Gazebo backend via
// spawnTopic), exactly as the 2D map click handler always did. Callers are
// responsible for their own activeAppMode/currentMode gating first.
function placeMode1EntityAt(rx, ry) {
    // Enforce Lake Open-Water Constraints (Keep goals & buoys inside water body).
    // 330 keeps the same ~5% margin inside LAKE_RADIUS's own 350 pre-scale
    // reference that the old 285/300 pair had.
    const distFromOrigin = Math.hypot(rx, ry);
    if (distFromOrigin > 330.0 * WORLD_SCALE) {
        rx = (rx / distFromOrigin) * (330.0 * WORLD_SCALE);
        ry = (ry / distFromOrigin) * (330.0 * WORLD_SCALE);
    }

    // Reject placement on/near the Central Island outright (r < 33.0m from
    // island center) instead of silently sliding it to the nearest open-
    // water edge — that used to place something at a spot the player never
    // actually pointed at, which read as confusing/broken rather than
    // deliberate (worse for a kid using the gamepad cursor, where the
    // crosshair itself sits right over the island when this fires).
    const distFromIsland = Math.hypot(rx - ISLAND_X, ry - ISLAND_Y);
    if (distFromIsland < ISLAND_KEEP_OUT) {
        showAlert('Too close to the island — pick a spot in open water.', '🌲');
        return;
    }

    if (currentMode === 'goal') {
        entities = entities.filter(ent => ent.type !== 'goal');
        currentGoal = { type: 'goal', ros_x: rx, ros_y: ry, id: 'goal_node' };
        entities.push(currentGoal);
    } else {
        // Cap player-placed buoys/moving boats (MAX_BUOYS/MAX_MOVING_BOATS,
        // config.js) — excludes the baked-in moored marina boats
        // (isParkedShip), which aren't player-placed. Checked here, ahead of
        // the hull-collision check below, so both the mouse-click and
        // gamepad placement paths (both funnel through this one function)
        // get the same limit for free.
        const maxForType = currentMode === 'dynamic' ? MAX_MOVING_BOATS : MAX_BUOYS;
        const countOfType = entities.filter(ent => ent.type === currentMode && !ent.isParkedShip).length;
        if (countOfType >= maxForType) {
            const label = currentMode === 'dynamic' ? 'moving boats' : 'buoys';
            showAlert(`Max ${maxForType} ${label} reached — remove one before adding more.`, '🚧');
            return;
        }

        // Reject a spawn point that would already be touching the boat's
        // own hull at its current (stationary, design-phase) pose — see
        // wouldObstaclePlacementCollideWithBoat() for why this matters now
        // that hull-touch detection is hull-box-aware instead of a flat
        // center-to-center radius.
        if (wouldObstaclePlacementCollideWithBoat(rx, ry, currentMode)) {
            showAlert('Too close to your boat — pick a spot further away.', '🚤');
            return;
        }

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
}

// Only way to get back under MAX_BUOYS/MAX_MOVING_BOATS once you've hit the
// cap — the Reset button clears everything, this clears just one. Removes
// whichever player-placed buoy/moving-boat is nearest (rx, ry), goal and the
// baked-in moored marina boats excluded, if anything is within snap range.
// Silently no-ops otherwise (Design Level tool is trial-and-error by
// nature — a miss isn't an error worth an alert). Removes the real Gazebo
// model too via a 'remove' spawn message (obstacle_spawner.py), the
// single-entity counterpart to Reset's remove-everything 'reset' message.
// Wider than a placement snap radius on purpose — a moving boat entity
// drifts (updateDynamicEntities(), navigation.js), so by the time a press
// registers it may no longer be exactly where it looked when aimed at;
// generous range keeps that from reading as "removal doesn't work."
const REMOVE_SNAP_RADIUS = 12.0;

function removeMode1EntityAt(rx, ry) {
    let closest = null;
    let closestDist = REMOVE_SNAP_RADIUS;
    entities.forEach(ent => {
        if (ent.isParkedShip || (ent.type !== 'static' && ent.type !== 'dynamic')) return;
        const d = Math.hypot(ent.ros_x - rx, ent.ros_y - ry);
        if (d < closestDist) { closestDist = d; closest = ent; }
    });
    if (!closest) return;

    entities = entities.filter(ent => ent !== closest);
    const mesh = threeEntities.get(closest.id);
    if (mesh) {
        scene.remove(mesh);
        threeEntities.delete(closest.id);
    }

    spawnTopic.publish(new ROSLIB.Message({
        data: JSON.stringify({ type: 'remove', id: closest.id })
    }));
}

canvas.addEventListener('click', (e) => {
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    let { rx, ry } = canvasToRos(cx, cy);

    // In Mode 3, click near any pier edge to calculate dynamic autonomous
    // parking — attemptDockAt() (docking.js) does the actual detection/
    // validation/kickoff, shared with the gamepad's Cross-press (thrusterLoop).
    if (activeAppMode === 3) {
        attemptDockAt(rx, ry);
        return;
    }

    if (activeAppMode !== 1 || !currentMode) return;
    if (currentMode === 'remove') removeMode1EntityAt(rx, ry);
    else placeMode1EntityAt(rx, ry);
});

// Place obstacles/goal directly in the 3D Object Detection camera view, same
// tool selection (🟡/🚢/🏁) as the 2D tactical map — click a spot in the 3D
// view and it raycasts against the water plane (y=0) using the SAME `camera`
// that follows the boat (scene-environment.js), turning the click into a
// ROS-frame (x, y) point the exact same way canvasToRos() does for the 2D map
// (Three.js z = -ROS y, matching every mesh placement in this codebase, e.g.
// boatGroup/entities-3d.js). Reuses placeMode1EntityAt() so all placement
// surfaces share the exact same clamping/spawn/publish logic — clicking any
// of them has identical effect on the actual simulation.
//
// Bound to detectionOverlayCanvas rather than renderer.domElement: the plain
// "3D Realistic FPV" panel is no longer shown on its own (its canvas now
// renders off-screen purely as the detection panel's pixel source — see
// index.html/style.css), so it can no longer receive clicks; the detection
// view is the only visible 3D surface left.
const raycaster = new THREE.Raycaster();
const raycastMouse = new THREE.Vector2();
const waterPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
detectionOverlayCanvas.addEventListener('click', (e) => {
    if (activeAppMode !== 1 || !currentMode) return;

    const rect = detectionOverlayCanvas.getBoundingClientRect();
    raycastMouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    raycastMouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(raycastMouse, camera);
    const hit = new THREE.Vector3();
    if (!raycaster.ray.intersectPlane(waterPlane, hit)) return; // camera looking away from the water plane

    if (currentMode === 'remove') removeMode1EntityAt(hit.x, -hit.z);
    else placeMode1EntityAt(hit.x, -hit.z);
});

// ▶️ RUN ASV DEMO Button
document.getElementById('btn-run').addEventListener('click', () => {
    if (!currentGoal) {
        showAlert("Please set a Target Goal first before running!", '🏁');
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
    ilosState = { k: 1, y_int: 0 };
    update3DPathLine(); // draw the 3D route line immediately, not just once the first 250ms replan tick fires

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

// Debounces rapid repeated Reset presses (mashing the mouse button, or
// Options on the gamepad) — each press fires its own async 'set_pose'/
// 'reset' calls to Gazebo (subprocess.Popen in obstacle_spawner.py, no
// queueing), and resetBoatToPose()'s ignoreOdomUntil window is exactly what
// stops a stale pre-reset /odom reading from clobbering the fresh pose in
// the meantime — pressing Reset again WITHIN that window doesn't reset any
// faster, it just fires a second overlapping teleport that can land after
// the first one's ignoreOdomUntil has already expired, and briefly show
// wherever THAT second one hasn't finished landing yet. Matches
// resetBoatToPose's own 1000ms window exactly, so a new Reset is only ever
// accepted once the previous one has fully settled.
let lastMode1ResetAt = 0;
document.getElementById('btn-reset').addEventListener('click', () => {
    if (Date.now() - lastMode1ResetAt < 1000) return;
    lastMode1ResetAt = Date.now();

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

// Persisted across reloads (see the restore block below this) so refreshing
// the page returns to whichever mode was active beforehand, at that mode's
// own starting pose, instead of always reloading into Mode 1.
const ACTIVE_MODE_STORAGE_KEY = 'avraActiveMode';

if (mode1Btn) {
    mode1Btn.addEventListener('click', () => {
        if (activeAppMode === 2) stopThrusters();
        activeAppMode = 1;
        gamepadCursor.x = MODE1_START.x + 15; gamepadCursor.y = MODE1_START.y; // same "start clean" rule every mode switch follows; +15 so it doesn't start exactly on the boat (state.js)
        gamepadPerspective = 'map';
        resetBoatToPose(MODE1_START);
        clearMode1Design();
        localStorage.setItem(ACTIVE_MODE_STORAGE_KEY, '1');
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
        gamepadThrusterMode = false; // always re-enter Mode 2 in Cruise, same "start clean" rule every mode switch already follows
        resetBoatToPose(MODE2_START);
        clearMode1Design();
        localStorage.setItem(ACTIVE_MODE_STORAGE_KEY, '2');
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
        gamepadCursor.x = MODE3_START.x; gamepadCursor.y = MODE3_START.y; // same "start clean" rule every mode switch follows
        resetBoatToPose(MODE3_START);
        clearMode1Design();
        localStorage.setItem(ACTIVE_MODE_STORAGE_KEY, '3');
        mode3Btn.classList.add('active');
        if (mode1Btn) mode1Btn.classList.remove('active');
        if (mode2Btn) mode2Btn.classList.remove('active');
        if (mode3Tools) mode3Tools.style.display = 'block';
        if (mode1Tools) mode1Tools.style.display = 'none';
        if (mode2Tools) mode2Tools.style.display = 'none';
        if (dofPanel) dofPanel.style.display = 'block';
    });
}

// Restore whichever mode was active before a refresh. state.js already
// boots activeAppMode/boatPos at Mode 1's own start pose, so only 2/3 need
// to replay their button's full switch logic (pose reset, tool-panel
// visibility, active-button highlight). Guarded in case a browser has
// localStorage disabled (throws instead of returning null).
try {
    const savedMode = localStorage.getItem(ACTIVE_MODE_STORAGE_KEY);
    if (savedMode === '2' && mode2Btn) mode2Btn.click();
    else if (savedMode === '3' && mode3Btn) mode3Btn.click();
} catch (e) { /* localStorage unavailable — just stay on Mode 1's default start */ }

// Mode 2 Controls: continuous thruster + water-friction velocity model.
// A held control ramps quickly toward its fixed end velocity (thrusters
// "open"); releasing it lets velocity decay gradually toward zero at the
// water-friction rate instead of snapping to zero immediately.

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
    // Safety-net path (blur/reset) — release the mixer override here too, or
    // losing focus mid-W/R-press would leave cmd_vel_thrust_mixer.py paused
    // forever with no thruster key left held to ever trigger the release above.
    if (manualThrustOverrideActive) {
        manualThrustOverrideActive = false;
        manualOverrideTopic.publish(new ROSLIB.Message({ data: false }));
    }
    // Same reasoning: Flash Mode's boosted mixer ceiling must never survive
    // into Mode 1/3's autonomous /cmd_vel nav, which shares this same mixer
    // — every path that leaves Mode 2 (mode-switch, blur, STOP) routes
    // through here, so this is the one place that needs to guarantee it off.
    setFlashBoost(false);
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

// Mode 2's STOP button — not just a thruster kill, also snaps the boat back
// to Mode 2's own starting pose, same "stop == back to this mode's start"
// behavior as Mode 1's btn-reset and Mode 3's btn-reset-dock.
function stopAndResetMode2() {
    stopThrusters();
    gamepadThrusterMode = false;
    resetBoatToPose(MODE2_START);
}

const btnStop = document.getElementById('btn-stop');
if (btnStop) {
    btnStop.addEventListener('click', stopAndResetMode2);
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

// ================= PS4 / GAMEPAD CONTROL (all modes) =================
// Uses the browser's Gamepad API directly — a controller connected to this
// machine (USB or Bluetooth-paired to the OS) shows up here with no Docker/
// container involvement at all, unlike the README's "physical joystick"
// caveat (that's about a joystick recognized inside the container via a ROS
// `joy` node, a separate and much more involved path we don't need here
// since every mode already does its driving/UI client-side in JS).
const GAMEPAD_DEADZONE = 0.15; // ignore stick drift/noise near center
// Standard Gamepad API mapping — index, not the OS's button icon, so this
// holds regardless of platform. See https://w3c.github.io/gamepad/#remapping
const GP_BTN = {
    CROSS: 0, CIRCLE: 1, SQUARE: 2, TRIANGLE: 3,
    L1: 4, R1: 5, L2: 6, R2: 7, OPTIONS: 9, R3: 11,
    DPAD_UP: 12, DPAD_DOWN: 13, DPAD_LEFT: 14, DPAD_RIGHT: 15,
};
// Cursor speed in canvas PIXELS/sec (converted to ROS meters via the
// current view's own scale below) rather than a flat meters/sec figure — so
// the cursor covers the same visual distance per second in both Mode 1's
// wide 1.35x view and Mode 3's zoomed-in 3.5x marina view, instead of
// feeling twitchy-fast once zoomed in (a flat meters/sec figure tuned for
// Mode 1 would fly across Mode 3's much smaller on-screen area).
const GAMEPAD_CURSOR_PIXEL_SPEED = 220;
// Camera perspective doesn't use the pixel/view-scale conversion above at
// all — that conversion exists to match the 2D map's own orthographic
// scale, which has no real equivalent for a perspective camera view. A
// flat, slower ROS m/s figure instead: placing something precisely while
// watching the camera needs finer control than sweeping the wide-open 2D
// map does, so this is deliberately well under the map's own effective
// speed (220px / 1.35 view-scale ≈ 163 m/s at Mode 1's default zoom).
const GAMEPAD_CURSOR_CAMERA_SPEED = 10; // ROS meters/sec
const GAMEPAD_CURSOR_MAX_RADIUS = 330.0 * WORLD_SCALE; // matches placeMode1EntityAt's own open-water clamp
// How far in from each edge of the camera canvas counts as "off-frame" for
// containment purposes (isRosPointVisibleInCamera below) — matches
// renderGamepadCursorDetection's own off-screen cutoff, so the cursor never
// gets clamped to sit right at a point it's already invisible.
const CAMERA_VIEW_EDGE_MARGIN = 20;
// How far in front of the boat (ROS meters, along the camera's own flat
// forward direction) the cursor snaps to when switching TO camera
// perspective (R3) finds its current spot isn't visible in that view — the
// boat itself is always dead-center of this camera (main.js's
// camera.lookAt(boatPos...)), so a modest offset from it reliably lands
// somewhere both visible and not immediately hull-colliding.
const GAMEPAD_CAMERA_DEFAULT_DISTANCE = 12;
// Reusable scratch vectors for the camera-relative cursor movement below —
// avoids allocating a fresh THREE.Vector3 every frame just to throw it away.
const gamepadCamForward = new THREE.Vector3();
const gamepadCamRight = new THREE.Vector3();
const gamepadCamCheckPoint = new THREE.Vector3();
const gamepadCamCheckForward = new THREE.Vector3();
const GAMEPAD_WORLD_UP = new THREE.Vector3(0, 1, 0);

// True if (rx, ry) would currently render somewhere inside the camera
// view's own frame (not behind the camera, not off either edge) — used to
// keep the gamepad cursor from wandering somewhere the player can't
// actually see it while Camera perspective is selected, and to decide
// whether switching TO that perspective needs to relocate the cursor first
// (see the R3 handler below). Reuses projectToOverlay() (detection-
// overlay.js, loads before this file) — the exact same projection the
// crosshair's own on-screen drawing and the live detection boxes use, so
// this can never disagree with what's actually visible.
function isRosPointVisibleInCamera(rx, ry) {
    gamepadCamCheckPoint.set(rx, 0, -ry); // world Z = -ROS Y, same convention as everywhere else
    camera.getWorldDirection(gamepadCamCheckForward);
    const toPoint = gamepadCamCheckPoint.clone().sub(camera.position);
    if (toPoint.dot(gamepadCamCheckForward) <= 0) return false; // behind the camera
    const p = projectToOverlay(gamepadCamCheckPoint); // mutates gamepadCamCheckPoint via .project(camera) — fine, it's a scratch var
    return p.x >= CAMERA_VIEW_EDGE_MARGIN && p.x <= detectionOverlayCanvas.width - CAMERA_VIEW_EDGE_MARGIN &&
        p.y >= CAMERA_VIEW_EDGE_MARGIN && p.y <= detectionOverlayCanvas.height - CAMERA_VIEW_EDGE_MARGIN;
}

// Places the cursor a fixed distance ahead of the boat along the camera's
// own flat forward direction — see GAMEPAD_CAMERA_DEFAULT_DISTANCE above
// for why this reliably lands somewhere visible.
function snapCursorToCameraDefault() {
    camera.getWorldDirection(gamepadCamForward);
    gamepadCamForward.y = 0;
    gamepadCamForward.normalize();
    gamepadCursor.x = boatPos.x + gamepadCamForward.x * GAMEPAD_CAMERA_DEFAULT_DISTANCE;
    gamepadCursor.y = boatPos.y - gamepadCamForward.z * GAMEPAD_CAMERA_DEFAULT_DISTANCE;
}

function applyDeadzone(v) {
    return Math.abs(v) < GAMEPAD_DEADZONE ? 0 : v;
}

// First connected gamepad, or null. Good enough for a single-controller
// exhibit booth — doesn't try to track/prefer a specific one.
function getActiveGamepad() {
    const pads = navigator.getGamepads ? navigator.getGamepads() : [];
    for (const gp of pads) {
        if (gp) return gp;
    }
    return null;
}

// Edge-detects a button press (true only on the frame it goes from up to
// down) using the shared gamepadButtonPrev map (state.js) — every button
// gets updated every frame regardless of which mode acts on it, so a button
// physically held while switching modes doesn't read as a fresh press the
// instant the new mode starts looking at it.
function gamepadButtonJustPressed(gp, index) {
    const pressed = !!(gp.buttons[index] && gp.buttons[index].pressed);
    const wasPressed = !!gamepadButtonPrev[index];
    gamepadButtonPrev[index] = pressed;
    return pressed && !wasPressed;
}

// Kept deliberately short (fits 1-2 lines at the sidebar's 250px width,
// unlike an earlier longer-sentence version that pushed the Boat Speed
// controls below the fold) — this panel is read on a kiosk touchscreen at a
// public exhibit, so it can't rely on a hover tooltip a finger can't
// trigger, and each mode's mapping is stated right in the live status line.
// Terse on purpose — per-button mappings live on the buttons themselves
// (data-label-gamepad swaps via applyGamepadUiMode()) rather than being
// spelled out again here too. Only the one thing that's genuinely live
// state — which of the two driving schemes is active — needs a per-frame
// update at all.
function updateGamepadStatusMode2(gp) {
    const el = document.getElementById('gamepad-status');
    if (!el) return;
    el.textContent = !gp ? '🎮 Connect a controller' : (gamepadThrusterMode ? '🎮 Twin Thruster' : '🎮 Cruise');
}

// The chip itself is now static markup (index.html swaps "Connect a
// controller" for "Stick = Cursor" purely via the gamepad-mode CSS class —
// see applyGamepadUiMode()); this only has to keep the live buoy/moving-boat
// counts on their own buttons up to date, e.g. "🟡 Buoy (□) 3/10".
function updateGamepadStatusMode1(gp) {
    if (!gp) return;
    const buoyCount = entities.filter(e => e.type === 'static' && !e.isParkedShip).length;
    const boatCount = entities.filter(e => e.type === 'dynamic').length;
    const buoyBtn = document.getElementById('btn-static-buoy');
    const boatBtn = document.getElementById('btn-moving-boat');
    if (buoyBtn) buoyBtn.textContent = `🟡 Buoy (□) ${buoyCount}/${MAX_BUOYS}`;
    if (boatBtn) boatBtn.textContent = `🚢 Boat (△) ${boatCount}/${MAX_MOVING_BOATS}`;
}

// Applies gamepadPerspective (state.js, toggled by R3) as a yellow frame
// around whichever view panel is currently "live" — only touches the DOM on
// an actual change, not every frame. `active` is false outside Mode 1 or
// with no gamepad connected, clearing the frame from both panels.
let gamepadPerspectiveApplied = undefined; // deliberately not null — null IS a valid "cleared" state, this just forces the very first call through
function updateGamepadPerspectiveUI(active) {
    const wanted = active ? gamepadPerspective : null;
    if (wanted === gamepadPerspectiveApplied) return;
    gamepadPerspectiveApplied = wanted;
    document.getElementById('map-view-panel')?.classList.toggle('gamepad-perspective-active', wanted === 'map');
    document.getElementById('camera-view-panel')?.classList.toggle('gamepad-perspective-active', wanted === 'camera');
}

// Swaps every button carrying data-label-mouse/data-label-gamepad (Mode 1's
// tool/Run/Reset buttons, Mode 3's Cancel button) to its controller-icon
// wording, and toggles body.gamepad-mode — the CSS hook (style.css) that
// hides mouse/keyboard-only help text once a controller is doing the work.
// Called only on an actual connect/disconnect transition (thrusterLoop
// tracks that), not every frame — this rewrites text nodes, which a 60fps
// loop shouldn't be doing when nothing has changed.
function applyGamepadUiMode(connected) {
    document.body.classList.toggle('gamepad-mode', connected);
    document.querySelectorAll('[data-label-gamepad]').forEach(el => {
        el.textContent = connected ? el.dataset.labelGamepad : el.dataset.labelMouse;
    });
}
let gamepadWasConnected = false;

// Mode 1's Reset/Run/place-tool buttons all reuse the existing mouse-driven
// buttons/handlers via .click() rather than duplicating their logic — same
// approach the mode-restore-on-reload code above already uses
// (mode2Btn.click()), and it means a gamepad press can never drift out of
// sync with what a mouse click does.
function gamepadResetActiveMode() {
    if (activeAppMode === 1) document.getElementById('btn-reset')?.click();
    else if (activeAppMode === 2) document.getElementById('btn-stop')?.click();
    else if (activeAppMode === 3) document.getElementById('btn-reset-dock')?.click();
}

// L1/R1 (previous/next) cycle between the three modes — same "click the
// real button" reuse as everything else here, so it goes through the exact
// same reset/pose/UI logic a mouse click on that mode's tab does. Global
// (checked every frame regardless of activeAppMode, like Options above),
// not gated behind gamepadCursor.active or any per-mode block.
function cycleMode(direction) {
    const nextMode = ((activeAppMode - 1 + direction + 3) % 3) + 1;
    document.getElementById(`mode${nextMode}-btn`)?.click();
}

function thrusterLoop() {
    const now = performance.now();
    const dt = Math.min((now - lastThrusterTime) / 1000, 0.1); // clamp so a stalled tab doesn't jump velocity
    lastThrusterTime = now;

    // A custom alert is showing — treat it as fully modal (matching what a
    // blocking window.alert() used to do to this entire loop) except for the
    // gamepad's own Cross-to-dismiss, which needs the loop still running to
    // ever detect it. Everything below (driving, cursor, placement, D-pad)
    // is skipped entirely until it's dismissed.
    if (isCustomAlertVisible()) {
        const gpForAlert = getActiveGamepad();
        if (gpForAlert && gamepadButtonJustPressed(gpForAlert, GP_BTN.CROSS)) confirmCustomPopup();
        else if (gpForAlert && gamepadButtonJustPressed(gpForAlert, GP_BTN.CIRCLE)) cancelCustomPopup();
        requestAnimationFrame(thrusterLoop);
        return;
    }

    // Polled every frame regardless of mode: Options (☰) is the one gamepad
    // button that means the same thing everywhere (STOP & RESET), so its
    // edge-detection has to keep running even in modes with no other
    // gamepad handling, or it'll misfire the moment some mode's handling
    // starts caring about button state again.
    const gp = getActiveGamepad();
    if (!!gp !== gamepadWasConnected) { applyGamepadUiMode(!!gp); gamepadWasConnected = !!gp; }
    if (gp && gamepadButtonJustPressed(gp, GP_BTN.OPTIONS)) gamepadResetActiveMode();
    // L1/R1 switch modes — also global, also unconditional on activeAppMode.
    if (gp && gamepadButtonJustPressed(gp, GP_BTN.L1)) cycleMode(-1);
    if (gp && gamepadButtonJustPressed(gp, GP_BTN.R1)) cycleMode(1);

    if (activeAppMode === 2) {
        updateGamepadStatusMode2(gp);
        // Circle (◯) toggles Cruise <-> Twin Thruster.
        if (gp && gamepadButtonJustPressed(gp, GP_BTN.CIRCLE)) gamepadThrusterMode = !gamepadThrusterMode;
        // D-Pad Left/Right cycles the Boat Speed presets (Slow..Flash) —
        // reuses the same buttons a mouse click would use, see cycleSpeedPreset().
        if (gp && gamepadButtonJustPressed(gp, GP_BTN.DPAD_LEFT)) cycleSpeedPreset(-1);
        if (gp && gamepadButtonJustPressed(gp, GP_BTN.DPAD_RIGHT)) cycleSpeedPreset(1);
    } else if (activeAppMode === 1) {
        updateGamepadStatusMode1(gp);
    }
    updateGamepadPerspectiveUI(activeAppMode === 1 && !!gp);
    const gamepadDrivingThrusters = activeAppMode === 2 && !!gp && gamepadThrusterMode;
    const gamepadDrivingCruise = activeAppMode === 2 && !!gp && !gamepadThrusterMode;

    // ---- Mode 1 & Mode 3: shared gamepad cursor (left stick) ----
    // Pixel-speed converted via the CURRENT view's scale (render-2d.js) so
    // it covers the same on-screen distance per second in both Mode 1's wide
    // view and Mode 3's zoomed-in marina view.
    gamepadCursor.active = (activeAppMode === 1 || activeAppMode === 3) && !!gp;
    if (gamepadCursor.active) {
        const moveRight = applyDeadzone(gp.axes[0]);    // stick right = positive
        const moveForward = -applyDeadzone(gp.axes[1]); // stick up (negative axis) = positive "forward"
        const inCameraPerspective = activeAppMode === 1 && gamepadPerspective === 'camera';

        if (inCameraPerspective) {
            // Camera perspective selected (R3, gamepadPerspective): "left"
            // on the stick means the camera's own screen-left, not map-west
            // — matches what the player is actually looking at instead of
            // needing to mentally translate map-north into whichever way
            // the boat's camera currently happens to be facing. The
            // crosshair itself still shows on BOTH panels either way
            // (renderGamepadCursor2D()/renderGamepadCursorDetection()) —
            // only the STICK's meaning changes with the selected view, not
            // where the resulting shared cursor is drawn. Deliberately
            // slower than map movement (GAMEPAD_CURSOR_CAMERA_SPEED) —
            // precise placement while watching this view needs finer
            // control than sweeping the wide-open map does.
            camera.getWorldDirection(gamepadCamForward);
            gamepadCamForward.y = 0;
            gamepadCamForward.normalize();
            gamepadCamRight.crossVectors(gamepadCamForward, GAMEPAD_WORLD_UP).normalize();
            const worldDX = gamepadCamForward.x * moveForward + gamepadCamRight.x * moveRight;
            const worldDZ = gamepadCamForward.z * moveForward + gamepadCamRight.z * moveRight;
            const candidateX = gamepadCursor.x + worldDX * GAMEPAD_CURSOR_CAMERA_SPEED * dt;
            const candidateY = gamepadCursor.y - worldDZ * GAMEPAD_CURSOR_CAMERA_SPEED * dt; // world Z = -ROS Y

            // Applied per-axis, not as one combined step: keeps the cursor
            // from wandering out of the camera's own frame (this is the
            // ONLY clamp in camera perspective — GAMEPAD_CURSOR_MAX_RADIUS
            // below still applies too, but the view itself is always the
            // tighter constraint) while still letting it slide along
            // whichever edge it's hit instead of freezing outright the
            // moment either axis alone would leave the frame.
            if (isRosPointVisibleInCamera(candidateX, gamepadCursor.y)) gamepadCursor.x = candidateX;
            if (isRosPointVisibleInCamera(gamepadCursor.x, candidateY)) gamepadCursor.y = candidateY;
        } else {
            // Map perspective (default) and Mode 3 (no camera-relative
            // option there): plain map-aligned movement — stick right/up
            // moves the cursor map-east/map-north, same as before.
            const cursorSpeed = GAMEPAD_CURSOR_PIXEL_SPEED / getCurrentViewParams().scale;
            gamepadCursor.x += moveRight * cursorSpeed * dt;
            gamepadCursor.y += moveForward * cursorSpeed * dt;
        }
        const cursorDist = Math.hypot(gamepadCursor.x, gamepadCursor.y);
        if (cursorDist > GAMEPAD_CURSOR_MAX_RADIUS) {
            const k = GAMEPAD_CURSOR_MAX_RADIUS / cursorDist;
            gamepadCursor.x *= k;
            gamepadCursor.y *= k;
        }

        if (activeAppMode === 1) {
            // Square/Triangle/Circle place a buoy/moving-boat/goal at the
            // cursor — selecting the tool via the real tool-btn's own
            // .click() (so the sidebar's active-tool highlight stays in
            // sync, exactly like a mouse-driven selection) and then placing
            // immediately, one press, no separate confirm step.
            if (gamepadButtonJustPressed(gp, GP_BTN.SQUARE)) {
                document.getElementById('btn-static-buoy')?.click();
                placeMode1EntityAt(gamepadCursor.x, gamepadCursor.y);
            }
            if (gamepadButtonJustPressed(gp, GP_BTN.TRIANGLE)) {
                document.getElementById('btn-moving-boat')?.click();
                placeMode1EntityAt(gamepadCursor.x, gamepadCursor.y);
            }
            if (gamepadButtonJustPressed(gp, GP_BTN.CIRCLE)) {
                document.getElementById('btn-goal')?.click();
                placeMode1EntityAt(gamepadCursor.x, gamepadCursor.y);
            }
            if (gamepadButtonJustPressed(gp, GP_BTN.CROSS)) {
                document.getElementById('btn-run')?.click();
            }
            // L2 removes the nearest buoy/moving-boat to the cursor,
            // regardless of which placement tool is currently selected —
            // deleting shouldn't require switching tools first. Same snap
            // radius and target set as the mouse's Remove tool
            // (removeMode1EntityAt() above). Not L1 — that's now global
            // mode-switching (see cycleMode()), same button in every mode.
            if (gamepadButtonJustPressed(gp, GP_BTN.L2)) {
                removeMode1EntityAt(gamepadCursor.x, gamepadCursor.y);
            }
            // R3 (right stick click) swaps which way the stick moves the
            // cursor (map-aligned vs camera-relative — see above) — a
            // yellow frame (updateGamepadPerspectiveUI()) marks whichever
            // is live. The crosshair itself keeps drawing on both panels
            // either way. Switching TO camera perspective specifically can
            // land the cursor somewhere that view's own frame doesn't
            // currently cover at all (e.g. it was out past the map's own
            // wide-open edges) — relocate it to a known-visible spot ahead
            // of the boat (snapCursorToCameraDefault()) rather than leaving
            // it stuck outside the containment clamp above from the very
            // first frame.
            if (gamepadButtonJustPressed(gp, GP_BTN.R3)) {
                gamepadPerspective = gamepadPerspective === 'map' ? 'camera' : 'map';
                if (gamepadPerspective === 'camera' && !isRosPointVisibleInCamera(gamepadCursor.x, gamepadCursor.y)) {
                    snapCursorToCameraDefault();
                    // Camera.lookAt(boatPos...) targets the boat every frame
                    // regardless of how far the camera's OWN position has
                    // lerped toward its resting spot (main.js's draw()) —
                    // right after a fresh page load/mode switch, before
                    // that lerp has caught up, even the default offset
                    // point above can transiently fall just outside frame.
                    // boatPos itself is always dead-center in that case, so
                    // it's the one guaranteed fallback.
                    if (!isRosPointVisibleInCamera(gamepadCursor.x, gamepadCursor.y)) {
                        gamepadCursor.x = boatPos.x;
                        gamepadCursor.y = boatPos.y;
                    }
                }
            }
        } else if (activeAppMode === 3) {
            // Drives the SAME hoveredBerth the mouse's mousemove listener
            // sets (above) — render-2d.js's hover preview (open/occupied/
            // blocked) doesn't need to know or care whether it came from a
            // mouse or the gamepad cursor.
            const dockTypeSel = document.getElementById('dock-type-selector');
            const isParallel = dockTypeSel ? (dockTypeSel.value === 'parallel') : true;
            hoveredBerth = probeBerthHoverStatus(gamepadCursor.x, gamepadCursor.y, isParallel);

            // Cross (✕) confirms/selects the berth under the cursor — same
            // validation + kickoff as a mouse click (docking.js's attemptDockAt()).
            if (gamepadButtonJustPressed(gp, GP_BTN.CROSS)) {
                attemptDockAt(gamepadCursor.x, gamepadCursor.y);
            }
        }
    }

    if (activeAppMode === 2 && (heldThrusterKeys.size > 0 || gamepadDrivingThrusters)) {
        // Independent per-thruster control takes priority over the combined
        // arrow-key drive whenever any W/A/R/D is held, so the two schemes
        // never both publish to the same thrust topics in the same frame.
        // Tell cmd_vel_thrust_mixer.py to stand down too — its 20Hz timer
        // otherwise keeps publishing to these same thrust topics from a
        // stale /cmd_vel target the whole time a thruster key is held,
        // fighting whatever's commanded below (only sent once per
        // press/release transition, not every frame).
        if (!manualThrustOverrideActive) {
            manualThrustOverrideActive = true;
            manualOverrideTopic.publish(new ROSLIB.Message({ data: true }));
        }
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
        // boatSpeedMultiplier applies here too (it didn't before — the speed
        // presets had no effect at all on W/A/R/D, only on the arrow-key
        // drive), so both control schemes honor the same slow/medium/fast
        // choice instead of W/A/R/D silently always running at full thrust.
        const fwdThrustScale = Math.max(0, Math.min(1, boundaryCappedSpeed(1, MAX_LINEAR_FWD) / MAX_LINEAR_FWD)) * boatSpeedMultiplier;
        const revThrustScale = Math.max(0, Math.min(1, boundaryCappedSpeed(-1, Math.abs(MAX_LINEAR_REV)) / Math.abs(MAX_LINEAR_REV))) * boatSpeedMultiplier;
        let leftThrust = heldThrusterKeys.has('leftFwd') ? THRUSTER_MAX_FWD_N * fwdThrustScale
            : heldThrusterKeys.has('leftRev') ? THRUSTER_MIN_REV_N * revThrustScale : 0.0;
        let rightThrust = heldThrusterKeys.has('rightFwd') ? THRUSTER_MAX_FWD_N * fwdThrustScale
            : heldThrusterKeys.has('rightRev') ? THRUSTER_MIN_REV_N * revThrustScale : 0.0;

        // Gamepad Twin Thruster mode: each stick's Y-axis drives one hull's
        // thruster directly, analog in both directions (unlike W/A/R/D,
        // which is on/off) — push up for forward, down for reverse, with the
        // same boundary/speed-preset scale (fwdThrustScale/revThrustScale)
        // the digital scheme uses. Overrides the digital leftThrust/
        // rightThrust above rather than combining with them, so a stray
        // WASD press doesn't fight the stick.
        if (gamepadDrivingThrusters) {
            const axisToThrust = (axis) => axis >= 0
                ? axis * THRUSTER_MAX_FWD_N * fwdThrustScale
                : axis * Math.abs(THRUSTER_MIN_REV_N) * revThrustScale;
            leftThrust = axisToThrust(-applyDeadzone(gp.axes[1]));  // stick up = negative axis = forward
            rightThrust = axisToThrust(-applyDeadzone(gp.axes[3]));
        }

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
                teleStatusEl.textContent = '⚙️ Twin Thruster Mode';
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

        // Hand control of the thrust topics back to cmd_vel_thrust_mixer.py
        // now that no thruster key is held — mirrors the override=true sent
        // above on entry, so the mixer's timer only ever sits out exactly
        // the span a thruster key was actually down.
        if (manualThrustOverrideActive) {
            manualThrustOverrideActive = false;
            manualOverrideTopic.publish(new ROSLIB.Message({ data: false }));
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
        // boatSpeedMultiplier is a THRUST fraction (same meaning W/A/R/D
        // gives it — see fwdThrustScale/revThrustScale above), not a speed
        // fraction: MAX_LINEAR_FWD*multiplier used to be commanded directly
        // as the velocity target, but because drag is quadratic, "60% of
        // top speed" only takes ~37% of top thrust — so the two control
        // schemes' identical-looking slow/medium/fast presets landed on
        // genuinely different real-world speeds (confirmed empirically:
        // arrows settled ~40% thrust vs W/R's exact 60% at the "medium"
        // preset). speedForThrust() solves the same drag equation
        // MAX_LINEAR_FWD/REV themselves come from, in reverse, to find the
        // velocity that actually draws that fraction of max combined
        // thrust, so both schemes now agree.
        let targetLinear = heldAxes.has('fwd') ? boundaryCappedSpeed(1, speedForThrust(THRUSTER_MAX_FWD_N * 2 * boatSpeedMultiplier))
            : heldAxes.has('rev') ? -boundaryCappedSpeed(-1, speedForThrust(Math.abs(THRUSTER_MIN_REV_N) * 2 * boatSpeedMultiplier))
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
        let targetAngular = heldAxes.has('left') ? 15.0 : heldAxes.has('right') ? -15.0 : 0.0;

        // Gamepad Cruise mode: left stick is analog and proportional (a
        // light push creeps, a full push hits the same cap the keyboard's
        // on/off arrow keys land on) instead of the keyboard's fixed target
        // — same combined-drive scheme, just smoother. Overrides the
        // keyboard's digital target above rather than combining with it.
        if (gamepadDrivingCruise) {
            const fwdAxis = -applyDeadzone(gp.axes[1]);  // stick up = negative axis = forward
            const turnAxis = -applyDeadzone(gp.axes[0]); // stick left = negative axis = turn left (positive angular)
            targetLinear = fwdAxis >= 0
                ? fwdAxis * boundaryCappedSpeed(1, speedForThrust(THRUSTER_MAX_FWD_N * 2 * boatSpeedMultiplier))
                : fwdAxis * boundaryCappedSpeed(-1, speedForThrust(Math.abs(THRUSTER_MIN_REV_N) * 2 * boatSpeedMultiplier));
            targetAngular = turnAxis * 15.0;
        }

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
                teleStatusEl.textContent = '🚤 Cruise Mode';
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
