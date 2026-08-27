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
}

canvas.addEventListener('click', (e) => {
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    let { rx, ry } = canvasToRos(cx, cy);

    // In Mode 3, click near any pier edge to calculate dynamic autonomous parking
    if (activeAppMode === 3) {
        const dockTypeSel = document.getElementById('dock-type-selector');
        const isParallel = dockTypeSel ? (dockTypeSel.value === 'parallel') : true;

        // Berth detection/validation moved to docking.js's detectBerthAtClick()
        // (same logic, same alert() messages on an invalid/occupied/blocked
        // spot — it returns null in those cases instead of returning here).
        const bestBerth = detectBerthAtClick(rx, ry, isParallel);
        if (!bestBerth) return;

        // Start Dynamic Docking Sequence!
        startDynamicDocking(bestBerth);
        return;
    }

    if (activeAppMode !== 1 || !currentMode) return;
    placeMode1EntityAt(rx, ry);
});

// Place obstacles/goal directly in the 3D FPV camera view, same tool
// selection (🟡/🚢/🏁) as the 2D tactical map — click a spot in the 3D view
// and it raycasts against the water plane (y=0) using the SAME `camera` that
// follows the boat (scene-environment.js), turning the click into a ROS-frame
// (x, y) point the exact same way canvasToRos() does for the 2D map (Three.js
// z = -ROS y, matching every mesh placement in this codebase, e.g. boatGroup/
// entities-3d.js). Reuses placeMode1EntityAt() so both views share the exact
// same clamping/spawn/publish logic — clicking either view has identical
// effect on the actual simulation.
const raycaster = new THREE.Raycaster();
const raycastMouse = new THREE.Vector2();
const waterPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
renderer.domElement.addEventListener('click', (e) => {
    if (activeAppMode !== 1 || !currentMode) return;

    const rect = renderer.domElement.getBoundingClientRect();
    raycastMouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    raycastMouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

    raycaster.setFromCamera(raycastMouse, camera);
    const hit = new THREE.Vector3();
    if (!raycaster.ray.intersectPlane(waterPlane, hit)) return; // camera looking away from the water plane

    placeMode1EntityAt(hit.x, -hit.z);
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
