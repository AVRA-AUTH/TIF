// ================= MODE 3: DYNAMIC DOCKING =================
// Berth detection (from a map click) + the actual docking-run kickoff.
// Depends on config.js, state.js, pathfinding.js (findOptimalPath),
// navigation.js (update3DPathLine), scene-marina.js (dynamic3DBerthMesh).

// Pre-existing dead code, kept as-is (not called from anywhere, and
// `berth3DMeshes` is never declared anywhere in this app either — this
// predates the file split and would throw if ever invoked; not touched
// here, just moved intact along with everything else per "no lost
// functionality, no silent behavior changes"): a leftover from the earlier
// static availableBerths/activeBerthIdx berth-selector UI, superseded by
// the dynamic click-to-dock flow below.
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

// Shared nearest-pier/jetty-face search, used by both the real click handler
// and the hover-preview probe below so the two can never disagree about
// where a berth candidate actually is. Given a map click/hover point already
// converted to ROS coordinates (rx, ry) and the user's parking-style choice,
// finds the nearest pier/jetty face within snap distance and returns the
// full berth descriptor (or null if nothing is within snap distance) — no
// occupied/obstacle validation here, see isBerthOccupied()/isBerthBlocked().
function findBerthDescriptor(rx, ry, isParallel) {
    let bestBerth = null;
    let minDist = 3.5; // 3.5m snap distance

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

    return bestBerth;
}

// Does this berth overlap a parked vessel on the finger jetties? Shared by
// the click handler and the hover probe so both agree on what's occupied.
// Checks the real moored-boat entities (isParkedShip) instead of a
// hardcoded "whole jetty y-range minus one named gap" heuristic — that
// heuristic predates scene-marina.js thinning each jetty to every OTHER
// slot (idx % 2 !== 0 skip, "more open choices"), so it was calling every
// one of those now-empty in-between slots "Occupied" too, with no boat
// actually there. 4.0m covers the real x offset between a boat's own
// position and this berth's computed docking-target x (up to ~2.4m) plus
// a little slack, while staying well under the ~6.5m gap to the next
// slot along the jetty (occupied or empty) so neighbors don't bleed in.
function isBerthOccupied(berth) {
    if (!berth.name.includes('Jetty')) return false;
    return entities.some(ent => ent.isParkedShip &&
        Math.hypot(ent.ros_x - berth.ros_x, ent.ros_y - berth.ros_y) < 4.0);
}

// Is this berth blocked by a Mode-1-placed static/dynamic obstacle? Shared
// the same way as isBerthOccupied() above.
// Excludes isParkedShip entities (the permanent marina fleet) — those are
// already handled, more precisely, by isBerthOccupied() above (a tighter
// 4.0m radius, scoped to the berth's own jetty slot). Without this
// exclusion, a boat legitimately moored in a NEIGHBORING slot (jetty boats
// are packed only ~2.6m apart) fell within this 5.5m radius too and marked
// an actually-open berth "Blocked" — real Mode-1-placed obstacles (buoys,
// patrol boats) never carry isParkedShip, so this only narrows the check to
// what the name/comment always said it was for.
function isBerthBlocked(berth) {
    return entities.some(ent => (ent.type === 'static' || ent.type === 'dynamic') && !ent.isParkedShip &&
        Math.hypot(ent.ros_x - berth.ros_x, ent.ros_y - berth.ros_y) < 5.5);
}

// Silent version for hover preview — same berth-search + validity logic as
// detectBerthAtClick() below (via the shared helpers above), no alert()s.
// Reports occupied/blocked spots too (with which reason), not just
// dockable-or-null, so sweeping the mouse along a packed jetty immediately
// shows red "already has a boat" / orange "blocked by a placed obstacle" vs
// green "open" in the hover preview (render-2d.js's renderMarina2D), instead
// of going silent on an invalid spot until you click and read an alert().
function probeBerthHoverStatus(rx, ry, isParallel) {
    const bestBerth = findBerthDescriptor(rx, ry, isParallel);
    if (!bestBerth) return null;
    if (isBerthOccupied(bestBerth)) return { berth: bestBerth, status: 'occupied' };
    if (isBerthBlocked(bestBerth)) return { berth: bestBerth, status: 'blocked' };
    return { berth: bestBerth, status: 'open' };
}

// Mode 3 berth detection — given a map click already converted to ROS
// coordinates (rx, ry) and the user's parking-style choice, finds the
// nearest pier/jetty face within snap distance, validates it's not already
// occupied or blocked, and returns the berth descriptor (or null if the
// click/spot was invalid — matches the original inline click-handler
// behavior exactly, including showing the same alert() messages here rather
// than at the call site, so behavior is identical either way).
function detectBerthAtClick(rx, ry, isParallel) {
    const bestBerth = findBerthDescriptor(rx, ry, isParallel);

    if (!bestBerth) {
        showAlert("No valid pier edge detected! Please click closer to a rigid wooden dock.", '🧭');
        return null;
    }

    // Check if selected spot overlaps a parked vessel on finger jetties
    if (isBerthOccupied(bestBerth)) {
        showAlert("Space Occupied! A parked vessel is currently docked at this location. Please select an open spot.", '⛔');
        return null;
    }

    // 3. Perception / Collision Check for obstacles
    if (isBerthBlocked(bestBerth)) {
        showAlert("OBSTACLE DETECTED! There is another vessel parked there. Choose an empty space.", '🚫');
        return null;
    }

    return bestBerth;
}

// Shared by the map click handler (input.js) and the gamepad's Cross-press
// (thrusterLoop, input.js) so both trigger a docking run through the exact
// same validation + kickoff, whether the point came from a mouse click or
// the gamepad cursor.
function attemptDockAt(rx, ry) {
    const dockTypeSel = document.getElementById('dock-type-selector');
    const isParallel = dockTypeSel ? (dockTypeSel.value === 'parallel') : true;
    const bestBerth = detectBerthAtClick(rx, ry, isParallel);
    if (!bestBerth) return;
    startDynamicDocking(bestBerth);
}

function startDynamicDocking(chosen) {
    isNavigating = false;
    isAutoDocking = true;
    dockingBerthName = chosen.name;
    // Full turn-thrust-reserve, Mode 3 only (see cmd_vel_thrust_mixer.py):
    // guarantees the boat can always turn at its full MAX_ANGULAR rate,
    // even at commanded cruise speed, at the cost of a lower real top speed
    // for the whole docking run (transit + docking legs alike). Turned back
    // off in lifecycle.js's resetBoatToPose() and navigation.js's
    // docking-complete branch.
    turnReserveTopic.publish(new ROSLIB.Message({ data: true }));

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

    // buildDockingApproachWaypoints() (pathfinding.js) validates the final
    // align->creep hop and A*-routes the whole approach instead if that
    // straight line would actually cross solid structure (e.g. a
    // neighboring jetty) — see its own comment for why.
    const dockWaypoints = buildDockingApproachWaypoints(
        { x: boatPos.x, y: boatPos.y },
        alignX, alignY,
        { x: chosen.ros_x, y: chosen.ros_y },
        chosen.parkedYaw,
        entities
    );

    plannedPath = dockWaypoints;
    pathIndex = 1;
    ilosState = { k: 1, y_int: 0 };
    dockingTarget = { alignX, alignY, chosen };
    update3DPathLine();
    isNavigating = true;
}

const resetDockBtn = document.getElementById('btn-reset-dock');
if (resetDockBtn) {
    resetDockBtn.addEventListener('click', () => {
        // Cancel any in-progress docking run and snap back to Mode 3's own
        // start pose (the fairway entrance) — must NOT reuse btn-reset's
        // handler (Mode 1's Reset): that resets to MODE1_START, clears Mode
        // 1's design entities, and stamps "Design Phase" onto the telemetry
        // panel, none of which belong to Mode 3.
        resetBoatToPose(MODE3_START);
        dockingBerthName = '';

        const statusEl = document.getElementById('tele-status');
        if (statusEl) {
            statusEl.textContent = '⚓ Idle — Fairway Entrance';
            statusEl.style.color = '#ffc107';
        }
        document.getElementById('tele-x').textContent = MODE3_START.x.toFixed(2);
        document.getElementById('tele-y').textContent = MODE3_START.y.toFixed(2);
        document.getElementById('tele-speed').textContent = '0.00';
    });
}
