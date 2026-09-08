// ================= NAVIGATION / DOCKING STEP =================
// The nav/docking state machine, extracted from the original app.js's
// single draw() function into standalone functions called once per frame
// by main.js. This is the one genuinely tangled piece of the original file
// (confirmed by a full read before extracting): the live-replan block can
// reassign plannedPath/pathIndex and the very next block's transitEndIdx
// scan depends on reading that post-replan array; currentLinear/currentAngular
// flow through brake-distance calc -> clamp -> obstacle-block override ->
// publish as one atomic sequence. Moved as one cohesive function, not split
// further. Depends on config.js, state.js, boundaries.js, pathfinding.js,
// guidance.js, ros.js (cmdVelTopic), and scene-boat.js (boatGroup, for the
// roll-bank effect below).

// Stuck-recovery maneuver — see runNavigationStep()'s entry into
// recoveryUntil/recoveryYaw (state.js) below. Turn-first, not
// reverse-first: pivot decisively toward the escape heading picked once at
// entry (boundaries.js's findRecoveryYaw(), hull-aware — not just a point
// check), same "stop and pivot in place" idea the docking align state
// already uses, and only ease in a little reverse if the hull is STILL
// touching something even while turning (genuinely wedged nose-first, not
// just needing to reorient) — enough to create clearance to complete the
// turn, not a committed multi-second back-out by default. The caller exits
// this maneuver the moment the boat is aligned and no longer blocked going
// forward (checked every frame before this even runs), so a plain "turn the
// other way, then carry on forward" case never reverses at all; ended a
// mistake doesn't need an elaborate undo, just a correction.
// RECOVERY_DURATION_MS is a safety ceiling only, for the rare case it can't
// fully clear — not the normal, always-used duration.
function driveRecovery(navDt) {
    const yd = recoveryYaw - boatPos.yaw;
    const yawDiff = Math.atan2(Math.sin(yd), Math.cos(yd));
    const targetAngular = Math.sign(yawDiff) * Math.min(MAX_ANGULAR, Math.abs(yawDiff) * 3.0);
    currentAngular = approachVelocity(currentAngular, targetAngular, navDt);

    const stillTouching = isTouchingObstacle();
    const reverseBlocked = isMovingIntoObstacle(-1, false) || isExitingLake(-1);
    const targetLinear = (stillTouching && !reverseBlocked) ? RECOVERY_REVERSE_SPEED : 0.0;
    currentLinear = approachVelocity(currentLinear, targetLinear, navDt);
    currentLinear = Math.max(MAX_LINEAR_REV, Math.min(MAX_LINEAR_FWD, currentLinear));

    publishRecoveryFrame('↩️ Recovering — backing clear of obstruction...');
}

// Follow-on phase once driveRecovery() reports aligned and clear: creep
// forward on the new heading for a bit (RECOVERY_FORWARD_DURATION_MS,
// config.js) before handing back to the live replan — real separation from
// whatever the boat just turned away from, instead of resuming
// plan-following the instant it was JUST barely clear and still right next
// to it (which risked the very next replan routing straight back toward the
// same spot). Aborts straight back into a fresh driveRecovery() turn if the
// creep itself finds something new, rather than pushing into it.
function driveRecoveryForward(navDt) {
    const yd = recoveryYaw - boatPos.yaw;
    const yawDiff = Math.atan2(Math.sin(yd), Math.cos(yd));
    const targetAngular = Math.sign(yawDiff) * Math.min(MAX_ANGULAR, Math.abs(yawDiff) * 1.5);
    currentAngular = approachVelocity(currentAngular, targetAngular, navDt, AUTONOMOUS_ANGULAR_RAMP_RATE);

    if (isMovingIntoObstacle(1, false) || isExitingLake(1)) {
        recoveryForwardUntil = 0;
        recoveryYaw = findRecoveryYaw(boatPos.x, boatPos.y, boatPos.yaw) ??
            localCorrectedYaw(boatPos.x, boatPos.y, currentLinear, boatPos.yaw, entities, false);
        recoveryUntil = Date.now() + RECOVERY_DURATION_MS;
        driveRecovery(navDt);
        return;
    }

    currentLinear = approachVelocity(currentLinear, RECOVERY_FORWARD_SPEED, navDt);
    currentLinear = Math.max(MAX_LINEAR_REV, Math.min(MAX_LINEAR_FWD, currentLinear));

    publishRecoveryFrame('↩️ Recovering — creeping clear...');
}

// Shared publish + telemetry tail for driveRecovery()/driveRecoveryForward()
// — both drive currentLinear/currentAngular themselves and just need this
// same boilerplate to actually reach ROS and the HUD each frame.
function publishRecoveryFrame(statusText) {
    if (boatGroup) boatGroup.rotation.z = -currentAngular * 0.15;
    cmdVelTopic.publish(new ROSLIB.Message({
        linear: { x: currentLinear, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: currentAngular }
    }));

    const statusEl = document.getElementById('tele-status');
    if (statusEl) {
        statusEl.textContent = statusText;
        statusEl.style.color = '#ff8800';
    }
    document.getElementById('tele-x').textContent = boatPos.x.toFixed(2);
    document.getElementById('tele-y').textContent = boatPos.y.toFixed(2);
    document.getElementById('tele-speed').textContent = boatPos.speed.toFixed(2);
}

// Still-active stuck-spot breadcrumbs (state.js's recoveryBreadcrumbs), as
// synthetic obstacle points for the live replan to route around — see
// where recoveryBreadcrumbs is pushed to below for why this exists. Also
// prunes expired entries in place, so the array can't grow unbounded across
// a long session.
function activeRecoveryBreadcrumbObstacles(now) {
    recoveryBreadcrumbs = recoveryBreadcrumbs.filter(b => b.until > now);
    return recoveryBreadcrumbs.map(b => ({ ros_x: b.x, ros_y: b.y }));
}

function update3DPathLine() {
    if (threePathLine) {
        scene.remove(threePathLine);
        threePathLine = null;
    }
    if (plannedPath && plannedPath.length > 1 && pathIndex < plannedPath.length) {
        const from = new THREE.Vector3(boatPos.x, 0.6, -boatPos.y);
        const target = plannedPath[pathIndex];
        const to = new THREE.Vector3(target.x, 0.6, -target.y);
        const dir = to.clone().sub(from);
        const length = Math.min(dir.length(), 15);
        if (length > 0.5) {
            threePathLine = new THREE.ArrowHelper(dir.normalize(), from, length, 0x00ffcc, 2.5, 1.5);
            scene.add(threePathLine);
        }
    }
}

// Runs the whole nav/docking step for one frame — live replan, ILOS target
// selection + DWA local correction, movement physics + docking state
// machine, hard-obstacle backstop, and the /cmd_vel publish. Caller (main.js)
// only invokes this while `isNavigating && plannedPath.length > 0` — same
// gate the original code had, just moved to the call site instead of being
// the first line of this function, so main.js's own `if` stays visible.
function runNavigationStep(navDt) {
        // Mode 1 only: the moment the hull actually touches another ship
        // (moored or patrolling) or a buoy — separate from the generic
        // island/pier/shore hull-contact backstop further down, which
        // already handles the physical stop/back-off for ALL solid
        // obstacles but never tells the user WHAT was hit — treat it as
        // terminal: hard-stop the boat, start the sinking animation
        // (main.js's boat-transform step reads playerBoatCrashed/
        // playerCrashTime), then prompt to reset. Latched on
        // shipCollisionAlertShown so this fires once per contact, not every
        // frame the hull stays touching.
        if (activeAppMode === 1) {
            const hit = isTouchingShipOrBuoyAt(boatPos.x, boatPos.y, boatPos.yaw);
            if (hit && !shipCollisionAlertShown) {
                shipCollisionAlertShown = true;
                playerBoatCrashed = true;
                playerCrashTime = Date.now();

                // Hard stop — a crashed hull shouldn't keep driving into
                // whatever it just hit. Same stop sequence used below when
                // a path finishes normally (isNavigating=false branch).
                isNavigating = false;
                currentLinear = 0;
                currentAngular = 0;
                boatPos.speed = 0;
                cmdVelTopic.publish(new ROSLIB.Message({
                    linear: { x: 0.0, y: 0.0, z: 0.0 },
                    angular: { x: 0.0, y: 0.0, z: 0.0 }
                }));

                // Delayed so the sinking animation gets a few rendered
                // frames in before this synchronous dialog freezes the tab.
                setTimeout(() => {
                    showConfirm(
                        (hit === 'ship' ? 'Ship Collided! You crashed into another vessel.' : 'Ship Collided! You hit a buoy.') +
                        '\n\nYour boat is taking on water. Stop and reset the simulation?',
                        () => document.getElementById('btn-reset').click(),
                        hit === 'ship' ? '🚢' : '🛟'
                    );
                }, 1200);
            } else if (!hit) {
                shipCollisionAlertShown = false;
            }
        }

        const now = Date.now();

        // A committed recovery maneuver is in progress (entered below, near
        // the hull-contact backstop) — run it instead of everything else
        // this frame (live replan included: replanning from a position the
        // boat is actively backing away from would just recompute a route
        // back toward the same spot it's trying to clear).
        if (isNavigating && now < recoveryUntil) {
            // End the maneuver the moment it's actually done its job —
            // turned toward daylight and no longer blocked going forward —
            // instead of always riding out the full RECOVERY_DURATION_MS
            // ceiling. A plain "wrong heading, needs to turn" case clears
            // this almost immediately and falls straight through to normal
            // forward progress this same frame; only a genuinely stubborn
            // wedge keeps driveRecovery() running past a frame or two.
            const yd = recoveryYaw - boatPos.yaw;
            const headingError = Math.atan2(Math.sin(yd), Math.cos(yd));
            const turnedAndClear = Math.abs(headingError) < 0.15 && !isMovingIntoObstacle(1, false);
            if (turnedAndClear) {
                // Turn phase done — hand straight to the forward-creep phase
                // below THIS SAME frame (not next frame) for real separation
                // before trusting the live replan's route again.
                recoveryUntil = 0;
                recoveryForwardUntil = now + RECOVERY_FORWARD_DURATION_MS;
            } else {
                driveRecovery(navDt);
                return;
            }
        }

        if (isNavigating && now < recoveryForwardUntil) {
            driveRecoveryForward(navDt);
            return;
        }

        // Receding Horizon Sensor Scan: Recalculate path live every 250ms based on local 25m sensor horizon.
        // Mode 3 docking used to be fully excluded from this — its own
        // dedicated branch below now gives it the same live replanning
        // instead (see dockingTarget), but re-targeted at the docking's own
        // align point, not currentGoal, and only while still on the transit
        // leg. Mode 1's own currentGoal-based replan must still never touch
        // a docking run, or a currentGoal left over from a prior Mode 1 run
        // would silently replace the docking path with an A* route back to
        // that old goal and the boat would never reach the berth.
        if (!isAutoDocking && now - lastRecalcTime > 250 && currentGoal) {
            lastRecalcTime = now;
            const sensedObstacles = entities.filter(ent => {
                if (ent.type === 'goal' || ent.isCrashed) return false;
                const dist = Math.hypot(ent.ros_x - boatPos.x, ent.ros_y - boatPos.y);
                return dist <= 25.0; // 25-meter sensor horizon
            }).concat(activeRecoveryBreadcrumbObstacles(now));

            const newPath = findOptimalPath(
                { x: boatPos.x, y: boatPos.y },
                { x: currentGoal.ros_x, y: currentGoal.ros_y },
                sensedObstacles
            );
            if (newPath && newPath.length > 1) {
                plannedPath = newPath;
                pathIndex = 1; // Point directly to upcoming waypoint to prevent waypoint 0 snap
                // Only the active-segment pointer resets — the replanned path
                // always starts at the boat's current position, so segment 1
                // is correct again immediately. y_int (the disturbance
                // estimate) deliberately survives the replan; it's tracking
                // a real physical effect, not progress along a specific path.
                ilosState.k = 1;
                update3DPathLine();
            }
        } else if (isAutoDocking && dockingTarget && now - lastRecalcTime > 250 &&
                   plannedPath[pathIndex] &&
                   (plannedPath[pathIndex].mode === 'transit' || plannedPath[pathIndex].mode === undefined)) {
            // Docking's own live replan — same receding-horizon idea as Mode
            // 1's above, but re-routes toward the docking's own align point
            // (dockingTarget, saved by startDynamicDocking; docking never
            // uses currentGoal) and only while the boat is still somewhere
            // in the transit prefix. Once past that (align/creep), this
            // stops firing entirely — those are hand-tuned precision
            // maneuvers, not a line to replan around. This is what lets a
            // real deviation (a hull-contact backoff, a disturbance the
            // ILOS/local-correction steering couldn't fully absorb, physics
            // knocking the boat off course) get a genuinely new route
            // instead of the boat forever chasing a transit line planned
            // from a position it's no longer anywhere near.
            lastRecalcTime = now;
            const sensedObstacles = entities.filter(ent => {
                if (ent.type === 'goal' || ent.isCrashed) return false;
                const dist = Math.hypot(ent.ros_x - boatPos.x, ent.ros_y - boatPos.y);
                return dist <= 25.0;
            }).concat(activeRecoveryBreadcrumbObstacles(now));

            // buildDockingApproachWaypoints() (pathfinding.js) validates the
            // final align->creep hop and A*-routes the whole approach
            // instead if that straight line would actually cross solid
            // structure — same as startDynamicDocking()'s initial kickoff,
            // kept in sync since a live replan reconstructs this same tail
            // every 250ms.
            const chosen = dockingTarget.chosen;
            const newDockWaypoints = buildDockingApproachWaypoints(
                { x: boatPos.x, y: boatPos.y },
                dockingTarget.alignX, dockingTarget.alignY,
                { x: chosen.ros_x, y: chosen.ros_y },
                chosen.parkedYaw,
                sensedObstacles
            );
            if (newDockWaypoints && newDockWaypoints.length > 1) {
                plannedPath = newDockWaypoints;
                pathIndex = 1;
                ilosState.k = 1; // same reasoning as Mode 1's replan above — y_int survives
                update3DPathLine();
            }
        }

        if (pathIndex < plannedPath.length) {
            // Which prefix of plannedPath is a continuous transit line
            // (mode 'transit' or unset, i.e. the A*-planned lead-in) vs a
            // discrete docking maneuver (align/creep/etc.) — recomputed
            // each frame (cheap linear scan over a short array), not
            // cached, since plannedPath can be replaced wholesale by the
            // live 250ms replan at any time.
            let transitEndIdx = -1;
            for (let i = 0; i < plannedPath.length; i++) {
                const m = plannedPath[i].mode;
                if (m === 'transit' || m === undefined) transitEndIdx = i;
                else break;
            }
            const inTransitPhase = transitEndIdx >= 1 && pathIndex <= transitEndIdx;

            let target, dx, dy, dist, targetYaw, nextTarget;
            let advancePath = false;

            if (inTransitPhase) {
                // ILOS path-following (ilosGuidance(), ported from the
                // user's MATLAB ILOS_wrapper) instead of naive
                // point-to-point pursuit — tracks the actual planned LINE
                // across the whole transit prefix in one continuous pass,
                // with cross-track error + integral disturbance rejection,
                // rather than re-aiming at a single dot every tick (the
                // pursuit-curve divergence documented above).
                const transitWaypoints = plannedPath.slice(0, transitEndIdx + 1);
                const ilos = ilosGuidance(boatPos.x, boatPos.y, transitWaypoints, ilosState, navDt);
                target = transitWaypoints[ilos.seg];
                dx = target.x - boatPos.x;
                dy = target.y - boatPos.y;
                dist = Math.hypot(dx, dy);
                nextTarget = transitWaypoints[ilos.seg + 1];

                // COLREGS (colregs.js, ported from the user's Python
                // VO_collision_avoidance source): is there a nearby dynamic
                // ship where this boat is give-way (or the encounter is
                // head-on, where BOTH vessels must turn starboard)? If so,
                // restrict localCorrectedYaw()'s fan below to starboard-turn
                // candidates only — never let the "avoid a fresh obstacle"
                // correction cut across the other vessel's bow to port. Only
                // dynamic ships are COLREGS encounters; static buoys aren't
                // vessels and don't have a heading/give-way side. Cheap
                // per-tick loop over `entities` (already bounded by however
                // many the user has placed), same sensor horizon (25m) as
                // the live replan above.
                let colregsGiveWay = false;
                let colregsSituation = null;
                for (const ent of entities) {
                    if (ent.type !== 'dynamic' || ent.isCrashed || ent.heading === undefined) continue;
                    if (Math.hypot(ent.ros_x - boatPos.x, ent.ros_y - boatPos.y) > 25.0) continue;
                    const bearingDeg = relativeBearingDeg(boatPos.x, boatPos.y, boatPos.yaw, ent.ros_x, ent.ros_y);
                    const situation = encounterSituation(boatPos.yaw, ent.heading, bearingDeg);
                    if (situation === 'Head-on' || isGiveWayShip(situation, bearingDeg, Math.abs(currentLinear), ent.speed || 0)) {
                        colregsGiveWay = true;
                        colregsSituation = situation;
                        break;
                    }
                }

                // Testing/visibility aid: this is otherwise invisible (it only
                // restricts WHICH candidate headings localCorrectedYaw() below
                // tries, not a new obstacle check of its own) — surface it on
                // the existing tele-status HUD so give-way engagement can
                // actually be seen live, same element input.js's Run button
                // and the docking states below already drive. Left untouched
                // during docking (isAutoDocking owns tele-status there).
                if (!isAutoDocking) {
                    const teleStatusEl = document.getElementById('tele-status');
                    if (teleStatusEl) {
                        if (colregsGiveWay) {
                            teleStatusEl.textContent = `🧭 COLREGS Give-Way (${colregsSituation}) — Starboard Only`;
                            teleStatusEl.style.color = '#ff66cc';
                        } else if (teleStatusEl.textContent.startsWith('🧭')) {
                            teleStatusEl.textContent = '▶️ Navigating...';
                            teleStatusEl.style.color = '#00ffcc';
                        }
                    }
                }

                // Lightweight tick-rate local correction (DWA-style fan of
                // candidate headings, localCorrectedYaw() above) layered
                // under ILOS's heading — catches a fresh obstacle or
                // disturbance-induced drift immediately instead of waiting
                // for the next 250ms global replan. No-op when ILOS's own
                // heading is already clear.
                targetYaw = localCorrectedYaw(boatPos.x, boatPos.y, currentLinear, ilos.psi_d, entities, colregsGiveWay);

                // Same tolerances the old per-point advance used for a
                // standard/docking transit leg — only checked against the
                // FINAL transit waypoint now, since ILOS drives the whole
                // prefix as one continuous leg rather than one waypoint at
                // a time.
                const finalLegTolerance = isAutoDocking ? 1.2 : 1.8;
                if (ilos.seg >= transitWaypoints.length - 1 && dist < finalLegTolerance) {
                    advancePath = true;
                }
            } else {
                target = plannedPath[pathIndex];
                dx = target.x - boatPos.x;
                dy = target.y - boatPos.y;
                dist = Math.hypot(dx, dy);
                targetYaw = (target.mode === 'align' && target.targetYaw !== undefined) ? target.targetYaw : Math.atan2(dy, dx);
                nextTarget = plannedPath[pathIndex + 1];

                // State Machine Progression Logic
                if (target.mode === 'align') {
                    const yd = targetYaw - boatPos.yaw;
                    const wrapped = Math.atan2(Math.sin(yd), Math.cos(yd));
                    if (Math.abs(wrapped) < 0.05) advancePath = true; // Advance only when heading is locked!
                } else if (target.mode === 'creep' || target.mode === 'reverse_swing') {
                    if (dist < 0.3) advancePath = true; // High precision finish line
                } else if (target.mode === 'approach') {
                    if (dist < 0.5) advancePath = true; // Wait to reach near the pier
                } else if (isAutoDocking) {
                    if (dist < 1.2) advancePath = true;
                } else {
                    if (dist < 1.8) advancePath = true; // Standard transit tolerance
                }
            }

            let yawDiff = targetYaw - boatPos.yaw;
            while (yawDiff > Math.PI) yawDiff -= 2 * Math.PI;
            while (yawDiff < -Math.PI) yawDiff += 2 * Math.PI;

                        // Njord rule: on the FINAL docking waypoint, don't finish the run
            // the instant distance/heading tolerance is met — hold there,
            // stationary and parallel to the dock, for DOCK_HOLD_DURATION_MS
            // first. Drifting back out of tolerance before the hold completes
            // resets the clock (a boat that wanders off isn't "holding").
            const isFinalDockingWaypoint = isAutoDocking && !inTransitPhase && pathIndex === plannedPath.length - 1;
            if (advancePath && isFinalDockingWaypoint) {
                if (!dockHoldStartTime) dockHoldStartTime = now;
                if (now - dockHoldStartTime < DOCK_HOLD_DURATION_MS) {
                    advancePath = false; // not done yet — keep holding station
                }
            } else if (!advancePath && isFinalDockingWaypoint) {
                dockHoldStartTime = null; // drifted out of tolerance — restart the 5s clock
            }

            if (advancePath) {
                pathIndex = inTransitPhase ? transitEndIdx + 1 : pathIndex + 1;
                dockHoldStartTime = null;
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
                    // Hard ceiling, independent of MAX_LINEAR_FWD/REV — only
                    // 'transit' (2.49 m/s) actually exceeds this today.
                    cruiseSpeed = Math.max(-DOCK_MAX_SPEED, Math.min(DOCK_MAX_SPEED, cruiseSpeed));
                }

                 // Live countdown while holding station on the final waypoint —
                // overrides whatever status text the state machine above just set.
                if (isFinalDockingWaypoint && dockHoldStartTime) {
                    const secsLeft = Math.max(0, Math.ceil((DOCK_HOLD_DURATION_MS - (now - dockHoldStartTime)) / 1000));
                    const statusEl = document.getElementById('tele-status');
                    if (statusEl) {
                        statusEl.textContent = `⚓ STATE 5: Holding Position, Parallel to Dock (${secsLeft}s)...`;
                        statusEl.style.color = '#28a745';
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
                // CURRENT heading error above, and cap it at what the boat can
                // PHYSICALLY turn through — not an ad-hoc cosine guess. An
                // A*-planned route can string together several short,
                // sharp-angled legs (e.g. threading between marina jetties);
                // approaching one at full cruise speed leaves only
                // brakeDist ~= v^2/(2*brakeDecel) of room to slow down in,
                // which a short leg doesn't have.
                if (nextTarget && !pivotOnly) {
                    const legDx = target.x - boatPos.x, legDy = target.y - boatPos.y;
                    const nextDx = nextTarget.x - target.x, nextDy = nextTarget.y - target.y;
                    const legLen = Math.hypot(legDx, legDy), nextLen = Math.hypot(nextDx, nextDy);
                    if (legLen > 0.01 && nextLen > 0.01) {
                        const cosTurn = (legDx * nextDx + legDy * nextDy) / (legLen * nextLen); // 1 = straight, -1 = reversal
                        const turnAngle = Math.acos(Math.max(-1, Math.min(1, cosTurn)));
                        if (turnAngle > 0.01) {
                            // Arc-length estimate: this leg is legLen long and
                            // has to turn by turnAngle before/through the
                            // corner, so its effective radius of curvature is
                            // about legLen/turnAngle. MAX_ANGULAR (the boat's
                            // real max turn rate) then bounds how fast it can
                            // go around a corner that tight — cornerRadius *
                            // MAX_ANGULAR is the fastest linear speed that
                            // still keeps angular rate within the boat's limit.
                            const cornerRadius = legLen / turnAngle;
                            const kinematicMaxSpeed = cornerRadius * MAX_ANGULAR;
                            cruiseSpeed = Math.min(cruiseSpeed, Math.max(kinematicMaxSpeed, MAX_LINEAR_FWD * 0.3));
                        }
                    }
                }

                // --- Linear: cruise, then glide to a stop exactly at the waypoint ---
                if (pivotOnly) {
                    currentLinear = approachVelocity(currentLinear, 0.0, navDt);
                } else {
                    // Speed cap consistent with stopping exactly at the target under
                    // brakeDecel (v = sqrt(2*a*d)) — the SAME graduated-braking formula
                    // boundaries.js already uses for the island/shore/marina keep-outs
                    // ("real stopping-distance kinematics"), recomputed fresh from the
                    // live `dist` every frame instead of open-loop-decremented from a
                    // separately-computed brakeDist. The old two-branch version (cruise
                    // until inside brakeDist, then subtract a fixed step, with a
                    // "requiredDecel too high" fallback that slammed in up to -1.0 m/s
                    // of reverse thrust) could overshoot the fallback's trigger,
                    // reverse away from the target, re-enter the "plenty of room, cruise
                    // toward cruiseSpeed" branch, and repeat — a real bang-bang limit
                    // cycle, which is what read as the boat "going front and back" and
                    // never settling right at the target. Recomputing the cap directly
                    // from real remaining distance every frame is self-correcting by
                    // construction and can't develop that cycle: far away it just
                    // reduces to cruiseSpeed (sqrt term is large), and near the target it
                    // smoothly converges to 0, never needing a separate reverse fallback.
                    const maxSpeedForStop = Math.sqrt(Math.max(0, 2 * brakeDecel * dist));
                    const speedTarget = Math.sign(cruiseSpeed) * Math.min(Math.abs(cruiseSpeed), maxSpeedForStop);
                    currentLinear = approachVelocity(currentLinear, speedTarget, navDt);
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
                // Mode 3's final docking legs (align/approach/creep/reverse_swing)
                // deliberately drive the hull right up against the pier/jetty wall
                // it's docking at — that contact is the intended destination, not a
                // collision to fight. Without this, the boat would reach real hull
                // contact just short of the creep target, get read as "grazing an
                // obstacle," and get braked/backed off every frame — the exact
                // "never quite stops / settles" behavior this exists to fix. Every
                // OTHER contact (moored vessels, buoys, the island) still applies —
                // this only exempts the one wall the boat is actively docking
                // against, and only during those close-quarters legs.
                const dockingAgainstWall = isAutoDocking &&
                    (target.mode === 'align' || target.mode === 'approach' ||
                     target.mode === 'creep' || target.mode === 'reverse_swing');
                const forwardStuck = currentLinear > 0 && (isMovingIntoObstacle(1, dockingAgainstWall) || isExitingLake(1));
                const reverseStuck = currentLinear < 0 && (isMovingIntoObstacle(-1, dockingAgainstWall) || isExitingLake(-1));
                if (forwardStuck) currentLinear = Math.max(MAX_LINEAR_REV, -0.6);
                else if (reverseStuck) currentLinear = 0;

                // Genuinely wedged against something outside an active docking
                // maneuver (e.g. clipped a jetty/pier corner mid-transit): enter
                // a committed recovery maneuver (driveRecovery(), defined near
                // the top of this file) instead of the single-frame -0.6 cap
                // above being all that ever happens. That single-frame nudge
                // wasn't real distance — the instant contact briefly cleared,
                // the normal plan-following logic below immediately steered
                // straight back into it, a tight "twitch and reapproach" loop
                // that never actually got clear. The escape heading
                // (findRecoveryYaw(), boundaries.js — hull-aware, unlike
                // localCorrectedYaw()'s point-only every-tick check, which could
                // call the current blocked heading "already clear" and never
                // actually turn) is picked ONCE here and held for the whole
                // maneuver, not recomputed every frame.
                if ((forwardStuck || reverseStuck) && !dockingAgainstWall) {
                    // Mark this spot so the live replan (above) is forced to
                    // route around it for a while instead of just re-finding
                    // the same route back through it — see
                    // activeRecoveryBreadcrumbObstacles() and
                    // RECOVERY_BREADCRUMB_DURATION_MS (config.js) for why:
                    // without this, backing off here did nothing on its own
                    // when the replan's TARGET sits behind this exact choke
                    // point — the very next 250ms replan just recomputed the
                    // same route straight back into it.
                    recoveryBreadcrumbs.push({ x: boatPos.x, y: boatPos.y, until: now + RECOVERY_BREADCRUMB_DURATION_MS });
                    recoveryYaw = findRecoveryYaw(boatPos.x, boatPos.y, boatPos.yaw) ??
                        localCorrectedYaw(boatPos.x, boatPos.y, currentLinear, boatPos.yaw, entities, false);
                    recoveryUntil = now + RECOVERY_DURATION_MS;
                    driveRecovery(navDt);
                    return;
                }

                // --- Angular: proportional heading control, capped at MAX_ANGULAR ---
                const targetAngular = Math.sign(yawDiff) * Math.min(MAX_ANGULAR, Math.abs(yawDiff) * angularGain);
                // Ordinary transit (mode 'transit' or unset — the A*-planned
                // lead-in, whether Mode 1's own path or docking's transit
                // prefix) eases into a new heading (AUTONOMOUS_ANGULAR_RAMP_RATE,
                // config.js) instead of snapping to it — this is the leg a
                // 250ms live replan keeps nudging, so it's what read as
                // "dramatic." The hand-tuned docking maneuvers
                // (align/approach/creep/reverse_swing) keep the fast,
                // decisive THRUST_RAMP_RATE default — those are deliberate
                // state transitions, not routine replan noise.
                const isTransitLeg = target.mode === 'transit' || target.mode === undefined;
                currentAngular = isTransitLeg
                    ? approachVelocity(currentAngular, targetAngular, navDt, AUTONOMOUS_ANGULAR_RAMP_RATE)
                    : approachVelocity(currentAngular, targetAngular, navDt);

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
                dockingTarget = null;
                // Docking run finished on its own (not via Reset/mode-switch,
                // which already turn this off in resetBoatToPose()) — turn
                // the Mode-3-only turn-thrust-reserve back off now.
                turnReserveTopic.publish(new ROSLIB.Message({ data: false }));
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

// Pre-existing behavior, kept as-is (not a new bug, not fixed here — see
// the file-split plan's notes): the roll-bank line below
// (boatGroup.rotation.z = -currentAngular * 0.15) is unconditionally
// overwritten back to 0 every frame in main.js's boat-transform step,
// before the scene is ever rendered, so the roll-bank visually never
// actually happens. Flagging it here since this is where it's set.

// Dynamic (patrol) boat simulation: buoy-collision crash check, patrol
// waypoint AI, and per-frame 3D mesh sync (heave/pitch/roll). Called once
// per frame from main.js, unconditionally (not gated on isNavigating —
// dynamic boats patrol regardless of whether the ASV itself is navigating).
function updateDynamicEntities() {
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
                entity.speedMode = Math.random() < 0.5 ? 'constant' : 'variable';
                entity.baseSpeed = 1.0;
                entity.speed = entity.baseSpeed;
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
           
            if (entity.speedMode === 'variable') {
            entity.speed = entity.baseSpeed * (0.6 + 0.4 * Math.sin(Date.now() * 0.0005 + entity.ros_x));
            }

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
}
