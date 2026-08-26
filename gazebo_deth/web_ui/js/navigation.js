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

// Runs the whole nav/docking step for one frame — live replan, ILOS target
// selection + DWA local correction, movement physics + docking state
// machine, hard-obstacle backstop, and the /cmd_vel publish. Caller (main.js)
// only invokes this while `isNavigating && plannedPath.length > 0` — same
// gate the original code had, just moved to the call site instead of being
// the first line of this function, so main.js's own `if` stays visible.
function runNavigationStep(navDt) {
        const now = Date.now();
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
            });

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
            });

            const newTransit = findOptimalPath(
                { x: boatPos.x, y: boatPos.y },
                { x: dockingTarget.alignX, y: dockingTarget.alignY },
                sensedObstacles
            );
            if (newTransit && newTransit.length > 1) {
                const chosen = dockingTarget.chosen;
                plannedPath = [
                    ...newTransit.map(p => ({ x: p.x, y: p.y, mode: 'transit' })),
                    { x: dockingTarget.alignX, y: dockingTarget.alignY, mode: 'align', targetYaw: chosen.parkedYaw },
                    { x: chosen.ros_x, y: chosen.ros_y, mode: 'creep' }
                ];
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

                // Lightweight tick-rate local correction (DWA-style fan of
                // candidate headings, localCorrectedYaw() above) layered
                // under ILOS's heading — catches a fresh obstacle or
                // disturbance-induced drift immediately instead of waiting
                // for the next 250ms global replan. No-op when ILOS's own
                // heading is already clear.
                targetYaw = localCorrectedYaw(boatPos.x, boatPos.y, currentLinear, ilos.psi_d, entities);

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

            if (advancePath) {
                pathIndex = inTransitPhase ? transitEndIdx + 1 : pathIndex + 1;
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
                dockingTarget = null;
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
}
