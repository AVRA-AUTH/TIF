// ================= ISLAND / SHORE / MARINA COLLISION + BRAKING =================
// Everything that stops the boat sailing into the island, past the
// shoreline, or into the marina dock structure — both the hard hull
// collision (isTouchingObstacle*/isMovingIntoObstacle, used by every drive
// mode) and Mode 2's graduated approach-and-stop braking. Depends on
// config.js (ISLAND_*/MARINA_*/JETTY_*/LAKE_RADIUS/ISLAND_BRAKE_DECEL) and
// state.js (boatPos).

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

// Signed distance from (x,y) to an axis-aligned box: positive outside
// (ordinary distance to the nearest boundary point), NEGATIVE once inside
// (magnitude = depth to the nearest edge — how far a straight shot to the
// closest exit would have to travel). distanceToMarinaStructure() above
// clamps this at 0 once inside, which is fine for a speed cap (0 just means
// "fully braked") but throws away exactly the information needed below to
// tell "still driving deeper into the pier" from "already inside, now
// heading back out" — hence a separate function instead of reusing that one.
function boxSDF(x, y, xmin, xmax, ymin, ymax) {
    const cx = (xmin + xmax) / 2, cy = (ymin + ymax) / 2;
    const hx = (xmax - xmin) / 2, hy = (ymax - ymin) / 2;
    const qx = Math.abs(x - cx) - hx, qy = Math.abs(y - cy) - hy;
    const outside = Math.hypot(Math.max(qx, 0), Math.max(qy, 0));
    const inside = Math.min(Math.max(qx, qy), 0);
    return outside + inside;
}

// Same idea as distanceToMarinaStructure(), but signed (see boxSDF above).
// The pier's real collision rule is a half-plane (y <= MARINA_PIER_Y, no
// southern bound — nothing is ever that far south in practice), modeled
// here as a very deep box so boxSDF degenerates to the same half-plane
// behavior near the actual playing field.
function marinaPenetrationSDF(x, y) {
    let best = boxSDF(x, y, MARINA_PIER_X_MIN, MARINA_PIER_X_MAX, -LAKE_RADIUS - 50, MARINA_PIER_Y);
    for (const jx of JETTY_X_LIST) {
        best = Math.min(best, boxSDF(x, y, jx - JETTY_HALF_WIDTH, jx + JETTY_HALF_WIDTH, JETTY_Y_FAR, JETTY_Y_NEAR));
    }
    return best;
}

// Is `direction` (+1 forward, -1 reverse) making penetration into the
// marina structure WORSE? Unlike isMovingIntoObstacle()'s generic
// step-and-recheck-boolean test below (which has to fall back to "block
// neither" when both directions still read as touching after a small step —
// unavoidable for a long flat wall, where a 0.5m step parallel to the face
// doesn't cross out of the collision box either way, an ambiguity a
// circular obstacle like the island rarely hits), this compares the actual
// signed penetration depth before and after the step. That gradient stays
// meaningful even deep inside the pier/jetty box, which is what actually
// closes the bug this was written for: a boat driving straight at the pier
// could end up in that generic "both directions still touching, so allow
// both" case and sail right through.
//
// Gated on isTouchingObstacle() — the SAME real hull-inclusive contact test
// the generic check above uses — on purpose: this must never fire before
// the hull has actually made contact. An earlier version short-circuited on
// raw distance ("skip if > 5m away") instead, which meant it started
// throttling approach a full 5m out regardless of whether the hull was
// anywhere near touching — directly broke "should be able to get really
// close to dock." This is a backstop for genuine contact only, not a
// pre-emptive keep-out; it just fills the one gap the generic ambiguous-case
// fallback leaves on a long flat wall.
function isMovingDeeperIntoMarina(direction) {
    if (!isTouchingObstacle()) return false;
    const sdf0 = marinaPenetrationSDF(boatPos.x, boatPos.y);
    const stepX = boatPos.x + direction * Math.cos(boatPos.yaw) * 0.5;
    const stepY = boatPos.y + direction * Math.sin(boatPos.yaw) * 0.5;
    return marinaPenetrationSDF(stepX, stepY) < sdf0 - 0.01; // meaningfully worse, not float noise
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
function isTouchingObstacleAt(x, y, yaw, ignoreMarinaStructure) {
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

    // Main Spine Pier / Finger Jetties — canonical geometry (see MARINA_PIER_*/
    // JETTY_* consts, defined near the 3D mesh that also reads them, further
    // down this file). Skipped when ignoreMarinaStructure is set: Mode 3's
    // final docking legs (align/approach/creep/reverse_swing) deliberately
    // drive the hull right up against this exact geometry — that's what
    // "docked" means — so this generic backstop must not treat the intended
    // contact as a collision to fight. Every OTHER contact case (moored
    // vessels, buoys/dynamic entities, the island) still applies below,
    // unconditionally — this only carves out the wall the boat is actively
    // trying to dock against.
    if (!ignoreMarinaStructure) {
        if (y - hullYExtent <= MARINA_PIER_Y && x + hullXExtent >= MARINA_PIER_X_MIN && x - hullXExtent <= MARINA_PIER_X_MAX) {
            return true;
        }

        for (const jx of JETTY_X_LIST) {
            const overlapsX = (x + hullXExtent >= jx - JETTY_HALF_WIDTH) && (x - hullXExtent <= jx + JETTY_HALF_WIDTH);
            const overlapsY = (y + hullYExtent >= JETTY_Y_FAR) && (y - hullYExtent <= JETTY_Y_NEAR);
            if (overlapsX && overlapsY) return true;
        }
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

function isTouchingObstacle(ignoreMarinaStructure) {
    return isTouchingObstacleAt(boatPos.x, boatPos.y, boatPos.yaw, ignoreMarinaStructure);
}

// Narrower companion to isTouchingObstacleAt() above — only the "another
// vessel" categories (moored ships along the jetties, and dynamic/static
// entities placed in Mode 1), NOT the island/pier/jetty/shoreline geometry.
// Returns 'ship', 'buoy', or null, so a Mode 1 collision popup (input.js)
// can tell "hit a boat" from "hit a buoy" apart from a plain grounding.
//
// Box-aware, not a flat center-to-center radius: the old version only fired
// once the boat's CENTER point got within a fixed radius of the target's
// center, which in practice only ever tripped nose-on (or fully
// overlapping) — a moored/patrol boat's hull sticks out well past that
// radius along its own length, so scraping its side never registered. Now
// every hull is modeled as its actual (yaw-aware) oriented footprint — same
// idiom isTouchingObstacleAt() above already uses for the pier/jetty boxes
// (hullXExtent/hullYExtent from yaw) — so real hull-to-hull contact counts
// from any angle, not just bow-to-bow.
function isTouchingShipOrBuoyAt(x, y, yaw) {
    const hullXExtent = Math.abs(Math.cos(yaw)) * 2.8 + Math.abs(Math.sin(yaw)) * 0.8;
    const hullYExtent = Math.abs(Math.sin(yaw)) * 2.8 + Math.abs(Math.cos(yaw)) * 0.8;

    // Cheap bounding check before the ~42-iteration moored-ship loop below —
    // the whole marina footprint sits within x:[-100.2, -55.8] (the
    // parkedLocs values below, at this WORLD_SCALE), so anywhere comfortably
    // outside that (Mode 2's entire course sits at x:[25,132]; Mode 1's own
    // start is x:77.8) can never actually hit it. This function is now
    // called every frame Mode 2's challenge is running (checkMode2GameState(),
    // mode2-game.js) — previously only Mode 1's isNavigating step called it,
    // so skipping real, unavoidable dead work here matters more than it used
    // to. Margin is generous (40m) precisely because it only needs to rule
    // out "obviously nowhere near," not pinpoint the boundary.
    if (x > -40 || x < -140) {
        // skip the moored-ship loop entirely — falls through to the buoy/
        // dynamic-boat loop below, which is already cheap (bounded by
        // however many entities actually exist).
    } else {
        for (let b = -168 * WORLD_SCALE; b <= -128 * WORLD_SCALE; b += 6.5 * WORLD_SCALE) {
            if (Math.abs(b - (-147.5 * WORLD_SCALE)) > 3.0) {
                const parkedLocs = [
                    { x: -250.5 * WORLD_SCALE, y: b }, { x: -239.5 * WORLD_SCALE, y: b },
                    { x: -200.5 * WORLD_SCALE, y: b }, { x: -189.5 * WORLD_SCALE, y: b },
                    { x: -150.5 * WORLD_SCALE, y: b }, { x: -139.5 * WORLD_SCALE, y: b }
                ];
                // Moored hulls sit unrotated in their berth (createMooredBoat()
                // in scene-marina.js: rotation.y = 0, up to 3.6m long x 1.45m
                // wide) — a plain axis-aligned box vs. our own hull's box, same
                // overlap test as the jetty boxes above.
                for (const pl of parkedLocs) {
                    if (Math.abs(pl.x - x) <= hullXExtent + 1.8 && Math.abs(pl.y - y) <= hullYExtent + 0.73) return 'ship';
                }
            }
        }
    }

    for (const ent of entities) {
        if (ent.type === 'dynamic') {
            // Patrol boat has its own heading — approximate its footprint
            // the same oriented-box way as our own hull (hull box 3.8 long
            // x 1.5 wide, plus the tapered bow nose).
            const eh = ent.heading || 0;
            const entXExtent = Math.abs(Math.cos(eh)) * 2.6 + Math.abs(Math.sin(eh)) * 0.75;
            const entYExtent = Math.abs(Math.sin(eh)) * 2.6 + Math.abs(Math.cos(eh)) * 0.75;
            if (Math.abs(ent.ros_x - x) <= hullXExtent + entXExtent && Math.abs(ent.ros_y - y) <= hullYExtent + entYExtent) return 'ship';
        } else if (ent.type === 'static' && !ent.isParkedShip) {
            // Small round buoy body (0.55m radius, per its CylinderGeometry
            // in entities-3d.js) — box-vs-circle: clamp the buoy's center
            // into our hull box, then test the leftover distance against
            // its own radius (plus a small touch buffer).
            const closestX = Math.max(x - hullXExtent, Math.min(ent.ros_x, x + hullXExtent));
            const closestY = Math.max(y - hullYExtent, Math.min(ent.ros_y, y + hullYExtent));
            if (Math.hypot(closestX - ent.ros_x, closestY - ent.ros_y) < 0.65) return 'buoy';
        }
    }

    return null;
}

// Mode 1 design-phase placement guard — used by placeMode1EntityAt() (input.js)
// to reject a buoy/patrol-boat spawn point that would already be touching
// the player's own hull at its current (stationary, pre-RUN) pose. Without
// this, clicking right next to — or on top of — the boat spawned an
// obstacle already in contact, which isTouchingShipOrBuoyAt()'s improved
// box-aware detection above would then report as a collision the player
// never actually drove into.
// entityType is 'dynamic' or 'static' (the same currentMode values
// placeMode1EntityAt() already switches on) — 'goal' never reaches here.
function wouldObstaclePlacementCollideWithBoat(rx, ry, entityType) {
    const hullXExtent = Math.abs(Math.cos(boatPos.yaw)) * 2.8 + Math.abs(Math.sin(boatPos.yaw)) * 0.8;
    const hullYExtent = Math.abs(Math.sin(boatPos.yaw)) * 2.8 + Math.abs(Math.cos(boatPos.yaw)) * 0.8;

    // Patrol boats don't get a heading until the first sim tick after RUN
    // (navigation.js lazily sets entity.heading = random on first update),
    // so there's no orientation yet to build an oriented box from at
    // placement time — fall back to a circle sized to the hull's own
    // half-diagonal (covers 2.6 x 0.75 half-extents from entities-3d.js's
    // model regardless of which way it ends up facing). Buoys use the same
    // 0.65 body-radius-plus-buffer as isTouchingShipOrBuoyAt() above.
    const entRadius = entityType === 'dynamic' ? 2.7 : 0.65;

    const closestX = Math.max(boatPos.x - hullXExtent, Math.min(rx, boatPos.x + hullXExtent));
    const closestY = Math.max(boatPos.y - hullYExtent, Math.min(ry, boatPos.y + hullYExtent));
    return Math.hypot(closestX - rx, closestY - ry) < entRadius;
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
// both directions look blocked, treat it as ambiguous and block neither
// from THIS generic test. That ambiguity is rare for the circular island
// but common for the marina's long straight walls — a boat driving straight
// at the pier could land in exactly that "both sides still touching" case
// and sail right through it. isMovingDeeperIntoMarina() (above) closes that
// gap with a penetration-depth gradient instead of a boolean, so it's OR'd
// in independently rather than folded into the ambiguity fallback above.
// ignoreMarinaStructure: passed straight through to the pier/jetty backstop
// (see isTouchingObstacleAt) AND used to skip isMovingDeeperIntoMarina()
// below (the same wall, just checked via penetration depth instead of a
// boolean) — see runNavigationStep()'s docking close-quarters call in
// navigation.js for why this exists.
function isMovingIntoObstacle(direction, ignoreMarinaStructure) {
    let genericBlocked = false;
    if (isTouchingObstacle(ignoreMarinaStructure)) {
        const stepX = boatPos.x + direction * Math.cos(boatPos.yaw) * 0.5;
        const stepY = boatPos.y + direction * Math.sin(boatPos.yaw) * 0.5;
        const oppStepX = boatPos.x - direction * Math.cos(boatPos.yaw) * 0.5;
        const oppStepY = boatPos.y - direction * Math.sin(boatPos.yaw) * 0.5;
        const thisBlocked = isTouchingObstacleAt(stepX, stepY, boatPos.yaw, ignoreMarinaStructure);
        const oppBlocked = isTouchingObstacleAt(oppStepX, oppStepY, boatPos.yaw, ignoreMarinaStructure);
        genericBlocked = !(thisBlocked && oppBlocked) && thisBlocked;
    }
    return genericBlocked || (!ignoreMarinaStructure && isMovingDeeperIntoMarina(direction));
}

// Stuck-recovery heading search — used by runNavigationStep() (navigation.js)
// only once the linear backstop above has already found the boat genuinely
// wedged. guidance.js's localCorrectedYaw() (the normal every-tick local
// correction) is deliberately lightweight for that every-frame transit use:
// it checks the boat as a single POINT (pathfinding.js's isPointBlocked) over
// a short lookahead. Right against a wall that's not enough — the hull
// extends up to ~2.9m off-center when turning (hullXExtent/hullYExtent
// below), so the boat's CENTER can sit comfortably outside the exact
// structure box while the HULL is still touching it. That let the point-only
// search call the current (blocked) heading "already clear" and never
// actually turn — the boat just sat there re-trying the same heading every
// frame. This reuses the SAME hull-inclusive test the real backstop
// (isMovingIntoObstacle) uses, over a real ~1.5-hull-length lookahead
// (RECOVERY_LOOKAHEAD, sampled at a few points along the way so a heading
// that's clear at the end but clips something partway isn't picked), and
// sweeps a FULL 360deg fan in fixed steps rather than a narrow offset from
// the current heading — a genuinely wedged boat may need more than a small
// nudge to find daylight. Ties are broken by smallest turn from the current
// heading (cheapest escape to actually execute). Returns null if truly
// nowhere is clear (deeply wedged); the caller falls back to the lighter
// localCorrectedYaw() in that rare case.
const RECOVERY_HEADING_STEP = Math.PI / 12; // 15 degrees
const RECOVERY_LOOKAHEAD = 4.0;             // meters
const RECOVERY_SAMPLES = 4;
function findRecoveryYaw(x, y, currentYaw) {
    let best = null, bestTurn = Infinity;
    const steps = Math.round((2 * Math.PI) / RECOVERY_HEADING_STEP);
    for (let i = 0; i < steps; i++) {
        const candYaw = currentYaw + i * RECOVERY_HEADING_STEP;
        let clear = true;
        for (let s = 1; s <= RECOVERY_SAMPLES; s++) {
            const d = (RECOVERY_LOOKAHEAD * s) / RECOVERY_SAMPLES;
            const testX = x + Math.cos(candYaw) * d;
            const testY = y + Math.sin(candYaw) * d;
            if (isTouchingObstacleAt(testX, testY, candYaw)) { clear = false; break; }
        }
        if (!clear) continue;
        let turn = candYaw - currentYaw;
        turn = Math.atan2(Math.sin(turn), Math.cos(turn));
        if (Math.abs(turn) < bestTurn) { bestTurn = Math.abs(turn); best = candYaw; }
    }
    return best;
}
