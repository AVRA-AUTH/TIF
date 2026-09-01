// ================= UNIFIED A* + STRING-PULLING PATHFINDER ================= //
// Depends on config.js (SAFETY_RADIUS/SHORE_MARGIN/LAKE_RADIUS/ISLAND_*/
// MARINA_*/JETTY_*).

// Real marina dock structure (main spine pier + 3 finger jetties). This is
// the single source of truth for "is this point solid dock" — used by BOTH
// the direct-line fast path AND the grid search below, so the two can never
// disagree about whether a given spot is water or structure (they used to:
// one checked this exact rectangular shape, the other blocked a plain circle
// of a different, unit-mismatched size around the marina's center).
function isInsideMarinaStructure(x, y, extraMargin) {
    extraMargin = extraMargin || 0;
    // Main Spine Pier Wall — canonical geometry (MARINA_PIER_* consts), padded
    // by MARINA_PIER_PATH_MARGIN on the exposed north face and both ends (see
    // config.js for why: this used to be the pier's exact literal footprint,
    // the only obstacle in the whole pathfinder with zero planning buffer).
    // Suppressed directly in front of a finger jetty (within its own 5.5m
    // half-width below): jetties connect right at this same face with no gap
    // by design, and this marina is a deliberately tight, packed layout — the
    // pier's extra margin stacked on top of the jetty's own clearance there
    // closed off the real lane a boat needs to reach a jetty berth entirely,
    // which is what got a boat stuck immediately in front of a jetty mouth
    // the first time this margin was added. The jetty's own keepout below is
    // already generous (5.5m vs. its real 2.0m half-width) and covers this
    // stretch on its own.
    const jettyHalfBuffer = 5.5 + extraMargin;
    const nearAJetty = JETTY_X_LIST.some(jx => Math.abs(x - jx) < jettyHalfBuffer);
    const pierMargin = (nearAJetty ? 0 : MARINA_PIER_PATH_MARGIN) + extraMargin;
    if (y <= MARINA_PIER_Y + pierMargin &&
        x >= MARINA_PIER_X_MIN - pierMargin &&
        x <= MARINA_PIER_X_MAX + pierMargin) return true;
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
        if (Math.abs(x - jx) < jettyHalfBuffer && y <= JETTY_Y_NEAR && y >= JETTY_Y_FAR) return true;
    }
    return false;
}

// Single obstacle predicate for the whole pathfinder: outside the navigable
// lake (shoreline/greenery), the island, the marina structure, or within a
// keepout radius of any live obstacle (buoys, moving boats, and the ~30
// baked-in moored boats, all passed in via `obstacles`).
//
// The ~30 baked-in moored boats (entities with isParkedShip, see
// scene-marina.js's createMooredBoat) get their OWN smaller
// MOORED_SHIP_SAFETY_RADIUS instead of the generic SAFETY_RADIUS every other
// obstacle uses — SAFETY_RADIUS (5.5m) is sized for a buoy/boat the player
// can drop anywhere in open water, but these are only ~2.4m off their
// jetty's own centerline by design (packed marina, ~2.6m same-jetty
// spacing). Stacking the full 5.5m on top of the jetty's own 5.5m keepout
// pinched the open lane between two adjacent jetties down to ~4m — thinner
// than the 4m pathfinder grid cell, which could fail to route through it at
// all — which is what read as the boat getting stuck right at a jetty mouth
// with moored boats on either side. MOORED_SHIP_SAFETY_RADIUS keeps a real
// margin over their actual ~2.0m hull-contact radius (boundaries.js) without
// adding to what the jetty's own keepout already covers.
// extraMargin: optional additional standoff on top of every keepout below —
// see findOptimalPath()'s SHORTCUT_MARGIN for why this exists (the
// string-pulling smoothing pass uses it to prefer a route with real
// breathing room around a corner instead of the tightest legally-clear
// shortcut; every other caller passes 0/omits it, unchanged).
function isPointBlocked(x, y, obstacles, extraMargin) {
    extraMargin = extraMargin || 0;
    if (Math.hypot(x, y) > LAKE_RADIUS - SHORE_MARGIN - extraMargin) return true;
    if (Math.hypot(x - ISLAND_X, y - ISLAND_Y) < ISLAND_KEEP_OUT + extraMargin) return true;
    if (isInsideMarinaStructure(x, y, extraMargin)) return true;
    for (const obs of obstacles) {
        const radius = (obs.isParkedShip ? MOORED_SHIP_SAFETY_RADIUS : SAFETY_RADIUS) + extraMargin;
        if (Math.hypot(x - obs.ros_x, y - obs.ros_y) < radius) return true;
    }
    return false;
}

// Segment-blocked check, sampled at sub-grid resolution against the SAME
// predicate the grid search uses — this is what keeps the "is a straight
// line clear?" fast path and the A* grid in agreement.
function isSegmentBlocked(p1, p2, obstacles, extraMargin) {
    const dx = p2.x - p1.x;
    const dy = p2.y - p1.y;
    const len = Math.hypot(dx, dy);
    if (len === 0) return isPointBlocked(p1.x, p1.y, obstacles, extraMargin);
    const steps = Math.max(1, Math.ceil(len / 1.0)); // sample every ~1m
    for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        if (isPointBlocked(p1.x + t * dx, p1.y + t * dy, obstacles, extraMargin)) return true;
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

// Extra standoff (on top of every real keepout — SAFETY_RADIUS,
// MOORED_SHIP_SAFETY_RADIUS, the marina structure) used ONLY by the
// direct-line fast path and the string-pulling shortcut pass below, never by
// the raw A* grid search itself. Without this, string-pulling's greedy
// "farthest visible point" shortcut takes the single tightest diagonal cut
// that's still LEGALLY clear — by construction that hugs right along
// whatever corner/obstacle edge made a shorter cut illegal, so the boat's
// planned route (and the ILOS tracking it) rode the exact boundary of a
// keepout zone rather than taking a wider, more open turn around it. The
// raw grid search underneath is deliberately left untouched (extraMargin=0)
// so it can still find a way through a real tight-but-legal corridor (e.g. a
// docking berth) — this only asks the SMOOTHING step to prefer more
// breathing room when the room is actually there, falling back to following
// the raw (tight but valid) grid path more closely — never fewer than the
// original vetted grid segments — when it isn't.
const SHORTCUT_MARGIN = 3.0;

function findOptimalPath(start, goal, obstacles) {
    const validObs = obstacles.filter(o => o.type !== 'goal');

    // Direct line check first (instant if unblocked, with real breathing room)
    if (!isSegmentBlocked(start, goal, validObs, SHORTCUT_MARGIN)) {
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
        // Prefer a shortcut with real breathing room (SHORTCUT_MARGIN) first;
        // only if NONE exists anywhere in the remaining path does this fall
        // back to the tightest legally-clear one (0 margin) — still strictly
        // better than jumping straight to the currIdx+1 fallback, which would
        // needlessly follow the raw grid path turn-by-turn even where a
        // perfectly safe, less jagged shortcut (just not a wide-open one)
        // was actually available.
        let farthest = currIdx + 1;
        let found = false;
        for (let nextIdx = rawPath.length - 1; nextIdx > currIdx + 1; nextIdx--) {
            if (!isSegmentBlocked(rawPath[currIdx], rawPath[nextIdx], validObs, SHORTCUT_MARGIN)) {
                farthest = nextIdx;
                found = true;
                break;
            }
        }
        if (!found) {
            for (let nextIdx = rawPath.length - 1; nextIdx > currIdx + 1; nextIdx--) {
                if (!isSegmentBlocked(rawPath[currIdx], rawPath[nextIdx], validObs)) {
                    farthest = nextIdx;
                    break;
                }
            }
        }
        smoothPath.push(rawPath[farthest]);
        currIdx = farthest;
    }

    return smoothPath;
}

// Builds the final "approach the berth" tail of a Mode 3 docking plan —
// shared by docking.js's initial kickoff and navigation.js's live replan so
// the two can never disagree. The hand-tuned align->creep hop (a single,
// otherwise unchecked straight line from the align pivot point into the
// berth) is deliberately positioned clear of the wall the boat is actually
// docking AT, but nothing validated it against a DIFFERENT nearby obstacle
// sitting across that line — most commonly a neighboring jetty, when the
// align point ends up roughly the same x/y as the target (e.g. a bow-in
// spine-pier dock aligns due north of the berth, then creeps straight south
// through whatever's on that line). That let the boat's PLANNED route visibly
// cut through solid dock structure, even though real hull collision then
// correctly refused to actually drive through it — a planning bug, not a
// collision one. Falls back to a full A*-route from startPos straight to the
// creep target (skipping the align pivot state for this one run, still
// ending on the same high-precision 'creep' leg for the final insertion)
// only when the straight line is actually unsafe; the common, already-clear
// case keeps the original hand-tuned align+creep precision maneuver as-is.
function buildDockingApproachWaypoints(startPos, alignX, alignY, creepTarget, parkedYaw, obstacles) {
    if (!isSegmentBlocked({ x: alignX, y: alignY }, creepTarget, obstacles)) {
        const transit = findOptimalPath(startPos, { x: alignX, y: alignY }, obstacles);
        return [
            ...transit.map(p => ({ x: p.x, y: p.y, mode: 'transit' })),
            { x: alignX, y: alignY, mode: 'align', targetYaw: parkedYaw },
            { x: creepTarget.x, y: creepTarget.y, mode: 'creep' }
        ];
    }
    const fullRoute = findOptimalPath(startPos, creepTarget, obstacles);
    return [
        ...fullRoute.slice(0, -1).map(p => ({ x: p.x, y: p.y, mode: 'transit' })),
        { x: creepTarget.x, y: creepTarget.y, mode: 'creep' }
    ];
}
