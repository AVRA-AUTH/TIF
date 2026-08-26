// ================= UNIFIED A* + STRING-PULLING PATHFINDER ================= //
// Depends on config.js (SAFETY_RADIUS/SHORE_MARGIN/LAKE_RADIUS/ISLAND_*/
// MARINA_*/JETTY_*).

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
