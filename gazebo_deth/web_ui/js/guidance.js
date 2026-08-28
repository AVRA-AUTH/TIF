// ================= ILOS GUIDANCE (ported from the user's MATLAB ILOS_wrapper) =================
// Depends on config.js (ILOS_*/DWA_*) and pathfinding.js (isPointBlocked, used by localCorrectedYaw).
// Integral Line-of-Sight: a real marine guidance law (Fossen-style), not an
// ad-hoc heuristic — switches active path segment by along-track progress
// vs R_SWITCH, computes cross-track error `e` off the active segment's line,
// and folds a sigma-modified integral of `e` into the desired heading so
// steady disturbances (current/drift, or here: wave bounce, hull-contact
// backoffs, momentum overshoot) get rejected over time instead of just
// producing a constant offset. This REPLACES the nav loop's old
// point-to-point atan2(dy,dx) pursuit heading for transit legs only —
// align/creep/docking maneuvers are untouched, see the nav loop wiring.
//
// Starting tuning, scaled down from the MATLAB reference's real-world values
// (Delta=8, sigma=0.05, R_switch=2.0 at u_d0=1.0 m/s) for this sim's
// WORLD_SCALE-compressed world and this boat's real thrust-derived top speed
// (MAX_LINEAR_FWD=2.49 m/s, not 1.0) — same "port the algorithm exactly, then
// retune the constants for this scale" treatment every other empirically-set
// number in this file gets. sigma is a dimensionless ratio, left as-is.

// `state` is an explicit {k, y_int} object owned by the caller — JS has no
// equivalent of MATLAB's `persistent`. `k` is 1-indexed (segment k runs from
// waypoints[k-1] to waypoints[k] in this 0-indexed array), matching the
// MATLAB source exactly except for that indexing translation. Reset state
// when a fresh navigation run starts (see the nav-loop call sites) — NOT on
// every live A* replan, so the integral's disturbance estimate survives a
// replan even though `k` naturally restarts at segment 1 (the replanned path
// always starts at the boat's current position).
function ilosGuidance(x, y, waypoints, state, dt) {
    const nWpt = waypoints.length;
    let k = Math.min(Math.max(Math.round(state.k), 1), nWpt - 1);

    while (true) {
        const xk = waypoints[k - 1].x, yk = waypoints[k - 1].y;
        const xk1 = waypoints[k].x, yk1 = waypoints[k].y;
        const dx = xk1 - xk, dy = yk1 - yk;
        const Lk = Math.hypot(dx, dy);
        const alpha_k = Math.atan2(dy, dx);
        const s = (x - xk) * Math.cos(alpha_k) + (y - yk) * Math.sin(alpha_k);
        if (k < nWpt - 1 && (Lk - s) <= ILOS_R_SWITCH) {
            k++;
        } else {
            break;
        }
    }
    state.k = k;

    const xk = waypoints[k - 1].x, yk = waypoints[k - 1].y;
    const xk1 = waypoints[k].x, yk1 = waypoints[k].y;
    const alpha_k = Math.atan2(yk1 - yk, xk1 - xk);
    const e = -(x - xk) * Math.sin(alpha_k) + (y - yk) * Math.cos(alpha_k);

    const y_int_dot = (ILOS_DELTA * e) / (Math.pow(e + ILOS_SIGMA * state.y_int, 2) + ILOS_DELTA * ILOS_DELTA);
    state.y_int += dt * y_int_dot;

    let psi_d = alpha_k - Math.atan((e + ILOS_SIGMA * state.y_int) / ILOS_DELTA);
    psi_d = Math.atan2(Math.sin(psi_d), Math.cos(psi_d));

    return { psi_d, e, seg: k };
}

// Lightweight local safety/correction check — not a full velocity-space DWA,
// just a small fan of candidate headings sampled every tick (not just at the
// 250ms A* replan), so a sudden obstacle or disturbance-induced drift gets
// corrected immediately instead of waiting for the next global replan. Reuses
// isPointBlocked() — the SAME predicate the pathfinder trusts — so "clear
// according to this" can never disagree with "clear according to the plan."
// Returns the safest heading near `preferredYaw`; a no-op when that heading
// is already clear (checked first, so this matches ILOS/the planned heading
// exactly in the common case).
//
// restrictToStarboard (colregs.js): when the nav loop determines this boat is
// COLREGS give-way (or the encounter is head-on) against a nearby dynamic
// ship, only the 0/starboard-turning offsets are tried — never a port turn
// that would cut across the other vessel's bow. Falls back to preferredYaw,
// same as the unrestricted fan, if none of those are clear.
function localCorrectedYaw(x, y, speed, preferredYaw, obstacles, restrictToStarboard) {
    const travel = Math.max(Math.abs(speed), 0.5) * DWA_LOOKAHEAD_SEC; // assume at least a slow crawl so this still looks ahead near a stop
    const offsets = restrictToStarboard ? DWA_HEADING_OFFSETS.filter(o => o <= 0) : DWA_HEADING_OFFSETS;
    for (const offset of offsets) {
        const candidateYaw = preferredYaw + offset;
        const endX = x + Math.cos(candidateYaw) * travel;
        const endY = y + Math.sin(candidateYaw) * travel;
        let clear = true;
        const SAMPLES = 4;
        for (let i = 1; i <= SAMPLES; i++) {
            const t = i / SAMPLES;
            if (isPointBlocked(x + (endX - x) * t, y + (endY - y) * t, obstacles)) { clear = false; break; }
        }
        if (clear) return candidateYaw;
    }
    return preferredYaw; // nothing clear in the fan — fall back to the hard obstacle backstop already in place
}
