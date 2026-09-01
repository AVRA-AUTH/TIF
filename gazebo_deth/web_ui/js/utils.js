// ================= SHARED PHYSICS HELPER =================
// Used by both navigation.js's runNavigationStep() and input.js's
// thrusterLoop() — a simple rate-limited ramp toward a target velocity.
// Depends on config.js (THRUST_RAMP_RATE/WATER_FRICTION_RATE).

// overrideRate: optional — skips the THRUST_RAMP_RATE/WATER_FRICTION_RATE
// default entirely (see navigation.js's AUTONOMOUS_ANGULAR_RAMP_RATE use —
// THRUST_RAMP_RATE is tuned for Mode 2's snappy manual joystick feel, reaching
// full turn rate in ~0.2s, which read as an abrupt snap-turn when reused for
// autonomous steering: every 250ms live replan that picked a meaningfully
// different heading turned into a near-instant hard turn instead of a smooth
// arc into the new heading). Every other caller omits it, unchanged.
function approachVelocity(current, target, dt, overrideRate) {
    const rate = overrideRate !== undefined ? overrideRate : (target !== 0 ? THRUST_RAMP_RATE : WATER_FRICTION_RATE);
    const maxStep = rate * dt;
    const diff = target - current;
    if (Math.abs(diff) <= maxStep) return target;
    return current + Math.sign(diff) * maxStep;
}

// Inverse of the hull's own drag equation (xU*v + xUU*v^2 = thrust — same
// equation exhibition_water.sdf's derivation comment solves for
// MAX_LINEAR_FWD/REV, mirrored here as HULL_DRAG_LINEAR/QUADRATIC) — given a
// COMBINED thrust in Newtons, returns the steady-state speed it produces.
// Used by input.js's arrow-key drive to convert boatSpeedMultiplier (a
// THRUST fraction — same meaning W/A/R/D already gives it) into the
// matching /cmd_vel velocity target, so both control schemes' slow/medium/
// fast presets land on the same real percentage of max thrust instead of
// two different quantities (speed% vs thrust%) that happen to share a
// button label.
function speedForThrust(thrustN) {
    return (-HULL_DRAG_LINEAR + Math.sqrt(HULL_DRAG_LINEAR * HULL_DRAG_LINEAR + 4 * HULL_DRAG_QUADRATIC * thrustN)) / (2 * HULL_DRAG_QUADRATIC);
}

