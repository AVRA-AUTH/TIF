// ================= SHARED PHYSICS HELPER =================
// Used by both navigation.js's runNavigationStep() and input.js's
// thrusterLoop() — a simple rate-limited ramp toward a target velocity.
// Depends on config.js (THRUST_RAMP_RATE/WATER_FRICTION_RATE).

function approachVelocity(current, target, dt) {
    const rate = target !== 0 ? THRUST_RAMP_RATE : WATER_FRICTION_RATE;
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

