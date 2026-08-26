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

