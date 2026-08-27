// ================= COLREGS ENCOUNTER CLASSIFICATION =================
// Ported from the user's Python source at
// Matlab/Navigation-Guidance-and-Control/ROS_Code/src/VO_collision_avoidance
// (vo_factors.py: encounter_situation, is_give_way_ship) — classification and
// give-way determination ONLY. That source's actual velocity-obstacle safe-
// heading search (vo.py/colregs.py's calculate_vo/colregs_velocity_angle) was
// deliberately NOT ported — this file only answers "what kind of encounter is
// this, and does this boat have to give way", then guidance.js's
// localCorrectedYaw() uses that to restrict its candidate-heading fan to
// starboard-turning options, the classic "when in doubt, turn to starboard"
// COLREGS rule. isPointBlocked() (unchanged) still decides which of those
// candidates is actually safe — this only narrows which ones get tried.
//
// Bearing/heading convention translation: the Python source measures bearing
// CLOCKWISE from own heading (0=ahead, 90=starboard beam, 180=astern,
// 270=port beam) — standard maritime convention. This codebase's boatPos.yaw
// is the ordinary math convention instead (radians, CCW-positive, 0=+X world
// axis; confirmed by input.js's Mode 2 controls, where the 'left' key sends
// positive angular.z — i.e. increasing yaw turns left/port, matching ROS
// REP103). relativeBearingDeg() below does that conversion once so the
// classification functions can keep the Python source's original bearing
// thresholds unchanged. Course/heading DIFFERENCES (encounterSituation's
// courseDiff) don't need translating — an absolute angular difference between
// two headings is the same value whichever rotation direction is "positive".

function normalizeAngleDeg(deg) {
    return ((deg % 360) + 360) % 360;
}

// Clockwise bearing (degrees) from (ownX, ownY) facing ownYaw (radians, this
// codebase's CCW convention) to the point (tsX, tsY) — 0 = dead ahead,
// 90 = starboard beam, 180 = astern, 270 = port beam.
function relativeBearingDeg(ownX, ownY, ownYaw, tsX, tsY) {
    const trueBearingRad = Math.atan2(tsY - ownY, tsX - ownX); // CCW math bearing to target
    const relRad = trueBearingRad - ownYaw; // + = target to port (CCW), - = target to starboard (CW)
    return normalizeAngleDeg(-relRad * 180 / Math.PI); // flip CCW -> CW to match the Python convention
}

// Ported 1:1 from vo_factors.py::encounter_situation. ownYawRad/tsYawRad: own
// and target heading, radians, this codebase's convention (see file header —
// only their DIFFERENCE is used, so no conversion needed). bearingDeg: from
// relativeBearingDeg() above.
function encounterSituation(ownYawRad, tsYawRad, bearingDeg) {
    const ownDeg = normalizeAngleDeg(ownYawRad * 180 / Math.PI);
    const tsDeg = normalizeAngleDeg(tsYawRad * 180 / Math.PI);
    const diff = Math.abs(ownDeg - tsDeg);
    const courseDiff = diff <= 180 ? diff : 360 - diff;
    const b = normalizeAngleDeg(bearingDeg);

    if ((b >= 112.5 && b <= 247.5) || (b <= 67.5 || b >= 292.5)) {
        if (courseDiff < 45) return 'Overtaking';
    }
    if (courseDiff >= 155 && (b <= 25 || b >= 335)) return 'Head-on';
    return 'Crossing';
}

// Is the own ship (this boat) the give-way vessel? Ported 1:1 from
// vo_factors.py::is_give_way_ship — Head-on is intentionally NOT decided
// here (real COLREGS Rule 14 obligates BOTH vessels to turn starboard in a
// head-on encounter, not just whichever one is "give-way"); callers should
// treat encounterSituation() === 'Head-on' as its own always-turn-starboard
// case, same as the Python source's avoidance_action() does. ownSpeed/
// tsSpeed: any consistent unit — only their ratio/comparison matters.
function isGiveWayShip(encSituation, bearingDeg, ownSpeed, tsSpeed) {
    const b = normalizeAngleDeg(bearingDeg);
    if (encSituation === 'Crossing' && b <= 180) return true;
    if (encSituation === 'Overtaking') {
        if (ownSpeed > tsSpeed || b <= 67.5 || b >= 292.5) return true;
    }
    return false;
}
