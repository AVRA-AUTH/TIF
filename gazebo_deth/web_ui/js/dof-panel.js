// ============== DOF Panel (Mode 2 only — real /odom, not the JS sim) ============== //
// Scoped to surge and yaw rate specifically — heave/pitch are the interesting
// DOFs for the buoyancy bug hunted down in HANDOFF.md's Session update, but
// surge/yaw are what matter for actually driving the boat, which is this
// panel's purpose. Heave/pitch stay diagnosable via a raw /odom capture if
// that investigation ever needs to resume.
const DOF_HISTORY_LEN = 150; // ~3s of history at the odom plugin's ~50Hz rate
const dofPanel = document.getElementById('dof-panel');
const dofHist = { surge: [], yawRate: [] };
const dofCanvas = {
    surge: document.getElementById('dof-surge-chart'),
    yawRate: document.getElementById('dof-yawrate-chart'),
};
const dofValEl = {
    surge: document.getElementById('dof-surge-val'),
    yawRate: document.getElementById('dof-yawrate-val'),
};

function pushDof(key, value) {
    const arr = dofHist[key];
    arr.push(value);
    if (arr.length > DOF_HISTORY_LEN) arr.shift();
}

// Simple auto-scaled sparkline with a zero reference line — good enough to see
// oscillation, bias, and decay at a glance without pulling in a charting lib.
function drawSparkline(canvas, hist) {
    if (!canvas || hist.length < 2) return;
    const ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    let min = Math.min(...hist, 0);
    let max = Math.max(...hist, 0);
    const range = (max - min) || 1;
    const pad = range * 0.15;
    min -= pad;
    max += pad;

    const zy = h - ((0 - min) / (max - min)) * h;
    ctx.strokeStyle = '#333344';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, zy);
    ctx.lineTo(w, zy);
    ctx.stroke();

    ctx.strokeStyle = '#00ffcc';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    hist.forEach((v, i) => {
        const x = (i / (hist.length - 1)) * w;
        const y = h - ((v - min) / (max - min)) * h;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
}

function updateDofPanel(surge, yawRate) {
    pushDof('surge', surge);
    pushDof('yawRate', yawRate);

    dofValEl.surge.textContent = surge.toFixed(2);
    dofValEl.yawRate.textContent = yawRate.toFixed(2);

    drawSparkline(dofCanvas.surge, dofHist.surge);
    drawSparkline(dofCanvas.yawRate, dofHist.yawRate);
}

// ============== Live thrust/power readout (real /odom, not commanded target) ============== //
// "How hard are the real thrusters working right now" — solved from the
// boat's actual measured speed via the same drag equation MAX_LINEAR_FWD/REV
// were derived from (thrust = xU*v + xUU*v^2, both thrusters combined), not
// from whatever speed was merely requested. That matters because a commanded
// target can ask for more than the hardware can deliver (it just saturates)
// — this readout only ever reports what the real thrusters are actually
// doing. Current/power are a linear estimate off that (see config.js's
// T200_MAX_CURRENT_FWD_A/REV_A comment for why it stops there and doesn't
// also claim an efficiency/"% battery wasted" number).
const thrustReadoutEl = {
    pct: document.getElementById('tele-thrust-pct'),
    n: document.getElementById('tele-thrust-n'),
    a: document.getElementById('tele-thrust-a'),
    w: document.getElementById('tele-thrust-w'),
};

function updateThrustReadout(surge) {
    if (!thrustReadoutEl.pct) return;

    const forward = surge >= 0;
    const thrustN = Math.sign(surge) * (HULL_DRAG_LINEAR * Math.abs(surge) + HULL_DRAG_QUADRATIC * surge * surge);
    const maxThrustN = forward ? THRUSTER_MAX_FWD_N * 2 : Math.abs(THRUSTER_MIN_REV_N) * 2;
    const pct = Math.min(100, (Math.abs(thrustN) / maxThrustN) * 100);
    const maxCurrentA = (forward ? T200_MAX_CURRENT_FWD_A : T200_MAX_CURRENT_REV_A) * 2;
    const estCurrentA = (pct / 100) * maxCurrentA;
    const estPowerW = estCurrentA * BATTERY_VOLTAGE_V;

    thrustReadoutEl.pct.textContent = pct.toFixed(0);
    thrustReadoutEl.n.textContent = Math.abs(thrustN).toFixed(1);
    thrustReadoutEl.a.textContent = estCurrentA.toFixed(1);
    thrustReadoutEl.w.textContent = estPowerW.toFixed(0);
}
