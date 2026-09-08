// ================= MODE 2 "BUOY RUN" CHALLENGE =================
// Turns Mode 2's free-drive joystick sandbox into an actual objective: the
// course (buoys, patrol boats, danger zones, numbered checkpoints, a finish
// goal) lays out the moment Mode 2 is entered; the timer/win-fail challenge
// arms either by pressing START CHALLENGE or just by starting to drive (see
// autoStartMode2Challenge(), called from input.js's thrusterLoop). Reach
// every numbered checkpoint in order, then the goal, without touching
// anything or leaving the marked channel. Reuses Mode 1's existing
// entity system for buoys/boats/goal (same collision, same 3D meshes via
// entities-3d.js's sync3DEntities(), same patrol AI via navigation.js's
// updateDynamicEntities(), same 2D map icons via render-2d.js's
// renderEntities2D() — none of those needed to change) — only danger zones
// are new, since Mode 1 has no equivalent concept.
// Depends on config.js (MODE2_*), state.js (mode2GameStarted/Result/etc,
// entities, obsCounter), ros.js (spawnTopic), scene-environment.js (scene),
// entities-3d.js (threeEntities), boundaries.js (isTouchingShipOrBuoyAt),
// lifecycle.js (resetBoatToPose), input.js (stopThrusters/showAlert).

// Builds the course: pushes buoy/dynamic/goal entities into the SAME
// `entities` array Mode 1 uses (tagged `mode2Game: true` so
// teardownMode2Course() below can find exactly these and nothing else), and
// spawns the real Gazebo-side obstacle models the same way
// placeMode1EntityAt() does. Danger zones aren't an `entities` type (nothing
// else in the app knows what one is), so they get their own array + 3D mesh.
function spawnMode2Course() {
    MODE2_COURSE_BUOYS.forEach(pt => {
        const id = `obs_${obsCounter++}`;
        entities.push({ type: 'static', ros_x: pt.x, ros_y: pt.y, id, mode2Game: true });
        spawnTopic.publish(new ROSLIB.Message({ data: JSON.stringify({ type: 'static', x: pt.x, y: pt.y, id }) }));
    });
    MODE2_COURSE_BOATS.forEach(pt => {
        const id = `obs_${obsCounter++}`;
        // baseSpeed carries the course's medium/fast tier through to
        // navigation.js's patrol-init (updateDynamicEntities()), which
        // respects a pre-set value instead of always defaulting to 1.0.
        entities.push({ type: 'dynamic', ros_x: pt.x, ros_y: pt.y, id, mode2Game: true, baseSpeed: pt.speed });
        spawnTopic.publish(new ROSLIB.Message({ data: JSON.stringify({ type: 'dynamic', x: pt.x, y: pt.y, id }) }));
    });
    // Client-side only, same as Mode 1's own goal placement
    // (placeMode1EntityAt() in input.js) — a goal marker is never published
    // to the real Gazebo backend via spawnTopic, only buoys/boats are.
    entities.push({ type: 'goal', ros_x: MODE2_GOAL.x, ros_y: MODE2_GOAL.y, id: 'mode2_goal', mode2Game: true });

    mode2DangerZones = MODE2_DANGER_ZONES.map(z => ({ ...z }));
    build3DDangerZoneMeshes();
    build3DCheckpointMeshes();
    build3DBoundaryWalls();
}

// Removes every course entity/mesh/zone/checkpoint and resets the challenge
// state back to "not started" — called on Restart (via startMode2Challenge
// below), on Mode 2's own STOP button, and on every mode switch (input.js),
// so a half-finished course never survives into a fresh run or a different
// mode.
function teardownMode2Course() {
    entities = entities.filter(ent => {
        if (!ent.mode2Game) return true;
        const mesh = threeEntities.get(ent.id);
        if (mesh) { scene.remove(mesh); threeEntities.delete(ent.id); }
        return false;
    });
    removeMode2DangerZoneMeshes();
    mode2DangerZones = [];
    removeMode2CheckpointMeshes();
    removeMode2BoundaryMeshes();
    mode2NextCheckpoint = 0;
    mode2GameStarted = false;
    mode2Result = null;
    mode2StartTime = 0;
}

// Tears down whatever course currently exists and lays out a fresh one —
// "the arena," independent of whether the timer/win-fail challenge is
// actually running. Called both when just entering/re-entering Mode 2
// (input.js's mode2Btn handler) so the course is visible immediately rather
// than empty water until the player starts driving, and by
// startMode2Challenge() below so a start/restart always hands back the SAME
// fresh layout — no leftover sunk boats, no patrol boats mid-loop from a
// previous attempt.
function resetMode2Arena() {
    teardownMode2Course();
    resetBoatToPose(MODE2_START);
    spawnMode2Course();
}

// Arms the timer/win-fail checking — the part shared by both ways a run can
// begin (see below). Does NOT touch the arena itself; the caller decides
// whether one is needed first.
function armMode2Challenge() {
    mode2GameStarted = true;
    mode2Result = null;
    mode2StartTime = Date.now();
    showMode2Toast('🏁 Challenge Started! Reach every checkpoint, then the goal.', '#28a745');
}

// START CHALLENGE / RESTART button — one button does both: resetting the
// arena before arming the timer makes a mid-run restart and a first-ever
// start the same code path, so there's no separate "reset" case to keep in
// sync with this one. Always rebuilds the arena (fresh buoys, patrol boats
// back at their spawns, no leftover sunk boats) so "the arena starts the
// same" every time this is pressed.
function startMode2Challenge() {
    resetMode2Arena();
    armMode2Challenge();
}

document.getElementById('btn-mode2-start')?.addEventListener('click', startMode2Challenge);

// The OTHER way a run can begin: the player just starts driving without
// pressing START CHALLENGE first. Called from input.js's thrusterLoop the
// moment real driving intent is detected in Mode 2 while the challenge isn't
// armed yet — arms the clock on whatever arena is already sitting there
// (already fresh, since Mode 2 always lays one out the moment it's entered —
// see input.js's mode2Btn handler) rather than resetting it, so the boat
// doesn't jump back to the start line out from under a player who's already
// mid-maneuver on their very first input.
function autoStartMode2Challenge() {
    if (mode2GameStarted) return;
    armMode2Challenge();
}

// ---- Non-blocking toast (index.html's #mode2-toast) — see the CSS/HTML
// comments for why this isn't the modal custom-alert-overlay. ----
let mode2ToastHideTimer = null;
function showMode2Toast(text, color) {
    const el = document.getElementById('mode2-toast');
    if (!el) return;
    el.textContent = text;
    el.style.borderColor = color || '#00d9ff';
    el.style.color = color || '#00d9ff';
    el.style.display = 'block';
    requestAnimationFrame(() => el.classList.add('visible')); // one frame so the display:block above has already painted, or the opacity transition never plays
    clearTimeout(mode2ToastHideTimer);
    mode2ToastHideTimer = setTimeout(() => {
        el.classList.remove('visible');
        setTimeout(() => { el.style.display = 'none'; }, 300); // matches the CSS transition duration
    }, 1800);
}

// ---- Danger zone 3D rendering: one translucent disc per zone (no existing
// entity type covers this). Was a disc+rim pair per zone — merged into a
// single mesh (slightly higher opacity stands in for the rim's extra
// definition) since transparent meshes are real GPU cost repeated every
// frame regardless of camera angle, and this course adds several of these
// on top of everything else already in the scene. ----
function build3DDangerZoneMeshes() {
    mode2DangerZones.forEach(zone => {
        const discMat = new THREE.MeshBasicMaterial({ color: 0xcc2233, transparent: true, opacity: 0.22, side: THREE.DoubleSide });
        const disc = new THREE.Mesh(new THREE.CircleGeometry(zone.radius, 24), discMat);
        disc.rotation.x = -Math.PI / 2;
        disc.position.set(zone.x, 0.05, -zone.y);
        disc.userData.discMat = discMat;
        scene.add(disc);
        mode2DangerZoneMeshes.push(disc);
    });
}
function removeMode2DangerZoneMeshes() {
    mode2DangerZoneMeshes.forEach(m => scene.remove(m));
    mode2DangerZoneMeshes = [];
}

// ---- Channel boundary in 3D: a low, translucent wall along each of the 4
// edges of MODE2_COURSE_BOUNDS — the 2D map's dashed red rectangle only
// helps while looking at that panel; from the FPV camera (which is the main
// view while actually driving) the boundary was otherwise entirely
// invisible. Same muted red + low opacity as the danger zones for a
// consistent, calm "hazard" language rather than a bright fence. ----
function build3DBoundaryWalls() {
    const b = MODE2_COURSE_BOUNDS;
    const WALL_HEIGHT = 2.5;
    const wallMat = new THREE.MeshBasicMaterial({ color: 0xcc2233, transparent: true, opacity: 0.16, side: THREE.DoubleSide });
    const edges = [
        { x1: b.xMin, y1: b.yMin, x2: b.xMax, y2: b.yMin }, // south
        { x1: b.xMax, y1: b.yMin, x2: b.xMax, y2: b.yMax }, // east
        { x1: b.xMax, y1: b.yMax, x2: b.xMin, y2: b.yMax }, // north
        { x1: b.xMin, y1: b.yMax, x2: b.xMin, y2: b.yMin }, // west
    ];
    edges.forEach(e => {
        const length = Math.hypot(e.x2 - e.x1, e.y2 - e.y1);
        const wall = new THREE.Mesh(new THREE.PlaneGeometry(length, WALL_HEIGHT), wallMat);
        const midX = (e.x1 + e.x2) / 2, midY = (e.y1 + e.y2) / 2;
        wall.position.set(midX, WALL_HEIGHT / 2, -midY);
        // Y-axis rotation only: a PlaneGeometry's local +X is its width
        // direction and local +Y (already world-up) is its height, so this
        // alone points the wall's width along the edge — same x/-y world
        // convention as every other mesh placement in this codebase.
        wall.rotation.y = Math.atan2(e.y2 - e.y1, e.x2 - e.x1);
        scene.add(wall);
        mode2BoundaryMeshes.push(wall);
    });
}
function removeMode2BoundaryMeshes() {
    mode2BoundaryMeshes.forEach(m => scene.remove(m));
    mode2BoundaryMeshes = [];
}
// Gentle opacity pulse so a hazard reads as "active/live," not a flat decal
// — kept subtle on purpose (small amplitude, slow) so it reads as a calm
// warning marker, not a flashing alert. Called once per frame from main.js,
// gated on Mode 2 there.
function pulseMode2DangerZones() {
    const t = Date.now() * 0.0015;
    const pulse = 0.18 + 0.06 * Math.sin(t);
    mode2DangerZoneMeshes.forEach(m => { m.userData.discMat.opacity = pulse; });
}

// ---- Checkpoint 3D rendering: just a floating numbered sprite — a
// THREE.Sprite always faces the camera on its own, which is what makes a
// legible number readable from any angle without hand-rotating anything, and
// the 2D map already shows each checkpoint's real radius, so a matching
// ground-level disc+rim here was redundant weight (2 more transparent
// meshes per checkpoint) rather than useful information. Colored per
// mode2NextCheckpoint each frame (updateMode2CheckpointVisuals()) via the
// sprite's own material.color tint rather than baked into the texture,
// since which checkpoint is "current" vs "already passed" changes as the
// run progresses. ----
function makeCheckpointNumberTexture(number) {
    const canvas = document.createElement('canvas');
    canvas.width = 128;
    canvas.height = 128;
    const c = canvas.getContext('2d');
    c.fillStyle = '#ffffff';
    c.beginPath();
    c.arc(64, 64, 56, 0, 2 * Math.PI);
    c.fill();
    c.fillStyle = '#001018';
    c.font = 'bold 76px sans-serif';
    c.textAlign = 'center';
    c.textBaseline = 'middle';
    c.fillText(String(number), 64, 70);
    return new THREE.CanvasTexture(canvas);
}
function build3DCheckpointMeshes() {
    MODE2_CHECKPOINTS.forEach(cp => {
        const spriteMat = new THREE.SpriteMaterial({ map: makeCheckpointNumberTexture(cp.number), transparent: true });
        const sprite = new THREE.Sprite(spriteMat);
        sprite.scale.set(3, 3, 1);
        sprite.position.set(cp.x, 4, -cp.y);
        scene.add(sprite);
        mode2CheckpointMeshes.push(sprite);
    });
}
function removeMode2CheckpointMeshes() {
    mode2CheckpointMeshes.forEach(m => scene.remove(m));
    mode2CheckpointMeshes = [];
}
// Recolors each checkpoint's number-badge sprite by state (passed = green,
// current target = teal, not-yet-relevant = dim gray) via a tint on the
// sprite's own material — the texture's dark text stays legible regardless
// (near-black * any tint is still near-black), only the white badge
// background actually picks up the color. Called once per frame from
// main.js, gated on Mode 2 there.
function updateMode2CheckpointVisuals() {
    mode2CheckpointMeshes.forEach((sprite, i) => {
        const passed = i < mode2NextCheckpoint;
        const isCurrent = i === mode2NextCheckpoint;
        const color = passed ? 0x28a745 : isCurrent ? 0x2a9fb5 : 0x888888;
        sprite.material.color.setHex(color);
    });
}

// ---- 2D tactical map rendering: course boundary + danger zones ----
// (buoys/patrol boats/goal already draw themselves via render-2d.js's
// renderEntities2D(), same as Mode 1 — nothing extra needed for those here.)
// Flat, plain-shapes style to match Mode 3's marina map (solid fills, thin
// solid outlines, no dashed lines, no per-frame animation) instead of the
// original's stacked dashed/pulsing rings — that read as "flashy" and
// cluttered once combined with 7 buoys + 4 boats worth of their own dashed
// keep-out rings (now suppressed for this course, see render-2d.js's
// renderEntities2D()), and dashed strokes are also the single most expensive
// thing to draw repeatedly on a Canvas2D map redrawn from scratch every
// frame — cutting them here is a real, measurable frame-cost saving, not
// just a style choice. One shared ctx.save()/restore() pair for the whole
// function instead of one per shape, for the same reason.
function renderMode2Extras(mapScale) {
    // Not gated on mode2GameStarted — the course/danger zones/boundary are
    // laid out (resetMode2Arena()) the moment Mode 2 is entered, before the
    // timer/win-fail challenge itself is armed, so they should already be
    // visible then too.
    if (activeAppMode !== 2) return;

    ctx.save();

    // Channel boundary — thin solid line, no fill.
    ctx.strokeStyle = 'rgba(220, 90, 90, 0.55)';
    ctx.lineWidth = 1.5;
    const tl = rosToCanvas(MODE2_COURSE_BOUNDS.xMin, MODE2_COURSE_BOUNDS.yMax);
    const br = rosToCanvas(MODE2_COURSE_BOUNDS.xMax, MODE2_COURSE_BOUNDS.yMin);
    ctx.strokeRect(tl.x, tl.y, br.x - tl.x, br.y - tl.y);

    // Danger zones — solid outline, light flat fill, no label (the red
    // color alone reads as "hazard," same as Mode 3's plain colored
    // rectangles carry all their own meaning with no extra decoration).
    ctx.fillStyle = 'rgba(204, 34, 51, 0.18)';
    ctx.strokeStyle = '#cc4455';
    ctx.lineWidth = 1.5;
    mode2DangerZones.forEach(zone => {
        const p = rosToCanvas(zone.x, zone.y);
        ctx.beginPath();
        ctx.arc(p.x, p.y, zone.radius * mapScale, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();
    });

    // Checkpoints — small numbered marker, thin outline, no fill (keeps it
    // visually lighter than the buoys/danger zones so it reads as "waypoint,"
    // not "obstacle").
    ctx.font = 'bold 12px sans-serif';
    ctx.textAlign = 'center';
    ctx.lineWidth = 1.5;
    MODE2_CHECKPOINTS.forEach((cp, i) => {
        const p = rosToCanvas(cp.x, cp.y);
        const passed = i < mode2NextCheckpoint;
        const isCurrent = i === mode2NextCheckpoint;
        const color = passed ? '#28a745' : isCurrent ? '#2a9fb5' : '#556677';
        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 9, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.fillText(passed ? '✓' : String(cp.number), p.x, p.y + 4);
    });

    ctx.restore();
}

// ---- Win/fail checking — called once per frame from main.js, gated on
// Mode 2 there. Mirrors Mode 1's ship/buoy-collision handling in
// navigation.js (same isTouchingShipOrBuoyAt() check, same crash/sink +
// delayed popup pattern) but adds the three fail conditions Mode 1 doesn't
// have (timeout, danger zone, out-of-bounds) plus the win condition. ----
let mode2LastGoalHintAt = 0; // throttles the "reach every checkpoint first" toast below
function checkMode2GameState() {
    if (activeAppMode !== 2 || !mode2GameStarted || mode2Result) return;

    const elapsed = Date.now() - mode2StartTime;

    if (elapsed >= MODE2_TIME_LIMIT_MS) {
        endMode2Challenge('lose', "⏱️ Time's Up! You didn't reach the goal in time.", '⏱️', false);
        return;
    }

    const b = MODE2_COURSE_BOUNDS;
    if (boatPos.x < b.xMin || boatPos.x > b.xMax || boatPos.y < b.yMin || boatPos.y > b.yMax) {
        endMode2Challenge('lose', '🌊 Out of Bounds! You left the safe channel.', '🌊', true);
        return;
    }

    for (const zone of mode2DangerZones) {
        if (Math.hypot(boatPos.x - zone.x, boatPos.y - zone.y) < zone.radius) {
            endMode2Challenge('lose', '☢️ Danger Zone! Your boat strayed into a hazard area.', '☢️', true);
            return;
        }
    }

    const hit = isTouchingShipOrBuoyAt(boatPos.x, boatPos.y, boatPos.yaw);
    if (hit) {
        endMode2Challenge(
            'lose',
            hit === 'ship' ? '💥 Collision! You crashed into a moving boat.' : '💥 Collision! You hit a buoy.',
            hit === 'ship' ? '🚢' : '🛟',
            true
        );
        return;
    }

    // Numbered checkpoints must be reached IN ORDER — advance one at a time,
    // never skipping ahead even if the boat happens to swing past a later
    // one's radius first.
    if (mode2NextCheckpoint < MODE2_CHECKPOINTS.length) {
        const cp = MODE2_CHECKPOINTS[mode2NextCheckpoint];
        if (Math.hypot(boatPos.x - cp.x, boatPos.y - cp.y) < MODE2_CHECKPOINT_RADIUS) {
            mode2NextCheckpoint++;
            showMode2Toast(`✅ Checkpoint ${cp.number}/${MODE2_CHECKPOINTS.length} reached!`, '#00d9ff');
        }
    }

    const atGoal = Math.hypot(boatPos.x - MODE2_GOAL.x, boatPos.y - MODE2_GOAL.y) < MODE2_GOAL_RADIUS;
    if (atGoal && mode2NextCheckpoint >= MODE2_CHECKPOINTS.length) {
        const seconds = elapsed / 1000;
        endMode2Challenge('win', `🏆 Challenge Complete! You reached the goal in ${seconds.toFixed(1)}s without a scratch.`, '🏆', false, seconds);
    } else if (atGoal) {
        // Reached the goal early, before every checkpoint — doesn't count
        // yet. Throttled (2s) so parking on the goal spot doesn't spam a
        // toast every single frame.
        const now = Date.now();
        if (now - mode2LastGoalHintAt > 2000) {
            mode2LastGoalHintAt = now;
            const remaining = MODE2_CHECKPOINTS.length - mode2NextCheckpoint;
            showMode2Toast(`🚧 ${remaining} checkpoint${remaining > 1 ? 's' : ''} left before the goal counts!`, '#ffc107');
        }
    }
}

// Freezes the boat and shows the outcome — a real confirm prompt (not a
// bare OK) whose primary button is labeled "Restart"/"Play Again" and
// actually starts a fresh run (startMode2Challenge()) rather than just
// dismissing, so getting back into it after a loss doesn't need a separate
// trip to the sidebar's START CHALLENGE button. Cancel still just closes,
// for "not now — I'll restart later." `sink` mirrors Mode 1's crash
// animation (playerBoatCrashed/playerCrashTime, read by main.js's
// boat-transform step) for an actual hull/hazard hit; a plain timeout
// doesn't get one — nothing physically happened to the boat, it just ran out
// of time. The popup itself is delayed the same 1200ms Mode 1 uses so the
// sink animation gets a few rendered frames in first; a non-sinking result
// (timeout) pops immediately. `finishSeconds` (win only) gets recorded on
// the persisted best-times leaderboard.
function endMode2Challenge(result, message, icon, sink, finishSeconds) {
    mode2Result = result;
    stopThrusters();
    if (sink) {
        playerBoatCrashed = true;
        playerCrashTime = Date.now();
    }
    if (result === 'win') saveMode2Time(finishSeconds);

    const statusEl = document.getElementById('tele-status');
    if (statusEl) {
        statusEl.textContent = result === 'win' ? '🏆 Challenge Complete!' : `${icon} Challenge Failed`;
        statusEl.style.color = result === 'win' ? '#28a745' : '#ff4d4d';
    }

    const restartLabel = result === 'win' ? 'Play Again' : 'Restart';
    setTimeout(() => showConfirm(message, startMode2Challenge, icon, restartLabel), sink ? 1200 : 200);
}

// ---- Best-times leaderboard — localStorage, so it survives both a mode
// switch and a page reload, not just staying in memory for the current
// session. Guarded with try/catch, same pattern as input.js's
// ACTIVE_MODE_STORAGE_KEY use, in case a browser has localStorage disabled. ----
const MODE2_LEADERBOARD_KEY = 'avraMode2BestTimes';
const MODE2_LEADERBOARD_SIZE = 5;

function loadMode2Leaderboard() {
    try {
        const raw = localStorage.getItem(MODE2_LEADERBOARD_KEY);
        return raw ? JSON.parse(raw) : [];
    } catch (e) {
        return [];
    }
}

function saveMode2Time(seconds) {
    const board = loadMode2Leaderboard();
    board.push({ seconds, date: new Date().toISOString() });
    board.sort((a, b) => a.seconds - b.seconds);
    board.length = Math.min(board.length, MODE2_LEADERBOARD_SIZE);
    try {
        localStorage.setItem(MODE2_LEADERBOARD_KEY, JSON.stringify(board));
    } catch (e) { /* localStorage unavailable — the run still counts, it just won't be remembered */ }
    renderMode2Leaderboard();
}

function renderMode2Leaderboard() {
    const el = document.getElementById('mode2-leaderboard');
    if (!el) return;
    const board = loadMode2Leaderboard();
    if (board.length === 0) {
        el.innerHTML = '<li style="color: #8892a6;">No times yet — finish a run!</li>';
        return;
    }
    const medals = ['🥇', '🥈', '🥉'];
    el.innerHTML = board.map((entry, i) =>
        `<li style="margin-bottom: 2px;">${medals[i] || `${i + 1}.`} ${entry.seconds.toFixed(1)}s</li>`
    ).join('');
}
renderMode2Leaderboard(); // populate from any past session immediately, even before Mode 2 is ever opened this session

// ---- Camera HUD: countdown timer row (index.html's #hud-row-timer, right
// on the boat's own camera feed rather than tucked in the sidebar) ----
function updateMode2Hud() {
    const row = document.getElementById('hud-row-timer');
    const valueEl = document.getElementById('hud-timer-value');
    const cpRow = document.getElementById('hud-row-checkpoint');
    const cpValueEl = document.getElementById('hud-checkpoint-value');
    if (!row || !valueEl || !cpRow || !cpValueEl) return;

    const inMode2 = activeAppMode === 2;
    row.style.display = inMode2 ? 'flex' : 'none';
    cpRow.style.display = inMode2 ? 'flex' : 'none';
    if (!inMode2) return;

    cpValueEl.textContent = mode2NextCheckpoint >= MODE2_CHECKPOINTS.length
        ? `${MODE2_CHECKPOINTS.length}/${MODE2_CHECKPOINTS.length} ✅`
        : `${mode2NextCheckpoint}/${MODE2_CHECKPOINTS.length}`;
    cpValueEl.style.color = mode2NextCheckpoint >= MODE2_CHECKPOINTS.length ? '#28a745' : '#00d9ff';

    if (!mode2GameStarted) {
        valueEl.textContent = 'Ready';
        valueEl.style.color = '#a0c4ff';
        return;
    }
    if (mode2Result === 'win') {
        valueEl.textContent = 'Finished!';
        valueEl.style.color = '#28a745';
        return;
    }
    if (mode2Result === 'lose') {
        valueEl.textContent = 'Failed';
        valueEl.style.color = '#ff4d4d';
        return;
    }

    const remainingMs = Math.max(0, MODE2_TIME_LIMIT_MS - (Date.now() - mode2StartTime));
    const secs = Math.ceil(remainingMs / 1000);
    const mm = Math.floor(secs / 60).toString().padStart(2, '0');
    const ss = (secs % 60).toString().padStart(2, '0');
    valueEl.textContent = `${mm}:${ss}`;
    valueEl.style.color = secs <= 15 ? '#ff4d4d' : '#00ffcc';
}

// ---- Land in Mode 2 on every fresh load/refresh — the primary exhibition
// experience, not the Mode 1 sandbox state.js boots into by default. This
// must be the LAST thing this script does (after every function above it is
// defined): mode2Btn's click handler (input.js) calls resetMode2Arena(),
// spawnMode2Course() etc. — all defined in THIS file — so firing the click
// any earlier than this would throw a ReferenceError and abort the handler
// partway through, before the course ever gets laid out. ----
if (typeof mode2Btn !== 'undefined' && mode2Btn) mode2Btn.click();
