// ================= 2D TACTICAL MAP RENDERING =================
// Coordinate transforms plus every 2D-canvas-drawing piece, extracted from
// the original app.js's single draw() function into standalone functions
// called once per frame by main.js's orchestrator. Each takes `mapView`/
// `mapScale` as a parameter rather than relying on module state, since
// those are computed fresh every frame in main.js (not persistent state).
// Depends on config.js and state.js (entities/boatPos/plannedPath/etc);
// entities-3d.js is NOT a dependency (that's the 3D-mesh sync, kept separate
// from this file's 2D canvas drawing).

// Canvas Setup. SCALE is chosen so the navigable lake circle (LAKE_RADIUS,
// defined below) fills most of the 550x500 map canvas instead of being a
// small circle lost in a sea of green land — per explicit user request.
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const SCALE = 1.35;

function getCurrentViewParams() {
    if (activeAppMode === 3) {
        // Zoomed in on Marina
        return { scale: 3.5, offsetX: -195.0 * WORLD_SCALE, offsetY: -145.0 * WORLD_SCALE }; // ROS center of marina
    }
    // Global view
    return { scale: SCALE, offsetX: 0.0, offsetY: 0.0 };
}

function rosToCanvas(rx, ry) {
    const view = getCurrentViewParams();
    const cx = canvas.width / 2 + (rx - view.offsetX) * view.scale;
    const cy = canvas.height / 2 - (ry - view.offsetY) * view.scale;
    return { x: cx, y: cy };
}

function canvasToRos(cx, cy) {
    const view = getCurrentViewParams();
    const rx = (cx - canvas.width / 2) / view.scale + view.offsetX;
    const ry = -(cy - canvas.height / 2) / view.scale + view.offsetY;
    return { rx, ry };
}

// Land background, sandy beach shore ring, circular lake water body, and the
// tactical grid overlay — the base layer everything else draws on top of.
function renderWorldBackground(mapView, mapScale) {
    // 1. Render Outer Land Mass Background (Forest Green)
    ctx.fillStyle = '#1b3b18';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    const centerP = rosToCanvas(0, 0);

    // 2. Render Sandy Beach Shore Ring (Radius: 315m to match 3D world)
    ctx.beginPath();
    ctx.arc(centerP.x, centerP.y, (LAKE_RADIUS + 15.0) * mapScale, 0, 2 * Math.PI);
    ctx.fillStyle = '#d2b48c';
    ctx.fill();

    // 3. Render Circular Blue Lake Water Body (Clean 2D Ocean Fill)
    const waterGrad = ctx.createRadialGradient(centerP.x, centerP.y, 0, centerP.x, centerP.y, LAKE_RADIUS * mapScale);
    waterGrad.addColorStop(0, '#005f9e');
    waterGrad.addColorStop(1, '#002952');

    ctx.beginPath();
    ctx.arc(centerP.x, centerP.y, LAKE_RADIUS * mapScale, 0, 2 * Math.PI);
    ctx.fillStyle = waterGrad;
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#001a33';
    ctx.stroke();

    // 4. Draw Tactical Grid Overlay
    ctx.strokeStyle = 'rgba(255,255,255,0.08)';
    ctx.lineWidth = 1;
    for (let i = 0; i < canvas.width; i += mapScale * 10) {
        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, canvas.height); ctx.stroke();
    }
    for (let i = 0; i < canvas.height; i += mapScale * 10) {
        ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(canvas.width, i); ctx.stroke();
    }
}

// Draw Planned Path Trajectory
function renderPlannedPathLine() {
    if (plannedPath.length > 1) {
        ctx.beginPath();
        ctx.setLineDash([6, 6]);
        ctx.strokeStyle = '#00ffcc';
        ctx.lineWidth = 3;
        const startP = rosToCanvas(plannedPath[0].x, plannedPath[0].y);
        ctx.moveTo(startP.x, startP.y);

        for (let i = 1; i < plannedPath.length; i++) {
            const p = rosToCanvas(plannedPath[i].x, plannedPath[i].y);
            ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
        ctx.setLineDash([]);
    }
}

// Draw 2D Marina using proper scaling for zoomed views
function renderMarina2D(mapScale) {
    ctx.save();
    const s = mapScale;

    // Main Horizontal Pier Spine — drawn from the same canonical MARINA_PIER_*
    // constants (and the pierLength derived from them) the 3D mesh and the
    // collision/pathfinding checks use, instead of separate hardcoded
    // width/height literals that used to draw this at the pre-WORLD_SCALE
    // size (140/58 real meters) regardless of how compressed the rest of
    // the map is — the actual bug behind "the map doesn't match the camera."
    ctx.fillStyle = '#5d4037';
    const spineTL = rosToCanvas(MARINA_PIER_X_MIN, MARINA_PIER_Y); // top-left = min X, max Y (larger ROS y draws higher on screen)
    ctx.fillRect(spineTL.x, spineTL.y, pierLength * s, MARINA_PIER_THICKNESS * s);

    // 3 Vertical Finger Jetties extending up into water — same fix.
    JETTY_X_LIST.forEach(jx => {
        const jettyTL = rosToCanvas(jx - JETTY_HALF_WIDTH, JETTY_Y_NEAR); // top-left = min X, max Y
        ctx.fillRect(jettyTL.x, jettyTL.y, jettyWidth * s, jettyLength * s);
    });

    // Draw Moored Ships Along All Jetties — reads the REAL entities array
    // (isParkedShip, populated by createMooredBoat() above) instead of a
    // separately hardcoded loop over synthetic positions. That old loop
    // always drew every slot regardless of what was actually spawned in the
    // 3D scene, so thinning the moored fleet there (fewer boats, more open
    // choices) would otherwise have left the 2D map showing boats that no
    // longer exist — the same "map doesn't match the camera" bug, just for
    // moored boats instead of the pier/jetty structure.
    const bColors2D = ['#1d3557', '#2a9d8f', '#e63946', '#457b9d', '#0f4c5c', '#3d5a80', '#9b5de5', '#f15bb5'];
    entities.forEach(ent => {
        if (!ent.isParkedShip) return;
        const p = rosToCanvas(ent.ros_x, ent.ros_y);
        // Real hull footprint (ROS x/y extents), not a guessed rectangle —
        // the old hardcoded 3.5x6 was ~4x wider than the real ~1.45m hull
        // along the jetty, so neighboring boats' rectangles visually
        // swallowed the genuinely-open gaps between them (packed only 2.6m
        // apart), making an "Open" hover spot look occupied on the map even
        // though the actual collision check (isBerthOccupied, 4.0m radius on
        // real positions) correctly saw it as clear.
        const w = (ent.hullLength || 3.5) * s, h = (ent.hullWidth || 1.45) * s;
        ctx.fillStyle = bColors2D[Math.abs(Math.floor(ent.ros_x * 10 + ent.ros_y)) % bColors2D.length];
        ctx.fillRect(p.x - w / 2, p.y - h / 2, w, h); // width 3.5, height 6, centered on the boat's real position
        // Thin dark outline so a "slot has a boat" reads clearly against the
        // jetty even when hulls are packed edge-to-edge — no separate
        // keep-out ring/dot needed on top (renderEntities2D skips these
        // entities entirely; this rectangle IS their whole representation).
        ctx.strokeStyle = 'rgba(0, 0, 0, 0.6)';
        ctx.lineWidth = 1;
        ctx.strokeRect(p.x - w / 2, p.y - h / 2, w, h);
    });

    // Draw Dynamic Target Berth Marker on 2D Map if active
    if (activeAppMode === 3 && plannedPath && plannedPath.length > 0) {
        const lastP = plannedPath[plannedPath.length - 1];
        const bp = rosToCanvas(lastP.x, lastP.y);

        ctx.save();
        ctx.beginPath();
        ctx.setLineDash([3, 3]);
        ctx.arc(bp.x, bp.y, 14, 0, 2 * Math.PI);
        ctx.strokeStyle = '#00ffcc';
        ctx.lineWidth = 2.5;
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.font = 'bold 11px sans-serif';
        ctx.fillStyle = '#00ffcc';
        ctx.textAlign = 'center';
        ctx.fillText('⚓ Selected Berth', bp.x, bp.y - 16);
        ctx.restore();
    }
        // Hover Preview: highlight whatever pier/jetty face is under the
    // cursor (Mode 3 only), color-coded by status — not just "is this
    // dockable," but WHY not, so scanning the packed jetty by eye (or
    // sweeping the mouse along it) tells occupied from open without
    // clicking each slot and reading an alert(). hoveredBerth is
    // {berth, status} from input.js's mousemove listener (status: 'open' |
    // 'occupied' | 'blocked'); guarded with typeof since this file loads
    // before input.js declares it (only matters at call-time, which happens
    // later in the draw() loop, by when it always exists).
    if (activeAppMode === 3 && typeof hoveredBerth !== 'undefined' && hoveredBerth) {
        const hp = rosToCanvas(hoveredBerth.berth.ros_x, hoveredBerth.berth.ros_y);
        const statusStyle = {
            open: { color: '#00ffcc', fill: 'rgba(0, 255, 204, 0.15)', label: '✅ Open — click to dock' },
            occupied: { color: '#ff4d4d', fill: 'rgba(255, 77, 77, 0.15)', label: '⛔ Occupied' },
            blocked: { color: '#ff9800', fill: 'rgba(255, 152, 0, 0.15)', label: '🚫 Blocked' }
        }[hoveredBerth.status];

        ctx.save();
        ctx.beginPath();
        ctx.arc(hp.x, hp.y, 10, 0, 2 * Math.PI);
        ctx.strokeStyle = statusStyle.color;
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.fillStyle = statusStyle.fill;
        ctx.fill();

        ctx.font = 'bold 10px sans-serif';
        ctx.fillStyle = statusStyle.color;
        ctx.textAlign = 'center';
        ctx.fillText(statusStyle.label, hp.x, hp.y - 14);
        ctx.restore();
    }

    // Label Text
    ctx.font = 'bold 10px sans-serif';
    ctx.fillStyle = '#00ffcc';
    const marinaBase = rosToCanvas(-195 * WORLD_SCALE, -175 * WORLD_SCALE);
    ctx.fillText('⚡ 30+ PACKED MARINA GRID (<40cm TOLERANCE)', marinaBase.x - 65, marinaBase.y + 12);
    ctx.restore();
}

// City Building Blocks (Strictly on Upper-Right Sandy Coast)
// Pre-existing oddity, kept as-is (behavior-preserving move, not a fix):
// this ctx.restore() has no matching ctx.save() in this function — the
// save/restore pair from renderMarina2D() above already closed. Harmless on
// Canvas2D (pops back to the default state) but dead/confusing.
function renderCoastalCity2D() {
    const cityCanvas = rosToCanvas(230, 230);
    ctx.fillStyle = '#37474f';
    ctx.fillRect(cityCanvas.x - 28, cityCanvas.y - 14, 62, 28);
    ctx.fillStyle = '#eceff1';
    ctx.font = 'bold 10px sans-serif';
    ctx.fillText('🏙️ Coastal City', cityCanvas.x - 24, cityCanvas.y + 3);
    ctx.restore();
}

// Draw 2D Core Island Land Mass (Obstacle at x: 20m, y: 15m, radius: 12m)
function renderIsland2D(mapScale) {
    const islandCanvas = rosToCanvas(ISLAND_X, ISLAND_Y);
    ctx.save();

    // Sand Border
    ctx.beginPath();
    ctx.arc(islandCanvas.x, islandCanvas.y, (ISLAND_RADIUS + 1.5) * mapScale, 0, 2 * Math.PI);
    ctx.fillStyle = '#d2b48c';
    ctx.fill();
    // Grass Island Top
    ctx.beginPath();
    ctx.arc(islandCanvas.x, islandCanvas.y, ISLAND_RADIUS * mapScale, 0, 2 * Math.PI);
    ctx.fillStyle = '#2e7d32';
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#1b5e20';
    ctx.stroke();
    // Island Label & Trees
    ctx.font = '14px sans-serif';
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';
    ctx.fillText('🌲 Island 🌲', islandCanvas.x, islandCanvas.y + 5);
    ctx.restore();
}

// Draw Entities & Imaginary Dotted Safety Keep-Out Circles
function renderEntities2D(mapScale) {
    entities.forEach(entity => {
        // The ~30 baked-in moored boats get their own compact rectangle +
        // occupied/open hover status further down in renderMarina2D()
        // (called earlier in the draw() order) — that's their whole
        // representation. They're also type 'static' (same as a
        // player-placed buoy), so without this early return they'd ALSO
        // fall into the generic buoy branches below and stack a keep-out
        // ring + dot per boat on top of the rectangle, which at the packed
        // marina's <40cm slot spacing turned into one cluttered blob per
        // jetty — the actual thing making it hard to tell "does this slot
        // already have a boat" at a glance.
        if (entity.isParkedShip) return;

        const p = rosToCanvas(entity.ros_x, entity.ros_y);

        // Draw Visual Restrictive Imaginary Dotted Safety Circle
        if (entity.type === 'static' || entity.type === 'dynamic') {
            ctx.save();
            ctx.beginPath();
            ctx.setLineDash([4, 4]);
            ctx.arc(p.x, p.y, SAFETY_RADIUS * mapScale, 0, 2 * Math.PI);
            ctx.strokeStyle = entity.type === 'dynamic' ? 'rgba(23, 162, 184, 0.7)' : 'rgba(255, 193, 7, 0.7)';
            ctx.lineWidth = 1.5;
            ctx.stroke();
            ctx.fillStyle = entity.type === 'dynamic' ? 'rgba(23, 162, 184, 0.1)' : 'rgba(255, 193, 7, 0.1)';
            ctx.fill();
            ctx.restore();
        }

        ctx.beginPath();
        if (entity.type === 'static') {
            ctx.arc(p.x, p.y, 8, 0, 2 * Math.PI);
            ctx.fillStyle = '#ffc107'; // Yellow buoy
        } else if (entity.type === 'dynamic') {
            ctx.save();
            ctx.translate(p.x, p.y);
            ctx.rotate(-entity.heading);
            ctx.rect(-10, -6, 20, 12);
            ctx.fillStyle = '#17a2b8'; // Blue boat
            ctx.fill();
            ctx.lineWidth = 2;
            ctx.strokeStyle = '#ffffff';
            ctx.stroke();
            ctx.restore();
            return;
        } else if (entity.type === 'goal') {
            ctx.arc(p.x, p.y, 12, 0, 2 * Math.PI);
            ctx.fillStyle = '#28a745'; // Green goal
        }
        ctx.fill();
        ctx.lineWidth = 2;
        ctx.strokeStyle = '#ffffff';
        ctx.stroke();
    });
}

// Mode 1's gamepad cursor (input.js's thrusterLoop) — a crosshair at
// gamepadCursor's ROS position, tinted by whichever tool is currently
// selected (matches the yellow buoy/blue boat/green goal colors
// renderEntities2D uses for the real thing) so it doubles as a preview of
// what pressing Square/Triangle/Circle will place there. Guarded with
// typeof since this file loads before state.js declares gamepadCursor only
// matters at call-time (main.js's draw loop), by which point it always
// exists.
function renderGamepadCursor2D() {
    if (typeof gamepadCursor === 'undefined' || !gamepadCursor.active) return;
    const p = rosToCanvas(gamepadCursor.x, gamepadCursor.y);
    const color = currentMode === 'dynamic' ? '#17a2b8' : currentMode === 'goal' ? '#28a745' : '#ffc107';

    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(p.x - 12, p.y);
    ctx.lineTo(p.x - 4, p.y);
    ctx.moveTo(p.x + 4, p.y);
    ctx.lineTo(p.x + 12, p.y);
    ctx.moveTo(p.x, p.y - 12);
    ctx.lineTo(p.x, p.y - 4);
    ctx.moveTo(p.x, p.y + 4);
    ctx.lineTo(p.x, p.y + 12);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(p.x, p.y, 3, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.restore();
}

// Draw ASV Boat (2D Map)
function renderBoatIcon2D() {
    const boatCanvas = rosToCanvas(boatPos.x, boatPos.y);
    ctx.save();
    ctx.translate(boatCanvas.x, boatCanvas.y);
    ctx.rotate(-boatPos.yaw);

    ctx.beginPath();
    ctx.moveTo(14, 0);
    ctx.lineTo(-10, -7);
    ctx.lineTo(-10, 7);
    ctx.closePath();
    ctx.fillStyle = '#dc3545';
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();
}
