// ================= OBJECT DETECTION OVERLAY (GROUND TRUTH) =================
// Draws labeled boxes for buoys/boats onto detectionOverlayCanvas, on top of
// the copied FPV frame (see the drawImage call in main.js's draw()). There's
// no image analysis here: every entity's exact 3D position and type is
// already known (it's the same data driving the map/3D views), so this just
// projects each one's real bounding box into 2D screen space via the shared
// FPV camera. It looks like live detection because the underlying data is
// exactly what a perfect detector would recover — it just skips the
// (unreliable, given these are flat, untextured sim primitives) step of
// actually inferring that from pixels.
// Depends on scene-environment.js (`camera`, `detectionOverlayCanvas/Ctx`),
// state.js (`entities`, `staticDetections`) and entities-3d.js
// (`threeEntities`); must load after all three.

function colorNameFromHex(hex) {
    const r = ((hex >> 16) & 255) / 255;
    const g = ((hex >> 8) & 255) / 255;
    const b = (hex & 255) / 255;
    const max = Math.max(r, g, b), min = Math.min(r, g, b);
    const l = (max + min) / 2;
    const d = max - min;
    if (d < 0.08) return l > 0.6 ? 'White' : (l < 0.25 ? 'Black' : 'Gray');
    let h;
    if (max === r) h = ((g - b) / d) % 6;
    else if (max === g) h = (b - r) / d + 2;
    else h = (r - g) / d + 4;
    h *= 60;
    if (h < 0) h += 360;
    if (h < 50 && l < 0.4) return 'Brown'; // dark red/orange (e.g. wood) reads as brown, not red/orange
    if (h < 15 || h >= 345) return 'Red';
    if (h < 45) return 'Orange';
    if (h < 65) return 'Yellow';
    if (h < 170) return 'Green';
    if (h < 255) return l < 0.3 ? 'Navy' : (h < 200 ? 'Teal' : 'Blue');
    if (h < 290) return 'Purple';
    return 'Pink';
}

const DETECTION_TYPE_INFO = {
    static: { label: 'Buoy', hex: 0xff8c00 },
    dynamic: { label: 'Boat', hex: 0x0f4c5c },
};
const MAX_DETECTION_DISTANCE = 90; // meters; farther objects don't get a box

const detectionBox3 = new THREE.Box3();
const detectionCenter = new THREE.Vector3();
const detectionToObject = new THREE.Vector3();
const detectionForward = new THREE.Vector3();

function projectToOverlay(v) {
    v.project(camera);
    return {
        x: (v.x + 1) / 2 * detectionOverlayCanvas.width,
        y: (1 - v.y) / 2 * detectionOverlayCanvas.height,
    };
}

function drawDetectionBox(coreMeshes, labelText) {
    const ctx = detectionOverlayCtx;

    detectionBox3.makeEmpty();
    coreMeshes.forEach(m => detectionBox3.expandByObject(m));
    if (detectionBox3.isEmpty()) return;

    detectionBox3.getCenter(detectionCenter);
    detectionToObject.copy(detectionCenter).sub(camera.position);
    if (detectionToObject.dot(detectionForward) <= 0) return; // behind the camera
    if (detectionToObject.length() > MAX_DETECTION_DISTANCE) return; // too far to plausibly detect

    const { min, max } = detectionBox3;
    const corners = [
        [min.x, min.y, min.z], [min.x, min.y, max.z],
        [min.x, max.y, min.z], [min.x, max.y, max.z],
        [max.x, min.y, min.z], [max.x, min.y, max.z],
        [max.x, max.y, min.z], [max.x, max.y, max.z],
    ];

    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    corners.forEach(([x, y, z]) => {
        const p = projectToOverlay(new THREE.Vector3(x, y, z));
        minX = Math.min(minX, p.x); maxX = Math.max(maxX, p.x);
        minY = Math.min(minY, p.y); maxY = Math.max(maxY, p.y);
    });

    minX = Math.max(0, minX); minY = Math.max(0, minY);
    maxX = Math.min(detectionOverlayCanvas.width, maxX);
    maxY = Math.min(detectionOverlayCanvas.height, maxY);
    if (maxX - minX < 4 || maxY - minY < 4) return; // offscreen or degenerate

    ctx.strokeStyle = '#00ffcc';
    ctx.lineWidth = 2;
    ctx.strokeRect(minX, minY, maxX - minX, maxY - minY);

    ctx.font = '13px sans-serif';
    const textWidth = ctx.measureText(labelText).width;
    ctx.fillStyle = '#00ffcc';
    ctx.fillRect(minX, minY - 18, textWidth + 8, 18);
    ctx.fillStyle = '#001a14';
    ctx.fillText(labelText, minX + 4, minY - 5);
}

// Mode 1's gamepad cursor (input.js's thrusterLoop), projected onto the
// boat's own camera view too — the crosshair on the 2D map (render-2d.js's
// renderGamepadCursor2D) only helps a kid looking at THAT panel; someone
// watching the camera feed instead needs the same aim point shown there.
// Placing itself already works from this view regardless (Square/Triangle/
// Circle call placeMode1EntityAt(gamepadCursor.x, gamepadCursor.y)
// unconditionally, same as the mouse's raycast click handler on this same
// canvas resolves TO) — this only adds the missing visual feedback for it.
// Reuses `detectionForward` as already set by drawDetectionOverlay() this
// same frame (see main.js's draw() call order) rather than recomputing it.
function renderGamepadCursorDetection() {
    if (typeof gamepadCursor === 'undefined' || !gamepadCursor.active || activeAppMode !== 1) return;

    const worldPoint = new THREE.Vector3(gamepadCursor.x, 0, -gamepadCursor.y); // y=0 water plane, matches the mouse raycast's own waterPlane
    const toCursor = worldPoint.clone().sub(camera.position);
    if (toCursor.dot(detectionForward) <= 0) return; // behind the camera — nothing to draw

    const p = projectToOverlay(worldPoint);
    if (p.x < -20 || p.x > detectionOverlayCanvas.width + 20 || p.y < -20 || p.y > detectionOverlayCanvas.height + 20) return; // well outside the frame

    const ctx = detectionOverlayCtx;
    // Always yellow here (unlike the 2D map's tool-tinted crosshair,
    // render-2d.js) — this view's sky/water/scenery background makes the
    // other tool colors (teal boat, green goal) harder to pick out at a
    // glance; plain high-contrast yellow reads clearly against all of them,
    // and matches the perspective-selection frame's own gold tint.
    const color = '#ffd700';
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(p.x - 14, p.y); ctx.lineTo(p.x - 5, p.y);
    ctx.moveTo(p.x + 5, p.y); ctx.lineTo(p.x + 14, p.y);
    ctx.moveTo(p.x, p.y - 14); ctx.lineTo(p.x, p.y - 5);
    ctx.moveTo(p.x, p.y + 5); ctx.lineTo(p.x, p.y + 14);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(p.x, p.y, 4, 0, 2 * Math.PI);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.restore();
}

function drawDetectionOverlay() {
    // Note: no clearRect here — the caller already refreshed the whole
    // canvas via drawImage(renderer.domElement, ...) just before this runs.
    camera.getWorldDirection(detectionForward);

    entities.forEach(ent => {
        if (ent.isParkedShip || ent.type === 'goal') return;
        const mesh = threeEntities.get(ent.id);
        const info = DETECTION_TYPE_INFO[ent.type];
        if (!mesh || !info) return;
        const label = `${colorNameFromHex(info.hex)} ${info.label}`;
        drawDetectionBox(mesh.userData.detectionMeshes || [mesh], label);
    });

    // Static marina scenery (moored boats) - not tracked in `entities`
    // since they never move or get placed by the player, but they're real,
    // known objects in the scene just like everything else.
    staticDetections.forEach(target => {
        const label = `${colorNameFromHex(target.hex)} ${target.label}`;
        drawDetectionBox(target.detectionMeshes, label);
    });
}
