// ================= MARINA 3D SCENE (pier, jetties, moored fleet, office, city) =================
// Depends on config.js (MARINA_PIER_*/JETTY_*/pierLength/jettyLength/etc) and
// scene-environment.js (`scene`). createMooredBoat() also registers each
// moored boat's collision entity into state.js's `entities` array.

// 3B. Ultra-Packed Real Marina Grid (30+ Vessels Packed Side-by-Side with 40cm Ultra-Small Tolerances)
const marinaGroup = new THREE.Group();

// Main Horizontal Floating Spine Pier — size AND position both derived from
// the canonical MARINA_PIER_* constants above, so this mesh can no longer
// drift out of sync with the collision/pathfinding geometry the way the old
// hardcoded BoxGeometry(130, 0.8, 6) did (that 130/6 were never scaled by
// WORLD_SCALE while the position was, rendering a pier far bigger than what
// actually blocked the boat).
const pierMat = new THREE.MeshLambertMaterial({ color: 0x5d4037, roughness: 0.8 });
// pierLength/pierCenterX/pierCenterY are defined in config.js (shared with
// render-2d.js's 2D marina draw, so the two can never disagree).
const mainPier = new THREE.Mesh(new THREE.BoxGeometry(pierLength, 0.8, MARINA_PIER_THICKNESS), pierMat);
mainPier.position.set(pierCenterX, 0.4, -pierCenterY); // Three.js z = -ROS y
marinaGroup.add(mainPier);

// 3 Vertical Finger Jetties extending perpendicularly up into water — same
// derive-from-constants fix as the pier above (was BoxGeometry(4, 0.8, 62),
// a real 62m-long jetty when the collision system only protected 24m).
// jettyWidth/jettyLength/jettyCenterY are defined in config.js (same
// shared-with-render-2d.js reasoning as the pier ones above).
JETTY_X_LIST.forEach(jx => {
    const jetty = new THREE.Mesh(new THREE.BoxGeometry(jettyWidth, 0.8, jettyLength), pierMat);
    jetty.position.set(jx, 0.4, -jettyCenterY);
    marinaGroup.add(jetty);

    // Mooring Pylons along each finger jetty, spanning its corrected length
    for (let pz = -JETTY_Y_NEAR; pz <= -JETTY_Y_FAR; pz += 6.5 * WORLD_SCALE) {
        const pylonL = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.3, 4.5, 8), new THREE.MeshLambertMaterial({ color: 0x3e2723 }));
        pylonL.position.set(jx - 1.8, 0.9, pz);
        const pylonR = pylonL.clone();
        pylonR.position.x = jx + 1.8;
        marinaGroup.add(pylonL, pylonR);
    }
});

// Helper to spawn realistic moored yachts, sailboats, & speedboats along berths
function createMooredBoat(x, z, hullColor, boatType = 'yacht') {
    const bGroup = new THREE.Group();
    const isSailboat = boatType === 'sailboat';
    const isSmall = boatType === 'speedboat';

    const bLen = isSmall ? 3.0 : 3.6;
    const bWid = isSmall ? 1.25 : 1.45;

    const hull = new THREE.Mesh(new THREE.BoxGeometry(bLen, 0.65, bWid), new THREE.MeshStandardMaterial({ color: hullColor, roughness: 0.2 }));
    hull.position.y = 0.2;
    bGroup.add(hull);

    // Detection box hugs the hull (+ cabin, when present); the tall thin
    // sailboat mast is excluded so it doesn't stretch the box unrealistically.
    const detectionMeshes = [hull];

    if (isSailboat) {
        // Tall Sailboat Mast
        const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.07, 7.5, 8), new THREE.MeshStandardMaterial({ color: 0xdddddd }));
        mast.position.set(0, 3.8, 0);
        bGroup.add(mast);
    } else {
        // Cabin Superstructure
        const cabin = new THREE.Mesh(new THREE.BoxGeometry(1.5, 0.7, 1.05), new THREE.MeshStandardMaterial({ color: 0xffffff }));
        cabin.position.set(-0.2, 0.7, 0);
        bGroup.add(cabin);
        detectionMeshes.push(cabin);
    }

    bGroup.position.set(x, 0.2, z);
    bGroup.rotation.y = 0; // Parked horizontally into berth
    staticDetections.push({ detectionMeshes, label: 'Boat', hex: hullColor });

    // Register physical collision obstacle (isParkedShip avoids yellow buoy rings)
    entities.push({ id: 'moored_' + x + '_' + z, type: 'static', isParkedShip: true, ros_x: x, ros_y: -z });
    return bGroup;
}

// Populate Jetty 1 (Left Jetty at x: -245.0) - Densely Packed Side-by-Side (z: 168 to 120)
const zList = [168, 161.5, 155, 148.5, 142, 135.5, 129, 122.5].map(z => z * WORLD_SCALE);
const colorsList = [0x1d3557, 0x2a9d8f, 0xe63946, 0x457b9d, 0x0f4c5c, 0x3d5a80, 0x9b5de5, 0xf15bb5];
const typesList = ['yacht', 'sailboat', 'speedboat', 'yacht', 'sailboat', 'speedboat', 'yacht', 'sailboat'];

// Populate Jetty 1 (Left Jetty at x: -245.0) - Open Berth #1 at z: 142.0.
// Thinned to every other slot (idx % 2 === 0), on top of the existing named
// berth gap, per explicit request for more open choices — the marina used
// to leave only the 4 named availableBerths open with every other slot
// packed solid.
zList.forEach((z, idx) => {
    if (idx % 2 !== 0) return;
    if (Math.abs(z - 142.0 * WORLD_SCALE) > 3.0) {
        marinaGroup.add(createMooredBoat(-251 * WORLD_SCALE, z, colorsList[idx % colorsList.length], typesList[idx % typesList.length]));
    }
    marinaGroup.add(createMooredBoat(-239 * WORLD_SCALE, z, colorsList[(idx + 2) % colorsList.length], typesList[(idx + 1) % typesList.length]));
});

// Populate Jetty 2 (Middle Jetty at x: -195.0) - Open Berth #2 at z: 142.2 (Tight 40cm Gap) and Open Berth #3 at z: 142.2 (Right Side)
// Thinned to every other hand-placed slot, same reasoning as Jetty 1 above.
marinaGroup.add(createMooredBoat(-201 * WORLD_SCALE, 168 * WORLD_SCALE, 0xf8f9fa, 'yacht'));
marinaGroup.add(createMooredBoat(-201 * WORLD_SCALE, 155 * WORLD_SCALE, 0x0f4c5c, 'speedboat'));

// ===> OPEN BERTH #2: X: -201.0, Y: -142.2 (z = 142.2) <=== (ROS coords, pre-WORLD_SCALE)

marinaGroup.add(createMooredBoat(-201 * WORLD_SCALE, 135.9 * WORLD_SCALE, 0x1d3557, 'yacht')); // BOTTOM BOUNDARY OF BERTH #2
marinaGroup.add(createMooredBoat(-201 * WORLD_SCALE, 122.9 * WORLD_SCALE, 0x457b9d, 'speedboat'));

// Right side berths of Jetty 2 - Open Berth #3 at z: 142.2 — thinned to every other slot
zList.forEach((z, idx) => {
    if (idx % 2 !== 0) return;
    if (Math.abs(z - 142.2 * WORLD_SCALE) > 3.0) {
        marinaGroup.add(createMooredBoat(-189 * WORLD_SCALE, z, colorsList[(idx + 3) % colorsList.length], typesList[idx % typesList.length]));
    }
});

// Populate Jetty 3 (Right Jetty at x: -145.0) - Open Berth #4 at z: 142.0 — thinned to every other slot
zList.forEach((z, idx) => {
    if (idx % 2 !== 0) return;
    if (Math.abs(z - 142.0 * WORLD_SCALE) > 3.0) {
        marinaGroup.add(createMooredBoat(-139 * WORLD_SCALE, z, colorsList[(idx + 1) % colorsList.length], typesList[(idx + 2) % typesList.length]));
    }
});

// Dynamic 3D Target Beacon Light for clicked berth
const bGeo = new THREE.TorusGeometry(1.6, 0.2, 16, 100);
const bMat = new THREE.MeshBasicMaterial({ color: 0x00ffcc, transparent: true, opacity: 0.8 });
const dynamic3DBerthMesh = new THREE.Mesh(bGeo, bMat);
dynamic3DBerthMesh.rotation.x = Math.PI / 2;
dynamic3DBerthMesh.position.set(-220, -10, 0); // Hide offscreen initially
const bLight = new THREE.PointLight(0x00ffcc, 2, 20);
bLight.position.set(0, 2, 0);
dynamic3DBerthMesh.add(bLight);
marinaGroup.add(dynamic3DBerthMesh);

// Harbor Master Control Office & Lighthouse Beacon on Spine Pier
const officeMesh = new THREE.Mesh(
    new THREE.BoxGeometry(10, 6, 8),
    new THREE.MeshStandardMaterial({ color: 0xf1faee, roughness: 0.3 })
);
officeMesh.position.set(-260 * WORLD_SCALE, 3.8, 175 * WORLD_SCALE);

const officeRoof = new THREE.Mesh(
    new THREE.ConeGeometry(7, 3, 4),
    new THREE.MeshStandardMaterial({ color: 0xe63946 })
);
officeRoof.rotation.y = Math.PI / 4;
officeRoof.position.set(-260 * WORLD_SCALE, 8.3, 175 * WORLD_SCALE);

const beaconLight = new THREE.PointLight(0x00ffff, 2.5, 30);
beaconLight.position.set(-260 * WORLD_SCALE, 9.0, 175 * WORLD_SCALE);
marinaGroup.add(officeMesh, officeRoof, beaconLight);

// Coastal Town Villas & City Buildings, nestled just past the beach on the
// NE shore (r ~= 165-200m from origin). Previously fixed at r ~= 305-330m —
// a leftover from before WORLD_SCALE existed, left behind when the lake
// shrank so the "coastal" city ended up nowhere near the actual coast.
// Repositioned (not just rescaled) to sit right at the new, smaller
// shoreline instead, on the opposite side of the lake from the marina.
const bldgColors = [0xfaf0e6, 0xdfc09f, 0xe8d8c8, 0xd7ccc8, 0xc05a46, 0xefebe9];
const bldgPositions = [
    { x: 120, z: -115, w: 16, h: 20, d: 14, colorIdx: 0 },
    { x: 140, z: -123, w: 20, h: 26, d: 18, colorIdx: 1 },
    { x: 112, z: -133, w: 15, h: 18, d: 14, colorIdx: 2 },
    { x: 150, z: -111, w: 18, h: 22, d: 16, colorIdx: 3 },
    { x: 124, z: -143, w: 16, h: 20, d: 14, colorIdx: 4 },
    { x: 160, z: -119, w: 22, h: 28, d: 20, colorIdx: 5 },
    { x: 142, z: -139, w: 18, h: 24, d: 16, colorIdx: 0 }
];

bldgPositions.forEach(b => {
    // Villa Wall
    const bldgMat = new THREE.MeshStandardMaterial({ color: bldgColors[b.colorIdx], roughness: 0.5 });
    const bldgMesh = new THREE.Mesh(new THREE.BoxGeometry(b.w, b.h, b.d), bldgMat);
    bldgMesh.position.set(b.x, b.h / 2 - 0.1, b.z);
    bldgMesh.castShadow = true;
    marinaGroup.add(bldgMesh);

    // Terracotta Pitched Roof
    const roofMat = new THREE.MeshStandardMaterial({ color: 0x8d3c1b, roughness: 0.4 }); // Terracotta tile
    const roofGeo = new THREE.ConeGeometry(Math.max(b.w, b.d) * 0.75, 3.5, 4);
    const roofMesh = new THREE.Mesh(roofGeo, roofMat);
    roofMesh.rotation.y = Math.PI / 4;
    roofMesh.position.set(b.x, b.h + 1.7, b.z);
    marinaGroup.add(roofMesh);
});

scene.add(marinaGroup);
