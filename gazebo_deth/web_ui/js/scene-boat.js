// ================= BOAT 3D MODEL =================
// Depends on scene-environment.js (`scene`). `boatGroup`/`radarMesh` are
// read every frame by main.js's boat/camera transform and by
// navigation.js's roll-bank effect.

// 4. High-Detail ASV Vessel Model (Main Boat) — twin-hull catamaran styled
// after the real WAM-V: black cylindrical pontoons with an orange deck/trim
// (colors taken from the actual WAM-V texture), linked by an open frame
// rather than a solid slab, matching its real look instead of a plain box hull.
const boatGroup = new THREE.Group();
const HULL_Z_OFFSET = 0.55; // distance from centerline to each hull's centerline
const HULL_RADIUS = 0.28;

const hullGeo = new THREE.CylinderGeometry(HULL_RADIUS, HULL_RADIUS, 3.0, 16);
const hullMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.4, metalness: 0.3 });
const bowGeo = new THREE.ConeGeometry(HULL_RADIUS, 1.0, 16);
const accentMat = new THREE.MeshStandardMaterial({ color: 0xff7a1a, roughness: 0.4 });
const engineGeo = new THREE.BoxGeometry(0.5, 0.8, 0.35);
const engineMat = new THREE.MeshStandardMaterial({ color: 0x111111, metalness: 0.8 });

// Port + Starboard Pontoon Hulls (each with a tapered bow and stern engine)
[HULL_Z_OFFSET, -HULL_Z_OFFSET].forEach(z => {
    const hullMesh = new THREE.Mesh(hullGeo, hullMat);
    hullMesh.rotation.z = Math.PI / 2;
    hullMesh.position.set(-0.1, 0.2, z);
    hullMesh.castShadow = true;
    boatGroup.add(hullMesh);

    const bowMesh = new THREE.Mesh(bowGeo, hullMat);
    bowMesh.rotation.z = -Math.PI / 2;
    bowMesh.position.set(1.9, 0.2, z);
    boatGroup.add(bowMesh);

    const engMesh = new THREE.Mesh(engineGeo, engineMat);
    engMesh.position.set(-1.8, 0.2, z);
    boatGroup.add(engMesh);
});

// Open Frame Bridging the Two Hulls (crossbeams, not a solid deck)
const beamGeo = new THREE.CylinderGeometry(0.05, 0.05, HULL_Z_OFFSET * 2 + 0.2, 8);
[1.0, -1.0].forEach(x => {
    const beamMesh = new THREE.Mesh(beamGeo, accentMat);
    beamMesh.rotation.x = Math.PI / 2;
    beamMesh.position.set(x, 0.55, 0);
    boatGroup.add(beamMesh);
});

// Flat Equipment Deck on top of the frame
const deckGeo = new THREE.BoxGeometry(1.85, 0.08, HULL_Z_OFFSET * 2 - 0.1);
const deckMesh = new THREE.Mesh(deckGeo, accentMat);
deckMesh.position.set(0, 0.62, 0);
boatGroup.add(deckMesh);

// Electronics/Sensor Pod
const podGeo = new THREE.BoxGeometry(1.0, 0.5, 0.9);
const podMat = new THREE.MeshStandardMaterial({ color: 0xdddddd, roughness: 0.3 });
const podMesh = new THREE.Mesh(podGeo, podMat);
podMesh.position.set(-0.2, 0.95, 0);
boatGroup.add(podMesh);

// Navigation Arch & Lidar Mast
const archGeo = new THREE.CylinderGeometry(0.04, 0.04, 1.2);
const archMat = new THREE.MeshStandardMaterial({ color: 0x888888, metalness: 0.9 });
const mast1 = new THREE.Mesh(archGeo, archMat); mast1.position.set(-0.8, 1.4, 0.5);
const mast2 = new THREE.Mesh(archGeo, archMat); mast2.position.set(-0.8, 1.4, -0.5);
const topBar = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.1, 1.2), archMat);
topBar.position.set(-0.8, 2.0, 0);
boatGroup.add(mast1, mast2, topBar);

// Rotating Radar Dome
let radarMesh;
const radarGeo = new THREE.CylinderGeometry(0.4, 0.4, 0.15, 16);
const radarMat = new THREE.MeshStandardMaterial({ color: 0xffffff });
radarMesh = new THREE.Mesh(radarGeo, radarMat);
radarMesh.position.set(-0.8, 2.15, 0);
boatGroup.add(radarMesh);

scene.add(boatGroup);
