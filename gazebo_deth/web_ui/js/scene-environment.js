// ================= THREE.JS 3D FPV SETUP ================= //
// Renderer/camera/lights, sky/sun/clouds/seagulls, lake/beach/land, island.
// Purely declarative scene construction (no branching logic) — depends on
// config.js (LAKE_RADIUS/ISLAND_*) and runs immediately at load, so it must
// load AFTER config.js. `scene`/`camera`/`renderer` here are used by every
// other scene-*.js file, entities-3d.js, and main.js — this file must load
// before all of them.
const container3d = document.getElementById('threejs-container');
const scene = new THREE.Scene();

// 1. Realistic Atmosphere & Sky
scene.background = new THREE.Color(0x7ec0ee); // Lake Sky Blue
scene.fog = new THREE.FogExp2(0x7ec0ee, 0.001);

const camera = new THREE.PerspectiveCamera(68, 500 / 440, 0.1, 3000);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(500, 440);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
container3d.appendChild(renderer.domElement);

// "Object Detection Camera" panel: rather than a second live WebGL renderer
// (which doubles GPU cost and risks its own context issues), each frame just
// copies the already-rendered boat-camera canvas (container3d above, now kept
// off-screen — see style.css) onto this plain 2D canvas, then draws detection
// boxes on top of that copy (see detection-overlay.js and main.js's draw()).
const detectionOverlayCanvas = document.getElementById('detectionOverlayCanvas');
const detectionOverlayCtx = detectionOverlayCanvas.getContext('2d');

// Lighting System
scene.add(new THREE.AmbientLight(0xffffff, 1.1));
const sunLight = new THREE.DirectionalLight(0xfffaed, 1.3);
sunLight.position.set(300, 500, 200);
sunLight.castShadow = true;
sunLight.shadow.camera.left = -400;
sunLight.shadow.camera.right = 400;
sunLight.shadow.camera.top = 400;
sunLight.shadow.camera.bottom = -400;
scene.add(sunLight);

// 1B. Glowing 3D Sun & Sky Corona
const sunGeo = new THREE.SphereGeometry(22, 32, 32);
const sunMat = new THREE.MeshBasicMaterial({ color: 0xfff5cc });
const sunMesh = new THREE.Mesh(sunGeo, sunMat);
sunMesh.position.set(300, 450, -200);
scene.add(sunMesh);

const haloGeo = new THREE.SphereGeometry(35, 32, 32);
const haloMat = new THREE.MeshBasicMaterial({ color: 0xffea9f, transparent: true, opacity: 0.35 });
const haloMesh = new THREE.Mesh(haloGeo, haloMat);
haloMesh.position.set(300, 450, -200);
scene.add(haloMesh);

// 1C. Volumetric Fluffy Clouds
function createCloud(x, y, z, scale = 1.0) {
    const cloudGroup = new THREE.Group();
    const cloudMat = new THREE.MeshLambertMaterial({ color: 0xffffff, transparent: true, opacity: 0.85 });

    const puffPositions = [
        { x: 0, y: 0, z: 0, r: 12 },
        { x: 10, y: 3, z: 2, r: 10 },
        { x: -10, y: 2, z: -2, r: 11 },
        { x: 18, y: -2, z: 1, r: 8 },
        { x: -16, y: -1, z: 3, r: 9 },
        { x: 4, y: 6, z: -1, r: 9 }
    ];

    puffPositions.forEach(p => {
        const puff = new THREE.Mesh(new THREE.SphereGeometry(p.r * scale, 16, 16), cloudMat);
        puff.position.set(p.x * scale, p.y * scale, p.z * scale);
        cloudGroup.add(puff);
    });

    cloudGroup.position.set(x, y, z);
    return cloudGroup;
}

const clouds = [];
const cloudCoords = [
    { x: -200, y: 140, z: -300, s: 1.5 },
    { x: 100, y: 160, z: -400, s: 1.8 },
    { x: 300, y: 150, z: -150, s: 1.4 },
    { x: -350, y: 170, z: 100, s: 1.6 },
    { x: 50, y: 180, z: 250, s: 1.7 }
];
cloudCoords.forEach(c => {
    const cloud = createCloud(c.x, c.y, c.z, c.s);
    scene.add(cloud);
    clouds.push(cloud);
});

// 1C-2. Low-Lying Coastal Horizon Clouds (Hugging distant shoreline hills & mountains)
function createHorizonCloud(x, y, z, scale = 1.0) {
    const horizonGroup = new THREE.Group();
    const mistMat = new THREE.MeshLambertMaterial({
        color: 0xf0f4f8,
        transparent: true,
        opacity: 0.65
    });

    // Elongated horizontal stratus cloud bank
    const stratusPuffs = [
        { x: 0, y: 0, z: 0, rx: 35, ry: 10, rz: 15 },
        { x: 25, y: 3, z: 5, rx: 28, ry: 9, rz: 12 },
        { x: -25, y: -2, z: -4, rx: 30, ry: 8, rz: 14 },
        { x: 50, y: 1, z: 2, rx: 22, ry: 7, rz: 10 },
        { x: -50, y: 2, z: -3, rx: 24, ry: 8, rz: 11 }
    ];

    stratusPuffs.forEach(p => {
        const pGeo = new THREE.SphereGeometry(1, 16, 16);
        pGeo.scale(p.rx * scale, p.ry * scale, p.rz * scale);
        const puff = new THREE.Mesh(pGeo, mistMat);
        puff.position.set(p.x * scale, p.y * scale, p.z * scale);
        horizonGroup.add(puff);
    });

    horizonGroup.position.set(x, y, z);
    return horizonGroup;
}

// Generate 360-degree ring of low horizon clouds clinging to outer land mass (r ~ 450-600m, height 25-45m)
for (let angle = 0; angle < Math.PI * 2; angle += 0.45) {
    const dist = 450 + (Math.sin(angle * 4) * 50) + (Math.random() * 60);
    const hx = Math.cos(angle) * dist;
    const hz = Math.sin(angle) * dist;
    const hy = 25 + Math.random() * 20; // Low right above land horizon!

    const hCloud = createHorizonCloud(hx, hy, hz, 1.2 + Math.random() * 0.8);
    scene.add(hCloud);
    clouds.push(hCloud);
}

// 1D. Flying Seagull Flock with Wing-Flap Animation
function createSeagull() {
    const birdGroup = new THREE.Group();
    const wingMat = new THREE.MeshStandardMaterial({ color: 0x222222, side: THREE.DoubleSide });

    const leftWing = new THREE.Mesh(new THREE.PlaneGeometry(1.4, 0.5), wingMat);
    leftWing.position.set(-0.7, 0, 0);
    leftWing.rotation.y = -0.2;
    leftWing.name = 'leftWing';

    const rightWing = new THREE.Mesh(new THREE.PlaneGeometry(1.4, 0.5), wingMat);
    rightWing.position.set(0.7, 0, 0);
    rightWing.rotation.y = 0.2;
    rightWing.name = 'rightWing';

    birdGroup.add(leftWing, rightWing);
    return birdGroup;
}

const flock = [];
for (let b = 0; b < 9; b++) {
    const bird = createSeagull();
    bird.userData = {
        radius: 80 + Math.random() * 90,
        height: 50 + Math.random() * 35,
        speed: 0.008 + Math.random() * 0.005,
        angle: Math.random() * Math.PI * 2,
        phase: Math.random() * Math.PI * 2
    };
    scene.add(bird);
    flock.push(bird);
}

// 2. Circular Lake Water Body. LAKE_RADIUS is defined in config.js (see its
// comment there for the 350-vs-300 pre-scale reference derivation).
const waterGeo = new THREE.RingGeometry(0.001, LAKE_RADIUS, 128, 32); // High-density mesh for fluid wave motion

const waterMat = new THREE.MeshPhongMaterial({
    color: 0x003d73,       // Deep ocean blue with rich base tone
    emissive: 0x001020,    // Deep water shadow
    specular: 0x88d4ff,    // Bright ocean water specular reflection
    shininess: 80,         // High gloss for wave facet highlights
    transparent: true,
    opacity: 0.92,
    flatShading: true,     // Explicitly render wave facets so wave shapes are crisp & visible!
    side: THREE.DoubleSide
});
const waterMesh = new THREE.Mesh(waterGeo, waterMat);
waterMesh.rotation.x = -Math.PI / 2;
waterMesh.receiveShadow = true;
scene.add(waterMesh);

// 3. Surrounding Shoreline Land Mass & Forest (Trees strictly on land!)
// Outer Sandy Beach Ring, tied directly to LAKE_RADIUS (previously a fixed
// 295-330 left over from before WORLD_SCALE existed — that stranded the
// beach/land ring far outside the actual water body, leaving a bare gap
// between the lake edge and the shore with nothing rendered in it).
const beachGeo = new THREE.RingGeometry(LAKE_RADIUS - 5, LAKE_RADIUS + 15, 64);
const beachMat = new THREE.MeshLambertMaterial({ color: 0xd2b48c, side: THREE.DoubleSide });
const beachMesh = new THREE.Mesh(beachGeo, beachMat);
beachMesh.rotation.x = -Math.PI / 2;
beachMesh.position.y = -0.05;
scene.add(beachMesh);

// Surrounding Green Forest Hills Land Mass — starts right where the beach
// ends, also tied to LAKE_RADIUS for the same reason.
const landGeo = new THREE.RingGeometry(LAKE_RADIUS + 13, LAKE_RADIUS * 4, 64);
const landMat = new THREE.MeshLambertMaterial({ color: 0x3e5c26, side: THREE.DoubleSide });
const landMesh = new THREE.Mesh(landGeo, landMat);
landMesh.rotation.x = -Math.PI / 2;
landMesh.position.y = -0.1;
scene.add(landMesh);

// Helper function to build 3D Pine Trees
function createTree(x, z, scale = 1.0) {
    const treeGroup = new THREE.Group();
    // Trunk
    const trunkGeo = new THREE.CylinderGeometry(0.3 * scale, 0.5 * scale, 3.5 * scale, 8);
    const trunkMat = new THREE.MeshLambertMaterial({ color: 0x5c4033 });
    const trunkMesh = new THREE.Mesh(trunkGeo, trunkMat);
    trunkMesh.position.y = (3.5 * scale) / 2;
    treeGroup.add(trunkMesh);

    // Pine Foliage Cones
    const foliageMat = new THREE.MeshLambertMaterial({ color: 0x1e4d2b });
    const c1 = new THREE.Mesh(new THREE.ConeGeometry(2.2 * scale, 4 * scale, 8), foliageMat);
    c1.position.y = 3.5 * scale;
    const c2 = new THREE.Mesh(new THREE.ConeGeometry(1.7 * scale, 3 * scale, 8), foliageMat);
    c2.position.y = 5.0 * scale;
    treeGroup.add(c1, c2);

    treeGroup.position.set(x, 0, z);
    return treeGroup;
}

// Generate Trees STRICTLY on outer land terrain, starting right at the land
// ring's inner edge (tied to LAKE_RADIUS, same reasoning as beachGeo/landGeo
// above — a fixed radius here would leave trees stranded far from the
// now-smaller lake).
for (let angle = 0; angle < Math.PI * 2; angle += 0.05) {
    const r = (LAKE_RADIUS + 15) + Math.sin(angle * 6) * 9 + Math.random() * 37;
    const tx = Math.cos(angle) * r;
    const tz = Math.sin(angle) * r;
    scene.add(createTree(tx, tz, 1.2 + Math.random() * 1.0));
}

// Core Land Obstacle: Scenic Central Island in Ocean Bay. Placed dead center
// of the lake (was offset toward the marina corner) per explicit request
// that the island sit in the middle rather than off to one side.
const islandGroup = new THREE.Group();
const islandTerrainGeo = new THREE.CylinderGeometry(22, 26, 2, 32);
const islandTerrainMat = new THREE.MeshLambertMaterial({ color: 0x3a6024 }); // Grass
const islandTerrain = new THREE.Mesh(islandTerrainGeo, islandTerrainMat);
islandTerrain.position.y = 0.4;
islandGroup.add(islandTerrain);

// Island Sand Border
const islandSandGeo = new THREE.CylinderGeometry(26.5, 30, 1.2, 32);
const islandSandMat = new THREE.MeshLambertMaterial({ color: 0xc2b280 });
const islandSand = new THREE.Mesh(islandSandGeo, islandSandMat);
islandSand.position.y = 0.0;
islandGroup.add(islandSand);

islandGroup.add(createTree(4, 5, 1.2));
islandGroup.add(createTree(-6, -4, 1.1));
islandGroup.add(createTree(2, -8, 1.3));
islandGroup.position.set(ISLAND_X, 0, -ISLAND_Y);
scene.add(islandGroup);
