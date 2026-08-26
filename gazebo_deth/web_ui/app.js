// ================= MODE 2 (JOYSTICK DRIVE) TUNABLES ================= //
const MAX_LINEAR_FWD = 6.0;      // top forward speed (m/s)
const MAX_LINEAR_REV = -1.5;     // top reverse speed (m/s)
const MAX_ANGULAR = 1.2;         // top turn rate (rad/s)
const THRUST_RAMP_RATE = 6.0;    // units/sec: how fast velocity reaches its target while a thruster is held open
const WATER_FRICTION_RATE = 1.0; // units/sec: how fast velocity decays toward zero once released

// Connect to ROS via roslibjs
const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

const statusEl = document.getElementById('status');

ros.on('connection', function () {
    statusEl.textContent = 'Connected to ROS';
    statusEl.className = 'connected';
});

ros.on('error', function () {
    statusEl.textContent = 'Connection Error';
    statusEl.className = 'disconnected';
});

ros.on('close', function () {
    statusEl.textContent = 'Disconnected';
    statusEl.className = 'disconnected';
});

// ROS Topics
const spawnTopic = new ROSLIB.Topic({ ros: ros, name: '/exhibition/spawn_obstacle', messageType: 'std_msgs/String' });
const cmdVelTopic = new ROSLIB.Topic({ ros: ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' });
const odomTopic = new ROSLIB.Topic({ ros: ros, name: '/odom', messageType: 'nav_msgs/Odometry' });
const goalTopic = new ROSLIB.Topic({ ros: ros, name: '/goal_pose', messageType: 'geometry_msgs/PoseStamped' });

// State (Starting at Harbor Fairway Channel Entrance)
let boatPos = { x: -220.0, y: -95.0, yaw: -1.57, speed: 0 };
let currentMode = 'static';
let activeAppMode = 1;

// Pre-Defined Open Marina Docking Berths (Side Parking & Slip Parking)
const availableBerths = [
    { id: 0, name: 'Berth #1: Main Spine Pier (Side Parking)', ros_x: -215.0, ros_y: -171.5, fairway_x: -215.0, type: 'parallel' },
    { id: 1, name: 'Berth #2: Jetty 2 Left Pier Face (Side Parking)', ros_x: -198.2, ros_y: -142.2, fairway_x: -220.0, type: 'parallel' },
    { id: 2, name: 'Berth #3: Jetty 2 Right Slip (Bow-In Docking)', ros_x: -189.0, ros_y: -142.2, fairway_x: -172.0, type: 'slip' },
    { id: 3, name: 'Berth #4: Jetty 3 Left Slip (Bow-In Docking)', ros_x: -139.0, ros_y: -142.0, fairway_x: -158.0, type: 'slip' }
];
let activeBerthIdx = 1; // Default to Berth #2 Side Parking
let entities = [];
let plannedPath = [];
let isNavigating = false;
let isShipwrecked = false;
let pathIndex = 0;
let obsCounter = 1;
let currentGoal = null;
let lastRecalcTime = 0;
let threePathLine = null;

function update3DPathLine() {
    if (threePathLine) {
        scene.remove(threePathLine);
        if (threePathLine.geometry) threePathLine.geometry.dispose();
        threePathLine = null;
    }

    if (plannedPath && plannedPath.length > 1) {
        // Filter out duplicate points (like in 'align' states) that break computeLineDistances!
        const uniquePoints = [];
        plannedPath.forEach(p => {
            if (uniquePoints.length === 0) {
                uniquePoints.push(new THREE.Vector3(p.x, 0.4, -p.y));
            } else {
                const lastP = uniquePoints[uniquePoints.length - 1];
                if (Math.abs(lastP.x - p.x) > 0.01 || Math.abs(lastP.z - (-p.y)) > 0.01) {
                    uniquePoints.push(new THREE.Vector3(p.x, 0.4, -p.y));
                }
            }
        });

        if (uniquePoints.length > 1) {
            const lineGeo = new THREE.BufferGeometry().setFromPoints(uniquePoints);
            const lineMat = new THREE.LineDashedMaterial({
                color: 0x00ffcc,
                linewidth: 3,
                scale: 1,
                dashSize: 1,
                gapSize: 0.5
            });
            threePathLine = new THREE.Line(lineGeo, lineMat);
            threePathLine.computeLineDistances();
            scene.add(threePathLine);
        }
    }
}

function triggerShipwreck(entity) {
    if (isShipwrecked) return;
    isShipwrecked = true;
    isNavigating = false;
    boatPos.speed = 0;

    // Send zero velocity to stop motors in ROS
    const stopTwist = new ROSLIB.Message({
        linear: { x: 0.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
    });
    cmdVelTopic.publish(stopTwist);

    document.getElementById('tele-status').textContent = '💥 CRASH! ASV SUNK!';
    document.getElementById('tele-status').style.color = '#ff4444';

    const obsName = entity.type === 'quay_wall' ? 'the solid Quay Wall pier' : (entity.type === 'dynamic' ? 'a moving boat' : 'a moored vessel');
    setTimeout(() => {
        alert(`💥 SHIPWRECK! ASV collided with ${obsName} and sank!\n\nClick "🛑 STOP & RESET ALL" to clear level and restart.`);
    }, 100);
}

// Listen to boat Odometry (Only sync when Gazebo physics is actively driving in Mode 2)
odomTopic.subscribe((msg) => {
    const gzSpeed = Math.sqrt(
        Math.pow(msg.twist.twist.linear.x, 2) +
        Math.pow(msg.twist.twist.linear.y, 2)
    );
    if (activeAppMode === 2 && gzSpeed > 0.05) {
        boatPos.x = msg.pose.pose.position.x;
        boatPos.y = msg.pose.pose.position.y;
        boatPos.speed = gzSpeed;
        const q = msg.pose.pose.orientation;
        boatPos.yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }
});

// Canvas Setup (Massive World: 1 meter = 0.65 pixels for 600m wide ocean lake/bay)
const canvas = document.getElementById('mapCanvas');
const ctx = canvas.getContext('2d');
const SCALE = 0.65;

function getCurrentViewParams() {
    if (activeAppMode === 3) {
        // Zoomed in on Marina
        return { scale: 3.5, offsetX: -195.0, offsetY: -145.0 }; // ROS center of marina
    }
    // Global view
    return { scale: 0.65, offsetX: 0.0, offsetY: 0.0 };
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

// ================= THREE.JS 3D FPV SETUP ================= //
const container3d = document.getElementById('threejs-container');
const scene = new THREE.Scene();

// 1. Realistic Atmosphere & Sky
scene.background = new THREE.Color(0x7ec0ee); // Lake Sky Blue
scene.fog = new THREE.FogExp2(0x7ec0ee, 0.001);

const camera = new THREE.PerspectiveCamera(68, 550 / 500, 0.1, 3000);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(550, 500);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
container3d.appendChild(renderer.domElement);

// "3D FPV — Object Detection" panel: rather than a second live WebGL
// renderer (which doubles GPU cost and risks its own context issues), each
// frame just copies the already-rendered FPV canvas onto this plain 2D
// canvas, then draws detection boxes on top of that copy.
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

// 2. Circular Lake Water Body (300m radius ocean bay = 600m wide!)
const LAKE_RADIUS = 300.0;
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
// Outer Sandy Beach Ring (296m to 330m)
const beachGeo = new THREE.RingGeometry(295, 330, 64);
const beachMat = new THREE.MeshLambertMaterial({ color: 0xd2b48c, side: THREE.DoubleSide });
const beachMesh = new THREE.Mesh(beachGeo, beachMat);
beachMesh.rotation.x = -Math.PI / 2;
beachMesh.position.y = -0.05;
scene.add(beachMesh);

// Surrounding Green Forest Hills Land Mass (328m to 1200m)
const landGeo = new THREE.RingGeometry(328, 1200, 64);
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

// Generate Trees STRICTLY on outer land terrain (r >= 332m, 100% clear of water!)
for (let angle = 0; angle < Math.PI * 2; angle += 0.05) {
    const r = 332 + Math.sin(angle * 6) * 20 + Math.random() * 80;
    const tx = Math.cos(angle) * r;
    const tz = Math.sin(angle) * r;
    scene.add(createTree(tx, tz, 1.2 + Math.random() * 1.0));
}

// Core Land Obstacle: Scenic Central Island in Ocean Bay (x: 60m, z: -45m, radius: 25m)
const ISLAND_X = 60.0;
const ISLAND_Y = 45.0;
const ISLAND_RADIUS = 25.0;
const ISLAND_KEEP_OUT = ISLAND_RADIUS + 8.0; // 33.0m keep-out buffer (Guaranteed clearance, NO island crashes!)

// Core Structure Obstacle: Marina Dock & Pier at Bottom-Left (x: -215m, y: -165m, radius: 25m)
const MARINA_X = -215.0;
const MARINA_Y = -165.0;
const MARINA_RADIUS = 25.0;

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

// 3B. Ultra-Packed Real Marina Grid (30+ Vessels Packed Side-by-Side with 40cm Ultra-Small Tolerances)
const marinaGroup = new THREE.Group();

// Main Horizontal Floating Spine Pier (z = 175.0, ROS y = -175.0)
const pierMat = new THREE.MeshLambertMaterial({ color: 0x5d4037, roughness: 0.8 });
const mainPier = new THREE.Mesh(new THREE.BoxGeometry(130, 0.8, 6), pierMat);
mainPier.position.set(-195, 0.4, 175);
marinaGroup.add(mainPier);

// 3 Vertical Finger Jetties extending perpendicularly up into water (z = 175 down to z = 115)
const jettiesX = [-245.0, -195.0, -145.0];
jettiesX.forEach(jx => {
    const jetty = new THREE.Mesh(new THREE.BoxGeometry(4, 0.8, 62), pierMat);
    jetty.position.set(jx, 0.4, 144);
    marinaGroup.add(jetty);

    // Mooring Pylons along each finger jetty
    for (let pz = 115; pz <= 173; pz += 6.5) {
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
    }

    bGroup.position.set(x, 0.2, z);
    bGroup.rotation.y = 0; // Parked horizontally into berth

    // Register physical collision obstacle (isParkedShip avoids yellow buoy rings)
    entities.push({ id: 'moored_' + x + '_' + z, type: 'static', isParkedShip: true, ros_x: x, ros_y: -z });
    return bGroup;
}

// Populate Jetty 1 (Left Jetty at x: -245.0) - Densely Packed Side-by-Side (z: 168 to 120)
const zList = [168, 161.5, 155, 148.5, 142, 135.5, 129, 122.5];
const colorsList = [0x1d3557, 0x2a9d8f, 0xe63946, 0x457b9d, 0x0f4c5c, 0x3d5a80, 0x9b5de5, 0xf15bb5];
const typesList = ['yacht', 'sailboat', 'speedboat', 'yacht', 'sailboat', 'speedboat', 'yacht', 'sailboat'];

// Populate Jetty 1 (Left Jetty at x: -245.0) - Open Berth #1 at z: 142.0
zList.forEach((z, idx) => {
    if (Math.abs(z - 142.0) > 3.0) {
        marinaGroup.add(createMooredBoat(-251, z, colorsList[idx % colorsList.length], typesList[idx % typesList.length]));
    }
    marinaGroup.add(createMooredBoat(-239, z, colorsList[(idx + 2) % colorsList.length], typesList[(idx + 1) % typesList.length]));
});

// Populate Jetty 2 (Middle Jetty at x: -195.0) - Open Berth #2 at z: 142.2 (Tight 40cm Gap) and Open Berth #3 at z: 142.2 (Right Side)
marinaGroup.add(createMooredBoat(-201, 168, 0xf8f9fa, 'yacht'));
marinaGroup.add(createMooredBoat(-201, 161.5, 0x2a9d8f, 'sailboat'));
marinaGroup.add(createMooredBoat(-201, 155, 0x0f4c5c, 'speedboat'));
marinaGroup.add(createMooredBoat(-201, 148.5, 0xe63946, 'yacht')); // TOP BOUNDARY OF BERTH #2

// ===> OPEN BERTH #2: X: -201.0, Y: -142.2 (z = 142.2) <===

marinaGroup.add(createMooredBoat(-201, 135.9, 0x1d3557, 'yacht')); // BOTTOM BOUNDARY OF BERTH #2
marinaGroup.add(createMooredBoat(-201, 129.4, 0x37474f, 'sailboat'));
marinaGroup.add(createMooredBoat(-201, 122.9, 0x457b9d, 'speedboat'));

// Right side berths of Jetty 2 - Open Berth #3 at z: 142.2
zList.forEach((z, idx) => {
    if (Math.abs(z - 142.2) > 3.0) {
        marinaGroup.add(createMooredBoat(-189, z, colorsList[(idx + 3) % colorsList.length], typesList[idx % typesList.length]));
    }
});

// Populate Jetty 3 (Right Jetty at x: -145.0) - Open Berth #4 at z: 142.0
zList.forEach((z, idx) => {
    if (Math.abs(z - 142.0) > 3.0) {
        marinaGroup.add(createMooredBoat(-139, z, colorsList[(idx + 1) % colorsList.length], typesList[(idx + 2) % typesList.length]));
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
officeMesh.position.set(-260, 3.8, 175);

const officeRoof = new THREE.Mesh(
    new THREE.ConeGeometry(7, 3, 4),
    new THREE.MeshStandardMaterial({ color: 0xe63946 })
);
officeRoof.rotation.y = Math.PI / 4;
officeRoof.position.set(-260, 8.3, 175);

const beaconLight = new THREE.PointLight(0x00ffff, 2.5, 30);
beaconLight.position.set(-260, 9.0, 175);
marinaGroup.add(officeMesh, officeRoof, beaconLight);

// Coastal Town Villas & City Buildings nestled exactly on the Coast (r = ~305m)
const bldgColors = [0xfaf0e6, 0xdfc09f, 0xe8d8c8, 0xd7ccc8, 0xc05a46, 0xefebe9];
const bldgPositions = [
    { x: 215, z: -215, w: 16, h: 20, d: 14, colorIdx: 0 },
    { x: 235, z: -225, w: 20, h: 26, d: 18, colorIdx: 1 },
    { x: 210, z: -235, w: 15, h: 18, d: 14, colorIdx: 2 },
    { x: 245, z: -205, w: 18, h: 22, d: 16, colorIdx: 3 },
    { x: 220, z: -250, w: 16, h: 20, d: 14, colorIdx: 4 },
    { x: 260, z: -210, w: 22, h: 28, d: 20, colorIdx: 5 },
    { x: 240, z: -245, w: 18, h: 24, d: 16, colorIdx: 0 }
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

// ================= OBJECT DETECTION OVERLAY (GROUND TRUTH) ================= //
// Draws labeled boxes for buoys/boats onto detectionOverlayCanvas, on top of
// the copied FPV frame (see the drawImage call in draw()). There's no image
// analysis here: every entity's exact 3D position and type is already known
// (it's the same data driving the map/3D views), so this just projects each
// one's real bounding box into 2D screen space via the shared FPV camera. It
// looks like live detection because the underlying data is exactly what a
// perfect detector would recover — it just skips the (unreliable, given
// these are flat, untextured sim primitives) step of actually inferring
// that from pixels.
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

function drawDetectionOverlay() {
    // Note: no clearRect here — the caller already refreshed the whole
    // canvas via drawImage(renderer.domElement, ...) just before this runs.
    const ctx = detectionOverlayCtx;
    camera.getWorldDirection(detectionForward);

    entities.forEach(ent => {
        if (ent.isParkedShip || ent.type === 'goal') return;
        const mesh = threeEntities.get(ent.id);
        const info = DETECTION_TYPE_INFO[ent.type];
        if (!mesh || !info) return;

        detectionBox3.setFromObject(mesh);
        if (detectionBox3.isEmpty()) return;

        detectionBox3.getCenter(detectionCenter);
        detectionToObject.copy(detectionCenter).sub(camera.position);
        if (detectionToObject.dot(detectionForward) <= 0) return; // behind the camera

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

        const label = `${colorNameFromHex(info.hex)} ${info.label}`;
        ctx.strokeStyle = '#00ffcc';
        ctx.lineWidth = 2;
        ctx.strokeRect(minX, minY, maxX - minX, maxY - minY);

        ctx.font = '13px sans-serif';
        const textWidth = ctx.measureText(label).width;
        ctx.fillStyle = '#00ffcc';
        ctx.fillRect(minX, minY - 18, textWidth + 8, 18);
        ctx.fillStyle = '#001a14';
        ctx.fillText(label, minX + 4, minY - 5);
    });
}

// 3D Entity Tracker Map (High-Detail Obstacles)
const threeEntities = new Map();

function sync3DEntities() {
    entities.forEach(ent => {
        if (ent.isParkedShip) return; // Skip rendering yellow buoy rings for parked quay ships!
        if (!threeEntities.has(ent.id)) {
            let mesh;
            if (ent.type === 'static') {
                // High-detail Nautical Buoy
                const buoyGroup = new THREE.Group();
                const buoyGeo = new THREE.CylinderGeometry(0.55, 0.4, 2.2, 16);
                const buoyMat = new THREE.MeshLambertMaterial({ color: 0xff8c00 }); // Nautical Orange
                const buoyMesh = new THREE.Mesh(buoyGeo, buoyMat);
                buoyGroup.add(buoyMesh);

                // Dotted Safety Keep-Out Ring in 3D
                const ringGeo = new THREE.RingGeometry(5.8, 6.0, 32);
                const ringMat = new THREE.MeshBasicMaterial({ color: 0xffa500, side: THREE.DoubleSide, transparent: true, opacity: 0.6 });
                const ringMesh = new THREE.Mesh(ringGeo, ringMat);
                ringMesh.rotation.x = Math.PI / 2;
                ringMesh.position.y = -0.4;
                buoyGroup.add(ringMesh);

                // Blinking Beacon Light
                const light = new THREE.PointLight(0xff0000, 1.2, 12);
                light.position.set(0, 1.4, 0);
                buoyGroup.add(light);

                mesh = buoyGroup;
            } else if (ent.type === 'dynamic') {
                // Ultra-Realistic Dynamic Patrol Vessel Model
                const obsBoatGroup = new THREE.Group();

                // 1. Hydrodynamic Sleek V-Hull (Deep Navy / Marine Teal)
                const hullMat = new THREE.MeshStandardMaterial({ color: 0x0f4c5c, roughness: 0.15, metalness: 0.3 });
                const hullMesh = new THREE.Mesh(new THREE.BoxGeometry(3.8, 0.7, 1.5), hullMat);
                hullMesh.position.set(0, 0.2, 0);
                obsBoatGroup.add(hullMesh);

                // 2. Red Anti-Fouling Waterline Stripe
                const stripeMat = new THREE.MeshStandardMaterial({ color: 0xc1121f, roughness: 0.3 });
                const stripeMesh = new THREE.Mesh(new THREE.BoxGeometry(3.82, 0.22, 1.52), stripeMat);
                stripeMesh.position.set(0, 0.05, 0);
                obsBoatGroup.add(stripeMesh);

                // 3. Tapered Pointed Bow Nose
                const bowGeo = new THREE.ConeGeometry(0.75, 1.3, 4);
                const bowMesh = new THREE.Mesh(bowGeo, hullMat);
                bowMesh.rotation.x = Math.PI / 2;
                bowMesh.rotation.z = -Math.PI / 2;
                bowMesh.position.set(2.5, 0.2, 0);
                obsBoatGroup.add(bowMesh);

                // 4. White Superstructure Cabin Bridge & Tinted Glass
                const cabinBase = new THREE.Mesh(
                    new THREE.BoxGeometry(1.6, 0.85, 1.2),
                    new THREE.MeshStandardMaterial({ color: 0xf8f9fa, roughness: 0.1 })
                );
                cabinBase.position.set(-0.2, 0.825, 0);
                obsBoatGroup.add(cabinBase);

                // Tinted Glass Windshield
                const glassMat = new THREE.MeshStandardMaterial({ color: 0x1d2d44, metalness: 0.9, roughness: 0.05, transparent: true, opacity: 0.85 });
                const glassMesh = new THREE.Mesh(new THREE.BoxGeometry(0.8, 0.5, 1.15), glassMat);
                glassMesh.position.set(0.2, 0.98, 0);
                obsBoatGroup.add(glassMesh);

                // 5. Radar Arch & Rotating Radar Antenna Bar
                const archMat = new THREE.MeshStandardMaterial({ color: 0xd1d5db, metalness: 0.8, roughness: 0.2 });
                const mastLeft = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.9), archMat);
                mastLeft.position.set(-0.8, 1.45, 0.45);
                const mastRight = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.9), archMat);
                mastRight.position.set(-0.8, 1.45, -0.45);
                const archTop = new THREE.Mesh(new THREE.BoxGeometry(0.08, 0.08, 1.0), archMat);
                archTop.position.set(-0.8, 1.9, 0);
                obsBoatGroup.add(mastLeft, mastRight, archTop);

                // Rotating Radar Scanner
                const radarBar = new THREE.Mesh(new THREE.BoxGeometry(0.6, 0.08, 0.15), new THREE.MeshStandardMaterial({ color: 0xffffff }));
                radarBar.position.set(-0.8, 2.0, 0);
                radarBar.name = 'radarBar';
                obsBoatGroup.add(radarBar);

                // 6. Stainless Steel Bow Guard Railings
                const railMat = new THREE.MeshStandardMaterial({ color: 0xe5e7eb, metalness: 0.9, roughness: 0.1 });
                const railLeft = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 1.8), railMat);
                railLeft.rotation.z = Math.PI / 2;
                railLeft.position.set(1.4, 0.7, 0.7);
                const railRight = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 1.8), railMat);
                railRight.rotation.z = Math.PI / 2;
                railRight.position.set(1.4, 0.7, -0.7);
                obsBoatGroup.add(railLeft, railRight);

                // 7. Orange LifeBuoy Rescue Ring on Cabin Side
                const buoyRingGeo = new THREE.TorusGeometry(0.18, 0.05, 8, 16);
                const buoyRingMat = new THREE.MeshLambertMaterial({ color: 0xff5722 });
                const buoyRing = new THREE.Mesh(buoyRingGeo, buoyRingMat);
                buoyRing.position.set(-0.2, 0.9, 0.62);
                buoyRing.rotation.y = Math.PI / 2;
                obsBoatGroup.add(buoyRing);

                // 8. Navigation Lights (Port/Red on left, Starboard/Green on right)
                const portLight = new THREE.Mesh(
                    new THREE.SphereGeometry(0.08, 8, 8),
                    new THREE.MeshBasicMaterial({ color: 0xff0000 })
                );
                portLight.position.set(0.5, 1.05, 0.6);
                const stbdLight = new THREE.Mesh(
                    new THREE.SphereGeometry(0.08, 8, 8),
                    new THREE.MeshBasicMaterial({ color: 0x00ff00 })
                );
                stbdLight.position.set(0.5, 1.05, -0.6);
                obsBoatGroup.add(portLight, stbdLight);

                // 9. Dual Black Outboard Engines at Stern
                const engineMat = new THREE.MeshStandardMaterial({ color: 0x111111, metalness: 0.8 });
                const eng1 = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.85, 0.35), engineMat);
                eng1.position.set(-2.1, 0.2, 0.4);
                const eng2 = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.85, 0.35), engineMat);
                eng2.position.set(-2.1, 0.2, -0.4);
                obsBoatGroup.add(eng1, eng2);

                // 10. Dynamic Trailing White Water Propeller Wake
                const wakeGeo = new THREE.PlaneGeometry(3.5, 1.8);
                const wakeMat = new THREE.MeshBasicMaterial({
                    color: 0xffffff,
                    transparent: true,
                    opacity: 0.45,
                    side: THREE.DoubleSide
                });
                const wakeMesh = new THREE.Mesh(wakeGeo, wakeMat);
                wakeMesh.rotation.x = -Math.PI / 2;
                wakeMesh.position.set(-3.6, -0.38, 0);
                obsBoatGroup.add(wakeMesh);

                mesh = obsBoatGroup;
            } else if (ent.type === 'goal') {
                const goalGeo = new THREE.TorusGeometry(1.5, 0.2, 16, 100);
                const goalMat = new THREE.MeshBasicMaterial({ color: 0x00ff00, transparent: true, opacity: 0.8 });
                mesh = new THREE.Mesh(goalGeo, goalMat);
                mesh.rotation.x = Math.PI / 2;
                const glow = new THREE.PointLight(0x00ff00, 2, 20);
                glow.position.set(0, 2, 0);
                mesh.add(glow);
            }
            mesh.position.set(ent.ros_x, 0.5, -ent.ros_y);
            scene.add(mesh);
            threeEntities.set(ent.id, mesh);
        }
    });
}

// ================= ULTIMATE GRID BFS + STRING-PULLING PATHFINDER ================= //
const SAFETY_RADIUS = 5.5; // Restrictive imaginary keep-out radius for buoys (meters)

function findOptimalPath(start, goal, obstacles) {
    const validObs = obstacles.filter(o => o.type !== 'goal');

    // Helper: Check line segment collision against buoys, Island land, and Marina Docks
    function lineCollides(p1, p2) {
        const dx = p2.x - p1.x;
        const dy = p2.y - p1.y;
        const len = Math.hypot(dx, dy);
        if (len > 0) {
            // Check Island Land Mass collision
            const uIsl = Math.max(0, Math.min(1, ((ISLAND_X - p1.x) * dx + (ISLAND_Y - p1.y) * dy) / (len * len)));
            const islClosestX = p1.x + uIsl * dx;
            const islClosestY = p1.y + uIsl * dy;
            if (Math.hypot(ISLAND_X - islClosestX, ISLAND_Y - islClosestY) < ISLAND_KEEP_OUT) {
                return true;
            }

            // Check Rigid Marina Pier Wall Collision (Main Spine Pier y <= -173.0 & Finger Jetties x: -245, -195, -145)
            const jettiesX = [-245.0, -195.0, -145.0];
            for (let t = 0; t <= 1; t += 0.1) {
                const px = p1.x + t * dx;
                const py = p1.y + t * dy;
                // Main Spine Pier Wall (y <= -173.0 between x = -265 and -125)
                if (py <= -173.0 && px >= -265.0 && px <= -125.0) return true;
                // Finger Jetties (2.5m clearance around jetty centerlines from y = -115 to -175)
                for (const jx of jettiesX) {
                    if (Math.abs(px - jx) < 2.5 && py <= -115.0 && py >= -175.0) return true;
                }
            }

            // Check buoy collisions
            for (const obs of validObs) {
                const u = Math.max(0, Math.min(1, ((obs.ros_x - p1.x) * dx + (obs.ros_y - p1.y) * dy) / (len * len)));
                const closestX = p1.x + u * dx;
                const closestY = p1.y + u * dy;
                if (Math.hypot(obs.ros_x - closestX, obs.ros_y - closestY) < SAFETY_RADIUS) {
                    return true;
                }
            }
        }
        return false;
    }

    // Direct line check first (Instant 0ms if unblocked)
    if (!lineCollides(start, goal)) {
        return [{ x: start.x, y: start.y }, { x: goal.x, y: goal.y }];
    }

    // 1. Setup Grid BFS Parameters (Massive 600m Ocean Bay Pathfinder: 560m x 560m)
    const STEP = 4.0; // 4.0 meter grid cells for fast vector search
    const MIN_X = -280, MAX_X = 280, MIN_Y = -280, MAX_Y = 280;

    function toKey(gx, gy) { return `${gx},${gy}`; }
    function toWorld(gx, gy) { return { x: gx * STEP, y: gy * STEP }; }

    const startGx = Math.round(start.x / STEP);
    const startGy = Math.round(start.y / STEP);
    const goalGx = Math.round(goal.x / STEP);
    const goalGy = Math.round(goal.y / STEP);

    // 2. Pre-mark blocked grid cells (Buoys + Core Island Land Mass + Marina Docks)
    const blocked = new Set();

    // Mark Island Land & Safety Buffer (15.0m radius)
    const islandRadiusCells = Math.ceil(ISLAND_KEEP_OUT / STEP);
    const islandGx = Math.round(ISLAND_X / STEP);
    const islandGy = Math.round(ISLAND_Y / STEP);
    for (let dx = -islandRadiusCells; dx <= islandRadiusCells; dx++) {
        for (let dy = -islandRadiusCells; dy <= islandRadiusCells; dy++) {
            const gx = islandGx + dx;
            const gy = islandGy + dy;
            const w = toWorld(gx, gy);
            if (Math.hypot(w.x - ISLAND_X, w.y - ISLAND_Y) < ISLAND_KEEP_OUT) {
                blocked.add(toKey(gx, gy));
            }
        }
    }

    // Mark Marina Dock Structure Cells
    const marinaRadiusCells = Math.ceil((MARINA_RADIUS + 1.5) / STEP);
    const marinaGx = Math.round(MARINA_X / STEP);
    const marinaGy = Math.round(MARINA_Y / STEP);
    for (let dx = -marinaRadiusCells; dx <= marinaRadiusCells; dx++) {
        for (let dy = -marinaRadiusCells; dy <= marinaRadiusCells; dy++) {
            const gx = marinaGx + dx;
            const gy = marinaGy + dy;
            const w = toWorld(gx, gy);
            if (Math.hypot(w.x - MARINA_X, w.y - MARINA_Y) < MARINA_RADIUS + 1.5) {
                blocked.add(toKey(gx, gy));
            }
        }
    }

    // Mark Buoy Safety Cells
    validObs.forEach(obs => {
        const radiusCells = Math.ceil(SAFETY_RADIUS / STEP);
        const obsGx = Math.round(obs.ros_x / STEP);
        const obsGy = Math.round(obs.ros_y / STEP);

        for (let dx = -radiusCells; dx <= radiusCells; dx++) {
            for (let dy = -radiusCells; dy <= radiusCells; dy++) {
                const gx = obsGx + dx;
                const gy = obsGy + dy;
                const w = toWorld(gx, gy);
                if (Math.hypot(w.x - obs.ros_x, w.y - obs.ros_y) < SAFETY_RADIUS) {
                    blocked.add(toKey(gx, gy));
                }
            }
        }
    });

    // Ensure start & goal cells aren't strictly blocked from starting/ending
    blocked.delete(toKey(startGx, startGy));

    // 3. BFS Search for guaranteed shortest topological path
    const queue = [{ gx: startGx, gy: startGy }];
    const visited = new Set([toKey(startGx, startGy)]);
    const cameFrom = new Map();
    let foundGoalKey = null;

    const neighbors = [
        { dx: 1, dy: 0 }, { dx: -1, dy: 0 }, { dx: 0, dy: 1 }, { dx: 0, dy: -1 },
        { dx: 1, dy: 1 }, { dx: -1, dy: 1 }, { dx: 1, dy: -1 }, { dx: -1, dy: -1 }
    ];

    while (queue.length > 0) {
        const curr = queue.shift();
        const currKey = toKey(curr.gx, curr.gy);

        if (curr.gx === goalGx && curr.gy === goalGy) {
            foundGoalKey = currKey;
            break;
        }

        for (const n of neighbors) {
            const ngx = curr.gx + n.dx;
            const ngy = curr.gy + n.dy;
            const nWorld = toWorld(ngx, ngy);

            if (nWorld.x < MIN_X || nWorld.x > MAX_X || nWorld.y < MIN_Y || nWorld.y > MAX_Y) continue;
            const nKey = toKey(ngx, ngy);

            if (!visited.has(nKey) && !blocked.has(nKey)) {
                visited.add(nKey);
                cameFrom.set(nKey, currKey);
                queue.push({ gx: ngx, gy: ngy });
            }
        }
    }

    // Fallback: If exact goal is blocked/unreachable, pick visited node closest to goal
    if (!foundGoalKey) {
        let minGoalDist = Infinity;
        visited.forEach(key => {
            const [gx, gy] = key.split(',').map(Number);
            const w = toWorld(gx, gy);
            const d = Math.hypot(goal.x - w.x, goal.y - w.y);
            if (d < minGoalDist) {
                minGoalDist = d;
                foundGoalKey = key;
            }
        });
    }

    // Reconstruct raw grid path
    let rawPath = [];
    let curr = foundGoalKey;
    while (curr) {
        const [gx, gy] = curr.split(',').map(Number);
        rawPath.unshift(toWorld(gx, gy));
        curr = cameFrom.get(curr);
    }
    if (rawPath.length === 0) {
        rawPath = [{ x: start.x, y: start.y }, { x: goal.x, y: goal.y }];
    }
    rawPath[0] = { x: start.x, y: start.y };

    // 4. String Pulling (Shortcut Optimization for smooth vector turns)
    const smoothPath = [rawPath[0]];
    let currIdx = 0;
    while (currIdx < rawPath.length - 1) {
        let farthest = currIdx + 1;
        for (let nextIdx = rawPath.length - 1; nextIdx > currIdx + 1; nextIdx--) {
            if (!lineCollides(rawPath[currIdx], rawPath[nextIdx])) {
                farthest = nextIdx;
                break;
            }
        }
        smoothPath.push(rawPath[farthest]);
        currIdx = farthest;
    }

    return smoothPath;
}

// Render & Motion Loop
function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 1. Render Outer Land Mass Background (Forest Green)
    ctx.fillStyle = '#1b3b18';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    const centerP = rosToCanvas(0, 0);

    // 2. Render Sandy Beach Shore Ring (Radius: 315m to match 3D world)
    ctx.beginPath();
    ctx.arc(centerP.x, centerP.y, 315.0 * SCALE, 0, 2 * Math.PI);
    ctx.fillStyle = '#d2b48c';
    ctx.fill();

    // 3. Render Circular Blue Lake Water Body (Clean 2D Ocean Fill)
    const waterGrad = ctx.createRadialGradient(centerP.x, centerP.y, 0, centerP.x, centerP.y, 300.0 * SCALE);
    waterGrad.addColorStop(0, '#005f9e');
    waterGrad.addColorStop(1, '#002952');

    ctx.beginPath();
    ctx.arc(centerP.x, centerP.y, 300.0 * SCALE, 0, 2 * Math.PI);
    ctx.fillStyle = waterGrad;
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#001a33';
    ctx.stroke();

    // 4. Draw Tactical Grid Overlay
    ctx.strokeStyle = 'rgba(255,255,255,0.08)';
    ctx.lineWidth = 1;
    for (let i = 0; i < canvas.width; i += SCALE * 10) {
        ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, canvas.height); ctx.stroke();
    }
    for (let i = 0; i < canvas.height; i += SCALE * 10) {
        ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(canvas.width, i); ctx.stroke();
    }

    // Active Navigation Kinematic & ROS Steer Loop (Receding Horizon Live Pathfinder)
    if (isNavigating && plannedPath.length > 0 && !isShipwrecked) {
        const now = Date.now();
        // Receding Horizon Sensor Scan: Recalculate path live every 250ms based on local 25m sensor horizon
        if (now - lastRecalcTime > 250 && currentGoal) {
            lastRecalcTime = now;
            const sensedObstacles = entities.filter(ent => {
                if (ent.type === 'goal' || ent.isCrashed) return false;
                const dist = Math.hypot(ent.ros_x - boatPos.x, ent.ros_y - boatPos.y);
                return dist <= 25.0; // 25-meter sensor horizon
            });

            const newPath = findOptimalPath(
                { x: boatPos.x, y: boatPos.y },
                { x: currentGoal.ros_x, y: currentGoal.ros_y },
                sensedObstacles
            );
            if (newPath && newPath.length > 1) {
                plannedPath = newPath;
                pathIndex = 1; // Point directly to upcoming waypoint to prevent waypoint 0 snap
                update3DPathLine();
            }
        }

        // Real-time Solid Physical Boundary Collision Checks
        const distToIsland = Math.hypot(ISLAND_X - boatPos.x, ISLAND_Y - boatPos.y);
        if (distToIsland < ISLAND_RADIUS + 0.5 && !isShipwrecked) {
            triggerShipwreck({ type: 'island' });
        }

        // Real-time Solid Physical Boundary Collision Checks (Main Spine Pier Wall & Finger Jetties)
        const hullXExtent = Math.abs(Math.cos(boatPos.yaw)) * 2.8 + Math.abs(Math.sin(boatPos.yaw)) * 0.8;
        const hullYExtent = Math.abs(Math.sin(boatPos.yaw)) * 2.8 + Math.abs(Math.cos(boatPos.yaw)) * 0.8;

        // Main Spine Pier (y <= -173.0, x from -265.0 to -125.0)
        if (boatPos.y - hullYExtent <= -173.0 && (boatPos.x + hullXExtent >= -265.0 && boatPos.x - hullXExtent <= -125.0) && !isShipwrecked) {
            triggerShipwreck({ type: 'quay_wall' });
        }

        // Finger Jetties (x = -245, -195, -145, width = 4m [jx - 2.0, jx + 2.0], y in [-175, -115])
        const jettiesXList = [-245.0, -195.0, -145.0];
        jettiesXList.forEach(jx => {
            const overlapsX = (boatPos.x + hullXExtent >= jx - 2.0) && (boatPos.x - hullXExtent <= jx - 2.0 + 4.0);
            const overlapsY = (boatPos.y + hullYExtent >= -175.0) && (boatPos.y - hullYExtent <= -115.0);
            if (overlapsX && overlapsY && !isShipwrecked) {
                triggerShipwreck({ type: 'quay_wall' });
            }
        });

        // Moored vessels collision check along jetties
        for (let b = -168; b <= -128; b += 6.5) {
            if (Math.abs(b - (-147.5)) > 3.0) {
                const parkedLocs = [
                    { x: -250.5, y: b }, { x: -239.5, y: b },
                    { x: -200.5, y: b }, { x: -189.5, y: b },
                    { x: -150.5, y: b }, { x: -139.5, y: b }
                ];
                parkedLocs.forEach(pl => {
                    if (Math.hypot(pl.x - boatPos.x, pl.y - boatPos.y) < 2.0 && !isShipwrecked) {
                        triggerShipwreck({ type: 'parked_vessel' });
                    }
                });
            }
        }

        entities.forEach(ent => {
            if (ent.type === 'static' || ent.type === 'dynamic') {
                const dist = Math.hypot(ent.ros_x - boatPos.x, ent.ros_y - boatPos.y);
                if (dist < 1.7 && !isShipwrecked) { // Direct crash hit!
                    triggerShipwreck(ent);
                }
            }
        });

        if (!isShipwrecked) {
            if (pathIndex < plannedPath.length) {
                const target = plannedPath[pathIndex];
                const dx = target.x - boatPos.x;
                const dy = target.y - boatPos.y;
                const dist = Math.hypot(dx, dy);

                const targetYaw = (target.mode === 'align' && target.targetYaw !== undefined) ? target.targetYaw : Math.atan2(dy, dx);
                let yawDiff = targetYaw - boatPos.yaw;
                while (yawDiff > Math.PI) yawDiff -= 2 * Math.PI;
                while (yawDiff < -Math.PI) yawDiff += 2 * Math.PI;

                // State Machine Progression Logic
                let advancePath = false;
                if (target.mode === 'align') {
                    if (Math.abs(yawDiff) < 0.05) advancePath = true; // Advance only when heading is locked!
                } else if (target.mode === 'creep' || target.mode === 'reverse_swing') {
                    if (dist < 0.3) advancePath = true; // High precision finish line
                } else if (target.mode === 'approach') {
                    if (dist < 0.5) advancePath = true; // Wait to reach near the pier
                } else if (isAutoDocking) {
                    if (dist < 0.4) advancePath = true; // Tight channel precision for docking transit!
                } else {
                    if (dist < 1.8) advancePath = true; // Standard transit tolerance
                }

                if (advancePath) {
                    pathIndex++;
                } else {
                    const maxRudderStep = 0.022;
                    let turnStep = Math.max(-maxRudderStep, Math.min(maxRudderStep, yawDiff * 0.06));

                    let baseTargetSpeed = 1.8;
                    let turnDragFactor = Math.max(0.45, Math.cos(yawDiff * 0.6));

                    // DOCKING STATE MACHINE
                    if (isAutoDocking) {
                        if (target.mode === 'transit') {
                            baseTargetSpeed = 1.2;
                            document.getElementById('tele-status').textContent = '⚓ STATE 1: Transit to Staging Area...';
                            document.getElementById('tele-status').style.color = '#00ffcc';
                        } else if (target.mode === 'align') {
                            // STATE 2: PIVOT ALIGNMENT (Zero forward speed, pivot in place!)
                            baseTargetSpeed = 0.0;
                            turnDragFactor = 0.0;
                            turnStep = Math.max(-0.035, Math.min(0.035, yawDiff * 0.1)); // Stronger pivot thrust
                            document.getElementById('tele-status').textContent = '🔵 STATE 2: Pivoting to Alignment...';
                            document.getElementById('tele-status').style.color = '#ffaa00';
                        } else if (target.mode === 'approach') {
                            // Angled Approach (Slow forward)
                            baseTargetSpeed = 0.45;
                            turnStep = Math.max(-0.015, Math.min(0.015, yawDiff * 0.1));
                            document.getElementById('tele-status').textContent = '🟢 STATE 3: Angled Approach...';
                            document.getElementById('tele-status').style.color = '#00ff00';
                        } else if (target.mode === 'reverse_swing') {
                            // Reverse Swing (Reverse speed + hard rudder)
                            baseTargetSpeed = -0.4;
                            turnStep = Math.max(-0.04, Math.min(0.04, yawDiff * 0.2)); // Hard pivot while reversing
                            document.getElementById('tele-status').textContent = '🟣 STATE 4: Reversing & Swinging Stern...';
                            document.getElementById('tele-status').style.color = '#cc00ff';
                        } else if (target.mode === 'creep') {
                            // STATE 3: CREEP INSERTION (Micro-speed linear slide)
                            baseTargetSpeed = 0.35;
                            turnStep = Math.max(-0.015, Math.min(0.015, yawDiff * 0.1)); // Extremely tight heading hold
                            document.getElementById('tele-status').textContent = '🟢 STATE 3: Creep Insertion...';
                            document.getElementById('tele-status').style.color = '#00ff00';
                        }
                    }

                    boatPos.yaw += turnStep;

                    // Hydrodynamic Roll Banking: Lean boat hull realistically into turns in 3D
                    if (boatGroup && !isShipwrecked) {
                        boatGroup.rotation.z = -turnStep * 8.5;
                    }

                    boatPos.speed = baseTargetSpeed * turnDragFactor;
                    
                    // Realistic Kinematics (No Magic Sideways Sliding)
                    boatPos.x += Math.cos(boatPos.yaw) * (boatPos.speed * 0.07);
                    boatPos.y += Math.sin(boatPos.yaw) * (boatPos.speed * 0.07);

                    // Send cmd_vel to ROS with smooth rudder angular velocity
                    const twist = new ROSLIB.Message({
                        linear: { x: boatPos.speed, y: 0.0, z: 0.0 },
                        angular: { x: 0.0, y: 0.0, z: turnStep * 10.0 }
                    });
                    cmdVelTopic.publish(twist);
                }
            } else {
                isNavigating = false;
                boatPos.speed = 0;

                // Publish zero velocity to ROS to freeze boat motors
                const stopTwist = new ROSLIB.Message({
                    linear: { x: 0.0, y: 0.0, z: 0.0 },
                    angular: { x: 0.0, y: 0.0, z: 0.0 }
                });
                cmdVelTopic.publish(stopTwist);

                if (isAutoDocking) {
                    isAutoDocking = false;
                    document.getElementById('tele-status').textContent = '🎉 DOCKED SAFELY AT MARINA BERTH #1!';
                    document.getElementById('tele-status').style.color = '#28a745';
                } else {
                    document.getElementById('tele-status').textContent = '🎉 Goal Reached!';
                    document.getElementById('tele-status').style.color = '#28a745';
                }
            }
        }

        document.getElementById('tele-x').textContent = boatPos.x.toFixed(2);
        document.getElementById('tele-y').textContent = boatPos.y.toFixed(2);
        document.getElementById('tele-speed').textContent = boatPos.speed.toFixed(2);
    }

    // Draw Planned Path Trajectory
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

    // Update Dynamic Moving Boats Realistic Heavy Ship Kinematics & Buoy Collision
    entities.forEach(entity => {
        if (entity.type === 'dynamic') {
            const mesh = threeEntities.get(entity.id);

            // Handle Crashed Dynamic Boats (Sunk in buoy collision)
            if (entity.isCrashed) {
                if (mesh) {
                    mesh.rotation.z = 0.8; // Tilt sideways
                    mesh.position.y = -0.3; // Submerge
                }
                return;
            }

            // Check if dynamic boat hits any static buoy
            entities.forEach(other => {
                if (other.type === 'static' && !entity.isCrashed) {
                    const distToBuoy = Math.hypot(other.ros_x - entity.ros_x, other.ros_y - entity.ros_y);
                    if (distToBuoy < 2.5) {
                        entity.isCrashed = true; // Dynamic boat crashes & sinks into buoy!
                    }
                }
            });

            if (entity.isCrashed) return;

            // Initialize realistic heavy ship patrol waypoints & heading if missing
            if (!entity.patrolPoints) {
                const r = 24.0;
                entity.patrolPoints = [
                    { x: entity.ros_x + r, y: entity.ros_y },
                    { x: entity.ros_x, y: entity.ros_y + r },
                    { x: entity.ros_x - r, y: entity.ros_y },
                    { x: entity.ros_x, y: entity.ros_y - r }
                ];
                entity.patrolIdx = 0;
                entity.heading = Math.random() * Math.PI * 2;
                entity.speed = 1.0; // Heavy ship cruising speed
            }

            // Target current patrol waypoint in loop
            const target = entity.patrolPoints[entity.patrolIdx];
            const dx = target.x - entity.ros_x;
            const dy = target.y - entity.ros_y;
            const dist = Math.hypot(dx, dy);

            if (dist < 5.0) {
                entity.patrolIdx = (entity.patrolIdx + 1) % entity.patrolPoints.length;
            }

            // Gradual heavy rudder turning towards target
            const targetHeading = Math.atan2(dy, dx);
            let headingDiff = targetHeading - entity.heading;

            while (headingDiff > Math.PI) headingDiff -= 2 * Math.PI;
            while (headingDiff < -Math.PI) headingDiff += 2 * Math.PI;

            entity.heading += headingDiff * 0.015; // Slow, wide ship turning radius

            // Advance boat position forward along its hull orientation
            entity.ros_x += Math.cos(entity.heading) * entity.speed * 0.05;
            entity.ros_y += Math.sin(entity.heading) * entity.speed * 0.05;

            // Sync 3D Mesh Position, Yaw, Pitch, Roll & Water Heave Physics in Three.js
            if (mesh) {
                const nowTime = Date.now() * 0.0025;
                const heave = Math.sin(nowTime * 1.5 + entity.ros_x * 0.1) * 0.06;
                const pitch = Math.sin(nowTime + entity.ros_y * 0.1) * 0.04;
                const roll = Math.cos(nowTime * 1.2 + entity.ros_x * 0.1) * 0.05;

                mesh.position.set(entity.ros_x, 0.4 + heave, -entity.ros_y);
                mesh.rotation.set(pitch, entity.heading, roll);

                // Spin radar antenna scanner
                const rBar = mesh.getObjectByName('radarBar');
                if (rBar) rBar.rotation.y += 0.08;
            }
        }
    });

    // Draw 2D Marina using proper scaling for zoomed views
    ctx.save();
    const view = getCurrentViewParams();
    const s = view.scale;

    // Main Horizontal Pier Spine: x from -265 to -125 (width 140), y from -173 to -177 (height 4). Center = (-195, -175).
    ctx.fillStyle = '#5d4037';
    // Draw Spine
    const spineTL = rosToCanvas(-265, -173); // Top-Left in Canvas means min X, max Y (since +Y is up in ROS, -173 is "above" -177)
    // Wait, rosToCanvas cy = height/2 - (ry - offsetY)*s. Larger ry -> smaller cy. 
    // So ry=-173 gives smaller cy (higher on screen) than ry=-177.
    // So width is 140*s, height is 4*s.
    ctx.fillRect(spineTL.x, spineTL.y, 140 * s, 4 * s);

    // 3 Vertical Finger Jetties extending up into water
    // x = -245, -195, -145. width 4m. y = -115 to -173. height = 58m.
    const jetties2D = [-245.0, -195.0, -145.0];
    jetties2D.forEach(jx => {
        // max Y is -115. min Y is -173.
        const jettyTL = rosToCanvas(jx - 2.0, -115.0);
        ctx.fillRect(jettyTL.x, jettyTL.y, 4 * s, 58 * s);
    });

    // Draw Moored Ships Along All Jetties (Leaving 4 Open Berths!)
    const bColors2D = ['#1d3557', '#2a9d8f', '#e63946', '#457b9d', '#0f4c5c', '#3d5a80', '#9b5de5', '#f15bb5'];
    
    // We will draw the boats using rosToCanvas directly. Boats are 6m x 3.5m.
    // Jetty 1 & 3 Left/Right, Jetty 2 Left/Right
    for (let b = -168; b <= -128; b += 6.5) { // y coordinates
        // Jetty 1 Left (-245 - 2 - 3.5/2 = -248.75 center? No, let's just draw them relative)
        if (Math.abs(b - (-147.5)) > 3.0) { // Keep berth 1 empty at -147.5
            const p = rosToCanvas(-250.5, b);
            ctx.fillStyle = bColors2D[Math.abs(Math.floor(b)) % bColors2D.length];
            ctx.fillRect(p.x, p.y, 3.5 * s, 6 * s); // width 3.5, height 6
        }
        // Jetty 1 Right
        const p1r = rosToCanvas(-239.5, b);
        ctx.fillRect(p1r.x, p1r.y, 3.5 * s, 6 * s);

        // Jetty 3 Left
        if (Math.abs(b - (-147.5)) > 3.0) { // berth 4 empty
            const p3l = rosToCanvas(-150.5, b);
            ctx.fillStyle = bColors2D[(Math.abs(Math.floor(b)) + 3) % bColors2D.length];
            ctx.fillRect(p3l.x, p3l.y, 3.5 * s, 6 * s);
        }
        // Jetty 3 Right
        const p3r = rosToCanvas(-139.5, b);
        ctx.fillRect(p3r.x, p3r.y, 3.5 * s, 6 * s);

        // Jetty 2 Right
        if (Math.abs(b - (-147.5)) > 3.0) { // berth 3 empty
            const p2r = rosToCanvas(-189.5, b);
            ctx.fillStyle = bColors2D[(Math.abs(Math.floor(b)) + 2) % bColors2D.length];
            ctx.fillRect(p2r.x, p2r.y, 3.5 * s, 6 * s);
        }
        // Jetty 2 Left
        if (Math.abs(b - (-147.5)) > 3.0) { // berth 2 empty
            const p2l = rosToCanvas(-200.5, b);
            ctx.fillStyle = bColors2D[Math.abs(Math.floor(b)) % bColors2D.length];
            ctx.fillRect(p2l.x, p2l.y, 3.5 * s, 6 * s);
        }
    }

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

    // Label Text
    ctx.font = 'bold 10px sans-serif';
    ctx.fillStyle = '#00ffcc';
    const marinaBase = rosToCanvas(-195, -175);
    ctx.fillText('⚡ 30+ PACKED MARINA GRID (<40cm TOLERANCE)', marinaBase.x - 65, marinaBase.y + 12);
    ctx.restore();

    // City Building Blocks (Strictly on Upper-Right Sandy Coast)
    const cityCanvas = rosToCanvas(230, 230);
    ctx.fillStyle = '#37474f';
    ctx.fillRect(cityCanvas.x - 28, cityCanvas.y - 14, 62, 28);
    ctx.fillStyle = '#eceff1';
    ctx.font = 'bold 10px sans-serif';
    ctx.fillText('🏙️ Coastal City', cityCanvas.x - 24, cityCanvas.y + 3);
    ctx.restore();

    // Draw 2D Core Island Land Mass (Obstacle at x: 20m, y: 15m, radius: 12m)
    const islandCanvas = rosToCanvas(ISLAND_X, ISLAND_Y);
    ctx.save();

    // Sand Border
    ctx.beginPath();
    ctx.arc(islandCanvas.x, islandCanvas.y, (ISLAND_RADIUS + 1.5) * SCALE, 0, 2 * Math.PI);
    ctx.fillStyle = '#d2b48c';
    ctx.fill();
    // Grass Island Top
    ctx.beginPath();
    ctx.arc(islandCanvas.x, islandCanvas.y, ISLAND_RADIUS * SCALE, 0, 2 * Math.PI);
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

    // Draw Entities & Imaginary Dotted Safety Keep-Out Circles
    entities.forEach(entity => {
        const p = rosToCanvas(entity.ros_x, entity.ros_y);

        // Draw Visual Restrictive Imaginary Dotted Safety Circle
        if (entity.type === 'static' || entity.type === 'dynamic') {
            ctx.save();
            ctx.beginPath();
            ctx.setLineDash([4, 4]);
            ctx.arc(p.x, p.y, SAFETY_RADIUS * SCALE, 0, 2 * Math.PI);
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

    // Draw ASV Boat (2D Map)
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

    // Sync 3D Scene & FPV Camera
    sync3DEntities();

    // Water Wave Animation (Realistic fluid harmonic swells)
    const time = Date.now() * 0.0015;
    const position = waterGeo.attributes.position;

    for (let i = 0; i < position.count; i++) {
        const u = position.getX(i);
        const v = position.getY(i);
        // Dramatic rolling sea swells with high amplitude and crisp wave crests
        const z = Math.sin(u * 0.14 + time) * 0.65
            + Math.cos(v * 0.14 + time * 1.3) * 0.45
            + Math.sin((u + v) * 0.22 + time * 1.8) * 0.25;
        position.setZ(i, z);
    }
    position.needsUpdate = true;
    waterGeo.computeVertexNormals();

    // Drift Clouds Slowly Across Sky
    clouds.forEach(cloud => {
        cloud.position.x += 0.05;
        if (cloud.position.x > 600) cloud.position.x = -600;
    });

    // Animate Flying Seagull Flock (Circling Sky & Flapping Wings)
    flock.forEach(bird => {
        bird.userData.angle += bird.userData.speed;
        const bx = Math.cos(bird.userData.angle) * bird.userData.radius;
        const bz = Math.sin(bird.userData.angle) * bird.userData.radius;
        const by = bird.userData.height + Math.sin(bird.userData.angle * 2) * 4;

        bird.position.set(bx, by, bz);
        bird.rotation.y = -bird.userData.angle + Math.PI / 2; // Facing flight vector

        // Flap Wings
        const flap = Math.sin(Date.now() * 0.012 + bird.userData.phase) * 0.45;
        const lw = bird.getObjectByName('leftWing');
        const rw = bird.getObjectByName('rightWing');
        if (lw) lw.rotation.z = flap;
        if (rw) rw.rotation.z = -flap;
    });

    // Simulate Buoy Blinking
    threeEntities.forEach((mesh, id) => {
        if (id.startsWith('obs_')) {
            const light = mesh.children.find(c => c.isPointLight);
            if (light) {
                light.intensity = Math.sin(time * 2) > 0 ? 1 : 0;
            }
        }
    });

    // 3D Boat Group Transform & Shipwreck Sinking Animation
    if (radarMesh) radarMesh.rotation.y += 0.05; // Spin radar scanner

    if (isShipwrecked) {
        boatGroup.rotation.z = Math.min(boatGroup.rotation.z + 0.05, 1.1); // Tilt boat onto side
        boatGroup.position.y = Math.max(boatGroup.position.y - 0.02, -1.2); // Sink below water surface
    } else {
        boatGroup.position.set(boatPos.x, 0.4, -boatPos.y);
        boatGroup.rotation.y = boatPos.yaw;
        boatGroup.rotation.z = 0;
    }

    const targetCamPos = new THREE.Vector3(-8, 4, 0);
    targetCamPos.applyAxisAngle(new THREE.Vector3(0, 1, 0), boatPos.yaw);
    targetCamPos.add(new THREE.Vector3(boatPos.x, 0, -boatPos.y));

    camera.position.lerp(targetCamPos, 0.12); // Smooth 3D FPV camera tracking lerp
    camera.lookAt(boatPos.x, 0.8, -boatPos.y);

    renderer.render(scene, camera);
    detectionOverlayCtx.drawImage(renderer.domElement, 0, 0, detectionOverlayCanvas.width, detectionOverlayCanvas.height);
    drawDetectionOverlay();

    requestAnimationFrame(draw);
}
draw();

// UI Interactions
document.querySelectorAll('.tool-btn').forEach(btn => {
    btn.addEventListener('click', (e) => {
        document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
        e.currentTarget.classList.add('active');
        currentMode = e.currentTarget.dataset.type;
    });
});

function selectBerth(idx) {
    activeBerthIdx = idx;
    const bSel = document.getElementById('berth-selector');
    if (bSel) bSel.value = idx.toString();

    // Update 3D Beacon Mesh Lights
    berth3DMeshes.forEach((mesh, i) => {
        const isAct = i === activeBerthIdx;
        mesh.material.color.setHex(isAct ? 0x00ffcc : 0xffc107);
        const l = mesh.children.find(c => c.isPointLight);
        if (l) l.color.setHex(isAct ? 0x00ffcc : 0xffc107);
    });
}

canvas.addEventListener('click', (e) => {
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    let { rx, ry } = canvasToRos(cx, cy);

    // In Mode 3, click near any pier edge to calculate dynamic autonomous parking
    if (activeAppMode === 3) {
        let bestBerth = null;
        let minDist = 3.5; // 3.5m snap distance

        const dockTypeSel = document.getElementById('dock-type-selector');
        const isParallel = dockTypeSel ? (dockTypeSel.value === 'parallel') : true;

        // 1. Check Spine Pier (Horizontal, y = -175.0)
        if (rx >= -265.0 && rx <= -125.0) {
            if (Math.abs(ry - (-175.0)) < minDist) {
                minDist = Math.abs(ry - (-175.0));
                bestBerth = {
                    name: 'Dynamic Docking (Main Spine Pier)',
                    ros_x: rx,
                    ros_y: isParallel ? -171.2 : -168.8,
                    type: isParallel ? 'parallel' : 'slip',
                    parkedYaw: isParallel ? 0.0 : -1.57, 
                    corridor_x: isParallel ? rx - 15.0 : rx,
                    corridor_y: -145.0,
                    staging_x: isParallel ? rx - 15.0 : rx,
                    staging_y: isParallel ? -171.2 : -145.0
                };
            }
        }

        // 2. Check Finger Jetties (Vertical, x = -245, -195, -145)
        const jetties = [-245.0, -195.0, -145.0];
        jetties.forEach((jx, index) => {
            if (ry <= -115.0 && ry >= -175.0) {
                // Left Face (wall at jx - 2.0)
                if (Math.abs(rx - (jx - 2.0)) < minDist) {
                    minDist = Math.abs(rx - (jx - 2.0));
                    const targetX = isParallel ? jx - 3.6 : jx - 5.5;
                    const stagingX = isParallel ? jx - 5.5 : targetX;
                    const openCorridorX = (index === 0) ? -265.0 : (index === 1 ? -220.0 : -170.0);
                    bestBerth = {
                        name: `Dynamic Docking (Jetty ${index+1} Left)`,
                        ros_x: targetX,
                        ros_y: ry,
                        corridor_x: openCorridorX,
                        corridor_y: -105.0,
                        staging_x: stagingX,
                        staging_y: ry,
                        type: isParallel ? 'parallel' : 'slip',
                        parkedYaw: isParallel ? -1.57 : 0.0
                    };
                }
                // Right Face (wall at jx + 2.0)
                if (Math.abs(rx - (jx + 2.0)) < minDist) {
                    minDist = Math.abs(rx - (jx + 2.0));
                    const targetX = isParallel ? jx + 3.6 : jx + 5.5;
                    const stagingX = isParallel ? jx + 5.5 : targetX;
                    const openCorridorX = (index === 2) ? -125.0 : (index === 0 ? -220.0 : -170.0);
                    bestBerth = {
                        name: `Dynamic Docking (Jetty ${index+1} Right)`,
                        ros_x: targetX,
                        ros_y: ry,
                        corridor_x: openCorridorX,
                        corridor_y: -105.0,
                        staging_x: stagingX,
                        staging_y: ry,
                        type: isParallel ? 'parallel' : 'slip',
                        parkedYaw: isParallel ? -1.57 : 3.14
                    };
                }
            }
        });

        if (!bestBerth) {
            alert("No valid pier edge detected! Please click closer to a rigid wooden dock.");
            return;
        }

        // Check if selected spot overlaps a parked vessel on finger jetties
        if (bestBerth.ros_y <= -128.0 && bestBerth.ros_y >= -168.0 && Math.abs(bestBerth.ros_y - (-147.5)) > 3.5) {
            if (bestBerth.name.includes('Jetty')) {
                alert("⛔ Space Occupied! A parked vessel is currently docked at this location. Please select an open spot.");
                return;
            }
        }

        // 3. Perception / Collision Check for obstacles
        let blocked = false;
        entities.forEach(ent => {
            if (ent.type === 'static' || ent.type === 'dynamic') {
                const dist = Math.hypot(ent.ros_x - bestBerth.ros_x, ent.ros_y - bestBerth.ros_y);
                if (dist < 5.5) blocked = true;
            }
        });

        if (blocked) {
            alert("🚫 OBSTACLE DETECTED! There is another vessel parked there. Choose an empty space.");
            return;
        }

        // 4. Start Dynamic Docking Sequence!
        startDynamicDocking(bestBerth);
        return;
    }

    if (activeAppMode !== 1 || !currentMode) return;

    // Enforce Lake Open-Water Constraints (Keep goals & buoys inside water body, r <= 285m)
    const distFromOrigin = Math.hypot(rx, ry);
    if (distFromOrigin > 285.0) {
        rx = (rx / distFromOrigin) * 285.0;
        ry = (ry / distFromOrigin) * 285.0;
    }

    // Keep clear of Central Island (r >= 33.0m from island center)
    const distFromIsland = Math.hypot(rx - ISLAND_X, ry - ISLAND_Y);
    if (distFromIsland < 33.0) {
        const angle = Math.atan2(ry - ISLAND_Y, rx - ISLAND_X);
        rx = ISLAND_X + Math.cos(angle) * 33.0;
        ry = ISLAND_Y + Math.sin(angle) * 33.0;
    }

    if (currentMode === 'goal') {
        entities = entities.filter(ent => ent.type !== 'goal');
        currentGoal = { type: 'goal', ros_x: rx, ros_y: ry, id: 'goal_node' };
        entities.push(currentGoal);
    } else {
        const obsId = `obs_${obsCounter++}`;
        const ent = {
            type: currentMode,
            ros_x: rx,
            ros_y: ry,
            id: obsId,
            vx: currentMode === 'dynamic' ? (Math.random() > 0.5 ? 0.14 : -0.14) : 0,
            vy: currentMode === 'dynamic' ? (Math.random() > 0.5 ? 0.08 : -0.08) : 0
        };
        entities.push(ent);

        const msg = new ROSLIB.Message({
            data: JSON.stringify({ type: currentMode, x: rx, y: ry, id: obsId })
        });
        spawnTopic.publish(msg);
    }
});

// ▶️ RUN ASV DEMO Button
document.getElementById('btn-run').addEventListener('click', () => {
    if (!currentGoal) {
        alert("Please set a Target Goal (🏁) first before running!");
        return;
    }

    document.getElementById('tele-status').textContent = '▶️ Navigating...';
    document.getElementById('tele-status').style.color = '#00ffcc';

    // Calculate A* Collision-Free Path around all buoys
    plannedPath = findOptimalPath(
        { x: boatPos.x, y: boatPos.y },
        { x: currentGoal.ros_x, y: currentGoal.ros_y },
        entities
    );

    // Start Smooth Execution
    isNavigating = true;
    pathIndex = 0;

    // Send Goal Pose to ROS
    const goalMsg = new ROSLIB.Message({
        header: { frame_id: 'map' },
        pose: {
            position: { x: currentGoal.ros_x, y: currentGoal.ros_y, z: 0.0 },
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
        }
    });
    goalTopic.publish(goalMsg);
});

document.getElementById('btn-reset').addEventListener('click', () => {
    // 1. Halt active navigation immediately & reset shipwreck status
    isNavigating = false;
    isShipwrecked = false;
    isAutoDocking = false;
    plannedPath = [];
    currentGoal = null;
    pathIndex = 0;

    // 2. Reset boat pose, speed & 3D mesh transforms back to Harbor Fairway Entrance
    boatPos = { x: -220.0, y: -95.0, yaw: -1.57, speed: 0 };
    if (boatGroup) {
        boatGroup.position.set(-220.0, 0.4, 95.0);
        boatGroup.rotation.set(0, -1.57, 0);
    }

    // 3. Publish zero velocity command to ROS
    const stopTwist = new ROSLIB.Message({
        linear: { x: 0.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
    });
    cmdVelTopic.publish(stopTwist);

    // 4. Send Reset signal to Gazebo Backend Spawner to delete physical models & reset boat pose
    const resetMsg = new ROSLIB.Message({
        data: JSON.stringify({ type: 'reset' })
    });
    spawnTopic.publish(resetMsg);

    // 5. Remove all 3D meshes from Three.js scene & clear 2D entity lists
    threeEntities.forEach(mesh => scene.remove(mesh));
    threeEntities.clear();
    entities = [];

    // 5. Reset UI Telemetry Display
    document.getElementById('tele-status').textContent = 'Design Phase';
    document.getElementById('tele-status').style.color = '#ffc107';
    document.getElementById('tele-x').textContent = '0.00';
    document.getElementById('tele-y').textContent = '0.00';
    document.getElementById('tele-speed').textContent = '0.00';
});

// Mode Switcher
const mode1Btn = document.getElementById('mode1-btn');
const mode2Btn = document.getElementById('mode2-btn');
const mode3Btn = document.getElementById('mode3-btn');
const mode1Tools = document.getElementById('mode1-tools');
const mode2Tools = document.getElementById('mode2-tools');
const mode3Tools = document.getElementById('mode3-tools');

if (mode1Btn) {
    mode1Btn.addEventListener('click', () => {
        if (activeAppMode === 2) stopThrusters();
        activeAppMode = 1;
        mode1Btn.classList.add('active');
        if (mode2Btn) mode2Btn.classList.remove('active');
        if (mode3Btn) mode3Btn.classList.remove('active');
        if (mode1Tools) mode1Tools.style.display = 'block';
        if (mode2Tools) mode2Tools.style.display = 'none';
        if (mode3Tools) mode3Tools.style.display = 'none';
    });
}

if (mode2Btn) {
    mode2Btn.addEventListener('click', () => {
        activeAppMode = 2;
        mode2Btn.classList.add('active');
        if (mode1Btn) mode1Btn.classList.remove('active');
        if (mode3Btn) mode3Btn.classList.remove('active');
        if (mode2Tools) mode2Tools.style.display = 'block';
        if (mode1Tools) mode1Tools.style.display = 'none';
        if (mode3Tools) mode3Tools.style.display = 'none';
    });
}

if (mode3Btn) {
    mode3Btn.addEventListener('click', () => {
        if (activeAppMode === 2) stopThrusters();
        activeAppMode = 3;
        mode3Btn.classList.add('active');
        if (mode1Btn) mode1Btn.classList.remove('active');
        if (mode2Btn) mode2Btn.classList.remove('active');
        if (mode3Tools) mode3Tools.style.display = 'block';
        if (mode1Tools) mode1Tools.style.display = 'none';
        if (mode2Tools) mode2Tools.style.display = 'none';
    });
}

// Mode 3: Autonomous Docking Action Trigger
let isAutoDocking = false;

function startDynamicDocking(chosen) {
    isNavigating = false;
    isShipwrecked = false;
    isAutoDocking = true;

    if (dynamic3DBerthMesh) {
        dynamic3DBerthMesh.position.set(chosen.ros_x, 0.4, -chosen.ros_y);
    }

    const isParallelMode = (chosen.type === 'parallel');

    const statusEl = document.getElementById('tele-status');
    if (statusEl) {
        statusEl.textContent = isParallelMode
            ? `⚓ Side-Docking into ${chosen.name}...`
            : `⚓ Bow-In Docking into ${chosen.name}...`;
        statusEl.style.color = '#00ffcc';
    }

    let dockWaypoints = [];
    if (chosen.name.includes('Spine Pier')) {
        if (isParallelMode) {
            dockWaypoints = [
                { x: boatPos.x, y: boatPos.y, mode: 'transit' },
                { x: chosen.corridor_x, y: -105.0, mode: 'transit' },
                { x: chosen.corridor_x, y: chosen.ros_y, mode: 'transit' },
                { x: chosen.corridor_x, y: chosen.ros_y, mode: 'align', targetYaw: chosen.parkedYaw },
                { x: chosen.ros_x, y: chosen.ros_y, mode: 'creep' }
            ];
        } else {
            dockWaypoints = [
                { x: boatPos.x, y: boatPos.y, mode: 'transit' },
                { x: chosen.ros_x, y: -105.0, mode: 'transit' },
                { x: chosen.ros_x, y: -145.0, mode: 'transit' },
                { x: chosen.ros_x, y: -145.0, mode: 'align', targetYaw: chosen.parkedYaw },
                { x: chosen.ros_x, y: chosen.ros_y, mode: 'creep' }
            ];
        }
    } else {
        // Finger Jetties (Vertical docks):
        // 1. Transit through North Open Water (-105.0) to open channel corridor_x
        // 2. Descend wide open fairway to berth Y-level
        // 3. Creep sideways/across channel into berth position (ros_x, ros_y)
        // 4. Align heading at berth position to match parallel/slip parkedYaw!
        dockWaypoints = [
            { x: boatPos.x, y: boatPos.y, mode: 'transit' },
            { x: chosen.corridor_x, y: -105.0, mode: 'transit' },
            { x: chosen.corridor_x, y: chosen.ros_y, mode: 'transit' },
            { x: chosen.ros_x, y: chosen.ros_y, mode: 'creep' },
            { x: chosen.ros_x, y: chosen.ros_y, mode: 'align', targetYaw: chosen.parkedYaw }
        ];
    }

    plannedPath = dockWaypoints;
    pathIndex = 1;
    update3DPathLine();
    isNavigating = true;
}

const resetDockBtn = document.getElementById('btn-reset-dock');
if (resetDockBtn) {
    resetDockBtn.addEventListener('click', () => {
        document.getElementById('btn-reset').click();
    });
}

// Mode 2 Controls: continuous thruster + water-friction velocity model.
// A held control ramps quickly toward its fixed end velocity (thrusters
// "open"); releasing it lets velocity decay gradually toward zero at the
// water-friction rate instead of snapping to zero immediately.
function sendCmdVel(linear, angular) {
    const twist = new ROSLIB.Message({
        linear: { x: linear, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angular }
    });
    cmdVelTopic.publish(twist);
}

let currentLinear = 0.0;
let currentAngular = 0.0;
const heldAxes = new Set(); // 'fwd' | 'rev' | 'left' | 'right'

function keyToAxis(key) {
    switch (key) {
        case 'w': case 'arrowup': return 'fwd';
        case 's': case 'arrowdown': return 'rev';
        case 'a': case 'arrowleft': return 'left';
        case 'd': case 'arrowright': return 'right';
        default: return null;
    }
}

function approachVelocity(current, target, dt) {
    const rate = target !== 0 ? THRUST_RAMP_RATE : WATER_FRICTION_RATE;
    const maxStep = rate * dt;
    const diff = target - current;
    if (Math.abs(diff) <= maxStep) return target;
    return current + Math.sign(diff) * maxStep;
}

function stopThrusters() {
    heldAxes.clear();
    currentLinear = 0.0;
    currentAngular = 0.0;
    sendCmdVel(0.0, 0.0);
}

// On-screen D-pad: press-and-hold, matching keyboard behavior
function bindThruster(el, axis) {
    if (!el) return;
    el.addEventListener('mousedown', () => heldAxes.add(axis));
    el.addEventListener('touchstart', (e) => { e.preventDefault(); heldAxes.add(axis); });
    ['mouseup', 'mouseleave', 'touchend', 'touchcancel'].forEach(evt =>
        el.addEventListener(evt, () => heldAxes.delete(axis))
    );
}
bindThruster(document.getElementById('btn-up'), 'fwd');
bindThruster(document.getElementById('btn-down'), 'rev');
bindThruster(document.getElementById('btn-left'), 'left');
bindThruster(document.getElementById('btn-right'), 'right');

const btnStop = document.getElementById('btn-stop');
if (btnStop) {
    btnStop.addEventListener('click', stopThrusters);
}

window.addEventListener('keydown', (e) => {
    if (activeAppMode !== 2) return;
    if (e.key === ' ') { stopThrusters(); return; }
    const axis = keyToAxis(e.key.toLowerCase());
    if (axis) heldAxes.add(axis);
});

window.addEventListener('keyup', (e) => {
    if (activeAppMode !== 2) return;
    const axis = keyToAxis(e.key.toLowerCase());
    if (axis) heldAxes.delete(axis);
});

let lastThrusterTime = performance.now();
let lastPublishedLinear = 0.0;
let lastPublishedAngular = 0.0;

function thrusterLoop() {
    const now = performance.now();
    const dt = Math.min((now - lastThrusterTime) / 1000, 0.1); // clamp so a stalled tab doesn't jump velocity
    lastThrusterTime = now;

    if (activeAppMode === 2) {
        const targetLinear = heldAxes.has('fwd') ? MAX_LINEAR_FWD : heldAxes.has('rev') ? MAX_LINEAR_REV : 0.0;
        const targetAngular = heldAxes.has('left') ? MAX_ANGULAR : heldAxes.has('right') ? -MAX_ANGULAR : 0.0;

        currentLinear = approachVelocity(currentLinear, targetLinear, dt);
        currentAngular = approachVelocity(currentAngular, targetAngular, dt);

        if (currentLinear !== 0.0 || currentAngular !== 0.0) {
            boatPos.x += Math.cos(boatPos.yaw) * currentLinear * dt;
            boatPos.y += Math.sin(boatPos.yaw) * currentLinear * dt;
            boatPos.yaw += currentAngular * dt;
        }

        // Publish while moving/decaying; once settled at zero, send one
        // final stop and go quiet rather than flooding the bridge forever.
        const stillish = currentLinear === 0.0 && currentAngular === 0.0;
        const wasPublishingMotion = lastPublishedLinear !== 0.0 || lastPublishedAngular !== 0.0;
        if (!stillish || wasPublishingMotion) {
            sendCmdVel(currentLinear, currentAngular);
            lastPublishedLinear = currentLinear;
            lastPublishedAngular = currentAngular;
        }

        // Telemetry panel: this is the only place driving it in Mode 2 (the
        // status-machine block further down only runs during Mode 1/3
        // auto-nav), so without this it just sits frozen on stale values.
        const teleStatusEl = document.getElementById('tele-status');
        if (teleStatusEl) {
            if (stillish) {
                teleStatusEl.textContent = '⚓ Idle (Manual Mode)';
                teleStatusEl.style.color = '#ffc107';
            } else {
                teleStatusEl.textContent = '🕹️ Manual Drive';
                teleStatusEl.style.color = '#00ffcc';
            }
        }
        document.getElementById('tele-x').textContent = boatPos.x.toFixed(2);
        document.getElementById('tele-y').textContent = boatPos.y.toFixed(2);
        document.getElementById('tele-speed').textContent = Math.abs(currentLinear).toFixed(2);
    }

    requestAnimationFrame(thrusterLoop);
}
requestAnimationFrame(thrusterLoop);
