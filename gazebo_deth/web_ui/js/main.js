// ================= MAIN RENDER/NAV LOOP ORCHESTRATOR =================
// The slim draw() loop: frame timing, then calls into render-2d.js's
// rendering functions, navigation.js's nav step, entities-3d.js's mesh sync,
// and finally the ambient animation + boat/camera transform + render. This
// is the LAST script tag (see index.html) — the only file allowed to assume
// every other file is fully loaded, since it's what actually kicks off the
// loop and every other file's construction/declarations must already have
// run by the time draw() is first called.
//
// This mirrors the original app.js's single draw() function exactly in
// execution order — the only change is that most of its body now lives in
// named functions in other files instead of being inline here.

let lastNavTime = performance.now();
function draw() {
    const navNow = performance.now();
    const navDt = Math.min((navNow - lastNavTime) / 1000, 0.1); // clamp so a stalled tab doesn't jump
    lastNavTime = navNow;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Current view's scale (global 1.35, or the 3.5x marina zoom used in Mode 3 —
    // see getCurrentViewParams). Every radius/spacing drawn below must use THIS,
    // not the fixed global SCALE, or it renders at the wrong size/place relative
    // to positions (which rosToCanvas already draws using this same view scale) —
    // that mismatch was why the marina looked stranded out in the forest in Mode 3.
    const mapView = getCurrentViewParams();
    const mapScale = mapView.scale;

    renderWorldBackground(mapView, mapScale);

    // Active Navigation Kinematic & ROS Steer Loop (Receding Horizon Live Pathfinder)
    if (isNavigating && plannedPath.length > 0) {
        runNavigationStep(navDt);
        // update3DPathLine() itself is only otherwise called at replan events
        // (a fresh A* route, docking kickoff, Reset) — its arrow's start
        // point is the boat position AT THAT CALL, frozen until the next one.
        // Between replans (up to 250ms, and align/creep don't replan at all)
        // the boat keeps moving every frame while that arrow doesn't, so it
        // visibly lagged behind — up to pointing back at the boat from
        // BEHIND it once the boat drove past its stale anchor, reading as a
        // sudden, dramatic direction change. Refreshing it every frame here
        // keeps its start point (and therefore its whole look) current with
        // wherever the boat actually is right now.
        update3DPathLine();
    }

    renderPlannedPathLine();

    // Update Dynamic Moving Boats Realistic Heavy Ship Kinematics & Buoy Collision
    updateDynamicEntities();

    renderMarina2D(mapScale);
    renderCoastalCity2D();
    renderIsland2D(mapScale);
    renderEntities2D(mapScale);
    renderBoatIcon2D();

    // Sync 3D Scene & FPV Camera
    sync3DEntities();

    // Water stays flat — matches the actual sim, which has no wave field
    // (vrx::Surface here is flat-fluid-level buoyancy only, no vrx::Wavefield
    // plugin is instantiated; see exhibition_water.sdf).
    const time = Date.now() * 0.0015;

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

    // 3D Boat Group Transform
    if (radarMesh) radarMesh.rotation.y += 0.05; // Spin radar scanner

    if (playerBoatCrashed) {
        // Same tilt/submerge look updateDynamicEntities() (navigation.js)
        // already gives a crashed patrol boat (rotation.z 0.8, position.y
        // -0.3), eased in over 1.2s instead of snapped instantly so it
        // reads as sinking rather than teleporting — the reset confirm()
        // dialog is deliberately delayed that long so this gets to play.
        const sinkT = Math.min((Date.now() - playerCrashTime) / 1200, 1);
        boatGroup.position.set(boatPos.x, 0.4 - 0.7 * sinkT, -boatPos.y);
        boatGroup.rotation.y = boatPos.yaw;
        boatGroup.rotation.z = 0.8 * sinkT;
    } else {
        boatGroup.position.set(boatPos.x, 0.4, -boatPos.y);
        boatGroup.rotation.y = boatPos.yaw;
        // Pre-existing behavior, kept as-is: this unconditionally overwrites
        // the roll-bank effect runNavigationStep() sets (navigation.js)
        // every frame, before render — so that roll-bank visually never
        // actually happens. Not a new bug, not fixed here; see the
        // file-split plan's notes.
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
