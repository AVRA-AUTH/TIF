// ================= LIFECYCLE / RESET =================
// Reset and mode-switch glue that touches many other modules' state at
// once (nav state, 3D path line, camera-independent boat pose, entities).
// Depends on state.js, navigation.js (update3DPathLine), ros.js
// (cmdVelTopic/spawnTopic), scene-boat.js (boatGroup), scene-marina.js
// (dynamic3DBerthMesh), entities-3d.js (threeEntities), scene-environment.js (scene).

// Stops any nav/dock in progress and snaps the boat to `pose`, both locally
// (so the map/3D view update instantly) and on the REAL Gazebo boat via the
// backend's 'set_pose' handler. odom now drives boatPos unconditionally in
// every mode (see ros.js's odomTopic.subscribe), so without ignoreOdomUntil
// below, boatPos would get overwritten straight back to wherever the boat
// physically still is by the next /odom tick — the backend's teleport is
// async and takes real time to land. Used both by Reset and by every
// mode-switch, so entering a mode never carries over position or
// navigation state from what came before.
function resetBoatToPose(pose) {
    isNavigating = false;
    isAutoDocking = false;
    dockHoldStartTime = null;
    plannedPath = [];
    currentGoal = null;
    dockingTarget = null;
    pathIndex = 0;
    ilosState = { k: 1, y_int: 0 };
    currentLinear = 0.0;
    currentAngular = 0.0;
    shipCollisionAlertShown = false;
    playerBoatCrashed = false;
    recoveryUntil = 0; // isNavigating=false above already guards driveRecovery/driveRecoveryForward, but avoid carrying stale recovery state into the next run regardless
    recoveryForwardUntil = 0;

    boatPos = { x: pose.x, y: pose.y, yaw: pose.yaw, speed: 0 };
    // Give the backend's async 'set_pose' teleport time to actually land in
    // Gazebo before trusting /odom again — otherwise a stale reading from
    // the boat's pre-reset position can win the race and clobber this.
    ignoreOdomUntil = Date.now() + 1000;
    if (boatGroup) {
        boatGroup.position.set(pose.x, 0.4, -pose.y);
        boatGroup.rotation.set(0, pose.yaw, 0);
    }
    update3DPathLine(); // clears any leftover path line from a prior nav/dock run
    if (dynamic3DBerthMesh) dynamic3DBerthMesh.position.set(-220, -10, 0); // hide stale berth marker

    cmdVelTopic.publish(new ROSLIB.Message({
        linear: { x: 0.0, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 }
    }));

    spawnTopic.publish(new ROSLIB.Message({
        data: JSON.stringify({ type: 'set_pose', x: pose.x, y: pose.y, yaw: pose.yaw })
    }));

    // Called on every mode switch and Reset — always leave the Mode-3-only
    // turn-thrust-reserve (see docking.js/cmd_vel_thrust_mixer.py) off
    // outside of an active docking run.
    turnReserveTopic.publish(new ROSLIB.Message({ data: false }));
}

// Clears everything placed in Mode 1 (buoys, moving boats, the goal marker)
// while keeping the ~30 permanent moored boats (isParkedShip) intact — those
// are baked-in marina scenery set up once at page load, not part of a
// design, and never get re-added if wiped. Used by Reset AND by every mode
// switch below, so a design from Mode 1 never silently carries over into
// Mode 2/3 (or into a fresh Mode 1 session) — same "always start clean"
// principle resetBoatToPose already applies to the boat's position.
function clearMode1Design() {
    threeEntities.forEach(mesh => scene.remove(mesh));
    threeEntities.clear();
    entities = entities.filter(ent => ent.isParkedShip);
    currentGoal = null;
}

