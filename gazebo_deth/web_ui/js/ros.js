// ================= ROS CONNECTION, TOPICS, ODOM =================
// Depends on config.js (topic names use no config, but odom subscription
// updates boatPos/DOF panel from state.js/dof-panel.js — load after both).

// Connect to ROS via roslibjs.
// Local dev (python3 -m http.server) hits rosbridge directly on 9090.
// Anything else (e.g. served through proxy/server.js for a tunnel) goes
// through the same origin's /rosbridge path, so a single tunnel/URL covers
// both the page and the websocket.
const isLocalDev = location.hostname === 'localhost' || location.hostname === '127.0.0.1';
const rosbridgeUrl = isLocalDev
    ? 'ws://localhost:9090'
    : `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/rosbridge`;

const ros = new ROSLIB.Ros({
    url: rosbridgeUrl
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
// Raw per-thruster topics — same ones cmd_vel_thrust_mixer.py publishes to,
// but written directly from the browser for independent thruster control
// (W/A/R/D below), bypassing /cmd_vel. The mixer itself is NOT bypassed by
// that alone — its 20Hz timer keeps running and publishing to these same
// topics regardless of what sends /cmd_vel, so without manualOverrideTopic
// below the two would race on the same topic every tick.
const leftThrustTopic = new ROSLIB.Topic({ ros: ros, name: '/asv_boat/thrusters/left/thrust', messageType: 'std_msgs/Float64' });
const rightThrustTopic = new ROSLIB.Topic({ ros: ros, name: '/asv_boat/thrusters/right/thrust', messageType: 'std_msgs/Float64' });
// Tells cmd_vel_thrust_mixer.py to stop publishing to the thrust topics
// above while independent thruster control (W/A/R/D) is driving them
// directly — see that script's manual_override handling. Without this, the
// mixer's own 20Hz control loop keeps writing to the same two topics from a
// stale /cmd_vel target, fighting whatever W/A/R/D just commanded.
const manualOverrideTopic = new ROSLIB.Topic({ ros: ros, name: '/asv_boat/manual_thrust_override', messageType: 'std_msgs/Bool' });
// Tells cmd_vel_thrust_mixer.py to reserve turning-authority headroom on
// each thruster (see that script's TURN_THRUST_RESERVE_N) — toggled on only
// for Mode 3 autonomous docking runs (docking.js/lifecycle.js), since it
// trades top speed for guaranteed turning at any commanded speed.
const turnReserveTopic = new ROSLIB.Topic({ ros: ros, name: '/asv_boat/turn_reserve_enable', messageType: 'std_msgs/Bool' });
// Mode 2's "FLASH (300%)" speed preset (input.js) — tells
// cmd_vel_thrust_mixer.py to swap its real 51.5N/-40.2N ceiling for the
// unrealistic 154.5N/-120.6N one exhibition_water.sdf's Thruster plugins
// now allow through, for the combined-drive (cmd_vel) control scheme only.
// W/A/R/D's raw thrust-topic path doesn't need this — it bypasses the mixer
// entirely, so it gets the same headroom for free straight from the SDF.
const flashBoostTopic = new ROSLIB.Topic({ ros: ros, name: '/asv_boat/flash_boost_enable', messageType: 'std_msgs/Bool' });
// Listen to boat Odometry — the sole source of boatPos, unconditionally, in
// every mode. This used to be gated to "only while Mode 2 or actively
// navigating" as a minor perf shortcut, but that let boatPos sit frozen on a
// stale/cosmetic value (e.g. right after a reset) while the REAL simulated
// boat was somewhere else entirely (still settling at its old spot, or
// mid-collision) — invisible until the next nav run suddenly synced to the
// true position out from under it. Collision detection below now also runs
// in every mode, so it needs boatPos to be live at all times, not just
// during Mode 2 or an active nav/dock run.
odomTopic.subscribe((msg) => {
    // Drop odom updates for a short window right after resetBoatToPose() —
    // it snaps boatPos locally AND fires an async 'set_pose' to the Gazebo
    // backend, but that physics teleport takes real time to land. Without
    // this guard, an /odom message reporting the boat's OLD (pre-reset)
    // pose could arrive first and clobber the reset right back, before the
    // teleport actually completes — a race that only sometimes loses,
    // which is why it looked like "the right spot, then the wrong one" on
    // different clicks instead of failing consistently.
    if (Date.now() < ignoreOdomUntil) return;

    // twist is in the child_frame (base_link, body frame) per this odometry
    // plugin's config (robot_base_frame: base_link) — linear.x is signed
    // surge speed (forward positive, reverse negative) directly, not a
    // magnitude. Using sqrt(x^2+y^2) here used to erase the sign, making
    // reverse motion indistinguishable from forward on the telemetry panel.
    boatPos.x = msg.pose.pose.position.x;
    boatPos.y = msg.pose.pose.position.y;
    boatPos.speed = msg.twist.twist.linear.x;
    const q = msg.pose.pose.orientation;
    boatPos.yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

    // DOF panel: yaw rate is twist.angular.z (body frame, already signed).
    // Updates whenever boatPos itself is being synced (all modes now),
    // matching the panel's visibility (Part B).
    const yawRate = msg.twist.twist.angular.z;
    updateDofPanel(boatPos.speed, yawRate);
    updateThrustReadout(boatPos.speed);
});
function sendCmdVel(linear, angular) {
    const twist = new ROSLIB.Message({
        linear: { x: linear, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: angular }
    });
    cmdVelTopic.publish(twist);
}

function publishThrust(topic, value) {
    topic.publish(new ROSLIB.Message({ data: value }));
}

