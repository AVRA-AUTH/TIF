#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import subprocess

# Harbor Fairway Entrance — Mode 3's own start pose (matches MODE3_START /
# DOCK_ENTRANCE_X/Y in web_ui/js/config.js). Used below ONLY as the default
# for an explicit 'set_pose' request that omits x/y/yaw — NOT as a general
# "reset" position; every mode now has its own real start pose
# (MODE1_START/MODE2_START/MODE3_START), and the web UI's own
# resetBoatToPose() (web_ui/js/lifecycle.js) already sends the correct one
# via its own 'set_pose' message before ever publishing a bare 'reset' — see
# that handler below for why it must NOT also reposition the boat itself.
HARBOR_X = -88.0
HARBOR_Y = -38.0
HARBOR_YAW = -1.57

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        self.get_logger().info('Obstacle Spawner Node initialized with Gazebo Level Cleanup.')
        
        self.spawned_ids = set()

        # Subscribe to obstacle placement requests from Web UI
        self.subscription = self.create_subscription(
            String,
            '/exhibition/spawn_obstacle',
            self.spawn_callback,
            10
        )

    def _set_boat_pose(self, x, y, yaw):
        # Yaw-only rotation -> quaternion (roll = pitch = 0)
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        cmd = [
            'gz', 'service',
            '-s', '/world/exhibition_water_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', f'name: "asv_boat" position {{ x: {x} y: {y} z: 0.2 }} orientation {{ w: {qw} z: {qz} }}'
        ]
        subprocess.Popen(cmd)

    def spawn_callback(self, msg):
        try:
            data = json.loads(msg.data)
            obs_type = data.get('type')
            
            # Handle Reset / Level Clear — obstacles only. The boat's pose is
            # deliberately NOT touched here: web_ui/js/input.js's btn-reset
            # handler (the only publisher of a bare 'reset') already calls
            # resetBoatToPose(MODE1_START) — which sends its own correct
            # 'set_pose' message — immediately before publishing this. This
            # used to ALSO reposition the boat, unconditionally, to the
            # Harbor Fairway Entrance (Mode 3's own start pose, not Mode 1's)
            # — a leftover from before per-mode start poses existed. Since
            # both messages arrive here in quick succession, that second,
            # wrong teleport almost always landed AFTER the correct one and
            # silently overrode it: the boat looked like it reset correctly
            # for a moment, then "moved on its own" back to the harbor once
            # the next /odom update reported its real (overridden) pose.
            if obs_type == 'reset':
                self.get_logger().info("🧹 Reset command received! Clearing all Gazebo obstacles...")

                # Remove all spawned obstacles from Gazebo world
                for obs_id in list(self.spawned_ids):
                    cmd = [
                        'gz', 'service',
                        '-s', '/world/exhibition_water_world/remove',
                        '--reqtype', 'gz.msgs.Entity',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '1000',
                        '--req', f'name: "{obs_id}" type: MODEL'
                    ]
                    subprocess.Popen(cmd)

                self.spawned_ids.clear()
                return

            # Remove one previously-spawned obstacle by id — Mode 1's
            # Remove tool / gamepad L1 (web_ui/js/input.js's
            # removeMode1EntityAt()), the single-entity counterpart to the
            # 'reset' branch's remove-everything loop above.
            if obs_type == 'remove':
                obs_id = data.get('id')
                if obs_id in self.spawned_ids:
                    cmd = [
                        'gz', 'service',
                        '-s', '/world/exhibition_water_world/remove',
                        '--reqtype', 'gz.msgs.Entity',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '1000',
                        '--req', f'name: "{obs_id}" type: MODEL'
                    ]
                    subprocess.Popen(cmd)
                    self.spawned_ids.discard(obs_id)
                return

            # Reposition the boat only — used when the web UI switches modes
            # (each mode starts fresh at its own pose) without touching any
            # obstacles already placed in the world.
            if obs_type == 'set_pose':
                x = data.get('x', HARBOR_X)
                y = data.get('y', HARBOR_Y)
                yaw = data.get('yaw', HARBOR_YAW)
                self.get_logger().info(f"↩️  Repositioning boat to x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
                self._set_boat_pose(x, y, yaw)
                return

            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            obs_id = data.get('id', f"obs_{len(self.spawned_ids) + 1}")

            self.spawned_ids.add(obs_id)
            self.get_logger().info(f"Spawning {obs_type} obstacle '{obs_id}' in Gazebo at x={x:.2f}, y={y:.2f}")

            if obs_type == 'static':
                sdf_string = f"""
                <sdf version='1.8'>
                  <model name='{obs_id}'>
                    <pose>{x} {y} 0.5 0 0 0</pose>
                    <static>true</static>
                    <link name='link'>
                      <visual name='visual'>
                        <geometry><cylinder><radius>0.4</radius><length>1.0</length></cylinder></geometry>
                        <material><ambient>1 0.8 0 1</ambient><diffuse>1 0.8 0 1</diffuse></material>
                      </visual>
                      <collision name='collision'>
                        <geometry><cylinder><radius>0.4</radius><length>1.0</length></cylinder></geometry>
                      </collision>
                    </link>
                  </model>
                </sdf>
                """
            else: # dynamic moving boat
                sdf_string = f"""
                <sdf version='1.8'>
                  <model name='{obs_id}'>
                    <pose>{x} {y} 0.3 0 0 0</pose>
                    <link name='link'>
                      <visual name='visual'>
                        <geometry><box><size>1.5 0.8 0.4</size></box></geometry>
                        <material><ambient>0 0.5 1 1</ambient><diffuse>0 0.5 1 1</diffuse></material>
                      </visual>
                      <collision name='collision'>
                        <geometry><box><size>1.5 0.8 0.4</size></box></geometry>
                      </collision>
                    </link>
                  </model>
                </sdf>
                """

            # Call Gazebo spawn service
            cmd = [
                'gz', 'service',
                '-s', '/world/exhibition_water_world/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', f'sdf: "{sdf_string.replace(chr(10), " ")}"'
            ]

            subprocess.Popen(cmd)

        except Exception as e:
            self.get_logger().error(f"Failed to handle spawner request: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
