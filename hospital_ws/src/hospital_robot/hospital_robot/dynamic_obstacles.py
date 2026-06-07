#!/usr/bin/env python3
"""
dynamic_obstacles.py — Moving obstacles in the hospital simulation.

Spawns colored boxes that move through the hospital corridors on
predefined paths. Some move predictably (fixed patrol routes),
others with random pauses/speed changes (unpredictable).

Obstacle types:
  - hospital_cart (blue) — predictable supply cart route
  - visitor (green) — unpredictable walking path with pauses
  - gurney (red) — predictable patient transport between rooms
"""

import rclpy
from rclpy.node import Node
import subprocess
import os
import math
import time
import random
from threading import Lock


# ── Obstacle definitions ────────────────────────────────────────────────
OBSTACLES = [
    {
        'name': 'hospital_cart',
        'color': (0.2, 0.5, 1.0),   # blue
        'size': '0.4 0.25 0.3',
        'predictable': True,
        'waypoints': [
            (0.0, 1.5),      # nurse station
            (9.0, 10.0),     # pharmacy
            (0.0, 1.5),      # back
        ],
        'speed': 0.3,         # m/s
        'pause': 2.0,         # seconds at each waypoint
    },
    {
        'name': 'visitor',
        'color': (0.2, 0.8, 0.3),   # green
        'size': '0.2 0.2 0.3',
        'predictable': False,
        'waypoints': [
            (0.0, -5.5),     # reception
            (-11.0, 0.0),    # patient_room4
            (0.0, -5.5),     # back
            (-11.0, -12.0),  # patient_room5
            (0.0, -5.5),     # back
        ],
        'speed': 0.2,
        'pause': (1.0, 8.0),  # random pause range
    },
    {
        'name': 'gurney',
        'color': (1.0, 0.3, 0.3),   # red
        'size': '0.6 0.3 0.25',
        'predictable': True,
        'waypoints': [
            (11.0, -2.0),    # patient_room1
            (-1.0, -21.0),   # lab
            (11.0, -2.0),    # back
        ],
        'speed': 0.5,
        'pause': 3.0,
    },
]


class DynamicObstacleManager(Node):

    def __init__(self):
        super().__init__('dynamic_obstacle_manager')

        # Path to box model SDF — fallback if package not found
        self._model_dir = None
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_hwb = get_package_share_directory('hospital_world_bridge')
            self._model_dir = os.path.join(pkg_hwb, 'models', 'obstacle_box')
        except Exception:
            pass
        if not self._model_dir:
            self._model_dir = '/tmp/obstacle_box'

        # Generate colored box SDF if not exists
        if self._model_dir and not os.path.exists(os.path.join(self._model_dir, 'model.sdf')):
            self._create_obstacle_models()

        self._lock = Lock()
        self._spawned = {}
        self._waypoint_idx = {}
        self._paused_until = {}
        self._timer = self.create_timer(0.5, self._update)
        self._shutdown = False

        # Spawn obstacles after a delay (let Gazebo stabilize) — one-shot
        self._spawned_once = False
        self.create_timer(15.0, self._spawn_all_once)

        self.get_logger().info(f'Dynamic obstacle manager ready. {len(OBSTACLES)} obstacles defined.')

    def destroy_node(self):
        self._shutdown = True
        super().destroy_node()

    def _create_obstacle_models(self):
        os.makedirs(self._model_dir, exist_ok=True)
        for obs in OBSTACLES:
            r, g, b = obs['color']
            sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="obstacle_box">
    <static>false</static>
    <link name="box_link">
      <visual name="box_visual">
        <geometry><box><size>{obs['size']}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1.0</ambient>
          <diffuse>{r} {g} {b} 1.0</diffuse>
        </material>
      </visual>
      <collision name="box_collision">
        <geometry><box><size>{obs['size']}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>'''
            path = os.path.join(self._model_dir, f"{obs['name']}.sdf")
            if not os.path.exists(path):
                with open(path, 'w') as f:
                    f.write(sdf)

        # Also create model.config
        config = '''<?xml version="1.0"?>
<model>
  <name>Obstacle Box</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
</model>'''
        config_path = os.path.join(self._model_dir, 'model.config')
        if not os.path.exists(config_path):
            with open(config_path, 'w') as f:
                f.write(config)

    def _spawn_all_once(self):
        """One-shot spawn wrapper."""
        if self._spawned_once:
            return
        self._spawned_once = True
        self._spawn_all()

    def _spawn_all(self):
        """Spawn all obstacles at their starting positions."""
        for obs in OBSTACLES:
            name = obs['name']
            x, y = obs['waypoints'][0]
            sdf_path = os.path.join(self._model_dir, f"{name}.sdf")
            if not os.path.exists(sdf_path):
                self.get_logger().warn(f'No SDF for {name}')
                continue

            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'hospital',
                '-name', f'dyn_obs_{name}',
                '-file', sdf_path,
                '-x', str(x), '-y', str(y), '-z', '0.01',
            ]
            try:
                subprocess.run(cmd, capture_output=True, timeout=5)
                with self._lock:
                    self._spawned[name] = (x, y)
                    self._waypoint_idx[name] = 0
                self.get_logger().info(f'Spawned {name} at ({x:.1f}, {y:.1f})')
            except Exception as e:
                self.get_logger().warn(f'Failed to spawn {name}: {e}')

    def _update(self):
        """Move obstacles along their waypoint paths."""
        try:
            self._update_impl()
        except Exception as e:
            self.get_logger().warn(f'Update error: {e}', throttle_duration_sec=10.0)

    def _update_impl(self):
        if self._shutdown:
            return
        now = time.time()

        for obs in OBSTACLES:
            name = obs['name']
            if name not in self._spawned:
                continue

            # Check if paused
            if name in self._paused_until and now < self._paused_until[name]:
                continue

            waypoints = obs['waypoints']
            idx = self._waypoint_idx.get(name, 0)
            current = self._spawned[name]
            target = waypoints[idx]

            dx = target[0] - current[0]
            dy = target[1] - current[1]
            dist = math.hypot(dx, dy)

            speed = obs['speed']
            if not obs['predictable']:
                speed *= random.uniform(0.7, 1.3)  # speed variation

            step = speed * 0.5  # 0.5s timer period

            if dist < 0.15:  # Reached waypoint
                # Move to next waypoint
                next_idx = (idx + 1) % len(waypoints)
                self._waypoint_idx[name] = next_idx

                # Pause at waypoint
                if obs['predictable']:
                    pause = obs['pause'] if isinstance(obs['pause'], (int, float)) else 2.0
                else:
                    lo, hi = obs['pause'] if isinstance(obs['pause'], tuple) else (1.0, 5.0)
                    pause = random.uniform(lo, hi)
                self._paused_until[name] = now + pause

                # Snap to target
                self._spawned[name] = target
                self._move_entity(f'dyn_obs_{name}', target[0], target[1], 0.01)
            else:
                # Move toward target
                new_x = current[0] + (dx / dist) * step
                new_y = current[1] + (dy / dist) * step
                self._spawned[name] = (new_x, new_y)
                self._move_entity(f'dyn_obs_{name}', new_x, new_y, 0.01)

    def _move_entity(self, entity_name, x, y, z):
        """Move a Gazebo entity via gz service."""
        import tempfile
        req_text = (
            f'name: "{entity_name}"\n'
            f'position {{\n'
            f'  x: {x}\n'
            f'  y: {y}\n'
            f'  z: {z}\n'
            f'}}\n'
        )
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
                f.write(req_text)
                tmp = f.name
            subprocess.run([
                'gz', 'service', '-s', '/world/hospital/set_pose',
                '--reqtype', 'gz.msgs.Pose',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '2000',
                '--reqfile', tmp,
            ], capture_output=True, timeout=3)
        except FileNotFoundError:
            # fallback: remove + respawn
            subprocess.run([
                'ros2', 'run', 'ros_gz_sim', 'remove',
                '-world', 'hospital', '-name', entity_name,
            ], capture_output=True, timeout=3)
            # Respawn at new position
            for obs in OBSTACLES:
                if f'dyn_obs_{obs["name"]}' == entity_name:
                    sdf_path = os.path.join(self._model_dir, f"{obs['name']}.sdf")
                    subprocess.run([
                        'ros2', 'run', 'ros_gz_sim', 'create',
                        '-world', 'hospital', '-name', entity_name,
                        '-file', sdf_path,
                        '-x', str(x), '-y', str(y), '-z', str(z),
                    ], capture_output=True, timeout=5)
                    break
        except Exception:
            pass
        finally:
            if 'tmp' in locals():
                try:
                    os.unlink(tmp)
                except OSError:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
