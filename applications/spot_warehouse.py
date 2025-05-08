from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import os
from pathlib import Path
import omni.appwindow
import cv2

from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from spot_policy import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
from isaacsim.sensors.camera import Camera

from omni.isaac.core.utils.extensions import enable_extension
#enable_extension('isaacsim.ros2.bridge')

# ROS 2 & sensor_msgs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SpotRunner(object):
    def __init__(self, physics_dt, render_dt) -> None:
        self._world = World(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt)

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        # Spawn warehouse
        prim = define_prim("/World/Warehouse", "Xform")
        asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
        prim.GetReferences().AddReference(asset_path)

        BASE_DIR = Path(__file__).resolve().parent.parent
        policy_path = os.path.join(BASE_DIR, "policies/spot/models", "spot_policy.pt")
        policy_params_path = os.path.join(BASE_DIR, "policies/spot/params", "env.yaml")
        usd_path = os.path.join(BASE_DIR, "assets", "spot.usd")

        self._spot = SpotFlatTerrainPolicy(
            prim_path="/World/Spot",
            name="Spot",
            usd_path=usd_path,
            policy_path=policy_path,
            policy_params_path=policy_params_path,
            position=np.array([1, 0, 0.7]),
        )

        self._base_command = np.zeros(3)
        self._input_keyboard_mapping = {
            "NUMPAD_8": [1.0, 0.0, 0.0], "UP": [1.0, 0.0, 0.0],
            "NUMPAD_2": [-1.0, 0.0, 0.0], "DOWN": [-1.0, 0.0, 0.0],
            "NUMPAD_6": [0.0, -1.0, 0.0], "RIGHT": [0.0, -1.0, 0.0],
            "NUMPAD_4": [0.0, 1.0, 0.0], "LEFT": [0.0, 1.0, 0.0],
            "NUMPAD_7": [0.0, 0.0, 1.0], "N": [0.0, 0.0, 1.0],
            "NUMPAD_9": [0.0, 0.0, -1.0], "M": [0.0, 0.0, -1.0],
        }

        self.needs_reset = False
        self.first_step = True

        # Initialize both cameras
        self.frontleft_camera = Camera(prim_path="/World/Spot/body/frontleft_fisheye", resolution=(640, 480))
        self.frontright_camera = Camera(prim_path="/World/Spot/body/frontright_fisheye", resolution=(640, 480))
        self.frontleft_camera.initialize()
        self.frontright_camera.initialize()
        self.width, self.height = self.frontleft_camera.get_resolution()

        # ROS 2 setup
        rclpy.init()
        self.ros_node = rclpy.create_node("spot_camera_publisher")
        self.cv_bridge = CvBridge()

        self.frontleft_pub = self.ros_node.create_publisher(Image, "/scar/camera/frontleft/image", 10)
        self.frontright_pub = self.ros_node.create_publisher(Image, "/scar/camera/frontright/image", 10)

    def setup(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._sub_keyboard_event
        )
        self._world.add_physics_callback("spot_forward", callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        if self.first_step:
            self._spot.initialize()
            self.first_step = False
        elif self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
            self.first_step = True
        else:
            self._spot.forward(step_size, self._base_command)

    def publish_camera(self, camera: Camera, publisher, frame_id: str):
        rgb = camera.get_rgb()
        if rgb is None or len(rgb) == 0:
            return

        rgb_np = np.array(rgb).reshape(self.height, self.width, 3)
        if rgb_np.dtype == np.float32:
            rgb_np = (rgb_np * 255).astype(np.uint8)

        bgr = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
        msg = self.cv_bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        publisher.publish(msg)

    def run(self) -> None:
        while simulation_app.is_running():
            self._world.step(render=True)

            self.publish_camera(self.frontleft_camera, self.frontleft_pub, "frontleft_camera")
            self.publish_camera(self.frontright_camera, self.frontright_pub, "frontright_camera")

            rclpy.spin_once(self.ros_node, timeout_sec=0.0)

            if self._world.is_stopped():
                self.needs_reset = True

        self.ros_node.destroy_node()
        rclpy.shutdown()
        return

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(self._input_keyboard_mapping[event.input.name])
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(self._input_keyboard_mapping[event.input.name])
        return True


def main():
    physics_dt = 1 / 200.0
    render_dt = 1 / 60.0

    runner = SpotRunner(physics_dt=physics_dt, render_dt=render_dt)
    simulation_app.update()
    runner._world.reset()
    simulation_app.update()
    runner.setup()
    simulation_app.update()
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
