# fmt: off


# preload attrs
import os
import arena_simulation_setup
import arena_simulation_setup.utils.cattrs

# Use the isaacsim to import SimulationApp
from isaacsim import SimulationApp

# Setting the config for simulation and make an simulation.
CONFIG = {
    "renderer": "Wireframe",
    "headless": False,
}
#import parent directory
import sys
from pathlib import Path

simulation_app = SimulationApp(CONFIG)
parent_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0,str(parent_dir))

# stdlib
import random
import traceback

# Import Isaac Sim dependencies

import carb
import omni.kit.commands as commands
import omni.timeline
import omni.usd
import yaml
from isaac_utils.utils.assets import get_assets_root_path_safe
from omni.importer.urdf import _urdf
from omni.isaac.core import SimulationContext, World
from omni.isaac.core.utils import extensions, prims, stage
from pxr import Sdf

EXTENSIONS_PEOPLE = [
    'omni.anim.people', 
    'omni.anim.navigation.bundle', 
    'omni.anim.timeline',
    'omni.anim.graph.bundle', 
    'omni.anim.graph.core', 
    'omni.anim.graph.ui',
    'omni.anim.retarget.bundle', 
    'omni.anim.retarget.core',
    'omni.anim.retarget.ui', 
    'omni.kit.scripting',
    'omni.graph.nodes',
    'omni.anim.curve.core',
    'omni.anim.navigation.core'
]

EXTENSIONS_MATERIAL = [
    'omni.kit.material.library',
    'omni.kit.browser.material',
    'omni.kit.browser.asset',
    'omni.kit.window.material'
]
for ext_people in EXTENSIONS_PEOPLE:
    extensions.enable_extension(ext_people)

for ext_material in EXTENSIONS_MATERIAL:
    extensions.enable_extension(ext_material)
# Update the simulation app with the new extensions
simulation_app.update()

# -------------------------------------------------------------------------------------------------
# These lines are needed to restart the USD stage and make sure that the people extension is loaded
# -------------------------------------------------------------------------------------------------
omni.usd.get_context().new_stage()

from omni.isaac.core.utils.prims import define_prim

# 显式定义根节点和分类容器，防止服务调用时这些路径不存在
define_prim("/World", "Xform")
define_prim("/World/Walls", "Xform")
define_prim("/World/Doors", "Xform")
define_prim("/World/Floors", "Xform")
define_prim("/World/Obstacles", "Xform")
define_prim("/World/Pedestrians", "Xform")

extensions.enable_extension("omni.isaac.ros2_bridge")

import random

import numpy as np

#Import world generation dependencies
import omni.anim.graph.core as ag

import omni.anim.navigation.core as nav  # kept for omni.anim.people compat, not used for baking
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd

# rclpy
import rclpy
import rclpy.node
import std_srvs.srv

# graphs
from isaac_utils.graphs.time import PublishTime
from isaac_utils.managers.door_manager import DoorManager
from isaac_utils.managers.elevator_manager import elevator_manager

#Import services
from .services import services
from pedestrian.simulator.logic.people_manager import PeopleManager
from rclpy.qos import QoSProfile

# fmt: on
# ======================================Base======================================
# Setting up world and enable ros2_bridge extentions.
# BACKGROUND_STAGE_PATH = "/background"
# BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
plane_material_paths = [
    'https://omniverse-content-production.s3.us-west-2.amazonaws.com/Materials/2023_1/Base/Wood/Walnut_Planks.mdl',
    # 'https://omniverse-content-production.s3.us-west-2.amazonaws.com/Materials/2023_1/vMaterials_2/Ceramic/Ceramic_Tiles_Glazed_Diamond.mdl',
    # 'https://omniverse-content-production.s3.us-west-2.amazonaws.com/Materials/2023_1/vMaterials_2/Ceramic/Ceramic_Tiles_Glazed_Diamond.mdl'
]
world = World()
world.scene.add_ground_plane(size=100, z_position=0.0)
_stage = omni.usd.get_context().get_stage()
plane_mdl_path = random.choice(plane_material_paths)
plane_mtl_name = plane_mdl_path.split('/')[-1][:-4]
plane_mtl_path = "/World/Looks/PlaneMaterial"
plane_mtl = _stage.GetPrimAtPath(plane_mtl_path)
# if not (plane_mtl and plane_mtl.IsValid()):
#     create_res = omni.kit.commands.execute('CreateMdlMaterialPrimCommand',
#                                                 mtl_url=plane_mdl_path,
#                                                 mtl_name=plane_mtl_name,
#                                                 mtl_path=plane_mtl_path)

#     bind_res = omni.kit.commands.execute('BindMaterialCommand',
#                                             prim_path="/World/groundPlane",
#                                             material_path=plane_mtl_path)
simulation_app.update()  # update the simulation once for update ros2_bridge.
simulation_context = SimulationContext(stage_units_in_meters=1.0)  # currently we use 1m for simulation.
light_1 = prims.create_prim(
    "/World/Light_1",
    "DomeLight",
    position=np.array([1.0, 1.0, 1.0]),
    attributes={
        "inputs:texture:format": "latlong",
        "inputs:intensity": 1000.0,
        "inputs:color": (1.0, 1.0, 1.0)
    }
)
assets_root_path = get_assets_root_path_safe()

# navmesh_enabled stays TRUE (default) so omni.anim.people properly registers characters
# and initializes animation graphs (ag.get_character() works). 
# dynamic_avoidance_enabled=False: disables agent-agent collision avoidance (handled by hunav SFM instead)
omni.kit.commands.execute(
    'ChangeSetting',
    path='/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled',
    value=False)
simulation_app.update()


# =================================================================================

# ===========================raycast obstacle publisher============================
# Publishes per-agent obstacle data from PhysX raycasts to hunav.py

import math as _math
import omni.physx
from arena_people_msgs.msg import Pedestrians
from hunav_msgs.msg import Agent, Agents
from geometry_msgs.msg import Point as GeoPoint


class RaycastObstaclePublisher(rclpy.node.Node):
    """Runs in Isaac Sim process: casts PhysX rays around each pedestrian
    and publishes obstacle hits to hunav.py's _obstacle_subscriber.

    Mirrors hunav_isaac_wrapper's get_closest_obstacles() approach:
      - 36 rays covering 360° (wrapper uses 90; 36 balances accuracy/perf)
      - 5 sensor heights to catch low/mid/high wall geometry
      - uses hit['position'] directly (exact world-space hit point)
    """

    NUM_RAYS = 36
    RAY_DISTANCE = 4.0                         # metres (same as wrapper)
    SENSOR_HEIGHTS = [0.05, 0.1, 0.25, 0.5, 1.0]  # metres above ped base Z

    def __init__(self, peds_topic: str, obstacles_topic: str):
        super().__init__('raycast_obstacle_publisher')
        self._publisher = self.create_publisher(Agents, obstacles_topic, 10)
        self._subscriber = self.create_subscription(
            Pedestrians, peds_topic, self._peds_callback, 10
        )
        self._physx_query = omni.physx.get_physx_scene_query_interface()
        self.get_logger().info(
            f'RaycastObstaclePublisher: peds={peds_topic}, obs={obstacles_topic}'
        )

    def _peds_callback(self, msg: Pedestrians):
        result = Agents()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = 'map'

        angle_step = 2.0 * _math.pi / self.NUM_RAYS

        for ped in msg.pedestrians:
            agent = Agent()
            agent.name = ped.name

            ox = ped.pose.position.x
            oy = ped.pose.position.y
            oz = ped.pose.position.z  # sensor heights added per-ray below

            for i in range(self.NUM_RAYS):
                angle = angle_step * i
                dx = _math.cos(angle)
                dy = _math.sin(angle)

                best_dist = self.RAY_DISTANCE
                best_pos = None

                # Cast at each sensor height; keep the closest hit
                for h in self.SENSOR_HEIGHTS:
                    hit = self._physx_query.raycast_closest(
                        carb.Float3(ox, oy, oz + h),
                        carb.Float3(dx, dy, 0.0),
                        self.RAY_DISTANCE,
                    )
                    if hit and hit.get('hit', False):
                        d = hit.get('distance', self.RAY_DISTANCE)
                        if d < best_dist:
                            best_dist = d
                            best_pos = hit.get('position')  # exact world-space hit

                if best_pos is not None:
                    pt = GeoPoint()
                    pt.x = float(best_pos[0])
                    pt.y = float(best_pos[1])
                    pt.z = float(best_pos[2])
                    agent.closest_obs.append(pt)

            result.agents.append(agent)

        self._publisher.publish(result)


# =================================================================================

# ===================================controller====================================
# create controller node for isaacsim.


class IsaacController(rclpy.node.Node):
    def __init__(self, *args, **kwargs):
        super().__init__(node_name="isaac", *args, **kwargs)
        self._running = False
        self._should_step_once = False

        self.__pause_srv = self.create_service(
            std_srvs.srv.Trigger,
            os.path.join('isaac/PauseSimulation'),
            self._cb_pause,
        )
        self.__unpause_srv = self.create_service(
            std_srvs.srv.Trigger,
            os.path.join('isaac/UnpauseSimulation'),
            self._cb_unpause,
        )
        self.__step_srv = self.create_service(
            std_srvs.srv.Trigger,
            os.path.join('isaac/StepSimulation'),
            self._cb_step,
        )

    def _cb_pause(self, request: std_srvs.srv.Trigger.Request, response: std_srvs.srv.Trigger.Response):
        self._running = False
        response.success = True
        return response

    def _cb_unpause(self, request: std_srvs.srv.Trigger.Request, response: std_srvs.srv.Trigger.Response):
        self._running = True
        response.success = True
        return response

    def _cb_step(self, request: std_srvs.srv.Trigger.Request, response: std_srvs.srv.Trigger.Response):
        self._should_step_once = True
        response.success = True
        return response

    @property
    def _step_once(self) -> bool:
        v = self._should_step_once
        self._should_step_once = False
        return v

    @property
    def running(self):
        return self._step_once or self._running


# ======================================main=======================================


def main(args=None):
    """
    Main function to initialize the simulation, create the ROS 2 node,
    and run the simulation loop.
    """

    sim = SimulationContext()

    rclpy.init()

    controller = IsaacController()
    door_manager = DoorManager.instance(controller)
    for service in services:
        service.create(controller, qos_profile=QoSProfile(depth=2000))

    # RaycastObstaclePublisher: publishes PhysX raycast hits to hunav's obstacle subscriber
    # Topic names must match: peds published by hunav.py, obstacles consumed by hunav.py
    raycast_pub = RaycastObstaclePublisher(
        peds_topic='/task_generator_node/arena_peds',
        obstacles_topic='/task_generator_node/hunav_closest_obstacles',
    )

    PublishTime('/World/publish_time')
    world.reset()
    world.pause()

    # set photoreal settings
    import isaac_utils.config.photoreal as photoreal
    if os.environ.get('RENDER_PRESET', 'photoreal') != 'boring':
        photoreal.PRESET_PHOTOREAL.apply()
    else:
        photoreal.PRESET_DEFAULT.apply()

    # mainloop
    was_playing: bool = False
    try:
        while simulation_app.is_running():
            rclpy.spin_once(controller, timeout_sec=0)
            rclpy.spin_once(raycast_pub, timeout_sec=0)
            if controller.running:
                if not was_playing:
                    world.play()
                    was_playing = True
                door_manager.update()
                elevator_manager.update()
                world.step(render=True)
            else:
                if was_playing:
                    world.pause()
                    was_playing = False
                simulation_app.update()

    except KeyboardInterrupt:
        controller.get_logger().info('Received KeyboardInterrupt, shutting down.')
    except Exception as e:
        controller.get_logger().error(f'Exception in main loop: {e}')
        controller.get_logger().error(traceback.format_exc())
        traceback.print_exc(file=sys.stdout)
    finally:
        controller.get_logger().info('Shutting down ROS 2 node and simulation.')
        controller.destroy_node()
        if raycast_pub is not None:
            raycast_pub.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


# =================================================================================
if __name__ == "__main__":
    main()
