# fmt: off


# preload attrs
import argparse
import os
import arena_simulation_setup
import arena_simulation_setup.utils.cattrs
import signal
import sys

# Use the isaacsim to import SimulationApp
from isaacsim import SimulationApp

# Setting the config for simulation and make an simulation.
CONFIG = {
    # "renderer": "Wireframe",
    "renderer": "RayTracedLighting", # 或者使用 "PathTracing" 获得更高质量
    "headless": False,
}
#import parent directory
import sys
from pathlib import Path

simulation_app = SimulationApp(CONFIG)
# --- 强制压制材质报错刷屏 ---
import carb
log = carb.logging.acquire_logging()
log.set_level_threshold(carb.logging.LEVEL_ERROR) # 仅显示严重错误
settings = carb.settings.get_settings()
settings.set("/log/level", "error")
settings.set("/log/fileLogLevel", "error")
settings.set("/plugins/rtx.materialdb.plugin/logLevel", "fatal") # 彻底杀掉那个插件的输出
# -------------------------
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

extensions.enable_extension("omni.isaac.ros2_bridge")

import random

import numpy as np

#Import world generation dependencies
import omni.anim.graph.core as ag

#imprt navmesh gen
import omni.anim.navigation.core as nav
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
import traceback
from omni.isaac.core.utils.prims import is_prim_path_valid
# from omni.isaac.core.utils.physics import get_physics_context
from .vln_dataset_utils import VLNDataLogger
from vln_dataset_logger_rosbag import VLNDataLoggerRosbag

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
# world = World(
#     physics_dt=1/60.0, 
#     rendering_dt=1/30.0, 
#     stage_units_in_meters=1.0
# )
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

# Navmesh config and baking
simulation_app.update()
stage = omni.usd.get_context().get_stage()

omni.kit.commands.execute("CreateNavMeshVolumeCommand",
                          parent_prim_path=Sdf.Path("/World"),
                          layer=stage.GetRootLayer()
                          )
simulation_app.update()

omni.kit.commands.execute(
    'ChangeSetting',
    path='/exts/omni.anim.navigation.core/navMesh/config/agentRadius',
    value=35.0)

omni.kit.commands.execute(
    'ChangeSetting',
    path='/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled',
    value=True)
omni.kit.commands.execute(
    'ChangeSetting',
    path='/exts/omni.anim.people/navigation_settings/navmesh_enabled',
    value=True)

inav = nav.acquire_interface()
x = inav.start_navmesh_baking()
simulation_app.update()


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

keep_running = True

def main(args=None):
    """
    Main function to initialize the simulation, create the ROS 2 node,
    and run the simulation loop.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--save-data', type=str, default='false', 
                       help='Enable VLN dataset logging')
    parser.add_argument('--log-level', type=str, default='info',
                       help='log level for IsaacSim (debug/info/warn/error)')
    parsed_args = parser.parse_args(args)
    
    enable_logging = parsed_args.save_data.lower() == 'true'
    # apply log level if requested
    try:
        ll = parsed_args.log_level.lower()
        # carb logging thresholds are uppercase
        log.set_level_threshold(getattr(carb.logging, f"LEVEL_{ll.upper()}"))
    except Exception:
        pass

    sim = SimulationContext()

    rclpy.init()

    controller = IsaacController()
    door_manager = DoorManager.instance(controller)
    for service in services:
        service.create(controller, qos_profile=QoSProfile(depth=2000))

    PublishTime('/World/publish_time')
    world.reset()
    world.pause()

    frame_step_counter = 0  # 初始化计数器
    logger = None
    target_camera_path = "/World/Robots/jackal/camera_link/front_camera"
    target_pedestrian_root_path = "/World/Pedestrians"
    target_lidar_path = "/World/Robots/jackal/lidar_link/gpu_lidar"
    # 定义标志位
    has_saved = False
    global keep_running
    # Replicator
    #处理Ctrl+C 退出
    '''def emergency_save_handler(signum, frame):        
        global has_saved
        sys.stderr.write(f"\n[URGENT] 收到终止信号 ({signum})! 正在保存数据...\n")
        sys.stderr.flush()
        
        if not has_saved and logger and len(logger.param_buffer) > 0:
            try:
                # 强制保存
                logger.save_episode()
                has_saved = True  # 标记已保存
                sys.stderr.write("[URGENT] ✅ 数据保存成功！\n")
            except Exception as e:
                sys.stderr.write(f"[URGENT] ❌ 保存失败: {e}\n")
        else:
            sys.stderr.write("[URGENT] Buffer 为空，无需保存。\n")
        
        sys.stderr.flush()
        # 保存完后，手动退出程序
        sys.exit(0)
        signal.signal(signal.SIGINT, emergency_save_handler)
        signal.signal(signal.SIGTERM, emergency_save_handler)
        '''
    
    # ROSBAG
    # 注册信号监听器
    # 监听 SIGINT (Ctrl+C) 和 SIGTERM (launch 系统常用的杀死命令)
    # 同时向 rosbag 转发信号，让其优雅退出
    rosbag_process = None
    def forward_signal_to_rosbag(signum, frame):
        global keep_running, rosbag_process
        sys.stderr.write(f"\n[Signal] 接收到信号 ({signum})，正在申请安全停止...\n")
        if rosbag_process:
            sys.stderr.write(f"[Signal] 向 rosbag (PID {rosbag_process.pid}) 转发 SIGINT...\n")
            rosbag_process.send_signal(signal.SIGINT)
        keep_running = False
    
    signal.signal(signal.SIGINT, forward_signal_to_rosbag)
    signal.signal(signal.SIGTERM, forward_signal_to_rosbag)
    
    sys.stderr.write("[System] 信号拦截器启动，随时准备保存数据。\n")

    # set photoreal settings
    import isaac_utils.config.photoreal as photoreal
    if os.environ.get('RENDER_PRESET', 'photoreal') != 'boring':
        photoreal.PRESET_PHOTOREAL.apply()
    else:
        photoreal.PRESET_DEFAULT.apply()

    # mainloop
    was_playing: bool = False
    try:
        while simulation_app.is_running() and keep_running:
            rclpy.spin_once(controller, timeout_sec=0)
            if controller.running:
                if not was_playing:
                    world.play()
                    was_playing = True
                door_manager.update()
                elevator_manager.update()

                world.step(render=True) # 确保在这一步结束时，画面已经渲染结束
                # 如果enable_logging开启，则开始数据采集和保存的流程
                if enable_logging:
                    # 延迟初始化逻辑：每隔 50 帧检查一次机器人是否出现
                    if logger is None and frame_step_counter % 50 == 0:
                        # 检查相机路径是否存在
                        if is_prim_path_valid(target_camera_path):
                            try:
                                # logger = VLNDataLogger(camera_prim_path=target_camera_path, pedestrian_root_path = target_pedestrian_root_path, lidar_prim_path=target_lidar_path) # utils逻辑
                                logger = VLNDataLoggerRosbag(
                                    topics=[
                                        "/task_generator_node/jackal/odom",
                                        "/task_generator_node/jackal/front_camera/camera_info",
                                        "/task_generator_node/jackal/front_camera/image",
                                        "/task_generator_node/jackal/front_camera/depth",
                                        "/task_generator_node/jackal/lidar/points",
                                        "/task_generator_node/human_states",
                                        "/tf",
                                        "/tf_static",
                                    ],
                                    output_dir="collected_data"
                                )
                                rosbag_process = logger.process  # 保存进程引用
                                logger.start_recording()  # 启动 rosbag 录制
                                controller.get_logger().info('✅ VLNDataLoggerRosbag 初始化成功')
                            except Exception as e:
                                print(f" VLNDataLoggerRosbag 初始化失败: {e}")
                        else:
                            if frame_step_counter % 300 == 0: 
                                print(f" 等待机器人生成... 正在寻找路径: {target_camera_path}")
                    if logger:
                        try:
                            # logger.step(step_idx=frame_step_counter) # utils逻辑
                            frame_step_counter += 1  # 每次采集后自增
                            '''
                            感觉frame_step_counter还需要斟酌 无法确保记录的第0帧数据就是仿真的真实原点
                            存在logger初始化还没完成已经开始运动的可能
                            '''
                        except Exception as e:
                            # 强制把错误堆栈打印到 stderr
                            sys.stderr.write(f"\n🔥 Logger Step 崩溃原因: {e}\n")
                            traceback.print_exception(type(e), e, e.__traceback__, file=sys.stderr)
                            sys.stderr.flush()
                                    
            else:
                if was_playing:
                    world.pause()
                    was_playing = False
                simulation_app.update()

    except KeyboardInterrupt:
        controller.get_logger().info('Received KeyboardInterrupt, shutting down.......')
    except Exception as e:
        controller.get_logger().error(f'Exception in main loop: {e}')
        controller.get_logger().error(traceback.format_exc())
        traceback.print_exc(file=sys.stdout)
    finally:
        # 在程序退出时保存数据
        sys.stderr.write("[Finally] 正常退出流程...\n")
        if enable_logging and logger and not has_saved: #开启数据保存+Logger存在+数据未保存
            sys.stderr.write(f"[SAVE] 正在将 {len(logger.param_buffer)} 帧数据写入磁盘...\n")
            
            # logger.save_episode() # utils逻辑
            logger.stop_recording()
            has_saved = True
            sys.stderr.write("[SAVE] rosbag 已保存完成！\n")
        else:
            sys.stderr.write("[INFO] logger 未初始化或已保存，跳过 rosbag 等待。\n")
            
        if rclpy.ok():
            sys.stderr.write("[Finally] Shutting down ROS 2 node and simulation....\n")
            controller.destroy_node() 
            rclpy.shutdown()
        
        simulation_app.close()


# =================================================================================
if __name__ == "__main__":
    main()
