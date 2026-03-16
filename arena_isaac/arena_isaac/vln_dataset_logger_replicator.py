import omni.replicator.core as rep
import numpy as np
import pandas as pd
import imageio.v3 as iio
import os
import sys
import json
import time
import subprocess
import signal

from omni.isaac.core.utils.prims import get_prim_at_path, get_all_matching_child_prims, is_prim_path_valid
from scipy.spatial.transform import Rotation as R
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core import SimulationContext
# from mcap_ros2.ros2_decoding import DecoderFactory
from mcap_ros2.writer import Writer as Ros2Writer

import rclpy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

try:
    from mcap.writer import Writer
    HAS_MCAP = True
except ImportError:
    HAS_MCAP = False

class VLNDataLogger:
    def __init__(self, camera_prim_path, pedestrian_root_path, lidar_prim_path, output_dir="collected_data"):
        """
        camera_prim_path: 你的相机在 USD 中的路径
        output_dir: 数据保存路径
        """
    
        # 检查 Prim 是否真的存在
        if not is_prim_path_valid(camera_prim_path):
            sys.stderr.write(f"❌ [CRITICAL] 路径无效! 请在 Stage 中确认路径是否正确。\n")
        else:
            sys.stderr.write(f"✅ [Logger] 尝试挂载相机路径: {camera_prim_path}\n")

        self.output_dir = output_dir
        self.episode_idx = 0
        self.param_buffer = []     # 存元数据 (Pose, Intrinsics)
        self.rgb_frame_buffer = [] # 存视频帧 (RGB)
        self.depth_frame_buffer = [] # 存视频帧 (Depth)
        
        # 创建 Render Product (连接到现有的相机 Prim)
        self.camera_rp = rep.create.render_product(camera_prim_path, (1280, 720)) #(640, 480)
        # 注册 Annotators (数据提取器)
        self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        self.depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        self.cam_params_annot = rep.AnnotatorRegistry.get_annotator("camera_params")

        # 绑定相机数据
        self.rgb_annot.attach(self.camera_rp)
        self.depth_annot.attach(self.camera_rp)
        self.cam_params_annot.attach(self.camera_rp)

        # # 注册并创建 LIDAR Annotator（点云数据）
        '''self.lidar_annot = None
        self.lidar_rp = None
        try:
            # 获取 LIDAR annotator
            self.lidar_annot = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacComputeRTXLidarPointCloud")
            # 创建 LIDAR 的 render product（如果有独立的 LIDAR 传感器）            
            if is_prim_path_valid(lidar_prim_path):
                self.lidar_rp = rep.create.render_product(lidar_prim_path, (1, 1))
                self.lidar_annot.attach(self.lidar_rp)
                sys.stderr.write(f"✅ LIDAR Annotator 已初始化: {lidar_prim_path}\n")
            else:
                sys.stderr.write(f"⚠️ LIDAR 路径不存在: {lidar_prim_path}，跳过点云采集\n")
                self.lidar_annot = None
        except Exception as e:
            sys.stderr.write(f"⚠️ LIDAR Annotator 初始化失败: {e}\n")
            self.lidar_annot = None'''

        # 确保目录存在
        os.makedirs(os.path.join(self.output_dir, "data"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "rgb_videos"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "depth_videos"), exist_ok=True)
        # os.makedirs(os.path.join(self.output_dir, "ply"), exist_ok=True)

        # 初始化行人列表，在 initialize_pedestrians() 中动态扫描
        self.pedestrian_prims = []
        self.pedestrian_root_path = pedestrian_root_path
        
        # MCAP
        '''# 初始化 MCAP LIDAR 点云保存器相关属性
        self._lidar_writer = None
        self._lidar_file = None
        self._lidar_channel_id = None
        self._pc2_schema_id = None
        self._lidar_message_count = 0
        # 调用内部初始化例程，如果 mcap 库不可用会打印警告
        self._init_lidar_writer()'''

        # json
        '''# 代码会自动把路径拼成：my_lidar_logs/lidar_episode_000.mcap
        output_path = os.path.join(self.output_dir, f"lidar_episode_{self.episode_idx:06d}.mcap")
        self.f = open(output_path, "wb")
        self.writer = Writer(self.f)
        self.writer.start() 

        # JSON 格式 注册 Schema 
        # 我们定义一个简单的结构：{"points": [[x,y,z], ...], "frame_id": "..."}
        self.schema_id = self.writer.register_schema(
            name="pointcloud_json",
            encoding="jsonschema",
            data=json.dumps({
                "type": "object",
                "properties": {
                    "points": {"type": "array", "items": {"type": "array", "minItems": 3, "maxItems": 3}},
                    "frame_id": {"type": "string"}
                }
            }).encode()
        )

        # 2. 注册 Channel
        self.channel_id = self.writer.register_channel(
            topic="/lidar/json_points",
            message_encoding="json",
            schema_id=self.schema_id,
        )'''
        # rosbag
        # 初始化时自动开启录制
        self.lidar_topic = "/task_generator_node/jackal/lidar/points"
        self.record_process = None
        self._start_rosbag_record()

    def json_write_points(self, points, sim_time, frame_id="jackal/base_link"):
            # 将 numpy 转换为 list 才能被 JSON 序列化
            points_list = points.tolist()
            
            data = {
                "points": points_list,
                "frame_id": frame_id,
                "timestamp": sim_time
            }
            
            # 写入消息
            # log_time 和 publish_time 使用纳秒
            ns_time = int(sim_time * 1e9)
            self.writer.add_message(
                channel_id=self.channel_id,
                log_time=ns_time,
                data=json.dumps(data).encode("utf-8"),
                publish_time=ns_time
            )

    def json_close(self):
        self.writer.finish()
        self.f.close()
        sys.stderr.write(f"✅ MCAP (JSON模式) 已成功封存\n")

    def _init_lidar_writer(self):
        """开启一个新的 MCAP 文件并设置 Foxglove 点云通道"""
        if not HAS_MCAP:
            sys.stderr.write("⚠️ mcap 库未安装，点云保存功能不可用\n")
            return
        try:
            # 为每个 episode 创建独立的文件名
            lidar_output_path = os.path.join(self.output_dir, f"lidar_episode_{self.episode_idx:06d}.mcap")
            os.makedirs(os.path.dirname(lidar_output_path) or ".", exist_ok=True)
            
            self._lidar_file = open(lidar_output_path, "wb")
            self._lidar_ros_writer = Ros2Writer(self._lidar_file)

            self._lidar_channel_id = self._lidar_ros_writer.register_msg_channel(
                    topic="/jackal/lidar_points",
                    msg_type="sensor_msgs/msg/PointCloud2",
                    frame_id="jackal/base_link" # 根据你的图纸
                )
            self._lidar_writer = self._lidar_ros_writer
            # self._lidar_writer = Writer(self._lidar_file)
            # self._lidar_writer.start()
            # # 注册 PointCloud2 schema 和频道
            # self._pc2_schema_id = self._lidar_writer.register_schema(
            #     name="sensor_msgs/msg/PointCloud2",
            #     encoding="ros2msg",
            #     data=b"",
            # )
            # self._lidar_channel_id = self._lidar_writer.register_channel(
            #     schema_id=self._pc2_schema_id,
            #     topic="/jackal/lidar_points",
            #     message_encoding="cdr",
            # )

            sys.stderr.write(f"✅ MCAP LIDAR Writer 初始化成功: {lidar_output_path}\n")
            self._lidar_message_count = 0  # 计数器用于调试
        except Exception as e:
            sys.stderr.write(f"⚠️ MCAP LIDAR Writer 初始化失败: {e}\n")
            self._lidar_writer = None

    '''def _lidar_write_point_cloud(self, sim_time, points):
        """将当前帧的点云写入 MCAP"""
        if self._lidar_writer is None:
            return
        try:
            ns_time = int(sim_time * 1e9)
            if points is None or points.size == 0:
                return
            # 确保点云是 numpy 数组且为 float32
            if isinstance(points, np.ndarray):
                pc_data = points.astype(np.float32).tobytes()
            else:
                pc_data = np.array(points, dtype=np.float32).tobytes()
            
            self._lidar_writer.add_message(
                channel_id=self._lidar_channel_id,
                log_time=ns_time,
                publish_time=ns_time,
                data=pc_data,
            )
            self._lidar_message_count = getattr(self, '_lidar_message_count', 0) + 1
        except Exception as e:
            sys.stderr.write(f"⚠️ LIDAR 点云写入失败: {e}\n")'''

    def _lidar_write_point_cloud(self, sim_time, points):
        """将当前帧的点云包装为 ROS 2 PointCloud2 格式并写入 MCAP"""
        if self._lidar_writer is None:
            return

        try:
            # 1. 基础检查
            if points is None or points.size == 0:
                return
            
            # 确保数据格式为 float32 (ROS 2 PointCloud2 标准)
            points_f32 = points.astype(np.float32)
            
            # 2. 构造 ROS 2 消息头部
            header = Header()
            header.frame_id = "jackal/base_link"  # 必须与你 Foxglove 里的坐标系对应
            
            # 将仿真秒转换为 ROS 2 时间戳 (sec, nanosec)
            seconds = int(sim_time)
            nanoseconds = int((sim_time - seconds) * 1e9)
            header.stamp.sec = seconds
            header.stamp.nanosec = nanoseconds

            # 3. 利用官方工具函数创建 PointCloud2 消息
            # 这会自动处理 fields (x, y, z), point_step, row_step 等复杂参数
            msg = pc2.create_cloud_xyz32(header, points_f32)

            # 4. 【核心步骤】将 ROS 2 消息对象序列化为 CDR 字节流
            # 这是 Foxglove 识别 "message_encoding='cdr'" 的关键
            serialized_msg = serialize_message(msg)

            # 5. 写入 MCAP
            # log_time 和 publish_time 统一使用纳秒
            ns_time = int(sim_time * 1e9)
            self._lidar_ros_writer.write_message(
            topic="/jackal/lidar_points",
            message=msg,
            log_time=ns_time,
            publish_time=ns_time
            )
            # self._lidar_writer.add_message(
            #     channel_id=self._lidar_channel_id,
            #     log_time=ns_time,
            #     publish_time=ns_time,
            #     data=serialized_msg,
            # )
            
            self._lidar_message_count = getattr(self, '_lidar_message_count', 0) + 1
            if self._lidar_message_count % 10 == 0:
                print(f"已成功向 MCAP 写入 {self._lidar_message_count} 帧点云")
        except Exception as e:
            sys.stderr.write(f"⚠️ LIDAR 包装或写入失败: {e}\n")

    def _lidar_close(self):
        """结束当前 MCAP 会话并关闭文件"""
        if hasattr(self, "_lidar_ros_writer") and self._lidar_ros_writer:
            try:
                msg_count = getattr(self, '_lidar_message_count', 0)
                self._lidar_ros_writer.finish()
                self._lidar_file.close()
                self._lidar_ros_writer = None
                self._lidar_file = None
                sys.stderr.write(f"[Save] LIDAR: 写入 {msg_count} 条点云消息\n")
            except Exception as e:
                sys.stderr.write(f"[Save] 关闭 LIDAR 文件失败: {e}\n")

    def save_points_as_ply(self, points, filename):
        """
        将 numpy 数组 (N, 3) 保存为 PLY 文件
        """
        if points is None or len(points) == 0:
            sys.stderr.write(f"⚠️ 警告: {filename} 数据为空，跳过保存。\n")
            return

        # 确保是 float32
        points = points.astype(np.float32)
        
        # PLY 文件头
        header = f"""ply
        format ascii 1.0
        element vertex {len(points)}
        property float x
        property float y
        property float z
        end_header
        """
        # 写入文件
        with open(filename, 'w') as f:
            f.write(header)
            # 将坐标逐行写入
            np.savetxt(f, points, fmt='%f %f %f')

        sys.stderr.write(f"✅ 已保存单帧点云至: {filename}\n")

    def _start_rosbag_record(self):
            """在后台启动 ros2 bag 录制进程"""
            os.makedirs(self.output_dir, exist_ok=True)
            
            # 定义一个不重复的 bag 文件夹名
            bag_name = f"episode_{int(time.time())}"
            self.bag_path = os.path.join(self.output_dir, bag_name)
            
            # 构造 ROS 2 录制命令
            # -s mcap: 指定保存为 mcap 格式
            # -o <path>: 指定保存路径
            # 强烈建议同时录制 /tf 和 /tf_static，这样 Foxglove 就能直接识别坐标系！
            cmd = [
                "ros2", "bag", "record",
                "-s", "mcap",
                "-o", self.bag_path,
                self.lidar_topic,
                "/tf", 
                "/tf_static"
            ]
            
            sys.stderr.write(f"🚀 [Logger] 正在启动后台录制: {' '.join(cmd)}\n")
            
            # 使用 Popen 启动后台进程
            self.record_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL, # 隐藏正常的输出刷屏
                stderr=subprocess.PIPE     # 保留错误信息以便排查
            )
            
            # ⚠️ 极其关键的一步：防止漏帧
            # ros2 bag 启动和寻找 Topic 需要大概 0.5~1 秒钟
            # 如果不等待，仿真器直接开始跑，前几帧数据就会丢失
            time.sleep(1.5)
            sys.stderr.write(f"✅ [Logger] 录制进程已就绪，开始采集数据！\n")

    def _rosbag_close(self):
        """安全关闭录制进程（相当于在终端按 Ctrl+C）"""
        if self.record_process and self.record_process.poll() is None:
            print(f"🛑 [Logger] 准备封存 MCAP 文件...")
            
            # 向子进程发送 SIGINT 信号（等同于 Ctrl+C）
            # rosbag 收到此信号后会执行 finish() 操作，确保文件索引不损坏
            self.record_process.send_signal(signal.SIGINT)
            
            try:
                # 等待进程安全退出，最多等 5 秒
                self.record_process.wait(timeout=5.0)
                print(f"💾 [Logger] MCAP 文件已安全保存至: {self.bag_path}")
            except subprocess.TimeoutExpired:
                print("⚠️ [Logger] 录制进程无响应，正在强制终止！")
                self.record_process.kill()

    def find_bone_root_recursive(self, prim):
        """递归遍历子节点，寻找包含 'RL_BoneRoot' 的路径"""
        if "RL_BoneRoot" in prim.GetName():
            return prim.GetPath().pathString
        for child in prim.GetChildren():
            res = self.find_bone_root_recursive(child)
            if res:
                return res
        return None

    def initialize_pedestrians(self):
        """
        专门负责在环境准备好后，扫描并绑定所有行人的移动节点
        """
        pedestrian_root_prim = get_prim_at_path(self.pedestrian_root_path)
        if not pedestrian_root_prim:
            # 环境还没加载好，直接返回，等下一帧 step 再试
            sys.stderr.write(f"\n🧋 行人未加载完全\n")
            return

        # 获取 Pedestrian_0, Pedestrian_1...
        child_paths = [c.GetPath().pathString for c in pedestrian_root_prim.GetChildren()]
        if not child_paths:
            return
            
        child_paths.sort()
        temp_list = []
        
        for ped_path in child_paths:
            # 递归寻找真正带动画位移的节点
            bone_path = self.find_bone_root_recursive(get_prim_at_path(ped_path))
            
            if bone_path:
                # 只有当路径确实存在时，才包装成 XFormPrim
                temp_list.append(XFormPrim(bone_path))
                sys.stderr.write(f"\n✅ 成功绑定动态节点: {bone_path}\n")
            else:
                # 找不到骨骼根节点时，至少绑定到 Pedestrian_x，保证数据流不中断
                temp_list.append(XFormPrim(ped_path))
                sys.stderr.write(f"\n❌ 使用根节点作为回退: {ped_path}\n")

        self.pedestrian_prims = temp_list

    def process_camera_data(self,params, width, height):
        """
        解析、清洗并标准化相机参数
        :param params: replicator 返回的 params 字典
        :param width: 图像宽度 (像素)
        :param height: 图像高度 (像素)
        :return: (清洗后的Pose列表, 清洗后的Intrinsics列表)
        """
        
        # -----------------------------
        # 1. 处理位姿 (Pose)
        # -----------------------------
        # Isaac Sim 返回的是 World-to-Camera (View Matrix)
        # 并且是 Row-Major (行优先)
        view_matrix = params['cameraViewTransform'].reshape(4, 4)
        
        # 求逆得到 Camera-to-World (Pose Matrix)
        # 此时它仍然是 Row-Major
        pose_matrix_row_major = np.linalg.inv(view_matrix)
        
        # 【关键】转置！变成 Column-Major (列优先)
        # 这样最后一列才是位移 [x, y, z, 1]
        pose_matrix_col_major = pose_matrix_row_major.T
        
        # 【清洗】保留6位小数 (足以满足机器人导航精度，同时把 1e-19 变成 0)
        # 这一步会把 0.9999999 -> 1.0, 1.2e-19 -> 0.0
        pose_clean = np.round(pose_matrix_col_major, decimals=6)
        
        # -----------------------------
        # 2. 处理内参 (Intrinsics)
        # -----------------------------
        # 获取 4x4 投影矩阵
        proj_4x4 = params['cameraProjection'].reshape(4, 4)
        
        # 投影矩阵的 [0,0] 对应 X轴焦距归一化因子
        # 投影矩阵的 [1,1] 对应 Y轴焦距归一化因子
        # 注意：Isaac Sim/OpenGL 的投影矩阵里，f = 1 / tan(fov/2)
        # 换算公式：fx = P[0,0] * width / 2
        
        fx = proj_4x4[0, 0] * width / 2.0
        fy = proj_4x4[1, 1] * height / 2.0
        
        # 主点 (Principal Point) 通常位于图像中心
        cx = width / 2.0
        cy = height / 2.0
        
        # 构建标准的 3x3 内参矩阵 K
        K = np.array([
            [fx,  0.0, cx],
            [0.0, fy,  cy],
            [0.0, 0.0, 1.0]
        ])
        
        # 【内参也保留6位小数
        K_clean = np.round(K, decimals=6)
        
        # camera_sim_time = SimulationContext.get_instance().current_time

        return pose_clean.flatten().tolist(), K_clean.flatten().tolist()

    def process_depth_for_video(self, depth_data, max_dist=30.0):
        """
        将 float32 深度图转换为可以保存为视频的 uint8 RGB 格式
        """
        # 1. 将 Inf (天空) 替换为最大距离，将 NaN 替换为 0
        depth_data = np.nan_to_num(depth_data, nan=0.0, posinf=max_dist, neginf=0.0)
        
        # 2. 裁剪数值范围到 [0, max_dist]
        depth_data = np.clip(depth_data, 0, max_dist)
        
        # 3. 线性映射：从 [0, max_dist] 映射到 [0, 255]
        # 注意：如果你希望近处黑、远处白，用这个公式；反之则用 255 - (...)
        depth_norm = (depth_data / max_dist) * 255.0
        
        # 4. 转换数据类型为 uint8
        depth_uint8 = depth_norm.astype(np.uint8)
        
        # 5. 视频编码器通常喜欢 3 通道数据
        # 将 (H, W) 堆叠成 (H, W, 3) 的灰度图
        depth_rgb = np.stack([depth_uint8] * 3, axis=-1)
    
        return depth_rgb
    
    def get_pedestrian_state(self):
        """
        返回所有人的 4x4 矩阵。如果尚未初始化，则尝试动态初始化。
        """
        # 当前仿真时间
        # sim_time = SimulationContext.get_instance().current_time

        if not self.pedestrian_prims:
            self.initialize_pedestrians()
        
        # 如果搜索后依然找不到路径（场景里真的没人），返回占位符防止 Parquet 报错
        if not self.pedestrian_prims:
            return {"none": [0.0] * 16}
            # return {"none": [0.0] * 16}, sim_time
        
        curr_peds_dict = {}
        for i, xform_prim in enumerate(self.pedestrian_prims):
            label = f"p_{i+1}"
            
            # 获取位姿
            position, orientation = xform_prim.get_world_pose()
            
            # 四元数 [x, y, z, w] 转旋转矩阵
            r = R.from_quat([orientation[1], orientation[2], orientation[3], orientation[0]])
            rotation_matrix = r.as_matrix()
            
            # 构建 4x4 矩阵
            T = np.eye(4, dtype=np.float32)
            T[:3, :3] = rotation_matrix
            T[:3, 3] = position
            
            # 保留6位小数精度，展平为列表
            T_flat = np.round(T, decimals=6).flatten().tolist()
            
            # 存入字典
            curr_peds_dict[label] = T_flat
            
        return curr_peds_dict
    
    def step(self, step_idx, language_instruction="navigate"):
        """
        在 env.step() 之后调用此函数
        """

        # 预初始化变量，防止 UnboundLocalError
        rgb = None
        params = None
        curr_ped_pos = None
        width = 1280 # 你设置的分辨率
        height = 720

        # 尝试获取数据
        try:
            rgb = self.rgb_annot.get_data()
            params = self.cam_params_annot.get_data()
            depth = self.depth_annot.get_data()
            curr_ped_pos = self.get_pedestrian_state()
            
            # 获取 LIDAR 点云数据
            '''lidar_points = None
            if self.lidar_annot is not None:
                lidar_data = self.lidar_annot.get_data()

                if lidar_data is not None: # 每10帧打印一次点云数据的调试信息
                    # 点云数据通常是 [N, 3] 的坐标数组
                    points = lidar_data.get("data", np.array([]))

                    # 存ply
                    # if step_idx % 100 == 0:
                    #     self.save_points_as_ply(points, f"step_{step_idx:06d}.ply")
                    # lidar_points = points.astype(np.float32) 

                else :
                    sys.stderr.write(f"[DEBUG] lidar_data 为 None.\n")
                    
            else:
                sys.stderr.write(f"[DEBUG] ⚠️ lidar_annot 为 None\n")'''
                        
        except Exception as e:
            sys.stderr.write(f"获取数据失败: {e}\n")
            return

        # 检查数据完整性
        if rgb is None or depth is None or params is None:
            sys.stderr.write(f"数据缺失: RGB={rgb is None}, Params={params is None}\n")
            sys.stderr.flush()
            return
        
        if curr_ped_pos is None: 
            sys.stderr.write(f"行人数据缺失。\n")
            sys.stderr.flush()

        # 处理 RGB (去除 Alpha 通道)
        if rgb.shape[2] == 4:
            rgb = rgb[..., :3]
        # 处理深度图：转换为 0-255 uint8 RGB 格式（10m 以外为纯白）
        if depth is not None and depth.size > 0:
            depth_processed = self.process_depth_for_video(depth, max_dist=10.0)
        
        try:
            pose_list, intrinsics_list = self.process_camera_data(
                self.cam_params_annot.get_data(),
                width, 
                height
            )

            # self.buffer.append({
            #     "episode_index": self.episode_idx,
            #     "frame_index": step_idx,
            #     # "timestamp": step_idx * (1.0/30.0), # 假设30FPS
            #     # "instruction": language_instruction,
            # })

            self.rgb_frame_buffer.append(rgb)
            self.depth_frame_buffer.append(depth_processed)
            self.param_buffer.append({
                "frame_index": step_idx, 
                "observation.camera_intrin": intrinsics_list,               
                "observation.camera_state": pose_list,
                "observation.peds_state": curr_ped_pos,
            })
            
            # 保存 MCAP
            '''if lidar_points is not None:
                try:
                    if step_idx == 0 or step_idx % 10 == 0:
                        if isinstance(lidar_points, np.ndarray):
                            sys.stderr.write(f"[DEBUG] 准备写入 LIDAR数据: shape={lidar_points.shape}, dtype={lidar_points.dtype}, size={lidar_points.size}\n")
                    
                    sim_time = SimulationContext.instance().current_time
                    self._lidar_write_point_cloud(sim_time, lidar_points)
                except Exception as lidar_save_err:
                    sys.stderr.write(f"⚠️ 保存 LIDAR 点云失败: {lidar_save_err}\n")'''

            # #保存 JSON
            # sim_time = SimulationContext.instance().current_time
            # self.json_write_points(points, sim_time, frame_id="jackal/base_link")

            # === 确认缓存增加了 ===
            if len(self.param_buffer) % 10 == 0:
                sys.stderr.write(f"✓ 捕获帧数: {len(self.param_buffer)}\n")
                sys.stderr.flush()
        except Exception as e:
            sys.stderr.write(f"缓存数据出错: {e}\n")

        return

    def save_episode(self):
        """
        在 episode 结束时调用
        """
        sys.stderr.write(f"\n[Save] 触发保存函数。Chunk Buffer 大小: {len(self.param_buffer)}\n")
        sys.stderr.flush()
        if not self.param_buffer:
            sys.stderr.write("[Save] 警告: Buffer 为空，放弃写入文件。\n")
            sys.stderr.flush()
            return

        # 保存视频和元数据
        rgb_video_path = os.path.join(self.output_dir, f"rgb_videos/episode_{self.episode_idx:06d}.mp4")
        iio.imwrite(rgb_video_path, np.stack(self.rgb_frame_buffer), fps=30, codec="libx264")
        
        depth_video_path = os.path.join(self.output_dir, f"depth_videos/episode_{self.episode_idx:06d}.mp4")
        iio.imwrite(depth_video_path, np.stack(self.depth_frame_buffer), fps=30, codec="libx264")

        df = pd.DataFrame(self.param_buffer)
        parquet_path = os.path.join(self.output_dir, f"data/chunk_{self.episode_idx:06d}.parquet")
        df.to_parquet(parquet_path)
        
        self._start_rosbag_record()

        # 保存并关闭 JSON
        # self.json_close()
        # 保存并关闭 MCAP 点云文件
        '''if getattr(self, "_lidar_writer", None) is not None:
            try:
                self._lidar_close()
                sys.stderr.write(f"[Save] ✅ LIDAR 点云已保存\n")
            except Exception as e:
                sys.stderr.write(f"[Save] ❌ 关闭 LIDAR 文件失败: {e}\n")
            # 重新初始化以备下一个 episode
            if HAS_MCAP:
                self._init_lidar_writer()
        
        sys.stderr.write(f"[Save] Episode {self.episode_idx:06d} 已保存\n")'''
        sys.stderr.flush()

        # 重置缓存
        self.episode_idx += 1
        self.param_buffer = []
        self.rgb_frame_buffer = []
        self.depth_frame_buffer = []