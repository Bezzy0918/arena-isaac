"""
VLN Dataset Logger (ROS 2 Rosbag variant)
==========================================
基于 ROS 2 rosbag 命令 + MCAP 的轻量级数据记录器。

特点：
- 使用 ros2 bag record 命令后台录制（无需 Python 订阅）
- 自动保存为 MCAP 格式
- 简单轻量，直接依赖 rosbag 工具链
- 支持多个 topics 和多 episode 管理
"""

import os
import sys
import time
import signal
import subprocess
from pathlib import Path
from typing import Dict, List, Optional
from collections import defaultdict


class VLNDataLoggerRosbag:
    """
    基于 rosbag 命令行的数据记录器（轻量级）
    
    使用示例：
    ```python
    logger = VLNDataLoggerRosbag(
        topics=[
            "/camera/image_raw",
            "/lidar/points",
            "/tf",
        ],
        output_dir="collected_data"
    )
    
    logger.start_recording()
    # ... 仿真运行 ...
    logger.stop_recording()
    ```
    """
    
    def __init__(self, 
                 topics: List[str],
                 output_dir: str = "collected_data",
                 initial_delay: float = 1.5):
        """
        初始化 rosbag 记录器
        
        Args:
            topics: 要录制的 topic 列表
            output_dir: 输出目录
            initial_delay: rosbag 启动延迟（秒），确保不漏帧
        """
        self.topics = topics
        self.output_dir = output_dir
        self.initial_delay = initial_delay
        self.episode_idx = 0
        
        # 确保目录存在
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.process = None
        self.bag_path = None
        self._recording = False
        
        sys.stderr.write(f"✅ VLNDataLoggerRosbag 初始化完毕，准备录制 {len(topics)} 个 topics\n")

    def start_recording(self):
        """启动后台 rosbag 录制"""
        if self._recording:
            sys.stderr.write("⚠️  已在录制中，跳过\n")
            return
        
        # 定义 bag 文件夹名（每个 episode 独立）
        bag_name = f"episode_{self.episode_idx:06d}"
        self.bag_path = os.path.join(self.output_dir, bag_name)
        
        # 构造 rosbag 命令
        # -s mcap: 以 MCAP 格式保存
        # -o: 输出路径
        cmd = [
            "ros2", "bag", "record",
            "-s", "mcap",
            "-o", self.bag_path,
        ] + self.topics
        
        sys.stderr.write(f"🚀 启动 rosbag 录制: {' '.join(cmd)}\n")
        
        try:
            # 启动后台进程
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE
            )
            
            # 等待 rosbag 连接到 topics（避免漏帧）
            time.sleep(self.initial_delay)
            self._recording = True
            sys.stderr.write(f"✅ rosbag 已就绪，开始采集\n")
            
        except Exception as e:
            sys.stderr.write(f"❌ 启动 rosbag 失败: {e}\n")
            self.process = None

    def stop_recording(self) -> str:
        """
        等待 rosbag 进程退出（不主动发送 SIGINT）
        
        前提：调用者应确保已发送 SIGINT 或 SIGTERM 信号给该进程
        
        Returns:
            保存的 bag 文件路径
        """
        if not self.process:
            sys.stderr.write("⚠️  rosbag 进程未启动\n")
            return None
        
        sys.stderr.write("⏹️  正在等待 rosbag 进程退出...\n")
        
        try:
            # 等待进程退出（最多 10 秒）
            try:
                self.process.wait(timeout=10.0)
                sys.stderr.write(f"✅ rosbag 已保存至: {self.bag_path}\n")
            except subprocess.TimeoutExpired:
                sys.stderr.write("⚠️  rosbag 未响应，强制终止\n")
                self.process.kill()
                self.process.wait()
            
            self._recording = False
            return self.bag_path
            
        except Exception as e:
            sys.stderr.write(f"❌ 等待进程失败: {e}\n")
            return None

    def next_episode(self):
        """结束当前 episode，为下一个 episode 准备"""
        self.stop_recording()
        self.episode_idx += 1
        self.start_recording()
        sys.stderr.write(f"🔄 开始新 episode: {self.episode_idx}\n")

    @property
    def is_recording(self) -> bool:
        """返回是否正在录制"""
        if self.process:
            return self.process.poll() is None
        return False


class VLNDataBufferRosbag:
    """
    异步数据缓冲器（不基于 Node，可用于后处理）
    
    这个类用于读取已有的 MCAP 文件并重新组织数据。
    """
    
    def __init__(self, mcap_file_path: str, topic_filters: List[str] = None):
        """
        初始化缓冲器
        
        Args:
            mcap_file_path: MCAP 文件路径
            topic_filters: 只读取这些 topics（None = 读取全部）
        """
        self.mcap_path = mcap_file_path
        self.topic_filters = topic_filters or []
        self.messages = defaultdict(list)
        self._load_mcap()

    def _load_mcap(self):
        """从 MCAP 文件读取消息"""
        try:
            from mcap.reader import Reader
            
            with open(self.mcap_path, "rb") as f:
                reader = Reader(f)
                
                for schema, channel, message in reader.iter_messages():
                    # 过滤 topic
                    if self.topic_filters and channel.topic not in self.topic_filters:
                        continue
                    
                    self.messages[channel.topic].append({
                        'timestamp': message.log_time,
                        'data': message.data
                    })
            
            print(f"✅ 已加载 MCAP: {self.mcap_path}")
            for topic, msgs in self.messages.items():
                print(f"   {topic}: {len(msgs)} 条消息")
                
        except Exception as e:
            print(f"❌ 加载 MCAP 失败: {e}")

    def get_messages_by_topic(self, topic: str) -> List[Dict]:
        """获取某个 topic 的所有消息"""
        return self.messages.get(topic, [])

    def synchronize_by_timestamp(self, time_tolerance_ns: int = 10_000_000) -> List[Dict]:
        """
        按时间戳同步多个 topics 的消息
        
        Args:
            time_tolerance_ns: 时间容差（纳秒），默认 10ms
            
        Returns:
            同步后的消息列表，每个元素包含该时刻所有 topics 的消息
        """
        # 收集所有消息和时间戳
        all_timestamps = set()
        for msgs in self.messages.values():
            for msg in msgs:
                all_timestamps.add(msg['timestamp'])
        
        all_timestamps = sorted(all_timestamps)
        synchronized = []
        
        for ts in all_timestamps:
            frame = {'timestamp': ts, 'topics': {}}
            
            for topic, msgs in self.messages.items():
                # 找到这个时刻最接近的消息
                closest = min(
                    msgs,
                    key=lambda m: abs(m['timestamp'] - ts),
                    default=None
                )
                
                if closest and abs(closest['timestamp'] - ts) <= time_tolerance_ns:
                    frame['topics'][topic] = closest
            
            if frame['topics']:  # 只保留至少有一个消息的帧
                synchronized.append(frame)
        
        return synchronized


def run_logger(topics: List[str], 
               duration_seconds: Optional[float] = None,
               output_dir: str = "collected_data"):
    """
    快速启动一个 rosbag 录制
    
    Args:
        topics: 要录制的 topic 列表
        duration_seconds: 录制时长（秒），None = 手动停止
        output_dir: 输出目录
    """
    logger = VLNDataLoggerRosbag(
        topics=topics,
        output_dir=output_dir
    )
    
    logger.start_recording()
    
    if duration_seconds:
        time.sleep(duration_seconds)
        logger.stop_recording()
    else:
        try:
            # 等待用户 Ctrl+C
            while logger.is_recording:
                time.sleep(1)
        except KeyboardInterrupt:
            sys.stderr.write("\n[Interrupt] 用户中断，正在停止录制...\n")
            logger.stop_recording()


if __name__ == "__main__":
    # 示例：录制 LIDAR + TF topics
    topics = [
        "/task_generator_node/jackal/lidar/points",
        "/tf",
        "/tf_static",
    ]
    
    run_logger(topics, duration_seconds=10)
