import omni.graph.core as og
from isaac_utils.graphs import Graph
from omni.isaac.core.utils import extensions

extensions.enable_extension("omni.isaac.core_nodes")
extensions.enable_extension("omni.isaac.ros2_bridge")


def PublishTime(graph_path: str):
    graph = Graph(graph_path)

    on_playback_tick = graph.node('on_playback_tick', 'omni.graph.action.OnPlaybackTick')
    read_simulation_time = graph.node('read_simulation_time', 'omni.isaac.core_nodes.IsaacReadSimulationTime')
    publish_clock = graph.node('publish_clock', 'omni.isaac.ros2_bridge.ROS2PublishClock')

    # Publish Real Time Factor (RTF) as std_msgs/Float32 on topic: isaac/rtf
    # Node/port names can vary between Isaac Sim versions, so connections are best-effort.
    real_time_factor = graph.node('real_time_factor', 'omni.isaac.core_nodes.IsaacRealTimeFactor')
    publish_rtf = graph.node('publish_rtf', 'omni.isaac.ros2_bridge.ROS2Publisher')

    on_playback_tick.connect('tick', publish_clock, 'execIn')
    read_simulation_time.connect('simulationTime', publish_clock, 'timeStamp')

    publish_rtf.create_attribute('data', 'float')
    publish_rtf.attribute('topicName', '/isaac/rtf')
    publish_rtf.attribute('messagePackage', 'std_msgs')
    publish_rtf.attribute('messageName', 'Float32')

    # Trigger publishing each tick.
    on_playback_tick.connect('tick', publish_rtf, 'execIn')
    def _connect_first_existing(src_attr: str, dst_attr: str, candidates: list[str], *, dst_is_input: bool = True):
        def _action():
            last_exc: Exception | None = None
            for name in candidates:
                try:
                    if dst_is_input:
                        og.Controller.connect(src_attr.replace('{name}', name), dst_attr)
                    else:
                        og.Controller.connect(src_attr, dst_attr.replace('{name}', name))
                    return
                except Exception as e:
                    last_exc = e
                    continue
            if last_exc is not None:
                # Best-effort only
                pass
        graph.add_action(_action)

    # Try common output port names for RTF value.
    _connect_first_existing(
        f"{real_time_factor.path}.outputs:{{name}}",
        f"{publish_rtf.path}.inputs:data",
        candidates=['rtf', 'realTimeFactor', 'value', 'data']
    )

    graph.execute(og.Controller())
