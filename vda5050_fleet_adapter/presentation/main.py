"""VDA5050 Fleet Adapter 진입점.

ROS 2 노드를 시작하고 graceful shutdown을 처리한다.
실행: ros2 run vda5050_fleet_adapter fleet_adapter --ros-args -p fleet_name:=my_fleet
"""

from __future__ import annotations

import sys

import rclpy

from vda5050_fleet_adapter.presentation.fleet_adapter_node import (
    FleetAdapterNode,
)


def main(args: list[str] | None = None) -> None:
    """Fleet Adapter를 시작한다."""
    rclpy.init(args=args)

    node = FleetAdapterNode()

    # TODO: AGV ID 목록을 설정 파일 또는 파라미터에서 로드
    # 현재는 ROS 2 파라미터로 전달받는 구조
    node.declare_parameter("agv_ids", [""])
    agv_ids_param = (
        node.get_parameter("agv_ids")
        .get_parameter_value().string_array_value
    )
    agv_ids = [aid for aid in agv_ids_param if aid]

    try:
        node.start(agv_ids=agv_ids)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main(sys.argv)
