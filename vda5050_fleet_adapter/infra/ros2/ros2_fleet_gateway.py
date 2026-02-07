"""ROS 2 Fleet Gateway 구현체 (FleetGateway 포트 구현).

rmf_fleet_adapter_python을 사용하여 Open-RMF와 통신한다.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any

from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.value_objects.position import AgvPosition
from vda5050_fleet_adapter.infra.ros2.fleet_adapter_handle import (
    RmfNavigationHandle,
    RmfTaskHandle,
)
from vda5050_fleet_adapter.usecase.ports.config_port import AppConfig
from vda5050_fleet_adapter.usecase.ports.fleet_gateway import (
    FleetGateway,
    NavigationHandle,
)

logger = logging.getLogger(__name__)


class Ros2FleetGateway(FleetGateway):
    """FleetGateway의 ROS 2 구현체.

    rmf_fleet_adapter_python API를 사용하여
    AGV 상태를 RMF에 보고하고 태스크/내비게이션 명령을 수신한다.

    Args:
        config: 애플리케이션 설정.
    """

    def __init__(self, config: AppConfig) -> None:
        self._config = config
        self._fleet_handle: Any = None
        self._robot_handles: dict[str, Any] = {}

        self._navigate_callback: Callable[
            [str, NavigationHandle], None
        ] | None = None
        self._stop_callback: Callable[[str], None] | None = None
        self._action_callback: Callable[
            [str, str, dict], None
        ] | None = None

    # -- 상태 보고 --

    def update_position(
        self, agv_id: str, position: AgvPosition
    ) -> None:
        handle = self._robot_handles.get(agv_id)
        if handle is None:
            logger.warning("Robot handle not found: %s", agv_id)
            return

        handle.update(
            {
                "position": {
                    "x": position.x,
                    "y": position.y,
                    "yaw": position.theta,
                },
                "map": position.map_id,
            }
        )

    def update_battery(self, agv_id: str, battery_percent: float) -> None:
        handle = self._robot_handles.get(agv_id)
        if handle is None:
            return

        handle.update({"battery": battery_percent / 100.0})

    def update_state(self, agv_id: str, state: AgvState) -> None:
        handle = self._robot_handles.get(agv_id)
        if handle is None:
            return

        update_data: dict[str, Any] = {}

        if state.agv_position is not None:
            update_data["position"] = {
                "x": state.agv_position.x,
                "y": state.agv_position.y,
                "yaw": state.agv_position.theta,
            }
            update_data["map"] = state.agv_position.map_id

        if state.battery_state is not None:
            update_data["battery"] = (
                state.battery_state.battery_charge / 100.0
            )

        if update_data:
            handle.update(update_data)

    # -- 명령 수신 콜백 등록 --

    def on_navigate(
        self,
        callback: Callable[[str, NavigationHandle], None],
    ) -> None:
        self._navigate_callback = callback

    def on_stop(
        self,
        callback: Callable[[str], None],
    ) -> None:
        self._stop_callback = callback

    def on_action(
        self,
        callback: Callable[[str, str, dict], None],
    ) -> None:
        self._action_callback = callback

    # -- Fleet 관리 --

    def register_agv(self, agv_id: str) -> None:
        if self._fleet_handle is None:
            logger.warning(
                "Fleet handle not initialized, "
                "storing %s for deferred registration", agv_id,
            )
            self._robot_handles[agv_id] = None
            return

        robot_handle = self._fleet_handle.add_robot(
            name=agv_id,
            navigate_handler=self._make_navigate_handler(agv_id),
            stop_handler=self._make_stop_handler(agv_id),
            action_handler=self._make_action_handler(agv_id),
        )
        self._robot_handles[agv_id] = robot_handle
        logger.info("AGV registered with RMF: %s", agv_id)

    def set_fleet_handle(self, fleet_handle: Any) -> None:
        """외부에서 초기화된 fleet handle을 설정한다.

        presentation 레이어에서 ROS 2 노드 초기화 후 호출한다.

        Args:
            fleet_handle: rmf_fleet_adapter의 FleetHandle.
        """
        self._fleet_handle = fleet_handle
        logger.info("Fleet handle set")

        # 대기 중이던 AGV 등록 처리
        for agv_id, handle in list(self._robot_handles.items()):
            if handle is None:
                self.register_agv(agv_id)

    def start(self) -> None:
        logger.info(
            "Ros2FleetGateway started (fleet=%s)",
            self._config.adapter.fleet_name,
        )

    def shutdown(self) -> None:
        self._fleet_handle = None
        self._robot_handles.clear()
        logger.info("Ros2FleetGateway shut down")

    # -- RMF 콜백 핸들러 생성 --

    def _make_navigate_handler(
        self, agv_id: str
    ) -> Callable:
        """RMF navigate 콜백을 생성한다."""
        def handler(
            waypoints: list[Any], completer: Any
        ) -> None:
            wp_tuples = [
                (wp.position[0], wp.position[1], wp.position[2])
                for wp in waypoints
            ]
            nav_handle = RmfNavigationHandle(wp_tuples, completer)

            if self._navigate_callback is not None:
                self._navigate_callback(agv_id, nav_handle)
            else:
                logger.warning(
                    "No navigate callback registered for %s", agv_id
                )

        return handler

    def _make_stop_handler(self, agv_id: str) -> Callable:
        """RMF stop 콜백을 생성한다."""
        def handler() -> None:
            if self._stop_callback is not None:
                self._stop_callback(agv_id)
            else:
                logger.warning(
                    "No stop callback registered for %s", agv_id
                )

        return handler

    def _make_action_handler(self, agv_id: str) -> Callable:
        """RMF action 콜백을 생성한다."""
        def handler(
            action_type: str, parameters: dict
        ) -> None:
            if self._action_callback is not None:
                self._action_callback(agv_id, action_type, parameters)
            else:
                logger.warning(
                    "No action callback registered for %s", agv_id
                )

        return handler
