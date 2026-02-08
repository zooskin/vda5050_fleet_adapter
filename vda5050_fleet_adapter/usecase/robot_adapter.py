"""RobotAdapter: RMF ↔ VDA5050 로봇 브릿지.

rmf_demos_fleet_adapter의 RobotAdapter 패턴을 따른다.
RMF 콜백(navigate, stop, execute_action)을 받아
RobotAPI를 통해 VDA5050 AGV에 명령을 전달한다.
"""

from __future__ import annotations

import logging
import threading
import uuid
from typing import Any

from vda5050_fleet_adapter.infra.nav_graph.graph_utils import (
    build_vda5050_nodes_edges,
    compute_path,
    find_nearest_node,
)
from vda5050_fleet_adapter.usecase.ports.robot_api import (
    RobotAPI,
    RobotUpdateData,
)

logger = logging.getLogger(__name__)


class RobotAdapter:
    """개별 로봇의 RMF-VDA5050 브릿지.

    RMF easy_full_control의 콜백을 처리하고,
    RobotAPI를 통해 AGV에 명령을 전달한다.

    Args:
        name: 로봇 이름.
        api: RobotAPI 구현체.
        node: ROS 2 노드 (로깅용).
        fleet_handle: RMF fleet handle.
        nav_nodes: nav graph 노드 dict.
        nav_edges: nav graph 엣지 dict.
        nav_graph: networkx 그래프.
    """

    def __init__(
        self,
        name: str,
        api: RobotAPI,
        node: Any,
        fleet_handle: Any,
        nav_nodes: dict,
        nav_edges: dict,
        nav_graph: Any,
    ) -> None:
        self.name = name
        self.api = api
        self.node = node
        self.fleet_handle = fleet_handle
        self.nav_nodes = nav_nodes
        self.nav_edges = nav_edges
        self.nav_graph = nav_graph

        self.execution: Any | None = None
        self.update_handle: Any | None = None
        self.configuration: Any | None = None
        self.cmd_id: int = 0
        self.position: list[float] | None = None

        # Order 라이프사이클 관리
        self._active_order_id: str | None = None
        self._order_update_id: int = 0
        self._final_destination: str | None = None

        # 명령 재시도 스레드 관리
        self._issue_cmd_thread: threading.Thread | None = None
        self._cancel_cmd_event = threading.Event()

        # 경로 캐시
        self._last_nodes: list[list] = []

    def update(self, state: Any, data: RobotUpdateData) -> None:
        """주기적 상태 업데이트.

        명령 완료를 감지하고 RMF에 상태를 보고한다.

        Args:
            state: rmf_easy.RobotState 인스턴스.
            data: 로봇 상태 데이터.
        """
        self.position = data.position
        activity_identifier = None

        if self.execution is not None:
            if self.api.is_command_completed(self.name, self.cmd_id):
                self.execution.finished()
                self.execution = None
                self._active_order_id = None
                self._order_update_id = 0
                self._final_destination = None
            else:
                activity_identifier = self.execution.identifier

        if self.update_handle is not None:
            self.update_handle.update(state, activity_identifier)

    def make_callbacks(self) -> Any:
        """RMF RobotCallbacks를 생성한다.

        Returns:
            rmf_easy.RobotCallbacks 인스턴스.
        """
        import rmf_adapter.easy_full_control as rmf_easy

        return rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(
                destination, execution
            ),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            ),
        )

    def navigate(self, destination: Any, execution: Any) -> None:
        """RMF 내비게이션 콜백.

        목적지까지의 VDA5050 Order를 생성하여 전송한다.
        Base/Horizon 분리: RMF Destination까지 Base, 최종 목적지까지 Horizon.
        1 RMF Task = 1 orderID (새 Destination 시 order_update_id 증가).

        Args:
            destination: RMF destination (position, map, dock 등).
            execution: RMF execution handle.
        """
        self.cmd_id += 1
        self.execution = execution
        self.node.get_logger().info(
            f'[{self.name}] navigate callback called, '
            f'dest={destination.position}, map={destination.map}'
        )

        goal_node = find_nearest_node(
            self.nav_nodes,
            destination.position[0],
            destination.position[1],
        )

        if goal_node is None:
            logger.error(
                'No nearest node found for robot [%s] at %s',
                self.name, destination.position,
            )
            return

        # 새 Task vs Order Update 판별
        if self._active_order_id is None:
            # 새 Task: 첫 navigate의 destination이 최종 목적지
            self._final_destination = goal_node
            self._active_order_id = (
                f'order_{self.cmd_id}_{uuid.uuid4().hex[:8]}'
            )
            self._order_update_id = 0
        else:
            # Order Update: 같은 orderID, update_id 증가
            self._order_update_id += 1

        # 현재 위치에서 가장 가까운 노드 찾기
        start_node = None
        if self._last_nodes:
            start_node = self._last_nodes[-1][0]
        elif self.position is not None:
            start_node = find_nearest_node(
                self.nav_nodes, self.position[0], self.position[1]
            )

        if start_node is None:
            start_node = goal_node

        # 경로 계산: current → final_destination
        target = self._final_destination or goal_node
        path = compute_path(self.nav_graph, start_node, target)
        if path is None:
            path = [start_node, target]

        # goal_node의 path 내 인덱스 찾기 → base_end_index
        if goal_node in path:
            base_end_index = path.index(goal_node)
        else:
            base_end_index = len(path) - 1

        # VDA5050 Node/Edge 생성 (Base/Horizon 분리)
        map_name = destination.map
        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            path, self.nav_nodes, map_name,
            base_end_index=base_end_index,
        )

        # 경로 캐시 업데이트
        self._last_nodes = [
            [n, (self.nav_nodes[n]['x'], self.nav_nodes[n]['y'])]
            for n in path
        ]

        logger.info(
            'Navigate [%s]: cmd_id=%d, order_id=%s, '
            'order_update_id=%d, dest=%s, final_dest=%s, '
            'map=%s, path=%s, base_end_index=%d',
            self.name, self.cmd_id, self._active_order_id,
            self._order_update_id, destination.position,
            self._final_destination, map_name, path,
            base_end_index,
        )

        self.attempt_cmd_until_success(
            cmd=self.api.navigate,
            args=(
                self.name,
                self.cmd_id,
                vda_nodes,
                vda_edges,
                map_name,
                self._active_order_id,
                self._order_update_id,
            ),
        )

    def stop(self, activity: Any) -> None:
        """RMF 정지 콜백.

        Order 상태를 초기화하고 cancelOrder를 전송한다.

        Args:
            activity: 정지할 activity identifier.
        """
        if self.execution is not None:
            if self.execution.identifier.is_same(activity):
                self.execution = None
                self._active_order_id = None
                self._order_update_id = 0
                self._final_destination = None
                self.cmd_id += 1
                self.attempt_cmd_until_success(
                    cmd=self.api.stop,
                    args=(self.name, self.cmd_id),
                )

    def execute_action(
        self, category: str, description: dict, execution: Any
    ) -> None:
        """RMF 액션 실행 콜백.

        Args:
            category: 액션 카테고리 (teleop, charge 등).
            description: 액션 설명 dict.
            execution: RMF execution handle.
        """
        self.cmd_id += 1
        self.execution = execution

        logger.info(
            'Execute action [%s]: category=%s, cmd_id=%d',
            self.name, category, self.cmd_id,
        )

        params = description if isinstance(description, dict) else {}

        self.attempt_cmd_until_success(
            cmd=self.api.start_activity,
            args=(self.name, self.cmd_id, category, params),
        )

    def attempt_cmd_until_success(
        self, cmd: Any, args: tuple
    ) -> None:
        """명령을 성공할 때까지 재시도한다.

        Args:
            cmd: 실행할 함수.
            args: 함수 인자.
        """
        self.cancel_cmd_attempt()

        def loop() -> None:
            from vda5050_fleet_adapter.usecase.ports.robot_api import (
                RobotAPIResult,
            )
            while True:
                result = cmd(*args)
                if result == RobotAPIResult.SUCCESS:
                    break
                if result == RobotAPIResult.IMPOSSIBLE:
                    logger.error(
                        'Command impossible for robot %s, giving up',
                        self.name,
                    )
                    break
                logger.warning(
                    'Failed to contact fleet manager for robot %s, '
                    'retrying...', self.name
                )
                if self._cancel_cmd_event.wait(1.0):
                    break

        self._issue_cmd_thread = threading.Thread(
            target=loop, daemon=True
        )
        self._issue_cmd_thread.start()

    def cancel_cmd_attempt(self) -> None:
        """진행 중인 명령 재시도를 취소한다."""
        if self._issue_cmd_thread is not None:
            self._cancel_cmd_event.set()
            if self._issue_cmd_thread.is_alive():
                self._issue_cmd_thread.join(timeout=5.0)
            self._issue_cmd_thread = None
        self._cancel_cmd_event.clear()
