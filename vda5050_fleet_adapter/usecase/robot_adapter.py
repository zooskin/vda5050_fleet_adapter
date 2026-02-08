"""RobotAdapter: RMF ↔ VDA5050 로봇 브릿지.

rmf_demos_fleet_adapter의 RobotAdapter 패턴을 따른다.
RMF 콜백(navigate, stop, execute_action)을 받아
RobotAPI를 통해 VDA5050 AGV에 명령을 전달한다.
"""

from __future__ import annotations

import logging
import threading
from typing import Any
import uuid

from vda5050_fleet_adapter.infra.nav_graph.graph_utils import (
    build_vda5050_nodes_edges,
    compute_path,
    find_nearest_node,
)
from vda5050_fleet_adapter.usecase.ports.robot_api import (
    RobotAPI,
    RobotUpdateData,
)
from vda5050_fleet_adapter.usecase.ports.task_tracker import TaskTracker

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
        task_tracker: Task 최종 목적지 추적기 (optional).
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
        task_tracker: TaskTracker | None = None,
    ) -> None:
        self.name = name
        self.api = api
        self.node = node
        self.fleet_handle = fleet_handle
        self.nav_nodes = nav_nodes
        self.nav_edges = nav_edges
        self.nav_graph = nav_graph
        self.task_tracker = task_tracker

        self.execution: Any | None = None
        self.update_handle: Any | None = None
        self.configuration: Any | None = None
        self.cmd_id: int = 0
        self.position: list[float] | None = None

        # Order 라이프사이클 관리
        self._active_order_id: str | None = None
        self._order_update_id: int = 0
        self._final_destination: str | None = None
        self._current_task_id: str | None = None

        # 명령 재시도 스레드 관리
        self._issue_cmd_thread: threading.Thread | None = None
        self._cancel_cmd_event = threading.Event()

        # 경로 캐시
        self._last_nodes: list[list] = []

    def update(self, state: Any, data: RobotUpdateData) -> None:
        """주기적 상태 업데이트.

        명령 완료를 감지하고 RMF에 상태를 보고한다.
        Order 상태는 task 단위로만 리셋한다 (command 완료 시 리셋하지 않음).

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
            else:
                activity_identifier = self.execution.identifier

        # Task 완료 감지: task_id가 변경되거나 비어있으면 order 리셋
        if self._active_order_id is not None and self.execution is None:
            current_task_id = self._get_current_task_id()
            task_ended = (
                current_task_id is None
                or current_task_id == ''
                or (
                    self._current_task_id is not None
                    and current_task_id != self._current_task_id
                )
            )
            if task_ended:
                self._reset_order_state()

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
            f'dest={destination.position}, map={destination.map}, '
            f'name={getattr(destination, "name", "")}'
        )

        # Use destination.name for exact waypoint match (if available)
        dest_name = getattr(destination, 'name', '')
        if dest_name and dest_name in self.nav_nodes:
            goal_node = dest_name
        else:
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

        # Task 경계 감지: current_task_id 변경 시 새 Task
        current_task_id = self._get_current_task_id()
        is_new_task = (
            self._active_order_id is None
            or (
                current_task_id
                and current_task_id != self._current_task_id
            )
        )

        if is_new_task:
            self._current_task_id = current_task_id
            self._final_destination = (
                self._resolve_final_destination(goal_node)
            )
            self._active_order_id = (
                f'order_{self.cmd_id}_{uuid.uuid4().hex[:8]}'
            )
            self._order_update_id = 0
            self._last_nodes = []
        else:
            # Order Update: 같은 orderID, update_id 증가
            self._order_update_id += 1
            # Re-resolve: tracker may have received fleet_state
            # data since the initial resolve
            self._final_destination = (
                self._resolve_final_destination(
                    self._final_destination or goal_node
                )
            )

        # 현재 위치 기반으로 start_node 결정
        start_node = None
        if self.position is not None:
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
                self._reset_order_state()
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

    def _get_current_task_id(self) -> str | None:
        """현재 RMF task ID를 조회한다.

        Returns:
            현재 task의 booking_id 또는 None.
        """
        if self.update_handle is None:
            return None
        try:
            return self.update_handle.more().current_task_id()
        except Exception:
            logger.debug(
                'Could not get current_task_id for %s', self.name
            )
            return None

    def _resolve_final_destination(self, fallback: str) -> str:
        """Task_tracker에서 최종 목적지를 조회한다.

        Task_tracker가 없거나 매핑이 없으면 fallback을 사용한다.

        Args:
            fallback: 조회 실패 시 사용할 목적지 (goal_node).

        Returns:
            최종 목적지 waypoint 이름.
        """
        if self.task_tracker is None:
            return fallback

        task_id = self._get_current_task_id()
        if task_id is None:
            return fallback

        destination = self.task_tracker.get_final_destination(task_id)
        if destination is not None and destination in self.nav_nodes:
            logger.info(
                'Final destination from task_tracker: '
                'robot=%s, task_id=%s, destination=%s',
                self.name, task_id, destination,
            )
            return destination

        if destination is not None and destination not in self.nav_nodes:
            logger.warning(
                'Destination from task_tracker not in nav_nodes: '
                'robot=%s, task_id=%s, destination=%s, '
                'using fallback=%s',
                self.name, task_id, destination, fallback,
            )
        else:
            logger.warning(
                'No final destination from task_tracker: '
                'robot=%s, task_id=%s, destination=%s, '
                'using fallback=%s',
                self.name, task_id, destination, fallback,
            )
        return fallback

    def _reset_order_state(self) -> None:
        """Order 라이프사이클 상태를 초기화한다."""
        old_task_id = self._current_task_id
        if self.task_tracker is not None and old_task_id:
            self.task_tracker.clear_booking(old_task_id)
        self._active_order_id = None
        self._order_update_id = 0
        self._final_destination = None
        self._current_task_id = None
        self._last_nodes = []
        logger.info(
            'Order state reset for robot %s (task_id=%s)',
            self.name, old_task_id,
        )

    def cancel_cmd_attempt(self) -> None:
        """진행 중인 명령 재시도를 취소한다."""
        if self._issue_cmd_thread is not None:
            self._cancel_cmd_event.set()
            if self._issue_cmd_thread.is_alive():
                self._issue_cmd_thread.join(timeout=5.0)
            self._issue_cmd_thread = None
        self._cancel_cmd_event.clear()
