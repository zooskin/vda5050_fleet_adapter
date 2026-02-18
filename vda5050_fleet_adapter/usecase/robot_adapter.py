"""RobotAdapter: RMF ↔ VDA5050 로봇 브릿지.

rmf_demos_fleet_adapter의 RobotAdapter 패턴을 따른다.
RMF 콜백(navigate, stop, execute_action)을 받아
RobotAPI를 통해 VDA5050 AGV에 명령을 전달한다.
"""

from __future__ import annotations

import logging
import math
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
    RobotAPIResult,
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
        arrival_threshold: float = 0.5,
    ) -> None:
        self.name = name
        self.api = api
        self.node = node
        self.fleet_handle = fleet_handle
        self.nav_nodes = nav_nodes
        self.nav_edges = nav_edges
        self.nav_graph = nav_graph
        self.arrival_threshold = arrival_threshold

        self.execution: Any | None = None
        self.update_handle: Any | None = None
        self.configuration: Any | None = None
        self.cmd_id: int = 0
        self.position: list[float] | None = None

        # Navigate 도착 판정 상태
        self._navigate_target_position: list[float] | None = None
        self._is_navigating: bool = False

        # Order 라이프사이클 관리
        self._active_order_id: str | None = None
        self._order_update_id: int = 0
        self._final_destination: str | None = None
        self._current_task_id: str | None = None

        # 명령 재시도 스레드 관리
        self._issue_cmd_thread: threading.Thread | None = None
        self._cancel_cmd_event = threading.Event()

        # Negotiation 상태 관리
        self._is_paused_for_negotiation: bool = False

        # 경로 캐시
        self._last_nodes: list[list] = []
        self._last_stitch_seq_id: int = 0  # 마지막 Base 노드의 sequenceId

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
            completed = False
            if self._is_navigating and self._navigate_target_position:
                dist = math.hypot(
                    data.position[0]
                    - self._navigate_target_position[0],
                    data.position[1]
                    - self._navigate_target_position[1],
                )
                if dist <= self.arrival_threshold:
                    completed = True
            else:
                completed = self.api.is_command_completed(
                    self.name, self.cmd_id
                )
            if completed:
                self.execution.finished()
                self.execution = None
                self._is_navigating = False
                self._navigate_target_position = None
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
        self._navigate_target_position = list(destination.position[:2])
        self._is_navigating = True
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

        # Negotiation 처리: pause 상태에서 navigate가 호출되면
        # cancelOrder 전송 → AGV 처리 대기 → 새 orderID로 명령
        is_negotiation = self._is_paused_for_negotiation
        if is_negotiation:
            self._is_paused_for_negotiation = False
            cancel_cmd_id = self.cmd_id
            self.cmd_id += 1
            logger.info(
                'Negotiation detected for robot %s: '
                'will send cancelOrder (cmd_id=%d) then new order',
                self.name, cancel_cmd_id,
            )
            current_task_id = self._get_current_task_id()
            self._current_task_id = current_task_id
            self._final_destination = (
                self._resolve_final_destination(destination, goal_node)
            )
            self._active_order_id = (
                f'order_{self.cmd_id}_{uuid.uuid4().hex[:8]}'
            )
            self._order_update_id = 0
            self._last_nodes = []
        else:
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
                    self._resolve_final_destination(
                        destination, goal_node
                    )
                )
                self._active_order_id = (
                    f'order_{self.cmd_id}_{uuid.uuid4().hex[:8]}'
                )
                self._order_update_id = 0
                self._last_nodes = []
            else:
                # Order Update: 같은 orderID, update_id 증가
                self._order_update_id += 1
                self._final_destination = (
                    self._resolve_final_destination(
                        destination,
                        self._final_destination or goal_node,
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

        # ── Step 1: destination에서 경로 추출 ──
        target = self._final_destination or goal_node
        raw_waypoints = getattr(destination, 'waypoint_names', None)
        rmf_path = None
        if raw_waypoints and isinstance(
            raw_waypoints, (list, tuple)
        ):
            valid = all(
                wp in self.nav_nodes for wp in raw_waypoints
            )
            if valid:
                rmf_path = list(raw_waypoints)
            else:
                logger.warning(
                    'waypoint_names has unknown nodes for %s: %s',
                    self.name, raw_waypoints,
                )

        if rmf_path is not None:
            rmf_path = self._interpolate_path(rmf_path)
            if start_node in rmf_path:
                start_idx = rmf_path.index(start_node)
                path = rmf_path[start_idx:]
            elif rmf_path:
                bridge = compute_path(
                    self.nav_graph, start_node, rmf_path[0]
                )
                if bridge:
                    path = bridge[:-1] + rmf_path
                else:
                    path = rmf_path
            else:
                path = None
            logger.info(
                'Using waypoint_names for %s: %s',
                self.name, path,
            )
        else:
            logger.info(
                'No waypoint_names for %s, '
                'fallback to compute_path(start=%s, target=%s)',
                self.name, start_node, target,
            )
            path = compute_path(
                self.nav_graph, start_node, target
            )

        if path is None:
            path = [start_node, target]

        # ── Step 2: 3-tier 경로 구성 ──
        # tier 1 (Base):    path[0 : base_end_index+1]
        # tier 2 (Horizon): path[base_end_index+1 : rmf_path_end+1]
        # tier 3 (Horizon): path[rmf_path_end+1 :]
        rmf_path_end = len(path) - 1

        # Tier 3 확장: path에 최종목적지가 없으면
        # 현재 경로 끝에서 최종목적지까지의 경로를 Horizon으로 추가
        if (
            target
            and path
            and target not in path
        ):
            extension = compute_path(
                self.nav_graph, path[-1], target
            )
            if extension and len(extension) > 1:
                path = path + extension[1:]
                logger.info(
                    'Tier3 extension for %s: %s -> %s '
                    '(appended %s)',
                    self.name, extension[0],
                    target, extension[1:],
                )
            else:
                # 경로 계산 불가 시 최종목적지만 직접 추가
                path.append(target)
                logger.warning(
                    'Cannot compute path to final destination '
                    '%s for robot %s, appended directly',
                    target, self.name,
                )

        # goal_node의 path 내 인덱스 찾기 → base_end_index
        if goal_node in path:
            base_end_index = path.index(goal_node)
        else:
            base_end_index = len(path) - 1

        logger.info(
            'Navigate [%s] 3-tier path: '
            'Base(tier1)=%s, Horizon-RMF(tier2)=%s, '
            'Horizon-ext(tier3)=%s',
            self.name,
            path[:base_end_index + 1],
            path[base_end_index + 1:rmf_path_end + 1],
            path[rmf_path_end + 1:],
        )

        # ── Step 3: VDA5050 Node/Edge 생성 ──
        # Order update 시 stitching node의 sequenceId를 유지한다.
        seq_start = (
            self._last_stitch_seq_id
            if self._order_update_id > 0
            else 0
        )

        map_name = destination.map
        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            path, self.nav_nodes, map_name,
            base_end_index=base_end_index,
            seq_start=seq_start,
        )

        # 다음 order update를 위해 stitching sequenceId 저장
        # stitching node = 현재 Base의 마지막 노드
        self._last_stitch_seq_id = seq_start + base_end_index * 2

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

        nav_args = (
            self.name,
            self.cmd_id,
            vda_nodes,
            vda_edges,
            map_name,
            self._active_order_id,
            self._order_update_id,
        )

        if is_negotiation:
            # Negotiation: cancelOrder 전송 후 AGV가 처리할 시간을 주고
            # 새 order를 전송한다. attempt_cmd_until_success의 1초 대기를
            # 활용하여 cancelOrder와 새 order 사이에 간격을 둔다.
            cancel_sent = [False]

            def cancel_then_navigate(*args: Any) -> RobotAPIResult:
                if not cancel_sent[0]:
                    self.api.stop(self.name, cancel_cmd_id)
                    cancel_sent[0] = True
                    logger.info(
                        'cancelOrder sent for robot %s (cmd_id=%d), '
                        'waiting for AGV to process before new order',
                        self.name, cancel_cmd_id,
                    )
                    return RobotAPIResult.RETRY
                return self.api.navigate(*args)

            self.attempt_cmd_until_success(
                cmd=cancel_then_navigate,
                args=nav_args,
            )
        else:
            self.attempt_cmd_until_success(
                cmd=self.api.navigate,
                args=nav_args,
            )

    def stop(self, activity: Any) -> None:
        """RMF 정지 콜백.

        Negotiation 규칙에 따라 startPause를 즉시 전송한다.
        Order 상태는 유지하여 이후 navigate()에서 cancelOrder +
        새 orderID로 처리할 수 있도록 한다.

        startPause는 직접 호출한다 (attempt_cmd_until_success 사용 안 함).
        navigate()가 즉시 뒤따라올 수 있어 retry 스레드가 취소될 수 있기
        때문이다.

        Args:
            activity: 정지할 activity identifier.
        """
        if self.execution is not None:
            if self.execution.identifier.is_same(activity):
                self.execution = None
                self._is_paused_for_negotiation = True
                self.cancel_cmd_attempt()
                self.cmd_id += 1
                logger.info(
                    'Stop (negotiation pause) for robot %s, '
                    'sending startPause, cmd_id=%d',
                    self.name, self.cmd_id,
                )
                self.api.pause(self.name, self.cmd_id)

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
        self._is_navigating = False

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

    def _resolve_final_destination(
        self, destination: Any, fallback: str,
    ) -> str:
        """Navigate callback의 destination에서 최종 목적지를 조회한다.

        C++ EasyFullControl 패치로 추가된 destination.final_name을
        사용한다. 없으면 fallback(goal_node)을 사용한다.

        Args:
            destination: RMF Destination 객체.
            fallback: 조회 실패 시 사용할 목적지 (goal_node).

        Returns:
            최종 목적지 waypoint 이름.
        """
        final_name = getattr(destination, 'final_name', '')
        if final_name and final_name in self.nav_nodes:
            logger.info(
                'Final destination from Destination.final_name: '
                'robot=%s, final_name=%s',
                self.name, final_name,
            )
            return final_name

        if final_name and final_name not in self.nav_nodes:
            logger.warning(
                'Destination.final_name not in nav_nodes: '
                'robot=%s, final_name=%s, using fallback=%s',
                self.name, final_name, fallback,
            )
        return fallback

    def _reset_order_state(self) -> None:
        """Order 라이프사이클 상태를 초기화한다.

        Pause 상태에서 navigate 없이 task가 종료된 경우
        cancelOrder를 전송하여 로봇의 order를 정리한다.
        """
        if self._is_paused_for_negotiation:
            self._is_paused_for_negotiation = False
            self.cmd_id += 1
            logger.info(
                'Task ended while paused, sending cancelOrder '
                'for robot %s, cmd_id=%d',
                self.name, self.cmd_id,
            )
            self.attempt_cmd_until_success(
                cmd=self.api.stop,
                args=(self.name, self.cmd_id),
            )

        self._navigate_target_position = None
        self._is_navigating = False

        old_task_id = self._current_task_id
        self._active_order_id = None
        self._order_update_id = 0
        self._final_destination = None
        self._current_task_id = None
        self._last_nodes = []
        self._last_stitch_seq_id = 0
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

    def _interpolate_path(self, sparse_path: list[str]) -> list[str]:
        """Sparse path의 인접하지 않은 노드 쌍 사이를 보간한다.

        RMF planned_path는 주요 waypoint만 포함할 수 있다
        (예: [1,3,5] → [1,2,3,4,5]). 각 연속 노드 쌍에 대해
        nav_graph에 직접 edge가 없으면 compute_path로 중간 노드를 채운다.

        Args:
            sparse_path: 중간 노드가 빠진 경로.

        Returns:
            중간 노드가 채워진 경로.
        """
        if len(sparse_path) < 2:
            return sparse_path

        full: list[str] = [sparse_path[0]]
        for i in range(len(sparse_path) - 1):
            a, b = sparse_path[i], sparse_path[i + 1]
            if self.nav_graph.has_edge(a, b):
                full.append(b)
            else:
                segment = compute_path(self.nav_graph, a, b)
                if segment and len(segment) > 1:
                    full.extend(segment[1:])
                else:
                    full.append(b)
        logger.debug(
            'Interpolated path for %s: %s -> %s',
            self.name, sparse_path, full,
        )
        return full
