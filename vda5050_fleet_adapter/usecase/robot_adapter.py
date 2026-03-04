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
    _normalize_angle,
    build_vda5050_nodes_edges,
    compute_path,
    find_nearest_node,
)
from vda5050_fleet_adapter.usecase.ports.robot_api import (
    CommissionState,
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
        recharge_soc: float = 1.0,
        turn_angle_threshold: float = 0.2618,
        allowed_deviation_theta: float = 0.2618,
    ) -> None:
        """Initialize."""
        self.name = name
        self.api = api
        self.node = node
        self.fleet_handle = fleet_handle
        self.nav_nodes = nav_nodes
        self.nav_edges = nav_edges
        self.nav_graph = nav_graph
        self.arrival_threshold = arrival_threshold
        self.recharge_soc = recharge_soc
        self.turn_angle_threshold = turn_angle_threshold
        self.allowed_deviation_theta = allowed_deviation_theta

        self.execution: Any | None = None
        self.update_handle: Any | None = None
        self.configuration: Any | None = None
        self.cmd_id: int = 0
        self.position: list[float] | None = None

        # Navigate 도착 판정 상태
        self._navigate_target_position: list[float] | None = None
        self._navigate_target_theta: float | None = None
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

        # 충전 상태 관리
        self._is_charging_pending: bool = False   # charger로 이동 중
        self._is_charging: bool = False           # startCharging 완료 대기
        self._charging_station_name: str | None = None
        self._charging_action_id: str | None = None
        self._was_charging: bool = False          # 다음 order에 stopCharging 필요
        self._is_charging_decommissioned: bool = False  # 충전 중 decommission

        # 경로 캐시
        self._last_nodes: list[list] = []
        self._last_stitch_seq_id: int = 0  # 마지막 Base 노드의 sequenceId
        self._last_map: str | None = None  # 마지막 navigate의 맵 이름

        # Commission 상태 추적
        self._last_commission: Any | None = None

        # pickDrop 상태 관리
        self._pick_drop_destination: str | None = None
        self._pick_drop_station_node: str | None = None

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
            if self._is_charging:
                # startCharging action 완료 대기
                # (AGV 도킹+충전 시작 완료 시 FINISHED)
                completed = self.api.is_command_completed(
                    self.name, self.cmd_id
                )
                if completed:
                    self._was_charging = True
                    self._is_charging_decommissioned = True
            elif self._is_navigating and self._navigate_target_position:
                dist = math.hypot(
                    data.position[0]
                    - self._navigate_target_position[0],
                    data.position[1]
                    - self._navigate_target_position[1],
                )
                if dist <= self.arrival_threshold:
                    theta_ok = True
                    if self._navigate_target_theta is not None:
                        angle_diff = abs(_normalize_angle(
                            data.position[2]
                            - self._navigate_target_theta
                        ))
                        theta_ok = (
                            angle_diff
                            <= self.allowed_deviation_theta
                        )
                    if not theta_ok:
                        pass  # 회전 미완료, 도착 보류
                    elif self._is_charging_pending:
                        # Pre-charger 도착 → Phase 2 전환
                        self._is_navigating = False
                        self._is_charging_pending = False
                        self._is_charging = True
                        logger.info(
                            'Pre-charger arrival for robot %s, '
                            'transitioning to charging mode '
                            '(station=%s)',
                            self.name,
                            self._charging_station_name,
                        )
                    else:
                        completed = True
            else:
                completed = self.api.is_command_completed(
                    self.name, self.cmd_id
                )
            if completed:
                self.execution.finished()
                self.execution = None
                self._is_navigating = False
                self._is_charging = False
                self._is_charging_pending = False
                self._charging_station_name = None
                self._charging_action_id = None
                self._navigate_target_position = None
                self._navigate_target_theta = None
            elif not self._is_charging:
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
            self._update_commission()
            self._apply_charging_decommission(data.battery_soc)

    def _update_commission(self) -> None:
        """VDA5050 상태 기반으로 RMF commission을 업데이트한다."""
        commission_state = self.api.get_commission_state(self.name)
        if commission_state is None:
            return

        if commission_state == self._last_commission:
            return

        handle = self.update_handle.more()
        commission = handle.commission()
        commission.accept_dispatched_tasks = (
            commission_state.accept_dispatched_tasks
        )
        commission.accept_direct_tasks = (
            commission_state.accept_direct_tasks
        )
        commission.perform_idle_behavior = (
            commission_state.perform_idle_behavior
        )
        handle.set_commission(commission)
        self._last_commission = commission_state

        logger.info(
            'Commission updated [%s]: dispatch=%s, direct=%s, idle=%s',
            self.name,
            commission_state.accept_dispatched_tasks,
            commission_state.accept_direct_tasks,
            commission_state.perform_idle_behavior,
        )

    def _apply_charging_decommission(
        self, battery_soc: float,
    ) -> None:
        """충전 중 SOC 기반 decommission/recommission을 적용한다.

        startCharging 완료 후 SOC가 recharge_soc에 도달할 때까지
        로봇을 decommission 상태로 유지한다.

        Args:
            battery_soc: 현재 배터리 SOC (0.0~1.0).
        """
        if not self._is_charging_decommissioned:
            return

        if battery_soc >= self.recharge_soc:
            self._is_charging_decommissioned = False
            self._last_commission = None
            logger.info(
                'Charging recommission [%s]: SOC %.2f >= %.2f',
                self.name, battery_soc, self.recharge_soc,
            )
            return

        decommission = CommissionState(False, False, False)
        if self._last_commission == decommission:
            return

        handle = self.update_handle.more()
        commission = handle.commission()
        commission.accept_dispatched_tasks = False
        commission.accept_direct_tasks = False
        commission.perform_idle_behavior = False
        handle.set_commission(commission)
        self._last_commission = decommission

        logger.info(
            'Charging decommission [%s]: SOC %.2f < %.2f',
            self.name, battery_soc, self.recharge_soc,
        )

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
            destination: RMF destination (position, map 등).
            execution: RMF execution handle.
        """
        self.cmd_id += 1
        self.execution = execution
        self._navigate_target_position = list(destination.position[:2])
        self._is_navigating = True

        # 충전 태스크 감지: nav_graph 속성 is_charger로 판단
        dest_name_raw = getattr(destination, 'name', '')
        dest_node_attrs = self.nav_nodes.get(
            dest_name_raw, {}
        ).get('attributes', {})
        dest_is_charger = dest_node_attrs.get('is_charger', False)
        if dest_is_charger and dest_name_raw:
            self._is_charging_pending = True
            self._charging_station_name = dest_name_raw
        else:
            self._is_charging_pending = False
            self._charging_station_name = None

        # pickDrop 감지: nav_graph 속성 pickDrop으로 판단
        dest_is_pick_drop = dest_node_attrs.get('pickDrop', False)
        if dest_is_pick_drop and dest_name_raw:
            self._pick_drop_destination = dest_name_raw
        else:
            self._pick_drop_destination = None

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

        # Station node removal: path[-2]가 pickDrop이면
        # station(path[-1])을 경로에서 제거 (charging 패턴 동일)
        if (
            self._pick_drop_destination is None
            and len(path) >= 2
        ):
            pre_dest = path[-2]
            pre_dest_attrs = self.nav_nodes.get(
                pre_dest, {}
            ).get('attributes', {})
            if pre_dest_attrs.get('pickDrop', False):
                self._pick_drop_station_node = path[-1]
                self._pick_drop_destination = pre_dest
                path = path[:-1]
                target = pre_dest
                self._final_destination = pre_dest
                logger.info(
                    'Station node removal for %s: removed %s, '
                    'pickDrop staging=%s',
                    self.name, self._pick_drop_station_node,
                    pre_dest,
                )

        # pickDrop: 로봇이 이미 destination에 있는 경우 조기 리턴
        if (
            self._pick_drop_destination is not None
            and len(path) == 1
            and path[0] == self._pick_drop_destination
        ):
            self._active_order_id = None
            self._is_navigating = False
            self.execution.finished()
            self.execution = None
            logger.info(
                'Robot %s already at pickDrop destination %s, '
                'completing immediately',
                self.name, self._pick_drop_destination,
            )
            return

        # ── Step 2: 3-tier 경로 구성 (경로 조립 관점) ──
        # tier 1 (Horizon - RMF 경로):      path[0 : rmf_path_end+1]
        # tier 2 (Horizon - 최종목적지 확장): path[rmf_path_end+1 :]
        # tier 3 (Base):                    path[0 : base_end_index+1]
        rmf_path_end = len(path) - 1

        # Tier 2 확장: path에 최종목적지가 없으면
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
                    'Tier2 extension for %s: %s -> %s '
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

        # ── pickDrop: dest를 horizon으로 유지, 직전 노드까지 base ──
        if (
            self._pick_drop_destination is not None
            and len(path) >= 2
            and base_end_index >= 1
        ):
            base_end_index -= 1
            pre_pick_drop = path[base_end_index]
            self._navigate_target_position = [
                self.nav_nodes[pre_pick_drop]['x'],
                self.nav_nodes[pre_pick_drop]['y'],
            ]
            logger.info(
                'pickDrop: base_end adjusted for robot %s, '
                'pre_dest=%s, pick_drop_dest=%s, '
                'base_end_index=%d',
                self.name, pre_pick_drop,
                self._pick_drop_destination,
                base_end_index,
            )

        # ── Charging: charger 노드를 경로에서 항상 제거 ──
        # _final_destination이 is_charger 속성 노드이면,
        # 경로 끝의 charger 노드를 제거한다 (모든 navigate 호출에서).
        _final_is_charger = (
            self._final_destination is not None
            and self.nav_nodes.get(
                self._final_destination, {}
            ).get('attributes', {}).get('is_charger', False)
        )
        if (
            _final_is_charger
            and len(path) >= 2
            and path[-1] == self._final_destination
        ):
            path = path[:-1]
            if rmf_path_end >= len(path):
                rmf_path_end = len(path) - 1
            if self._is_charging_pending:
                # dest가 charger: pre-charger 도착 타겟, 전체 base
                pre_charger = path[-1]
                self._navigate_target_position = [
                    self.nav_nodes[pre_charger]['x'],
                    self.nav_nodes[pre_charger]['y'],
                ]
                base_end_index = len(path) - 1
            logger.info(
                'Charging: removed charger node for robot %s, '
                'pre_charger=%s, station=%s, path=%s',
                self.name, path[-1],
                self._final_destination, path,
            )

        logger.info(
            'Navigate [%s] 3-tier path: '
            'Horizon-RMF(tier1)=%s, Horizon-ext(tier2)=%s, '
            'Base(tier3)=%s',
            self.name,
            path[:rmf_path_end + 1],
            path[rmf_path_end + 1:],
            path[:base_end_index + 1],
        )

        # ── Step 3: VDA5050 Node/Edge 생성 ──
        # Order update 시 stitching node의 sequenceId를 유지한다.
        seq_start = (
            self._last_stitch_seq_id
            if self._order_update_id > 0
            else 0
        )

        map_name = destination.map
        self._last_map = map_name
        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            path, self.nav_nodes, map_name,
            base_end_index=base_end_index,
            seq_start=seq_start,
            edges=self.nav_edges,
            turn_angle_threshold=self.turn_angle_threshold,
            allowed_deviation_theta=self.allowed_deviation_theta,
        )

        # Base 도착 노드의 theta를 도착 판정에 사용
        base_node_pos = vda_nodes[base_end_index].node_position
        self._navigate_target_theta = (
            base_node_pos.theta if base_node_pos else None
        )

        # ── Charging nodeAction 부착 ──
        if vda_nodes and (
            self._is_charging_pending or self._was_charging
        ):
            from vda5050_fleet_adapter.domain.entities.action import (
                Action, ActionParameter,
            )
            from vda5050_fleet_adapter.domain.enums import BlockingType

            # stopCharging: 이전 충전 상태 → 첫 번째 노드에 부착
            if self._was_charging:
                stop_action = Action(
                    action_type='stopCharging',
                    action_id=(
                        f'stopCharging_{self.cmd_id}'
                        f'_{uuid.uuid4().hex[:8]}'
                    ),
                    blocking_type=BlockingType.HARD,
                    action_parameters=[],
                )
                vda_nodes[0].actions.append(stop_action)
                logger.info(
                    'Attached stopCharging to node[0] for '
                    'robot %s (action_id=%s)',
                    self.name, stop_action.action_id,
                )
                self._was_charging = False
                self._is_charging_decommissioned = False

            # startCharging: 마지막 base 노드에 부착
            if self._is_charging_pending:
                charge_action = Action(
                    action_type='startCharging',
                    action_id=(
                        f'startCharging_{self.cmd_id}'
                        f'_{uuid.uuid4().hex[:8]}'
                    ),
                    blocking_type=BlockingType.HARD,
                    action_parameters=[
                        ActionParameter(
                            key='stationName',
                            value=self._charging_station_name,
                        ),
                    ],
                )
                last_base_idx = min(
                    base_end_index, len(vda_nodes) - 1,
                )
                vda_nodes[last_base_idx].actions.append(
                    charge_action
                )
                self._charging_action_id = (
                    charge_action.action_id
                )
                logger.info(
                    'Attached startCharging to node[%d] for '
                    'robot %s (station=%s, action_id=%s)',
                    last_base_idx, self.name,
                    self._charging_station_name,
                    charge_action.action_id,
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

        # 충전 navigate: startCharging action 완료를 추적
        track_id = (
            self._charging_action_id
            if self._is_charging_pending
            else None
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
                return self.api.navigate(
                    *args, track_action_id=track_id
                )

            self.attempt_cmd_until_success(
                cmd=cancel_then_navigate,
                args=nav_args,
            )
        else:
            if track_id:
                self.attempt_cmd_until_success(
                    cmd=lambda *a: self.api.navigate(
                        *a, track_action_id=track_id
                    ),
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
                # 충전 중이면 다음 order에 stopCharging 부착 예약
                if self._is_charging:
                    self._was_charging = True
                    self._is_charging_decommissioned = False
                    self._is_charging = False
                    self._is_charging_pending = False
                    self._charging_station_name = None
                    self._charging_action_id = None
                self.execution = None
                self._navigate_target_theta = None
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

        Active order가 있으면 VDA5050 nodeAction으로 order update를 전송한다.
        Active order가 없으면 기존 instantAction 방식을 사용한다.

        Args:
            category: 액션 카테고리 (pick, drop, charge 등).
            description: 액션 설명 dict.
            execution: RMF execution handle.
        """
        self.cmd_id += 1
        self.execution = execution
        self._is_navigating = False

        logger.info(
            'Execute action [%s]: category=%s, cmd_id=%d, '
            'active_order=%s',
            self.name, category, self.cmd_id,
            self._active_order_id,
        )

        params = description if isinstance(description, dict) else {}

        if (
            self._active_order_id is not None
            and self.position is not None
        ):
            self._execute_action_as_order_update(category, params)
        else:
            self.attempt_cmd_until_success(
                cmd=self.api.start_activity,
                args=(self.name, self.cmd_id, category, params),
            )

    def _execute_action_as_order_update(
        self, category: str, params: dict,
    ) -> None:
        """Active order에 nodeAction을 추가하는 order update를 전송한다.

        pickDrop destination이 설정된 경우: 현재 노드 → pickDrop dest로
        모든 노드를 base로 전송, action을 마지막 노드에 부착 후
        order 라이프사이클을 리셋한다.

        그 외: 현재 위치의 노드에 action을 HARD blocking으로 첨부하여
        order update로 전송한다. 완료 추적은 action_id 기반.

        Args:
            category: 액션 종류 (e.g. 'pick', 'drop').
            params: 액션 파라미터 dict.
        """
        from vda5050_fleet_adapter.domain.entities.action import (
            Action, ActionParameter,
        )
        from vda5050_fleet_adapter.domain.enums import BlockingType

        current_node = find_nearest_node(
            self.nav_nodes, self.position[0], self.position[1]
        )
        if current_node is None:
            logger.error(
                'No nearest node for action, robot %s', self.name,
            )
            return

        action_id = (
            f'{category}_{self.cmd_id}_{uuid.uuid4().hex[:8]}'
        )
        action_params = [
            ActionParameter(key=k, value=v)
            for k, v in params.items()
        ]
        action = Action(
            action_type=category,
            action_id=action_id,
            blocking_type=BlockingType.HARD,
            action_parameters=action_params,
        )

        self._order_update_id += 1

        if self._pick_drop_destination is not None:
            self._build_pick_drop_order_update(
                current_node, action, action_id, category,
            )
        else:
            self._build_default_order_update(
                current_node, action, action_id, category,
            )

    def _build_default_order_update(
        self,
        current_node: str,
        action: Any,
        action_id: str,
        category: str,
    ) -> None:
        """기본 order update: 현재 노드(Base) + 최종목적지(Horizon).

        Args:
            current_node: 현재 위치 노드.
            action: VDA5050 Action 객체.
            action_id: action 추적용 ID.
            category: 액션 종류.
        """
        # 경로 구성: 현재 노드 (Base) + 최종 목적지까지 Horizon
        path = [current_node]
        if (
            self._final_destination
            and self._final_destination != current_node
        ):
            extension = compute_path(
                self.nav_graph, current_node,
                self._final_destination,
            )
            if extension and len(extension) > 1:
                path = path + extension[1:]

        map_name = self._last_map or 'map1'
        seq_start = self._last_stitch_seq_id

        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            path, self.nav_nodes, map_name,
            base_end_index=0,
            seq_start=seq_start,
            edges=self.nav_edges,
        )

        # Base 노드에 action 첨부
        vda_nodes[0].actions.append(action)

        # Stitch seq 유지 (base_end_index=0 → seq_start + 0)
        self._last_stitch_seq_id = seq_start

        logger.info(
            'Action as order update [%s]: category=%s, '
            'action_id=%s, order_id=%s, update_id=%d, '
            'node=%s, path=%s',
            self.name, category, action_id,
            self._active_order_id, self._order_update_id,
            current_node, path,
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

        self.attempt_cmd_until_success(
            cmd=lambda *a: self.api.navigate(
                *a, track_action_id=action_id
            ),
            args=nav_args,
        )

    def _build_pick_drop_order_update(
        self,
        current_node: str,
        action: Any,
        action_id: str,
        category: str,
    ) -> None:
        """Build pick/drop order update with all nodes as base.

        전송 후 order 라이프사이클을 리셋하여 다음 navigate가
        새 orderID를 생성하도록 한다.

        Args:
            current_node: 현재 위치 노드.
            action: VDA5050 Action 객체.
            action_id: action 추적용 ID.
            category: 액션 종류.
        """
        pick_drop_dest = self._pick_drop_destination

        # 경로: 현재 노드 → pickDrop destination
        if current_node == pick_drop_dest:
            path = [current_node]
        else:
            computed = compute_path(
                self.nav_graph, current_node, pick_drop_dest,
            )
            if computed and len(computed) > 0:
                path = computed
            else:
                path = [current_node, pick_drop_dest]

        map_name = self._last_map or 'map1'
        seq_start = self._last_stitch_seq_id
        base_end_index = len(path) - 1  # 모든 노드 base

        vda_nodes, vda_edges = build_vda5050_nodes_edges(
            path, self.nav_nodes, map_name,
            base_end_index=base_end_index,
            seq_start=seq_start,
            edges=self.nav_edges,
        )

        # 마지막 노드(pickDrop dest)에 action 첨부
        vda_nodes[-1].actions.append(action)

        # nav_args에 order_id를 미리 캡처 (리셋 전)
        order_id = self._active_order_id
        update_id = self._order_update_id

        logger.info(
            'pickDrop action order update [%s]: category=%s, '
            'action_id=%s, order_id=%s, update_id=%d, '
            'dest=%s, path=%s',
            self.name, category, action_id,
            order_id, update_id,
            pick_drop_dest, path,
        )

        nav_args = (
            self.name,
            self.cmd_id,
            vda_nodes,
            vda_edges,
            map_name,
            order_id,
            update_id,
        )

        self.attempt_cmd_until_success(
            cmd=lambda *a: self.api.navigate(
                *a, track_action_id=action_id
            ),
            args=nav_args,
        )

        # Order 라이프사이클 리셋: 다음 navigate가 새 orderID 생성
        self._active_order_id = None
        self._order_update_id = 0
        self._last_stitch_seq_id = 0
        self._pick_drop_destination = None
        self._pick_drop_station_node = None

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

        if self._pick_drop_destination is not None:
            self._pick_drop_destination = None
            self._pick_drop_station_node = None
            self.cmd_id += 1
            logger.info(
                'Task ended with pickDrop pending, sending '
                'cancelOrder for robot %s, cmd_id=%d',
                self.name, self.cmd_id,
            )
            self.attempt_cmd_until_success(
                cmd=self.api.stop,
                args=(self.name, self.cmd_id),
            )

        self._navigate_target_position = None
        self._navigate_target_theta = None
        self._is_navigating = False
        self._is_charging = False
        self._is_charging_pending = False
        self._charging_station_name = None
        self._charging_action_id = None
        self._pick_drop_station_node = None
        # _was_charging은 보존: 다음 order에서 stopCharging 전송 필요

        old_task_id = self._current_task_id
        self._active_order_id = None
        self._order_update_id = 0
        self._final_destination = None
        self._current_task_id = None
        self._last_nodes = []
        self._last_stitch_seq_id = 0
        self._last_map = None
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
