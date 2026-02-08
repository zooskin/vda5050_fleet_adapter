"""ROS 2 Task API 리스너.

/task_api_requests, /task_api_responses, /fleet_states 토픽을 구독하여
Task의 최종 목적지를 추적한다.
"""

from __future__ import annotations

import json
import logging
import threading
from typing import Any

from vda5050_fleet_adapter.usecase.ports.task_tracker import TaskTracker

logger = logging.getLogger(__name__)


class TaskApiListener(TaskTracker):
    """ROS 2 토픽 기반 TaskTracker 구현.

    데이터 소스:
      1. /task_api_requests + /task_api_responses: 외부 dispatch task의
         request_id → booking_id → destination 상관관계.
      2. /fleet_states: 내부 task (ParkRobot 등) 포함 모든 task의
         task_id → path[-1] → destination 매핑.

    Args:
        node: ROS 2 노드 (구독 생성용).
        fleet_name: Fleet 이름 (fleet_states 필터링용).
        index_to_name: Nav graph vertex index → waypoint name 매핑.
    """

    def __init__(
        self,
        node: Any,
        fleet_name: str = '',
        index_to_name: list[str] | None = None,
    ) -> None:
        self._node = node
        self._fleet_name = fleet_name
        self._index_to_name = index_to_name or []
        self._lock = threading.Lock()

        # Phase 1 cache: request_id -> destination waypoint name
        self._request_destinations: dict[str, str] = {}
        # Final cache: booking_id/task_id -> destination waypoint name
        self._booking_destinations: dict[str, str] = {}

        self._setup_subscriptions()

    def _setup_subscriptions(self) -> None:
        """ROS 2 구독을 설정한다."""
        from rclpy.qos import QoSDurabilityPolicy as Durability
        from rclpy.qos import QoSHistoryPolicy as History
        from rclpy.qos import QoSProfile
        from rclpy.qos import QoSReliabilityPolicy as Reliability
        from rmf_fleet_msgs.msg import FleetState
        from rmf_task_msgs.msg import ApiRequest, ApiResponse

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        self._node.create_subscription(
            ApiRequest,
            'task_api_requests',
            self._on_task_request,
            transient_qos,
        )

        self._node.create_subscription(
            ApiResponse,
            'task_api_responses',
            self._on_task_response,
            transient_qos,
        )

        # fleet_states: 기본 QoS (RELIABLE, VOLATILE)
        default_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.VOLATILE,
        )

        self._node.create_subscription(
            FleetState,
            '/fleet_states',
            self._on_fleet_state,
            default_qos,
        )

        logger.info(
            'TaskApiListener subscribed to task_api and fleet_states '
            'topics (fleet=%s, index_to_name=%d entries)',
            self._fleet_name, len(self._index_to_name),
        )

    def get_final_destination(self, booking_id: str) -> str | None:
        """Task의 최종 목적지 waypoint 이름을 반환한다.

        Args:
            booking_id: RMF booking ID (current_task_id() 반환값).

        Returns:
            최종 목적지 waypoint 이름, 또는 아직 매핑 전이면 None.
        """
        with self._lock:
            return self._booking_destinations.get(booking_id)

    def clear_booking(self, booking_id: str) -> None:
        """완료된 task의 캐시를 정리한다.

        Args:
            booking_id: 정리할 booking ID.
        """
        with self._lock:
            self._booking_destinations.pop(booking_id, None)

    def _on_fleet_state(self, msg: Any) -> None:
        """Fleet_states 콜백.

        각 로봇의 planned path 마지막 waypoint에서
        최종 목적지를 추출한다.

        Args:
            msg: rmf_fleet_msgs/msg/FleetState 메시지.
        """
        if self._fleet_name and msg.name != self._fleet_name:
            return

        if not self._index_to_name:
            return

        for robot_state in msg.robots:
            task_id = robot_state.task_id
            if not task_id or not robot_state.path:
                continue

            last_loc = robot_state.path[-1]
            idx = int(last_loc.index)
            if idx >= len(self._index_to_name):
                continue

            dest_name = self._index_to_name[idx]
            with self._lock:
                if task_id not in self._booking_destinations:
                    self._booking_destinations[task_id] = dest_name
                    logger.info(
                        'Fleet state destination: task_id=%s, '
                        'robot=%s, destination=%s (index=%d)',
                        task_id, robot_state.name, dest_name, idx,
                    )

    def _on_task_request(self, msg: Any) -> None:
        """Task_api_requests 콜백.

        Request JSON에서 최종 목적지를 추출하여 캐시한다.

        Args:
            msg: rmf_task_msgs/msg/ApiRequest 메시지.
        """
        try:
            payload = json.loads(msg.json_msg)
            request_id = msg.request_id
            destination = self._extract_destination(payload)

            if destination is not None:
                with self._lock:
                    self._request_destinations[request_id] = destination
                logger.info(
                    'Task request cached: request_id=%s, '
                    'destination=%s',
                    request_id, destination,
                )
            else:
                logger.info(
                    'Task request has no extractable destination: '
                    'request_id=%s, payload_keys=%s',
                    request_id,
                    list(payload.get('request', {}).keys()),
                )
        except (json.JSONDecodeError, KeyError, TypeError):
            logger.warning(
                'Could not parse task request: request_id=%s',
                getattr(msg, 'request_id', 'unknown'),
            )

    def _on_task_response(self, msg: Any) -> None:
        """Task_api_responses 콜백.

        TYPE_RESPONDING 응답에서 booking_id를 추출하고,
        request_id와 결합하여 booking_id -> destination 매핑을 생성.

        Args:
            msg: rmf_task_msgs/msg/ApiResponse 메시지.
        """
        # TYPE_RESPONDING = 2
        if msg.type != 2:
            return

        try:
            payload = json.loads(msg.json_msg)
            request_id = msg.request_id

            if not payload.get('success', False):
                return

            state = payload.get('state', {})
            booking = state.get('booking', {})
            booking_id = booking.get('id', '')

            if not booking_id:
                return

            with self._lock:
                destination = self._request_destinations.pop(
                    request_id, None
                )
                if destination is not None:
                    self._booking_destinations[booking_id] = destination
                    logger.info(
                        'Task correlation complete: '
                        'booking_id=%s, destination=%s',
                        booking_id, destination,
                    )
                else:
                    logger.warning(
                        'Response without matching request: '
                        'request_id=%s, booking_id=%s',
                        request_id, booking_id,
                    )
        except (json.JSONDecodeError, KeyError, TypeError):
            logger.debug(
                'Could not parse task response: request_id=%s',
                getattr(msg, 'request_id', 'unknown'),
            )

    def _extract_destination(self, payload: dict) -> str | None:
        """Task request payload에서 최종 목적지를 추출한다.

        Supports go_to_place and patrol task types.

        Args:
            payload: 파싱된 task request JSON dict.

        Returns:
            최종 목적지 waypoint 이름, 또는 추출 불가 시 None.
        """
        request = payload.get('request', {})
        description = request.get('description', {})
        inner_category = description.get('category', '')

        if inner_category == 'go_to_place':
            return self._extract_go_to_place_dest(description)

        category = request.get('category', '')
        if category == 'patrol':
            return self._extract_patrol_dest(description)

        # Fallback: try both
        dest = self._extract_go_to_place_dest(description)
        if dest is None:
            dest = self._extract_patrol_dest(description)
        return dest

    def _extract_go_to_place_dest(
        self, description: dict
    ) -> str | None:
        """Go_to_place 요청에서 목적지를 추출한다.

        Path: phases[-1].activity.description.one_of[-1].waypoint

        Args:
            description: task description dict.

        Returns:
            Waypoint 이름 또는 None.
        """
        try:
            phases = description.get('phases', [])
            if not phases:
                return None
            last_phase = phases[-1]
            activity = last_phase.get('activity', {})
            activity_desc = activity.get('description', {})
            one_of = activity_desc.get('one_of', [])
            if not one_of:
                return None
            return one_of[-1].get('waypoint')
        except (KeyError, IndexError, TypeError):
            return None

    def _extract_patrol_dest(
        self, description: dict
    ) -> str | None:
        """Patrol 요청에서 최종 목적지를 추출한다.

        Path: places[-1]

        Args:
            description: task description dict.

        Returns:
            Waypoint 이름 또는 None.
        """
        try:
            places = description.get('places', [])
            if not places:
                return None
            return places[-1]
        except (KeyError, IndexError, TypeError):
            return None
