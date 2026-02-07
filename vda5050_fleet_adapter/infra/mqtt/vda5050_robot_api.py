"""VDA5050 MQTT 기반 RobotAPI 구현.

VDA5050 프로토콜을 통해 AGV와 통신하는 RobotAPI 어댑터.
MQTT로 Order/InstantActions를 발행하고, State를 구독하여
로봇 상태를 캐시한다.
"""

from __future__ import annotations

from datetime import datetime, UTC
import logging
import threading
from typing import Any
import uuid

from vda5050_fleet_adapter.domain.entities.action import Action
from vda5050_fleet_adapter.domain.entities.edge import Edge
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.entities.node import Node
from vda5050_fleet_adapter.domain.entities.order import Order
from vda5050_fleet_adapter.domain.enums import ActionStatus, BlockingType
from vda5050_fleet_adapter.infra.mqtt.message_serializer import (
    deserialize_state,
    serialize_instant_actions,
    serialize_order,
)
from vda5050_fleet_adapter.infra.mqtt.mqtt_client import MqttClient
from vda5050_fleet_adapter.usecase.ports.robot_api import (
    RobotAPI,
    RobotAPIResult,
    RobotUpdateData,
)

logger = logging.getLogger(__name__)

# VDA5050 프로토콜 버전
_VDA5050_VERSION = '2.0.0'


class Vda5050RobotAPI(RobotAPI):
    """VDA5050 MQTT 프로토콜 기반 RobotAPI 구현.

    Args:
        mqtt_client: MQTT 클라이언트.
        prefix: MQTT 토픽 prefix (e.g. 'uagv/v2/manufacturer').
        manufacturer: AGV 제조사명.
    """

    def __init__(
        self,
        mqtt_client: MqttClient,
        prefix: str,
        manufacturer: str = '',
    ) -> None:
        self._mqtt = mqtt_client
        self._prefix = prefix
        self._manufacturer = manufacturer
        self._lock = threading.Lock()

        # 로봇별 캐시 상태: {robot_name: AgvState}
        self._state_cache: dict[str, Any] = {}
        # 로봇별 명령 ID 매핑: {robot_name: {cmd_id: order_id}}
        self._cmd_order_map: dict[str, dict[int, str]] = {}
        # 로봇별 완료된 cmd_id: {robot_name: set(cmd_id)}
        self._completed_cmds: dict[str, set[int]] = {}
        # 토픽별 header ID 카운터
        self._header_ids: dict[str, int] = {}

    def subscribe_robot(self, robot_name: str) -> None:
        """로봇의 MQTT 토픽을 구독한다.

        Args:
            robot_name: 구독할 로봇 이름.
        """
        state_topic = self._build_topic(robot_name, 'state')
        self._mqtt.subscribe(
            state_topic,
            lambda t, p: self._on_state_message(robot_name, p),
            qos=0,
        )
        logger.info('Subscribed to %s', state_topic)

        connection_topic = self._build_topic(robot_name, 'connection')
        self._mqtt.subscribe(
            connection_topic,
            lambda t, p: self._on_connection_message(robot_name, p),
            qos=0,
        )
        logger.info('Subscribed to %s', connection_topic)

    def connect(self) -> None:
        """MQTT 브로커에 연결한다."""
        self._mqtt.connect()

    def disconnect(self) -> None:
        """MQTT 브로커 연결을 종료한다."""
        self._mqtt.disconnect()

    def navigate(
        self,
        robot_name: str,
        cmd_id: int,
        nodes: list[Node],
        edges: list[Edge],
        map_name: str,
    ) -> RobotAPIResult:
        """VDA5050 Order를 전송하여 내비게이션을 시작한다.

        Args:
            robot_name: 로봇 이름.
            cmd_id: 명령 ID.
            nodes: VDA5050 Node 목록.
            edges: VDA5050 Edge 목록.
            map_name: 대상 맵 이름.

        Returns:
            명령 결과.
        """
        if not self._mqtt.is_connected:
            logger.warning('MQTT not connected, will retry navigate')
            return RobotAPIResult.RETRY

        order_id = f'order_{cmd_id}_{uuid.uuid4().hex[:8]}'
        header = self._make_header(robot_name, 'order')

        order = Order(
            header=header,
            order_id=order_id,
            order_update_id=0,
            nodes=nodes,
            edges=edges,
        )

        payload = serialize_order(order)
        topic = self._build_topic(robot_name, 'order')
        self._mqtt.publish(topic, payload, qos=0)

        with self._lock:
            if robot_name not in self._cmd_order_map:
                self._cmd_order_map[robot_name] = {}
            self._cmd_order_map[robot_name][cmd_id] = order_id
            if robot_name not in self._completed_cmds:
                self._completed_cmds[robot_name] = set()

        logger.info(
            'Navigate order sent: robot=%s, cmd_id=%d, order_id=%s, '
            'nodes=%d, edges=%d',
            robot_name, cmd_id, order_id, len(nodes), len(edges),
        )
        return RobotAPIResult.SUCCESS

    def stop(self, robot_name: str, cmd_id: int) -> RobotAPIResult:
        """Cancel order instant action을 전송한다.

        Args:
            robot_name: 로봇 이름.
            cmd_id: 명령 ID.

        Returns:
            명령 결과.
        """
        if not self._mqtt.is_connected:
            logger.warning('MQTT not connected, will retry stop')
            return RobotAPIResult.RETRY

        header = self._make_header(robot_name, 'instantActions')
        action = Action(
            action_type='cancelOrder',
            action_id=f'cancel_{cmd_id}_{uuid.uuid4().hex[:8]}',
            blocking_type=BlockingType.HARD,
        )

        payload = serialize_instant_actions(header, [action])
        topic = self._build_topic(robot_name, 'instantActions')
        self._mqtt.publish(topic, payload, qos=0)

        logger.info('Stop (cancelOrder) sent: robot=%s', robot_name)
        return RobotAPIResult.SUCCESS

    def start_activity(
        self,
        robot_name: str,
        cmd_id: int,
        activity: str,
        action_params: dict,
    ) -> RobotAPIResult:
        """VDA5050 instant action을 전송한다.

        Args:
            robot_name: 로봇 이름.
            cmd_id: 명령 ID.
            activity: 액션 종류.
            action_params: 액션 파라미터 dict.

        Returns:
            명령 결과.
        """
        if not self._mqtt.is_connected:
            logger.warning('MQTT not connected, will retry activity')
            return RobotAPIResult.RETRY

        header = self._make_header(robot_name, 'instantActions')

        from vda5050_fleet_adapter.domain.entities.action import (
            ActionParameter,
        )

        params = [
            ActionParameter(key=k, value=v)
            for k, v in action_params.items()
        ]
        action = Action(
            action_type=activity,
            action_id=f'{activity}_{cmd_id}_{uuid.uuid4().hex[:8]}',
            blocking_type=BlockingType.HARD,
            action_parameters=params,
        )

        payload = serialize_instant_actions(header, [action])
        topic = self._build_topic(robot_name, 'instantActions')
        self._mqtt.publish(topic, payload, qos=0)

        with self._lock:
            if robot_name not in self._cmd_order_map:
                self._cmd_order_map[robot_name] = {}
            self._cmd_order_map[robot_name][cmd_id] = action.action_id

        logger.info(
            'Activity sent: robot=%s, activity=%s, cmd_id=%d',
            robot_name, activity, cmd_id,
        )
        return RobotAPIResult.SUCCESS

    def get_data(self, robot_name: str) -> RobotUpdateData | None:
        """로봇의 현재 상태 데이터를 반환한다.

        Args:
            robot_name: 로봇 이름.

        Returns:
            상태 데이터 또는 아직 수신 전이면 None.
        """
        with self._lock:
            state = self._state_cache.get(robot_name)

        if state is None:
            return None

        position = [0.0, 0.0, 0.0]
        map_name = ''
        if state.agv_position is not None:
            position = [
                state.agv_position.x,
                state.agv_position.y,
                state.agv_position.theta,
            ]
            map_name = state.agv_position.map_id

        battery_soc = 0.0
        if state.battery_state is not None:
            battery_soc = state.battery_state.battery_charge / 100.0

        return RobotUpdateData(
            robot_name=robot_name,
            map_name=map_name,
            position=position,
            battery_soc=battery_soc,
        )

    def is_command_completed(
        self, robot_name: str, cmd_id: int
    ) -> bool:
        """명령 완료 여부를 확인한다.

        AGV state의 nodeStates가 비어있고 driving=False이면
        내비게이션이 완료된 것으로 판단한다.
        instantAction의 경우 actionStates에서 FINISHED/FAILED를 확인한다.

        Args:
            robot_name: 로봇 이름.
            cmd_id: 확인할 명령 ID.

        Returns:
            완료 여부.
        """
        with self._lock:
            if robot_name in self._completed_cmds:
                if cmd_id in self._completed_cmds[robot_name]:
                    return True

            state = self._state_cache.get(robot_name)
            order_or_action_id = (
                self._cmd_order_map.get(robot_name, {}).get(cmd_id)
            )

        if state is None or order_or_action_id is None:
            return False

        # Order 완료 확인: nodeStates가 비어있고 driving=False
        if order_or_action_id.startswith('order_'):
            if state.order_id == order_or_action_id:
                if not state.node_states and not state.driving:
                    with self._lock:
                        self._completed_cmds.setdefault(
                            robot_name, set()
                        ).add(cmd_id)
                    return True
            # Order ID가 달라졌으면 이전 주문은 완료된 것
            elif state.order_id != order_or_action_id:
                if not state.order_id or state.order_id == '':
                    with self._lock:
                        self._completed_cmds.setdefault(
                            robot_name, set()
                        ).add(cmd_id)
                    return True
        else:
            # InstantAction 완료 확인
            for action_state in state.action_states:
                if action_state.action_id == order_or_action_id:
                    if action_state.action_status in (
                        ActionStatus.FINISHED, ActionStatus.FAILED
                    ):
                        with self._lock:
                            self._completed_cmds.setdefault(
                                robot_name, set()
                            ).add(cmd_id)
                        return True

        return False

    def _on_state_message(
        self, robot_name: str, payload: bytes
    ) -> None:
        """State 토픽 콜백."""
        try:
            state = deserialize_state(payload.decode('utf-8'))
            with self._lock:
                self._state_cache[robot_name] = state
            logger.debug(
                'State received: robot=%s, order=%s, driving=%s',
                robot_name, state.order_id, state.driving,
            )
        except Exception:
            logger.exception(
                'Failed to deserialize state: robot=%s', robot_name
            )

    def _on_connection_message(
        self, robot_name: str, payload: bytes
    ) -> None:
        """Handle connection topic callback."""
        try:
            from vda5050_fleet_adapter.infra.mqtt.message_serializer import (
                deserialize_connection,
            )
            connection = deserialize_connection(payload.decode('utf-8'))
            logger.info(
                'Connection update: robot=%s, state=%s',
                robot_name, connection.connection_state,
            )
        except Exception:
            logger.exception(
                'Failed to deserialize connection: robot=%s', robot_name
            )

    def _build_topic(self, robot_name: str, topic_name: str) -> str:
        """MQTT 토픽을 생성한다.

        Args:
            robot_name: 로봇 이름.
            topic_name: 토픽 이름 (order, state, instantActions 등).

        Returns:
            '{prefix}/{robot_name}/{topic_name}' 형태.
        """
        return f'{self._prefix}/{robot_name}/{topic_name}'

    def _make_header(self, robot_name: str, topic: str) -> Header:
        """VDA5050 메시지 헤더를 생성한다.

        Args:
            robot_name: 로봇 이름.
            topic: 토픽 이름 (header ID 카운터 키).

        Returns:
            Header 인스턴스.
        """
        key = f'{robot_name}/{topic}'
        with self._lock:
            hid = self._header_ids.get(key, 0)
            self._header_ids[key] = hid + 1

        return Header(
            version=_VDA5050_VERSION,
            manufacturer=self._manufacturer,
            serial_number=robot_name,
            header_id=hid,
            timestamp=datetime.now(UTC),
        )
