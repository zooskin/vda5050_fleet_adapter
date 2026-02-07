"""VDA5050 MQTT 어댑터 (AgvGateway 구현체).

VDA5050 토픽 규칙에 따라 MQTT 메시지를 송수신한다.
"""

from __future__ import annotations

import logging
from collections.abc import Callable
from datetime import UTC, datetime

from vda5050_fleet_adapter.domain.entities.action import Action
from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.entities.connection import Connection
from vda5050_fleet_adapter.domain.entities.header import Header
from vda5050_fleet_adapter.domain.entities.order import Order
from vda5050_fleet_adapter.domain.enums import ConnectionState
from vda5050_fleet_adapter.infra.mqtt.message_serializer import (
    deserialize_connection,
    deserialize_state,
    serialize_connection,
    serialize_instant_actions,
    serialize_order,
)
from vda5050_fleet_adapter.infra.mqtt.mqtt_client import MqttClient
from vda5050_fleet_adapter.usecase.ports.agv_gateway import AgvGateway
from vda5050_fleet_adapter.usecase.ports.config_port import Vda5050Config

logger = logging.getLogger(__name__)

# VDA5050 QoS 정책 (CODING_RULES.md 기준)
_QOS_ORDER = 2
_QOS_INSTANT_ACTIONS = 2
_QOS_STATE = 1
_QOS_CONNECTION = 1


class Vda5050MqttAdapter(AgvGateway):
    """AgvGateway의 MQTT 구현체.

    VDA5050 토픽 구조에 따라 AGV와 MQTT 통신을 수행한다.
    토픽 형식: {interface}/{version}/{manufacturer}/{serialNumber}/{topic}

    Args:
        mqtt_client: MQTT 클라이언트 래퍼.
        vda5050_config: VDA5050 프로토콜 설정.
    """

    def __init__(
        self,
        mqtt_client: MqttClient,
        vda5050_config: Vda5050Config,
    ) -> None:
        self._client = mqtt_client
        self._config = vda5050_config
        self._header_counters: dict[str, int] = {}

    def _build_topic(self, serial_number: str, topic_name: str) -> str:
        """VDA5050 토픽 경로를 생성한다."""
        return (
            f"{self._config.interface_name}"
            f"/{self._config.protocol_version}"
            f"/{self._config.manufacturer}"
            f"/{serial_number}"
            f"/{topic_name}"
        )

    def _next_header_id(self, topic_key: str) -> int:
        """토픽별 증가 카운터를 반환한다."""
        count = self._header_counters.get(topic_key, 0) + 1
        self._header_counters[topic_key] = count
        return count

    def _build_header(self, agv_id: str, topic_name: str) -> Header:
        """메시지 헤더를 생성한다."""
        topic_key = f"{agv_id}/{topic_name}"
        return Header(
            header_id=self._next_header_id(topic_key),
            timestamp=datetime.now(UTC),
            version="2.0.0",
            manufacturer=self._config.manufacturer,
            serial_number=agv_id,
        )

    # -- AgvGateway 구현: 명령 전송 --

    def send_order(self, agv_id: str, order: Order) -> None:
        """AGV에 주행 주문을 전송한다."""
        topic = self._build_topic(agv_id, "order")
        order.header = self._build_header(agv_id, "order")
        payload = serialize_order(order)

        self._client.publish(topic, payload, qos=_QOS_ORDER)
        logger.info(
            "Order sent: agv=%s, orderId=%s, updateId=%d",
            agv_id, order.order_id, order.order_update_id,
        )

    def send_instant_actions(
        self, agv_id: str, actions: list[Action]
    ) -> None:
        """AGV에 즉시 실행 액션을 전송한다."""
        topic = self._build_topic(agv_id, "instantActions")
        header = self._build_header(agv_id, "instantActions")
        payload = serialize_instant_actions(header, actions)

        self._client.publish(topic, payload, qos=_QOS_INSTANT_ACTIONS)
        action_types = [a.action_type for a in actions]
        logger.info(
            "InstantActions sent: agv=%s, actions=%s",
            agv_id, action_types,
        )

    # -- AgvGateway 구현: 상태 수신 구독 --

    def subscribe_state(
        self,
        agv_id: str,
        callback: Callable[[str, AgvState], None],
    ) -> None:
        """AGV 상태 메시지 수신을 구독한다."""
        topic = self._build_topic(agv_id, "state")

        def _handler(topic_name: str, payload: bytes) -> None:
            state = deserialize_state(payload.decode("utf-8"))
            callback(agv_id, state)

        self._client.subscribe(topic, _handler, qos=_QOS_STATE)
        logger.info("Subscribed to state: agv=%s", agv_id)

    def subscribe_connection(
        self,
        agv_id: str,
        callback: Callable[[str, Connection], None],
    ) -> None:
        """AGV 연결 상태 메시지 수신을 구독한다."""
        topic = self._build_topic(agv_id, "connection")

        def _handler(topic_name: str, payload: bytes) -> None:
            connection = deserialize_connection(payload.decode("utf-8"))
            callback(agv_id, connection)

        self._client.subscribe(topic, _handler, qos=_QOS_CONNECTION)
        logger.info("Subscribed to connection: agv=%s", agv_id)

    # -- AgvGateway 구현: 연결 관리 --

    def connect(self) -> None:
        """MQTT 브로커에 연결한다."""
        self._client.connect()

    def disconnect(self) -> None:
        """MQTT 브로커 연결을 종료한다."""
        self._client.disconnect()

    # -- Last Will 설정 (connect 전 호출) --

    def setup_last_will(self, agv_id: str) -> None:
        """AGV의 Last Will 메시지를 설정한다.

        connect() 호출 전에 설정해야 한다.

        Args:
            agv_id: AGV 식별자.
        """
        topic = self._build_topic(agv_id, "connection")
        connection = Connection(
            header=self._build_header(agv_id, "connection"),
            connection_state=ConnectionState.CONNECTIONBROKEN,
        )
        payload = serialize_connection(connection)
        self._client.set_last_will(topic, payload, qos=1, retain=True)

    def publish_online(self, agv_id: str) -> None:
        """AGV ONLINE 상태를 발행한다.

        Args:
            agv_id: AGV 식별자.
        """
        topic = self._build_topic(agv_id, "connection")
        connection = Connection(
            header=self._build_header(agv_id, "connection"),
            connection_state=ConnectionState.ONLINE,
        )
        payload = serialize_connection(connection)
        self._client.publish(topic, payload, qos=1, retain=True)
        logger.info("Published ONLINE: agv=%s", agv_id)

    def publish_offline(self, agv_id: str) -> None:
        """AGV OFFLINE 상태를 발행한다.

        Args:
            agv_id: AGV 식별자.
        """
        topic = self._build_topic(agv_id, "connection")
        connection = Connection(
            header=self._build_header(agv_id, "connection"),
            connection_state=ConnectionState.OFFLINE,
        )
        payload = serialize_connection(connection)
        self._client.publish(topic, payload, qos=1, retain=True)
        logger.info("Published OFFLINE: agv=%s", agv_id)
