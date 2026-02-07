"""VDA5050 Fleet Adapter ROS 2 노드.

DI 조립 및 전체 생명주기를 관리하는 오케스트레이션 노드이다.
모든 의존성은 이 레이어에서 조립(wire)되어 usecase에 주입된다.
"""

from __future__ import annotations

import logging
import uuid
from pathlib import Path

import rclpy
from rclpy.node import Node

from vda5050_fleet_adapter.infra.config import YamlConfigLoader
from vda5050_fleet_adapter.infra.event import InMemoryEventPublisher
from vda5050_fleet_adapter.infra.mqtt import MqttClient, Vda5050MqttAdapter
from vda5050_fleet_adapter.infra.repository import InMemoryStateRepository
from vda5050_fleet_adapter.infra.ros2 import Ros2FleetGateway
from vda5050_fleet_adapter.usecase import (
    HandleAction,
    ProcessOrder,
    UpdateAgvState,
)
from vda5050_fleet_adapter.usecase.ports.config_port import AppConfig
from vda5050_fleet_adapter.usecase.ports.fleet_gateway import NavigationHandle

logger = logging.getLogger(__name__)

_NODE_NAME = "vda5050_fleet_adapter_node"


class FleetAdapterNode(Node):
    """VDA5050 Fleet Adapter ROS 2 노드.

    DI 조립, MQTT/RMF 연결, 콜백 등록을 수행한다.

    Args:
        config_path: YAML 설정 파일 경로. None이면 기본 경로 사용.
    """

    def __init__(self, config_path: Path | None = None) -> None:
        super().__init__(_NODE_NAME)

        # -- 1. 설정 로드 --
        self._config = self._load_config(config_path)
        self.get_logger().info(
            "Config loaded: fleet=%s, mqtt=%s:%d",
            self._config.adapter.fleet_name,
            self._config.mqtt.broker_host,
            self._config.mqtt.broker_port,
        )

        # -- 2. 인프라 어댑터 생성 --
        self._state_repo = InMemoryStateRepository()
        self._event_publisher = InMemoryEventPublisher()

        self._mqtt_client = MqttClient(
            config=self._config.mqtt,
            client_id=f"{_NODE_NAME}_{uuid.uuid4().hex[:8]}",
        )
        self._agv_gateway = Vda5050MqttAdapter(
            mqtt_client=self._mqtt_client,
            vda5050_config=self._config.vda5050,
        )

        self._fleet_gateway = Ros2FleetGateway(config=self._config)

        # -- 3. 유스케이스 생성 (DI) --
        self._process_order = ProcessOrder(
            agv_gateway=self._agv_gateway,
            state_repo=self._state_repo,
            event_publisher=self._event_publisher,
            config=self._config,
        )

        self._update_agv_state = UpdateAgvState(
            state_repo=self._state_repo,
            fleet_gateway=self._fleet_gateway,
            event_publisher=self._event_publisher,
        )

        self._handle_action = HandleAction(
            agv_gateway=self._agv_gateway,
            state_repo=self._state_repo,
            event_publisher=self._event_publisher,
        )

        # -- 4. RMF 콜백 등록 --
        self._fleet_gateway.on_navigate(self._on_rmf_navigate)
        self._fleet_gateway.on_stop(self._on_rmf_stop)
        self._fleet_gateway.on_action(self._on_rmf_action)

        # -- 5. 상태 발행 타이머 --
        period_sec = 1.0 / self._config.adapter.state_publish_rate_hz
        self._timer = self.create_timer(period_sec, self._on_timer)

        self.get_logger().info("FleetAdapterNode initialized")

    def start(self, agv_ids: list[str] | None = None) -> None:
        """어댑터를 시작한다.

        MQTT 연결, AGV 구독, RMF Fleet 등록을 수행한다.

        Args:
            agv_ids: 관리할 AGV ID 목록. None이면 설정에서 읽는다.
        """
        agv_ids = agv_ids or []

        # MQTT 연결
        self._agv_gateway.connect()
        self.get_logger().info("MQTT connected")

        # AGV별 구독 및 RMF 등록
        for agv_id in agv_ids:
            self._register_agv(agv_id)

        # RMF Fleet Gateway 시작
        self._fleet_gateway.start()
        self.get_logger().info(
            "Fleet adapter started with %d AGV(s): %s",
            len(agv_ids), agv_ids,
        )

    def shutdown(self) -> None:
        """어댑터를 종료한다."""
        self.get_logger().info("Shutting down fleet adapter")

        # AGV OFFLINE 발행
        for agv_id in self._state_repo.get_all_agv_ids():
            try:
                self._agv_gateway.publish_offline(agv_id)
            except Exception:
                self.get_logger().warning(
                    "Failed to publish OFFLINE for %s", agv_id
                )

        self._agv_gateway.disconnect()
        self._fleet_gateway.shutdown()
        self.get_logger().info("Fleet adapter shut down")

    # -- AGV 등록 --

    def _register_agv(self, agv_id: str) -> None:
        """AGV를 MQTT 구독 및 RMF에 등록한다."""
        # MQTT 상태/연결 구독
        self._agv_gateway.subscribe_state(
            agv_id, self._on_agv_state
        )
        self._agv_gateway.subscribe_connection(
            agv_id, self._on_agv_connection
        )

        # ONLINE 발행
        self._agv_gateway.publish_online(agv_id)

        # RMF 등록
        self._fleet_gateway.register_agv(agv_id)

        self.get_logger().info("AGV registered: %s", agv_id)

    # -- MQTT 수신 콜백 --

    def _on_agv_state(self, agv_id: str, state: object) -> None:
        """AGV State 메시지 수신 핸들러."""
        from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
        if not isinstance(state, AgvState):
            return
        try:
            self._update_agv_state.handle_state(agv_id, state)
        except Exception:
            self.get_logger().exception(
                "Error handling state from %s", agv_id
            )

    def _on_agv_connection(self, agv_id: str, connection: object) -> None:
        """AGV Connection 메시지 수신 핸들러."""
        from vda5050_fleet_adapter.domain.entities.connection import (
            Connection,
        )
        if not isinstance(connection, Connection):
            return
        try:
            self._update_agv_state.handle_connection(agv_id, connection)
            self.get_logger().info(
                "AGV [%s] connection: %s",
                agv_id, connection.connection_state.value,
            )
        except Exception:
            self.get_logger().exception(
                "Error handling connection from %s", agv_id
            )

    # -- RMF 수신 콜백 --

    def _on_rmf_navigate(
        self, agv_id: str, nav_handle: NavigationHandle
    ) -> None:
        """RMF 내비게이션 명령 수신 핸들러."""
        waypoints = nav_handle.get_remaining_waypoints()
        self.get_logger().info(
            "RMF navigate: agv=%s, waypoints=%d", agv_id, len(waypoints),
        )

        try:
            order_id = f"rmf_{uuid.uuid4().hex[:12]}"
            map_id = self._get_current_map_id(agv_id)
            wp_with_map = [
                (x, y, theta, map_id) for x, y, theta in waypoints
            ]
            self._process_order.send_new_order(
                agv_id, order_id, wp_with_map
            )
        except Exception:
            self.get_logger().exception(
                "Error processing navigate for %s", agv_id
            )
            nav_handle.fail("Order creation failed")

    def _on_rmf_stop(self, agv_id: str) -> None:
        """RMF 정지 명령 수신 핸들러."""
        self.get_logger().info("RMF stop: agv=%s", agv_id)
        try:
            action_id = f"stop_{uuid.uuid4().hex[:12]}"
            self._handle_action.pause(agv_id, action_id)
        except Exception:
            self.get_logger().exception(
                "Error processing stop for %s", agv_id
            )

    def _on_rmf_action(
        self, agv_id: str, action_type: str, parameters: dict
    ) -> None:
        """RMF 액션 명령 수신 핸들러."""
        self.get_logger().info(
            "RMF action: agv=%s, type=%s", agv_id, action_type,
        )
        try:
            action_id = f"rmf_{uuid.uuid4().hex[:12]}"
            if action_type == "cancel":
                self._handle_action.cancel_order(agv_id, action_id)
            elif action_type == "charge":
                self._handle_action.start_charging(agv_id, action_id)
            else:
                from vda5050_fleet_adapter.domain.entities.action import (
                    Action,
                    ActionParameter,
                )
                from vda5050_fleet_adapter.domain.enums import BlockingType

                action_params = [
                    ActionParameter(key=k, value=v)
                    for k, v in parameters.items()
                ]
                action = Action(
                    action_type=action_type,
                    action_id=action_id,
                    blocking_type=BlockingType.HARD,
                    action_parameters=action_params,
                )
                self._handle_action.send_custom_action(agv_id, action)
        except Exception:
            self.get_logger().exception(
                "Error processing action %s for %s", action_type, agv_id,
            )

    # -- 타이머 --

    def _on_timer(self) -> None:
        """주기적 상태 체크 타이머 콜백."""
        # 향후 확장: 타임아웃 감지, heartbeat 등
        pass

    # -- 헬퍼 --

    def _get_current_map_id(self, agv_id: str) -> str:
        """AGV의 현재 맵 ID를 반환한다."""
        state = self._state_repo.get_state(agv_id)
        if state is not None and state.agv_position is not None:
            return state.agv_position.map_id
        return "default_map"

    def _load_config(self, config_path: Path | None) -> AppConfig:
        """설정을 로드하고 ROS 2 파라미터 오버라이드를 적용한다."""
        loader = YamlConfigLoader(config_path)
        config = loader.load()

        # ROS 2 파라미터 오버라이드
        self.declare_parameter("fleet_name", config.adapter.fleet_name)
        self.declare_parameter("mqtt.broker_host", config.mqtt.broker_host)
        self.declare_parameter("mqtt.broker_port", config.mqtt.broker_port)
        self.declare_parameter(
            "vda5050.manufacturer", config.vda5050.manufacturer
        )

        from vda5050_fleet_adapter.usecase.ports.config_port import (
            AdapterConfig,
            MqttConfig,
            Vda5050Config,
        )

        fleet_name = (
            self.get_parameter("fleet_name")
            .get_parameter_value().string_value
        )
        broker_host = (
            self.get_parameter("mqtt.broker_host")
            .get_parameter_value().string_value
        )
        broker_port = (
            self.get_parameter("mqtt.broker_port")
            .get_parameter_value().integer_value
        )
        manufacturer = (
            self.get_parameter("vda5050.manufacturer")
            .get_parameter_value().string_value
        )

        return AppConfig(
            mqtt=MqttConfig(
                broker_host=broker_host or config.mqtt.broker_host,
                broker_port=broker_port or config.mqtt.broker_port,
                keepalive_sec=config.mqtt.keepalive_sec,
                reconnect_max_delay_sec=config.mqtt.reconnect_max_delay_sec,
            ),
            vda5050=Vda5050Config(
                interface_name=config.vda5050.interface_name,
                protocol_version=config.vda5050.protocol_version,
                manufacturer=manufacturer or config.vda5050.manufacturer,
            ),
            adapter=AdapterConfig(
                fleet_name=fleet_name or config.adapter.fleet_name,
                state_publish_rate_hz=config.adapter.state_publish_rate_hz,
                order_timeout_sec=config.adapter.order_timeout_sec,
                max_retry_count=config.adapter.max_retry_count,
            ),
        )
