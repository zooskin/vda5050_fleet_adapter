"""AGV 통신 포트 인터페이스.

MQTT를 통한 AGV와의 통신을 추상화한다.
infra/mqtt/ 레이어에서 구현한다.
"""

from abc import ABC, abstractmethod
from collections.abc import Callable

from vda5050_fleet_adapter.domain.entities.action import Action
from vda5050_fleet_adapter.domain.entities.agv_state import AgvState
from vda5050_fleet_adapter.domain.entities.connection import Connection
from vda5050_fleet_adapter.domain.entities.order import Order


class AgvGateway(ABC):
    """AGV와의 통신을 담당하는 포트.

    VDA5050 프로토콜에 따라 AGV에 명령을 전송하고,
    AGV로부터 상태를 수신하는 인터페이스를 정의한다.
    """

    # -- 명령 전송 (Master → AGV) --

    @abstractmethod
    def send_order(self, agv_id: str, order: Order) -> None:
        """AGV에 주행 주문을 전송한다.

        Args:
            agv_id: 대상 AGV 식별자.
            order: 전송할 VDA5050 주문.

        Raises:
            MqttConnectionError: MQTT 연결 실패 시.
        """

    @abstractmethod
    def send_instant_actions(
        self, agv_id: str, actions: list[Action]
    ) -> None:
        """AGV에 즉시 실행 액션을 전송한다.

        Args:
            agv_id: 대상 AGV 식별자.
            actions: 즉시 실행할 액션 목록.

        Raises:
            MqttConnectionError: MQTT 연결 실패 시.
        """

    # -- 상태 수신 구독 (AGV → Master) --

    @abstractmethod
    def subscribe_state(
        self,
        agv_id: str,
        callback: Callable[[str, AgvState], None],
    ) -> None:
        """AGV 상태 메시지 수신을 구독한다.

        Args:
            agv_id: 대상 AGV 식별자.
            callback: 상태 수신 시 호출할 콜백 (agv_id, state).
        """

    @abstractmethod
    def subscribe_connection(
        self,
        agv_id: str,
        callback: Callable[[str, Connection], None],
    ) -> None:
        """AGV 연결 상태 메시지 수신을 구독한다.

        Args:
            agv_id: 대상 AGV 식별자.
            callback: 연결 상태 수신 시 호출할 콜백 (agv_id, connection).
        """

    # -- 연결 관리 --

    @abstractmethod
    def connect(self) -> None:
        """MQTT 브로커에 연결한다.

        Raises:
            MqttConnectionError: 연결 실패 시.
        """

    @abstractmethod
    def disconnect(self) -> None:
        """MQTT 브로커 연결을 종료한다."""
