"""즉시 액션 처리 유스케이스.

VDA5050 Instant Actions(일시정지, 취소, 충전 등)을
AGV에 전송하는 로직을 담당한다.
"""

from vda5050_fleet_adapter.domain.entities.action import Action
from vda5050_fleet_adapter.domain.enums import BlockingType
from vda5050_fleet_adapter.domain.events.agv_events import OrderCancelledEvent
from vda5050_fleet_adapter.domain.exceptions import AgvNotFoundError
from vda5050_fleet_adapter.usecase.ports.agv_gateway import AgvGateway
from vda5050_fleet_adapter.usecase.ports.event_publisher import EventPublisher
from vda5050_fleet_adapter.usecase.ports.state_repository import StateRepository


class HandleAction:
    """즉시 액션 처리 유스케이스.

    Args:
        agv_gateway: AGV 통신 포트.
        state_repo: 상태 저장소.
        event_publisher: 이벤트 발행자.
    """

    def __init__(
        self,
        agv_gateway: AgvGateway,
        state_repo: StateRepository,
        event_publisher: EventPublisher,
    ) -> None:
        self._agv_gateway = agv_gateway
        self._state_repo = state_repo
        self._event_publisher = event_publisher

    def pause(self, agv_id: str, action_id: str) -> None:
        """AGV를 일시 정지한다.

        Args:
            agv_id: 대상 AGV 식별자.
            action_id: 액션 고유 ID.
        """
        self._send_instant_action(
            agv_id, "startPause", action_id, BlockingType.HARD
        )

    def resume(self, agv_id: str, action_id: str) -> None:
        """AGV 일시 정지를 해제한다.

        Args:
            agv_id: 대상 AGV 식별자.
            action_id: 액션 고유 ID.
        """
        self._send_instant_action(
            agv_id, "stopPause", action_id, BlockingType.HARD
        )

    def cancel_order(self, agv_id: str, action_id: str) -> None:
        """AGV의 현재 주문을 취소한다.

        Args:
            agv_id: 대상 AGV 식별자.
            action_id: 액션 고유 ID.
        """
        current_order = self._state_repo.get_current_order(agv_id)

        self._send_instant_action(
            agv_id, "cancelOrder", action_id, BlockingType.HARD
        )

        self._state_repo.clear_order(agv_id)

        if current_order is not None:
            self._event_publisher.publish(
                OrderCancelledEvent(
                    agv_id=agv_id,
                    order_id=current_order.order_id,
                )
            )

    def start_charging(self, agv_id: str, action_id: str) -> None:
        """AGV 충전을 시작한다.

        Args:
            agv_id: 대상 AGV 식별자.
            action_id: 액션 고유 ID.
        """
        self._send_instant_action(
            agv_id, "startCharging", action_id, BlockingType.HARD
        )

    def stop_charging(self, agv_id: str, action_id: str) -> None:
        """AGV 충전을 중지한다.

        Args:
            agv_id: 대상 AGV 식별자.
            action_id: 액션 고유 ID.
        """
        self._send_instant_action(
            agv_id, "stopCharging", action_id, BlockingType.HARD
        )

    def send_custom_action(
        self,
        agv_id: str,
        action: Action,
    ) -> None:
        """사용자 정의 즉시 액션을 전송한다.

        Args:
            agv_id: 대상 AGV 식별자.
            action: 전송할 액션.
        """
        self._ensure_agv_exists(agv_id)
        self._agv_gateway.send_instant_actions(agv_id, [action])

    def _send_instant_action(
        self,
        agv_id: str,
        action_type: str,
        action_id: str,
        blocking_type: BlockingType,
    ) -> None:
        """표준 즉시 액션을 생성하여 전송한다."""
        self._ensure_agv_exists(agv_id)

        action = Action(
            action_type=action_type,
            action_id=action_id,
            blocking_type=blocking_type,
        )
        self._agv_gateway.send_instant_actions(agv_id, [action])

    def _ensure_agv_exists(self, agv_id: str) -> None:
        """AGV 등록 여부를 확인한다."""
        if self._state_repo.get_state(agv_id) is None:
            raise AgvNotFoundError(
                f"AGV [{agv_id}]가 등록되어 있지 않습니다."
            )
