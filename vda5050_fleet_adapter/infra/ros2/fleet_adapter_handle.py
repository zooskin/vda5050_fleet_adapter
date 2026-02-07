"""RMF Fleet Adapter 핸들 구현체.

rmf_fleet_adapter_python의 핸들을 래핑하여
usecase 포트의 NavigationHandle/TaskHandle을 구현한다.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

from vda5050_fleet_adapter.usecase.ports.fleet_gateway import (
    NavigationHandle,
    TaskHandle,
)

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


class RmfTaskHandle(TaskHandle):
    """TaskHandle의 RMF 구현체.

    Args:
        activity: RMF activity completion handle.
    """

    def __init__(self, activity: Any) -> None:
        self._activity = activity
        self._finished = False

    def is_finished(self) -> bool:
        return self._finished

    def complete(self) -> None:
        if self._activity is not None:
            self._activity.completed()
        self._finished = True
        logger.info("RMF task completed")

    def fail(self, error_message: str) -> None:
        if self._activity is not None:
            self._activity.failed(error_message)
        self._finished = True
        logger.error("RMF task failed: %s", error_message)


class RmfNavigationHandle(NavigationHandle):
    """NavigationHandle의 RMF 구현체.

    Args:
        waypoints: RMF에서 할당한 경유지 목록.
        completer: 내비게이션 완료 콜백.
    """

    def __init__(
        self,
        waypoints: list[tuple[float, float, float]],
        completer: Any = None,
    ) -> None:
        self._waypoints = waypoints
        self._completer = completer
        self._current_index = 0

    def get_remaining_waypoints(self) -> list[tuple[float, float, float]]:
        return self._waypoints[self._current_index:]

    def arrived_at_waypoint(self, waypoint_index: int) -> None:
        self._current_index = waypoint_index + 1
        logger.debug(
            "Arrived at waypoint %d/%d",
            self._current_index, len(self._waypoints),
        )

    def complete(self) -> None:
        if self._completer is not None:
            self._completer.completed()
        logger.info("RMF navigation completed")

    def fail(self, error_message: str) -> None:
        if self._completer is not None:
            self._completer.failed(error_message)
        logger.error("RMF navigation failed: %s", error_message)
