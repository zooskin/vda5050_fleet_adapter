"""설정 로더 포트 인터페이스.

애플리케이션 설정의 로딩을 추상화한다.
ROS 2 파라미터 또는 YAML 파일로 구현 가능하다.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass(frozen=True)
class MqttConfig:
    """MQTT 브로커 접속 설정.

    Args:
        broker_host: 브로커 호스트 주소.
        broker_port: 브로커 포트 번호.
        keepalive_sec: 연결 유지 간격 (초).
        reconnect_max_delay_sec: 재연결 최대 대기 시간 (초).
    """

    broker_host: str = "localhost"
    broker_port: int = 1883
    keepalive_sec: int = 60
    reconnect_max_delay_sec: int = 60


@dataclass(frozen=True)
class Vda5050Config:
    """VDA5050 프로토콜 설정.

    Args:
        interface_name: MQTT 토픽 인터페이스명.
        protocol_version: VDA5050 프로토콜 메이저 버전.
        manufacturer: AGV 제조사명.
    """

    interface_name: str = "uagv"
    protocol_version: str = "v2"
    manufacturer: str = "default_manufacturer"


@dataclass(frozen=True)
class AdapterConfig:
    """Fleet Adapter 동작 설정.

    Args:
        fleet_name: RMF Fleet 이름.
        state_publish_rate_hz: State 발행 주기 (Hz).
        order_timeout_sec: 주문 응답 타임아웃 (초).
        max_retry_count: 최대 재시도 횟수.
    """

    fleet_name: str = "vda5050_fleet"
    state_publish_rate_hz: float = 1.0
    order_timeout_sec: float = 30.0
    max_retry_count: int = 3


@dataclass(frozen=True)
class AppConfig:
    """애플리케이션 전체 설정.

    Args:
        mqtt: MQTT 설정.
        vda5050: VDA5050 프로토콜 설정.
        adapter: Adapter 동작 설정.
    """

    mqtt: MqttConfig = MqttConfig()
    vda5050: Vda5050Config = Vda5050Config()
    adapter: AdapterConfig = AdapterConfig()


class ConfigPort(ABC):
    """설정 로더 인터페이스."""

    @abstractmethod
    def load(self) -> AppConfig:
        """설정을 로드하여 반환한다.

        Returns:
            애플리케이션 전체 설정.
        """
