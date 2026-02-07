"""YAML 파일 기반 설정 로더 구현체."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import yaml

from vda5050_fleet_adapter.usecase.ports.config_port import (
    AdapterConfig,
    AppConfig,
    ConfigPort,
    MqttConfig,
    Vda5050Config,
)

logger = logging.getLogger(__name__)

_DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parent.parent.parent
    / "config"
    / "default_params.yaml"
)


class YamlConfigLoader(ConfigPort):
    """ConfigPort의 YAML 파일 구현체.

    YAML 파일에서 설정을 읽어 AppConfig로 변환한다.
    파일이 없으면 기본값을 사용한다.

    Args:
        config_path: YAML 설정 파일 경로. None이면 기본 경로 사용.
    """

    def __init__(self, config_path: Path | None = None) -> None:
        self._path = config_path or _DEFAULT_CONFIG_PATH

    def load(self) -> AppConfig:
        """YAML 파일에서 설정을 로드한다."""
        raw = self._read_yaml()
        params = self._extract_params(raw)

        mqtt_data = params.get("mqtt", {})
        vda5050_data = params.get("vda5050", {})
        adapter_data = params.get("adapter", {})

        config = AppConfig(
            mqtt=MqttConfig(
                broker_host=mqtt_data.get("broker_host", "localhost"),
                broker_port=mqtt_data.get("broker_port", 1883),
                keepalive_sec=mqtt_data.get("keepalive_sec", 60),
                reconnect_max_delay_sec=mqtt_data.get(
                    "reconnect_max_delay_sec", 60
                ),
            ),
            vda5050=Vda5050Config(
                interface_name=vda5050_data.get("interface_name", "uagv"),
                protocol_version=vda5050_data.get("protocol_version", "v2"),
                manufacturer=vda5050_data.get(
                    "manufacturer", "default_manufacturer"
                ),
            ),
            adapter=AdapterConfig(
                fleet_name=params.get("fleet_name", "vda5050_fleet"),
                state_publish_rate_hz=adapter_data.get(
                    "state_publish_rate_hz", 1.0
                ),
                order_timeout_sec=adapter_data.get("order_timeout_sec", 30.0),
                max_retry_count=adapter_data.get("max_retry_count", 3),
            ),
        )

        logger.info("Config loaded from %s", self._path)
        return config

    def _read_yaml(self) -> dict[str, Any]:
        """YAML 파일을 dict로 읽는다."""
        if not self._path.exists():
            logger.warning(
                "Config file not found: %s, using defaults", self._path
            )
            return {}

        with open(self._path, encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if not isinstance(data, dict):
            logger.warning("Invalid YAML format, using defaults")
            return {}

        return data

    def _extract_params(self, raw: dict[str, Any]) -> dict[str, Any]:
        """YAML 구조에서 ros__parameters 를 추출한다."""
        # vda5050_fleet_adapter.ros__parameters 구조 탐색
        node_data = raw.get("vda5050_fleet_adapter", raw)
        if isinstance(node_data, dict):
            return node_data.get("ros__parameters", node_data)
        return {}
