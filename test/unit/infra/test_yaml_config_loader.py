"""YamlConfigLoader 단위 테스트."""

from pathlib import Path

import pytest

from vda5050_fleet_adapter.infra.config import YamlConfigLoader


class TestYamlConfigLoader:
    def test_load_default_config(self):
        loader = YamlConfigLoader()
        config = loader.load()

        assert config.mqtt.broker_host == "localhost"
        assert config.mqtt.broker_port == 1883
        assert config.mqtt.keepalive_sec == 60
        assert config.vda5050.interface_name == "uagv"
        assert config.vda5050.protocol_version == "v2"
        assert config.adapter.fleet_name == "vda5050_fleet"
        assert config.adapter.state_publish_rate_hz == 1.0
        assert config.adapter.order_timeout_sec == 30.0
        assert config.adapter.max_retry_count == 3

    def test_missing_file_returns_defaults(self, tmp_path):
        loader = YamlConfigLoader(tmp_path / "nonexistent.yaml")
        config = loader.load()

        assert config.mqtt.broker_host == "localhost"
        assert config.adapter.fleet_name == "vda5050_fleet"

    def test_custom_config(self, tmp_path):
        config_file = tmp_path / "custom.yaml"
        config_file.write_text("""
vda5050_fleet_adapter:
  ros__parameters:
    fleet_name: "custom_fleet"
    mqtt:
      broker_host: "192.168.1.100"
      broker_port: 8883
    vda5050:
      manufacturer: "CustomCo"
    adapter:
      state_publish_rate_hz: 2.0
      max_retry_count: 5
""")
        loader = YamlConfigLoader(config_file)
        config = loader.load()

        assert config.mqtt.broker_host == "192.168.1.100"
        assert config.mqtt.broker_port == 8883
        assert config.vda5050.manufacturer == "CustomCo"
        assert config.adapter.fleet_name == "custom_fleet"
        assert config.adapter.state_publish_rate_hz == 2.0
        assert config.adapter.max_retry_count == 5

    def test_partial_config_uses_defaults(self, tmp_path):
        config_file = tmp_path / "partial.yaml"
        config_file.write_text("""
vda5050_fleet_adapter:
  ros__parameters:
    mqtt:
      broker_host: "10.0.0.1"
""")
        loader = YamlConfigLoader(config_file)
        config = loader.load()

        assert config.mqtt.broker_host == "10.0.0.1"
        assert config.mqtt.broker_port == 1883  # default

    def test_empty_file_returns_defaults(self, tmp_path):
        config_file = tmp_path / "empty.yaml"
        config_file.write_text("")

        loader = YamlConfigLoader(config_file)
        config = loader.load()

        assert config.mqtt.broker_host == "localhost"
