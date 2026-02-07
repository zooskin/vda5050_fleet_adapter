# CLAUDE.md - VDA5050 Fleet Adapter Project Rules

## Project Overview
VDA5050 Fleet Adapter: ROS 2 (Jazzy) + Open-RMF fleet adapter bridging VDA5050 AGVs via MQTT.

## Tech Stack
- Python 3.12+ / ROS 2 Jazzy Jalisco (Ubuntu 24.04)
- `rmf_fleet_adapter_python`, `rclpy`, MQTT (paho-mqtt)
- Build: `ament_python` / `colcon`

## Architecture: Clean Architecture
```
vda5050_fleet_adapter/
  domain/         # Entities, Value Objects, Domain Events (no external deps)
  usecase/        # Application logic, Port interfaces (ABC)
  infra/          # Adapters: MQTT client, ROS 2 nodes, config loaders
  presentation/   # Entry points, CLI, ROS 2 launch integration
```
- Dependencies flow inward only: infra -> usecase -> domain
- Domain layer MUST NOT import rclpy, paho-mqtt, or any framework
- Use Port/Adapter (ABC interfaces in usecase/, implementations in infra/)

## Coding Rules

### Style
- PEP 8 strict, enforced by flake8
- Line length: 99 characters max
- Docstrings: Google style, PEP 257 compliant
- Type hints required on all public functions/methods
- snake_case for functions/variables, PascalCase for classes, UPPER_SNAKE for constants

### Naming Conventions
- ROS 2 nodes: `vda5050_` prefix (e.g., `vda5050_fleet_adapter_node`)
- MQTT topics: follow VDA5050 spec (`/interfaceName/version/manufacturer/serialNumber/topic`)
- Config keys: snake_case in YAML

### Error Handling
- Never use bare `except:`; always catch specific exceptions
- ROS 2 logging (`self.get_logger()`) for node-level logging
- Python `logging` module for non-ROS components
- Log levels: DEBUG for protocol details, INFO for state changes, WARN for recoverable errors, ERROR for failures

### Testing
- pytest for unit tests, launch_testing for integration
- Unit tests MUST NOT depend on ROS 2 runtime or MQTT broker
- Mock external dependencies at Port boundaries
- Test file naming: `test_<module_name>.py`

### VDA5050 Specific
- All VDA5050 message models defined as dataclasses in domain/
- Strict validation of VDA5050 JSON schemas on send/receive
- AGV state machine transitions must be explicit and logged

## Build & Test Commands
```bash
# Build
cd ~/rmf_ws && colcon build --packages-select vda5050_fleet_adapter

# Test
colcon test --packages-select vda5050_fleet_adapter
colcon test-result --verbose

# Lint
cd ~/rmf_ws/src/vda5050_fleet_adapter && python -m flake8
python -m pytest test/
```

## Key Constraints
- Do NOT add ROS 2 message type definitions here; use existing rmf_fleet_msgs
- MQTT QoS: use QoS 1 for state/visualization, QoS 2 for orders/instant actions
- All config values must be loadable from YAML parameters, no hardcoded values
- Thread safety: use `threading.Lock` for shared state between MQTT callbacks and ROS 2 callbacks
