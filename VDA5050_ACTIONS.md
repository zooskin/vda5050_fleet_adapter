# VDA5050 Actions Definition

## Overview

VDA5050 fleet adapter에서 지원하는 action 목록과 정의를 관리하는 문서.
Action은 3가지 방식으로 AGV에 전달된다:

| 전달 방식 | 설명 | 사용 시점 |
|-----------|------|----------|
| **nodeAction** | Order의 Node에 첨부 | 특정 위치 도착 시 실행 |
| **edgeAction** | Order의 Edge에 첨부 | 이동 중 실행 |
| **instantAction** | Order와 독립적으로 전송 | 즉시 실행 (활성 order 없을 때) |

---

## Action 공통 구조

```json
{
  "actionType": "string",
  "actionId": "string (UUID)",
  "blockingType": "NONE | SOFT | HARD",
  "actionDescription": "string (optional)",
  "actionParameters": [
    { "key": "string", "value": "string | number | boolean | array" }
  ]
}
```

### BlockingType 정의

| Type | 설명 |
|------|------|
| `NONE` | 주행과 병렬 실행 가능 |
| `SOFT` | 다른 action과 병렬 가능, 주행은 차단 |
| `HARD` | 모든 것을 차단 (독점 실행) |

---

## 현재 구현된 Action

### System Actions (내부 사용)

| actionType | blockingType | 전달 방식 | 설명 | 비고 |
|------------|-------------|----------|------|------|
| `cancelOrder` | HARD | instantAction | 현재 order 취소 | negotiation 시 사용 |
| `startPause` | HARD | instantAction | AGV 일시 정지 | negotiation 시 사용 |
| `stopPause` | HARD | instantAction | 일시 정지 해제 | - |

### RMF Task Actions

| actionType | blockingType | 전달 방식 | 설명 | actionParameters |
|------------|-------------|----------|------|-----------------|
| `teleop` | HARD | instantAction | 원격 조종 | - |
| `pick` | HARD | nodeAction / instantAction | 화물 적재 | `loadType`, `loadID` |
| `drop` | HARD | nodeAction / instantAction | 화물 하역 | `stationName` |

#### `pick` Action 상세

카트/화물 적재 action. RMF Task 생성 시 매개변수로 전달받는다.

```json
{
  "actionType": "pick",
  "actionId": "pick-patrol.dispatch-0",
  "blockingType": "HARD",
  "actionParameters": [
    { "key": "loadType", "value": "Tool" },
    { "key": "loadID", "value": "SP4ECTR002" }
  ]
}
```

| Parameter | Type | 설명 | 예시 | 소스 |
|-----------|------|------|------|------|
| `loadType` | string | 적재물 타입 종류 | `"Tool"`, `"Pallet"`, `"Box"` | Task 매개변수 |
| `loadID` | string | 카트(적재물) 고유 ID | `"SP4ECTR002"` | Task 매개변수 |

#### `drop` Action 상세

카트/화물 하역 action. RMF Task 생성 시 매개변수로 전달받는다.

```json
{
  "actionType": "drop",
  "actionId": "drop-patrol.dispatch-0",
  "blockingType": "HARD",
  "actionParameters": [
    { "key": "stationName", "value": "1004" }
  ]
}
```

| Parameter | Type | 설명 | 예시 | 소스 |
|-----------|------|------|------|------|
| `stationName` | string | 하역할 스테이션 이름 | `"1004"` | Task 매개변수 |

---

## 추가 예정 Action

### Navigation / Positioning

| actionType | blockingType | 전달 방식 | 설명 | actionParameters | 구현 상태 |
|------------|-------------|----------|------|-----------------|----------|
| `initPosition` | HARD | instantAction | AGV 위치 초기화 | `x`, `y`, `theta`, `mapId`, `lastNodeId` | [ ] |
| `finePositioning` | HARD | nodeAction | 정밀 위치 조정 | `stationType`, `stationName` | [ ] |

### Charging

| actionType | blockingType | 전달 방식 | 설명 | actionParameters | 구현 상태 |
|------------|-------------|----------|------|-----------------|----------|
| `startCharging` | HARD | nodeAction | 충전 시작 (charger 앞 노드에 부착) | `stationName` | [x] |
| `stopCharging` | HARD | instantAction | 충전 중지 (SOC 도달 시 자동 전송) | - | [x] |

### Cargo Handling

> `pick`/`drop`은 RMF Task Actions로 이동됨 (위 섹션 참조)

### Map Management

| actionType | blockingType | 전달 방식 | 설명 | actionParameters | 구현 상태 |
|------------|-------------|----------|------|-----------------|----------|
| `downloadMap` | NONE | instantAction | 맵 다운로드 | `mapId`, `mapDownloadUrl` | [ ] |
| `enableMap` | NONE | instantAction | 맵 활성화 | `mapId` | [ ] |
| `deleteMap` | NONE | instantAction | 맵 삭제 | `mapId` | [ ] |

### Trigger / Wait

| actionType | blockingType | 전달 방식 | 설명 | actionParameters | 구현 상태 |
|------------|-------------|----------|------|-----------------|----------|
| `waitForTrigger` | HARD | nodeAction | 외부 트리거 대기 | `triggerType`, `timeout` | [ ] |

### Information

| actionType | blockingType | 전달 방식 | 설명 | actionParameters | 구현 상태 |
|------------|-------------|----------|------|-----------------|----------|
| `factsheetRequest` | NONE | instantAction | AGV factsheet 요청 | - | [ ] |

### Custom Actions (프로젝트 특화)

| actionType | blockingType | 전달 방식 | 설명 | actionParameters | 구현 상태 |
|------------|-------------|----------|------|-----------------|----------|
| | | | | | [ ] |

---

## Action 구현 체크리스트

새로운 action 추가 시 아래 항목을 모두 완료해야 한다:

- [ ] `domain/entities/action.py` - 필요시 action 관련 상수/헬퍼 추가
- [ ] `config/config.yaml` - `actions` 목록에 actionType 등록
- [ ] `usecase/robot_adapter.py` - execute_action 콜백에서 처리 로직 구현
- [ ] `infra/mqtt/vda5050_robot_api.py` - 전송 로직 (nodeAction / instantAction)
- [ ] `infra/mqtt/message_serializer.py` - 직렬화/역직렬화 확인
- [ ] `test/` - 단위 테스트 작성
- [ ] 이 문서(`VDA5050_ACTIONS.md`) 업데이트

---

## Config 등록 방법

`config.yaml`의 `actions` 목록에 actionType을 추가:

```yaml
rmf_fleet:
  actions: ["teleop", "pick", "drop", "NEW_ACTION_TYPE"]
```

---

## Action 흐름도

```
RMF Task
  │
  ├─ navigate() ──────────────────────────► Order (nodeAction/edgeAction)
  │                                              │
  ├─ execute_action() ─┬─ active order 있음 ──► Order Update (nodeAction)
  │                    └─ active order 없음 ──► InstantAction
  │
  └─ stop() ──────────────────────────────► InstantAction (startPause)

                         ┌──────────────────┐
                         │   AGV (VDA5050)   │
                         └────────┬─────────┘
                                  │
                         State (actionStates)
                                  │
                    ┌─────────────▼──────────────┐
                    │ WAITING → INITIALIZING →    │
                    │ RUNNING → FINISHED/FAILED   │
                    └─────────────────────────────┘
```
