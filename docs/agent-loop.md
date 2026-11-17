# 관찰 기반 에이전트 실행

`harness-run --execute`는 자연어 작업을 짧은 행동으로 나누고, 매번 실제 실행 결과와 새 관찰을 받아 다음 행동을 결정합니다. 기본 `harness-run`의 관측·제안 모드는 유지합니다. 팔 XYZ 목표는 Pinocchio와 OSQP로 관절값을 구한 뒤 같은 실행 경로를 사용합니다.

## 역할

```text
자연어 작업 → harness의 에피소드 루프 → MCP observe/execute
                                          ↓
                                  server의 ZeroMQ 중계
                                          ↓
                            lekiwi_runtime의 검증·실행·측정
                                          ↓
                              결과 + 새 관찰 → 다음 판단
```

- `harness`: 같은 작업 문맥에서 한 번에 하나의 판단을 받고, 행동·시간·단계 예산을 검사한 뒤 실행합니다. 모델은 모터 통신을 직접 수행하지 않습니다.
- `server`: 로봇별 요청을 중계합니다. 기존 `request_id`는 요청과 응답을 매칭하는 통신 ID입니다.
- `lekiwi_runtime`: 실제 측정값과 관찰 유효성을 검사하고, 관절·바퀴를 제어합니다. 관찰 ID와 행동 ID를 통신 ID와 별도로 관리합니다.

## 실행

server와 runtime을 먼저 실행합니다. 로봇 없이 시험하려면 runtime에 `--mock`을 지정합니다.

```bash
uv run --project server server
uv run --project lekiwi_runtime lekiwi-runtime \
  --server tcp://127.0.0.1:5555 --robot-id kiwi-01 --mock
```

다른 터미널에서 실행합니다. Codex 로그인과 사용 가능한 모델이 필요합니다.

```bash
uv run --project harness harness-run --execute \
  --robot-id kiwi-01 --max-actions 20 --max-steps 40 --max-seconds 300 \
  '현재 관측을 확인하고 100ms 대기한 뒤 결과를 확인해줘.'
```

실물 runtime에 연결된 `--execute`는 실제 동작을 허용합니다. 기본 모드와 `--snapshot`은 관측·제안용이며, 저장된 관찰로 실행하는 것은 허용하지 않습니다. 실행 기록은 기본 `runs/` 아래에 저장하고 `--run-dir`로 경로를 지정합니다. 기록을 읽는 것은 가능하지만, 과거 행동을 자동 재개하거나 재생하지 않습니다.

`uv run --project harness harness-gui`로 로컬 GUI를 열면 브라우저에서 같은 실행 루프를 시작·중지할 수 있습니다. 기본 주소는 http://127.0.0.1:8765 입니다. GUI는 실행 기록을 읽어 단계별 계획·판단 근거 요약·예상 변화·피드백·관찰 이미지를 표시합니다. 별도의 관찰 polling으로 실행 중인 관찰 ID를 바꾸지 않습니다. 과거 기록도 읽기 전용으로 조회합니다. `--demo`는 실제 모델·로봇 없이 고정된 모의 루프를 보여줍니다.

실행 모드의 모델은 로그인만 공유하는 임시 Codex 환경에서 동작합니다. 기존 사용자 MCP·플러그인·셸 설정은 상속하지 않습니다. `--model`로 실행 모델을 지정할 수 있습니다. `--max-actions`는 전송을 시도한 행동 수, `--max-steps`는 모델 판단 수, `--max-seconds`는 초기 연결을 포함한 실행 시간 예산입니다. SDK 초기화에는 최대 30초, 종료 정리에는 별도 유예 시간을 둡니다.

## 콘솔 loop에서 반영한 부분

참고한 구현은 `/home/jwshin/Documents/Codex/2026-11-06/z/outputs/lekiwi-console/agent_runtime.py`와 `post_motion_observation.py`입니다.

- 관찰 → 계획·근거 → 한 동작 → 측정 결과 → 새 관찰 순서를 사용합니다.
- 계획은 판단 기록이며, 미래 동작을 큐에 넣지 않습니다. 다음 동작은 새 관찰로 다시 결정합니다.
- 예상 변화와 관찰된 결과를 기록하고, 완료 주장은 한 번 더 관찰한 뒤 확인합니다.
- 모델의 완료 주장과 모터 명령 완료를 구분합니다. 영상상의 완료 후보가 물리적 성공 검증을 뜻하지는 않습니다.

기존 콘솔의 HTTP UI·세션·조그 갱신 루프를 새 ZeroMQ 경로에 직접 연결한 것은 아닙니다. 콘솔과 harness가 서로의 자율 루프를 호출하지 않도록 합니다. 콘솔의 FK/IK는 하드웨어 정렬 미검증인 미리보기 기능이므로 실행 기능으로 이전하지 않았습니다.

## 제어 단위와 피드백

| 값 | 의미 |
|---|---|
| `x`, `y` | 베이스 전방·왼쪽 속도, m/s. 위치 좌표가 아님 |
| `theta` | 반시계 각속도, deg/s |
| `duration_ms` | 팔/그리퍼 단독·XYZ는 100–20000ms, 베이스·복합 이동·대기는 최대 1000ms |
| `joints` | 팔의 절대 관절각, deg. 생략한 관절 유지 |
| `joints.gripper` | 보정된 그리퍼 값, 0–100% |

팔은 기존 보정·범위·속도 검사, 목표 보간과 관절 피드백 확인을 사용합니다. 베이스는 시간 제한 속도 명령이며, 정지 명령 전송과 실제 정지·거리 도달은 구분합니다. IMU 적분 변위도 정확한 위치 측정값으로 취급하지 않습니다.

## XYZ 팔 제어

에이전트의 `command.cartesian` 목표를 harness의 SO-101 IK가 관절각으로 변환한 뒤 기존 server·runtime의 guarded 실행으로 보냅니다. 외부 `lekiwi-console`에 있던 URDF/FK/IK를 패키지 안으로 가져왔으며 실행 중 그 외부 폴더에 의존하지 않습니다. `numpy`·`scipy`는 harness 의존성에 포함됩니다.

```json
{
  "action": "move", "x": 0, "y": 0, "theta": 0,
  "duration_ms": 500, "joints": null,
  "cartesian": {
    "frame": "arm_base", "mode": "relative",
    "xyz_m": [0, 0, 0.01], "approach_axis": null
  }
}
```

위 요청은 현재 FK 위치에서 arm_base의 +Z 방향으로 1cm 이동하는 끝점 목표입니다. `mode: absolute`는 같은 기준의 절대 XYZ입니다. `arm_base`는 포함된 URDF의 고정 팔 장착 원점이며 카메라·세계·바퀴 odometry 좌표가 아닙니다. `approach_axis`는 이 좌표계에서 그리퍼의 local +Z가 향할 단위 방향을 선택적으로 지정합니다. 팔 5자유도에 맞춘 기능으로 임의의 전체 6D 자세는 지원하지 않습니다. 그리퍼와 베이스는 XYZ 명령과 동시에 움직이지 않으며 별도 행동으로 명령합니다.

계산은 현재 관절각을 시작값으로 사용하고 URDF 제한, runtime 제한, `현재각 ± 최대속도 × duration`의 교집합 안에서 수행합니다. 허용 위치 오차는 모델상 2mm이며 실물 정확도 보장은 아닙니다. 현재값이 범위 밖이면 잘라 맞추지 않고 거부합니다. 해가 없거나 매핑 검증이 안 됐으면 모터 명령을 보내지 않고 원인·수치 해를 모델에 돌려주어 재관찰·수정을 이어갑니다. 같은 실패 목표의 반복도 제한합니다. FK·목표·관절 해·잔차·거부 원인은 GUI와 `kinematics_state`/`ik_result` 이벤트에 표시합니다.

이 구현은 **끝점 IK + 기존 관절 보간**입니다. ffw/Cyclo처럼 연속 Cartesian 기준 궤적을 추종하는 MoveL, 충돌 검사, 영상 픽셀에서 물체 XYZ를 구하는 깊이·카메라 변환은 포함하지 않습니다. 모델은 이미지와 결과를 보며 짧은 XYZ 행동을 순차적으로 결정합니다.

### 실물 매핑 설정

설정이 없으면 FK/IK 미리보기는 제공하지만 XYZ 실행은 비활성입니다. 다음 예시를 로봇별 파일로 복사하고 실물 관절 영점·회전 방향을 확인해 작성합니다.

```bash
cp harness/src/harness/data/cartesian.example.json /tmp/lekiwi-cartesian.json
```

- `joint_mapping`: 각 관절에 대해 `q_URDF = sign × q_runtime + offset_deg`. `sign`은 +1 또는 -1입니다. 알려진 기준 자세에서 URDF 각도와 관측 각도를 비교해 offset을 정하고, 다른 자세에서도 방향과 FK 위치가 맞는지 확인합니다. 예시의 +1/0은 실물 검증 결과가 아닙니다.
- `calibration_id`: 캘리브레이션 저장 결과 또는 GUI 관절 패널에 표시되는 현재 `ego.calibration_id` 전체값입니다. 범위·영점 파일이 바뀌면 ID가 달라지고 이전 매핑의 실행은 차단됩니다.
- `model_hash`: FK/IK 결과에 표시되는 URDF 모델 식별자입니다. 링크 길이·축 등 모델이 변경되면 기존 검증 설정으로 실행하지 않습니다.
- `hardware_alignment_verified`: 위 매핑과 그리퍼 기준점이 실물과 일치하는 것을 확인한 뒤 `true`로 설정합니다. ID만 복사하는 것으로 정합 검증을 대신하지 않습니다.

검증된 파일로 GUI 또는 CLI를 실행합니다. 설정 경로는 실행자가 시작 옵션으로만 지정하며 에이전트나 HTTP 작업 요청이 변경할 수 없습니다.

```bash
uv run --project harness harness-gui --server tcp://127.0.0.1:5556 \
  --robot-id kiwi-01 --kinematics-config /tmp/lekiwi-cartesian.json

uv run --project harness harness-run --server tcp://127.0.0.1:5556 \
  --robot-id kiwi-01 --execute --kinematics-config /tmp/lekiwi-cartesian.json \
  '관찰 후 가능하면 팔 끝을 arm_base +Z 방향으로 1cm 이동하고 결과를 확인해줘'
```

모델이나 로봇을 호출하지 않고 기록된 관찰로 IK 수치 결과만 검사할 수도 있습니다. 아래 CLI는 설정이 검증됐더라도 항상 기록 기반 미리보기로 표시하고 명령을 전송하지 않습니다.

```bash
uv run --project harness python -m harness.kinematics \
  --observation runs/실행ID/observations/0001/observation.json \
  --xyz 0 0 0.01 --mode relative --duration-ms 500 \
  --config /tmp/lekiwi-cartesian.json
```

## 실패와 재관찰

실행 모드는 guarded 실행을 지원한다고 응답하는 runtime에만 명령을 보냅니다. 오래된 runtime으로 자동 하향 전환하지 않습니다. 만료되거나 바뀐 관찰은 실행 전에 거부하고 새로 관찰합니다. 통신 timeout 등으로 실행 여부를 알 수 없으면 해당 에피소드의 추가 실행을 중단합니다.

모터 목표·토크를 쓰기 전 확인된 검증 실패는 `status: rejected`, `execution.outcome: rejected`, `command_sent: false`와 원인으로 반환합니다. 예를 들어 `Current elbow_flex is outside calibration`은 현재 관절값이 보정 범위를 벗어났다는 전송 전 거부입니다. 루프는 종료하지 않고 새 관찰을 받은 뒤 실패 명령·원인·수정 지침을 모델에 전달해 다른 명령을 판단하게 합니다. 새 명령에는 새 행동 ID를 사용하고 기존 행동·판단·시간 예산을 계속 적용합니다.

`ego.calibration_status`와 GUI는 현재 범위를 벗어난 관절 및 허용 범위를 표시합니다. 해당 관절은 재관찰에서 정상 범위를 확인하기 전까지 명령에서 제외합니다. 목표만 줄이거나 시간을 늘려 같은 현재 위치 오류를 해결하려 하지 않으며, 보정값을 자동 변경하지 않습니다. 같은 실패 명령이나 아직 사용할 수 없는 관절을 다시 제안하면 호스트가 전송 없이 막고 한 번 더 수정 기회를 제공합니다. 계속 반복하거나 대안이 없으면 이유를 남기고 중단합니다. 통신 단절·시간 초과·상태 확인 실패는 계속 실행 미확정으로 중단합니다.

팔 단독 동작의 작은 목표 오차는 경고로 이어갑니다. 모든 쓰기 응답을 확인했고 목표 오차가 관절별 10도 이내이며, 보정 범위·모터 상태·측정 자세 유지를 검증한 경우 `status: warning`, `execution.outcome: warning`, `arm_reached: false`를 반환합니다. hold 직후의 일시적인 움직임은 최대 2초 동안 읽기만 반복하며 기다리되, 0.2초 이상 3개 샘플의 관절 변동 0.5도 이하라는 안정성 기준은 유지합니다. 새 카메라·실측 관절값도 확인한 뒤 다음 이동을 허용합니다. 이 경고만으로 잠그거나 복구 제한을 새로 걸지 않습니다. 루프와 GUI는 `motion_warning`에 목표·실측·잔차를 표시하고, 모델이 실제 도달 위치에서 다음 동작을 계획하게 합니다. 정상 도달 허용오차 2도와 행동·시간 예산은 그대로이며, 경고를 도달 성공으로 기록하지 않습니다. 베이스·그리퍼 명령, 통신 불확실성, 보정 범위 위반, 계속 불안정한 자세는 이 경고에 해당하지 않습니다.

경고 범위를 벗어난 팔 단독 목표 미도달은 기존 복구 경로로 처리합니다. 런타임이 실제 진행, 보정 범위, 목표값 재읽기와 자세 유지까지 검증한 경우에만 `incomplete`와 `arm_hold_verified: true`를 반환합니다. 루프는 작업을 성공 처리하거나 같은 명령을 재전송하지 않고, 실패 행동 ID를 지정한 읽기 전용 `reconcile`로 상태를 다시 확인합니다. 새 관측을 얻으면 실패한 목표·실제 위치·잔차를 `motion_failure_feedback`으로 모델에 전달하고 수정한 다음 행동을 판단하게 합니다. GUI에는 목표 미도달, 상태 재확인, 새 계획 과정을 각각 기록합니다.

이 복구는 runtime당 최대 두 번이며 호스트의 행동·판단·시간 예산도 계속 적용됩니다. 바로 다음 팔 동작은 현재 측정 위치에서 각 관절 최대 40도, 최소 5초로 제한하고 베이스·그리퍼는 제외합니다. 원래 명령을 그대로 보내거나 요청 ID만 바꾸는 것으로 제한을 해제할 수 없습니다. 진행이 없거나 반대 방향으로 움직인 실패, 통신·hold·상태 검증 오류는 이 복구를 허용하지 않습니다. 자세 유지 확인은 장애물 없음의 증명이 아니므로 모델은 새 카메라 영상에서 이동 경로도 다시 판단해야 합니다.

이 경고 처리를 적용하려면 ORIN의 `lekiwi_runtime`과 워크스테이션의 `harness`를 업데이트하고 runtime 및 GUI/에이전트 프로세스를 다시 실행해야 합니다. v4의 서버는 응답을 그대로 전달하므로 서버 변경은 필요하지 않습니다. 관찰의 `ego.control.revision: arm-warning-v5`, `capabilities.arm_motion_warning: 1`로 적용 여부를 확인합니다. 기존 버전이 이미 `uncertain_execution`으로 중단한 runtime은 새 관찰만으로 해제되지 않습니다. 로봇 상태를 확인하고 팔을 지지한 뒤 runtime을 정상 종료·재시작하세요(종료 시 팔 토크가 해제됩니다). 과거 실행 기록은 자동 재생하지 않습니다.

같은 `action_id`와 같은 명령을 다시 보내면 runtime은 저장된 결과를 반환합니다. 같은 ID에 다른 명령을 넣으면 거부합니다. 이 기록은 runtime 프로세스 동안 유지되며, 재시작하면 `runtime_id`가 바뀝니다. 이전 관찰·명령으로 새 프로세스에서 실행하지 않습니다.

`Arm 2 write 46: communication=-6, device=0`은 어깨 관절의 이동 속도 설정에 대한 응답 timeout입니다. 설정이 적용되지 않았다는 증거는 아니며, 응답이 없을 때 `device=0`도 서보 정상 확인을 뜻하지 않습니다. 런타임은 보간 전 시간·속도 설정(44·46)에 한해 최대 두 번 읽어 요청값과 정확히 일치하면 같은 행동을 계속합니다. 쓰기나 행동을 재전송하지 않고, 이후 관절 도달과 새 관찰까지 확인해야 완료로 처리합니다. 값 불일치, 확인 불가, 서보 오류, 목표 위치·토크 쓰기 실패는 계속 실행 미확정으로 중단합니다. GUI와 실행 결과의 `arm_motion.communication_recoveries`에서 설정 재확인 내역을 볼 수 있습니다.

행동 기록은 runtime당 기본 128개 ID 또는 인코딩된 결과 32MiB로 제한합니다. 공간이 차면 guarded 실행을 차단하며, 이전 ID를 지워 다시 실행할 수 있게 만들지 않습니다. 새 에피소드도 같은 runtime의 누적 한도를 공유합니다.

guarded 관찰은 JSON 인코딩 기준 2MiB 이하로 제한합니다. 더 큰 관찰도 기존 조회에는 그대로 전달하지만 실행 허가는 내주지 않습니다. 실행 전에는 해당 로봇의 이미지 크기와 관찰 설정을 확인합니다.

`Ctrl+C`와 에피소드 예산은 다음 행동을 막습니다. 이미 전송한 짧은 명령은 runtime의 기존 종료·정리 경로를 따릅니다. 이번 변경은 실행 중인 명령을 즉시 선점하는 별도 네트워크 정지 API를 추가하지 않습니다.

## 실행 기록 확인

- `run.json`: 작업 지시와 실행 예산.
- `events.jsonl`: 관찰, 모델 판단, 전송 전 행동 ID·인자, 실행 결과, 피드백. 전송 전 기록을 디스크에 반영합니다.
- `action_finished.status`는 행동 응답의 결과를, `transport_status`는 요청·응답 처리 종료 상태를 나타냅니다. `transport_status: completed`만으로 물리적 동작 성공을 뜻하지 않습니다. 이전 기록에 남은 `status: completed, result_status: error`도 GUI에서는 실행 오류로 표시합니다.
- `observations/`: 각 관찰의 메타데이터와 이미지.
- `result.json`: 종료 사유, 행동·판단 수, 미확정 행동 ID, 완료 주장. 종료 기록을 쓰지 못한 강제 중단은 `running`으로 남을 수 있으므로 성공으로 취급하지 않습니다.

정상적인 완료 재확인은 `completion_claim`으로 기록합니다. `budget_actions`, `budget_steps`, `budget_seconds`는 예산 종료이며 작업 성공을 뜻하지 않습니다. `startup_error`는 모델·MCP 준비 중 실패해 실행을 시작하지 못한 상태입니다. `uncertain_execution`은 동작 결과를 확정할 수 없어 중단한 상태입니다. 이 경우 기록을 확인하고 로봇의 실제 상태를 점검한 뒤 새 작업으로 시작해야 합니다.

## 검증 범위

2026-11-16 기준 캘리브레이션·통신 복구·XYZ 제어 추가 후 runtime 테스트 100개, harness 테스트 114개를 통과했습니다. 실제 MCP stdio·ZeroMQ server·mock runtime을 연결해 관찰 ID, 중복 실행 방지, 동작 이후 프레임과 에피소드 기록을 검증했습니다. 팔꿈치 raw 3133의 전송 전 거부 후 재관찰·손목 대안 명령·완료 재확인, 프로파일 ACK 오류 후 읽기 검증, 실제 IK 목표→guarded 관절 명령→모의 엔코더 도달 경로도 확인했습니다. 캘리브레이션의 범위·영점·토크 해제 실패·취소·백업과 XYZ 매핑·보정/모델 해시·속도 제한·미리보기 차단·계산 중 관찰 만료를 검사했습니다. GUI는 합성 데이터로 FK/IK 결과와 미검증 요청의 미전송 표시를 확인했습니다. 실제 모델 판단과 실물 로봇 동작은 시험하지 않았습니다.

```bash
uv run --project lekiwi_runtime python -m unittest discover -s lekiwi_runtime/tests -v
uv run --project harness python -m unittest discover -s harness/tests -v
```

## 복구

작업 전 기준 커밋은 `06b996dc46ce31684f07cec7660db48c32628065`이며, 작업 브랜치는 `codex/observation-action-loop`입니다. 적용 전 추적 파일과 사용자의 미커밋 변경은 `.recovery/agent-loop-20261116-171242/`의 `before.tar.gz`, `working-tree.patch`, `manifest.json`에 보존했습니다. `Developement_log.md`의 기존 수정은 이번 변경에 포함하지 않습니다.

변경을 되돌릴 때는 이 기능을 추가한 커밋을 `git revert <기능-커밋>`으로 되돌릴 수 있습니다. 이후 작성한 파일을 덮어쓸 수 있으므로 백업 압축파일을 현재 디렉터리에 무조건 풀거나 `git reset --hard`를 사용하지 않습니다.

GUI는 에이전트 루프 커밋 `9e42025` 위에 별도 커밋으로 추가합니다. GUI 커밋만 되돌리면 기존 CLI 루프는 유지됩니다. 두 기능을 모두 되돌릴 때는 GUI 커밋을 먼저, `9e42025`를 다음으로 revert합니다. 실행 기록은 별도 `runs/` 디렉터리에 남습니다.
