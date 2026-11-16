# 관찰 기반 에이전트 실행

`harness-run --execute`는 자연어 작업을 짧은 행동으로 나누고, 매번 실제 실행 결과와 새 관찰을 받아 다음 행동을 결정합니다. 기본 `harness-run`의 관측·제안 모드는 유지합니다. 팔 XYZ 제어와 ROS/Cyclo는 추가하지 않습니다.

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
| `duration_ms` | 한 명령의 실행 시간, 최대 1000ms |
| `joints` | 팔의 절대 관절각, deg. 생략한 관절 유지 |
| `joints.gripper` | 보정된 그리퍼 값, 0–100% |

팔은 기존 보정·범위·속도 검사, 목표 보간과 관절 피드백 확인을 사용합니다. 베이스는 시간 제한 속도 명령이며, 정지 명령 전송과 실제 정지·거리 도달은 구분합니다. IMU 적분 변위도 정확한 위치 측정값으로 취급하지 않습니다.

## 실패와 재관찰

실행 모드는 guarded 실행을 지원한다고 응답하는 runtime에만 명령을 보냅니다. 오래된 runtime으로 자동 하향 전환하지 않습니다. 만료되거나 바뀐 관찰은 실행 전에 거부하고 새로 관찰합니다. 통신 timeout 등으로 실행 여부를 알 수 없으면 해당 에피소드의 추가 실행을 중단합니다.

같은 `action_id`와 같은 명령을 다시 보내면 runtime은 저장된 결과를 반환합니다. 같은 ID에 다른 명령을 넣으면 거부합니다. 이 기록은 runtime 프로세스 동안 유지되며, 재시작하면 `runtime_id`가 바뀝니다. 이전 관찰·명령으로 새 프로세스에서 실행하지 않습니다.

행동 기록은 runtime당 기본 128개 ID 또는 인코딩된 결과 32MiB로 제한합니다. 공간이 차면 guarded 실행을 차단하며, 이전 ID를 지워 다시 실행할 수 있게 만들지 않습니다. 새 에피소드도 같은 runtime의 누적 한도를 공유합니다.

guarded 관찰은 JSON 인코딩 기준 2MiB 이하로 제한합니다. 더 큰 관찰도 기존 조회에는 그대로 전달하지만 실행 허가는 내주지 않습니다. 실행 전에는 해당 로봇의 이미지 크기와 관찰 설정을 확인합니다.

`Ctrl+C`와 에피소드 예산은 다음 행동을 막습니다. 이미 전송한 짧은 명령은 runtime의 기존 종료·정리 경로를 따릅니다. 이번 변경은 실행 중인 명령을 즉시 선점하는 별도 네트워크 정지 API를 추가하지 않습니다.

## 실행 기록 확인

- `run.json`: 작업 지시와 실행 예산.
- `events.jsonl`: 관찰, 모델 판단, 전송 전 행동 ID·인자, 실행 결과, 피드백. 전송 전 기록을 디스크에 반영합니다.
- `observations/`: 각 관찰의 메타데이터와 이미지.
- `result.json`: 종료 사유, 행동·판단 수, 미확정 행동 ID, 완료 주장. 종료 기록을 쓰지 못한 강제 중단은 `running`으로 남을 수 있으므로 성공으로 취급하지 않습니다.

정상적인 완료 재확인은 `completion_claim`으로 기록합니다. `budget_actions`, `budget_steps`, `budget_seconds`는 예산 종료이며 작업 성공을 뜻하지 않습니다. `startup_error`는 모델·MCP 준비 중 실패해 실행을 시작하지 못한 상태입니다. `uncertain_execution`은 동작 결과를 확정할 수 없어 중단한 상태입니다. 이 경우 기록을 확인하고 로봇의 실제 상태를 점검한 뒤 새 작업으로 시작해야 합니다.

## 검증 범위

2026-11-16 기준 runtime 테스트 72개, harness 테스트 40개를 통과했습니다. 실제 MCP stdio·ZeroMQ server·mock runtime을 연결해 관찰 ID, 중복 실행 방지, 동작 이후 프레임과 에피소드 기록을 검증했습니다. 설치된 SDK의 격리 환경 초기화와 빈 스레드 생성도 확인했습니다. 실제 모델 판단과 실물 로봇 동작은 시험하지 않았습니다.

```bash
uv run --project lekiwi_runtime python -m unittest discover -s lekiwi_runtime/tests -v
uv run --project harness python -m unittest discover -s harness/tests -v
```

## 복구

작업 전 기준 커밋은 `06b996dc46ce31684f07cec7660db48c32628065`이며, 작업 브랜치는 `codex/observation-action-loop`입니다. 적용 전 추적 파일과 사용자의 미커밋 변경은 `.recovery/agent-loop-20261116-171242/`의 `before.tar.gz`, `working-tree.patch`, `manifest.json`에 보존했습니다. `Developement_log.md`의 기존 수정은 이번 변경에 포함하지 않습니다.

변경을 되돌릴 때는 이 기능을 추가한 커밋을 `git revert <기능-커밋>`으로 되돌릴 수 있습니다. 이후 작성한 파일을 덮어쓸 수 있으므로 백업 압축파일을 현재 디렉터리에 무조건 풀거나 `git reset --hard`를 사용하지 않습니다.
