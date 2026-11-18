# Codex Harness

Python Codex runtime → MCP `observe`/`execute` → ZMQ 서버 → observation.
기본 모드는 한 번 관측하고 `wait` 제안 JSON을 출력합니다. `harness-run --execute`는 별도의 호스트 루프로 한 행동씩 실행하고 새 관찰을 바탕으로 다시 판단합니다. 실행 모드는 행동·시간·단계 예산과 실행 기록을 관리합니다.

```bash
uv run --project harness harness-run --execute --robot-id kiwi-01 \
  --max-actions 20 --max-steps 40 --max-seconds 300 \
  '현재 관측을 확인하고 100ms 대기한 뒤 결과를 확인해줘.'
```

실행 모드에는 guarded 실행을 지원하는 runtime이 필요합니다. `--snapshot`과 함께 사용할 수 없습니다. 모델은 한 단계의 계획·근거·명령을 출력하고, 호스트가 관찰 ID와 예산을 검증한 뒤 MCP를 호출합니다. 완료 주장은 재관찰 후 다시 확인하며 물리적 성공 판정과 구분합니다. 상세 흐름, 기존 console loop에서 반영한 부분, 복구 방법은 [에이전트 루프 안내](../docs/agent-loop.md)에 있습니다.

## GUI에서 실행

server와 runtime을 켠 뒤, 터미널의 `harness-run --execute` 대신 다음을 실행합니다.

```bash
cd /home/jwshin/Lekiwi
uv run --project harness harness-gui \
  --server tcp://127.0.0.1:5556 --robot-id kiwi-01
```

브라우저에서 **http://127.0.0.1:8765**를 열고 지시·예산을 입력한 뒤 실행합니다. GUI도 `harness-run --execute`와 같은 루프를 호출합니다. 모델의 단계별 계획·판단 근거 요약·예상 변화·측정 피드백, 전방/손목 관찰 이미지와 이전 동작 전 이미지를 보여줍니다. 모델 판단 중에는 진행 단계와 경과 시간을 표시합니다.

모델·로봇 없이 화면을 확인하려면:

```bash
uv run --project harness harness-gui --demo --port 8766
```

이 경우 **http://127.0.0.1:8766**에서 고정된 합성 데이터와 스크립트 판단으로 같은 루프를 시연합니다. 입력한 지시를 모델이 수행하는 모드는 아닙니다.

한 번에 하나의 실행만 허용합니다. **루프 중지**는 모델 대기를 취소하고 다음 행동을 막지만, 이미 전송한 동작을 즉시 멈추는 비상정지는 아닙니다. 브라우저 탭을 닫아도 GUI 서버의 루프는 계속되므로 중지 버튼을 사용합니다. 실행 기록은 기본 `runs/`에 보관하며 화면을 새로 열어도 조회할 수 있습니다. 과거 기록은 자동 재개하지 않습니다. `--runs-dir`로 기록 루트, `--model`로 기본 모델을 지정할 수 있습니다.

GUI는 localhost에서만 열립니다. 기존 TEMP GUI가 8765를 사용하면 다른 포트를 지정하세요. TEMP GUI와 외부 lekiwi-console은 수정하지 않았습니다. 새 GUI의 화면 갱신은 실행 기록만 읽으며 별도의 로봇 관찰 요청을 보내지 않습니다.

전송 전 검증에서 거부된 명령은 원인·새 관찰을 받아 자동으로 수정 판단을 이어갑니다. GUI에도 명령 거부와 보정 범위 진단을 표시합니다. 같은 불가능한 명령은 반복 전송하지 않으며, 실제 실행 여부가 불확실한 오류는 중단합니다. 이 구분에는 업데이트된 ORIN runtime과 GUI 재시작이 필요합니다. [오류 복구와 재시작 안내](../docs/agent-loop.md#실패와-재관찰)

## 기본 제안 모드

```bash
# 터미널 1: 통신 서버
uv run --project server server

# 터미널 2: observation을 보고 제어신호 출력
uv run --project harness harness-run \
  --robot-id kiwi-01 '현재 센서를 확인하고 100ms 대기하는 제어신호를 출력해줘.'
```

저장된 observation으로 시험 (서버/ORIN 불필요):

```bash
uv run --project harness harness-run \
  --snapshot TEMP/zmq_server/captures/20261109-125911/message-0001/frame-00.json \
  '관측한 상태를 확인하고 100ms 대기 신호를 출력해줘.'
```

`--snapshot`은 파일을 MCP `observe`로 전달합니다. 결과에는 기록된 데이터임을
명시하며, 원래 관측 timestamp와 센서값은 유지합니다.

- Codex 로그인 필요. 공식 `openai-codex` Python SDK와 번들 runtime 사용.
- `--server tcp://127.0.0.1:5556`, `--model`로 연결 주소와 모델 지정 가능.
- `runtime.py`: 기본 제안 모드와 실행 모드 CLI, JSON 출력.
- `episode.py`: 행동·판단·시간 예산, 실행 기록, 관찰 검증과 피드백 루프.
- `episode_runtime.py`: 실행 모드의 Codex 판단과 호스트 MCP 연결. 로그인만 공유하는 임시 Codex 환경을 사용하며 사용자 MCP·도구 설정은 상속하지 않습니다. 실행 모델을 지정하려면 `--model`을 사용합니다.
- `tools.py`: `observe(robot_id)`와 `execute(robot_id, x, y, theta, duration_ms)` 노출. JPEG는 MCP 이미지로 반환. snapshot 모드에는 observe만 등록합니다.
- `client.py`: 독립된 ZMQ 서버에 요청. 다른 프로젝트의 Python 코드를 import하지 않음.
- MCP 관측 성공을 확인하지 못하면 제어신호 출력을 거절함.

ORIN은 `move`, `wait`을 지원합니다. `--execute` 없는 기본 `harness-run`은 observe만 허용하며 출력 스키마는 wait 전용입니다. 아래 JSON은 기본 모드의 제안 결과입니다. 실제 실행에는 `--execute` 또는 아래 execute MCP를 사용합니다.

```json
{"robot_id":"kiwi-01","command":{"action":"wait","duration_ms":100},"reason":"관측에 기반한 설명"}
```

MCP만 실행: `uv run --project harness harness --server tcp://127.0.0.1:5556`

테스트: `uv run --project harness python -m unittest discover -s harness/tests -v`

SDK 문서: https://learn.chatgpt.com/docs/codex-sdk

## 차체 속도 기반 execute MCP

```json
{"robot_id":"kiwi-01","x":0.2,"y":0.0,"theta":0.0,"duration_ms":500}
```

위 호출은 전진 0.2 m/s를 500 ms 동안 명령합니다. `x`는 전진(+)/후진(-), `y`는 왼쪽(+)/오른쪽(-) 평행이동(m/s), `theta`는 반시계(+)/시계(-) 회전(deg/s)입니다. 평행이동과 회전을 함께 지정할 수 있습니다. 평행이동 속도 `hypot(x,y)`는 최대 0.4 m/s, 회전은 ±150 deg/s, 시간은 1–1000 ms입니다. 모두 0인 속도도 허용합니다.

MCP는 서버에 `command: {action: "move", x, y, theta, duration_ms}`를 보내며, ORIN이 바퀴 ID 7·8·9의 속도로 변환합니다. 실행 종료 시 정지·토크 해제를 수행한 뒤 observation을 반환합니다. 완료는 명령 전송과 정지 처리의 성공을 의미하며, 실제 이동 거리 도달을 검증하는 방식은 아닙니다. 통신 오류나 시간 초과는 실행 결과가 불확실할 수 있어 자동 재시도하지 않습니다.

## 폴더에서 직접 실행

observe/execute MCP 서버(stdio)를 시작합니다. Codex 제어신호 출력은 기존 `harness-run` 명령을 사용합니다.

```bash
cd harness
uv run python main.py
```

`uv run`이 의존성과 프로젝트 설치를 준비합니다. 이미 설치된 Python 환경에서는 `python main.py`로 실행할 수 있습니다. 기존 CLI 옵션도 그대로 사용할 수 있습니다.

## 팔·그리퍼 제어

execute MCP 예시(절대 목표값):

```json
{"robot_id":"kiwi-01","action":"move","joints":{"wrist_roll":10,"gripper":60},"duration_ms":1000}
```

관절명은 shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper입니다. 팔은 도(deg), 그리퍼는 보정된 개폐율(%)입니다. 먼저 observe의 `ego.joints`와 `ego.limits`를 확인합니다. 생략한 관절은 건드리지 않습니다. 이동 중 속도는 관절 30deg/s, 그리퍼 50%/s 이하입니다. 베이스 속도가 모두 0인 팔·그리퍼 단독 명령은 100–20,000ms, 베이스와 팔 동시 이동은 100–1,000ms입니다. 1초를 넘는 명령은 갱신된 runtime/server가 필요합니다. 클라이언트 기본 응답 대기는 45초, 서버는 35초입니다.

팔은 목표 도달 피드백을 확인한 후 완료를 반환합니다. `ego.last_motion`에 목표·측정값·허용오차가 포함됩니다. 팔 자세 유지 토크는 자동으로 해제하지 않습니다. 관절 도달과 물체 잡기 성공은 서로 다릅니다. 아래 XYZ 기능은 끝점 IK를 제공하며 자동 파지 경로·충돌 회피는 포함하지 않습니다.

베이스와 팔을 동시에 움직이려면 execute에 `action="move"`, `x/y/theta`, `joints`, 공통 `duration_ms`(100–1000)를 전달합니다. 각 축의 기존 범위·속도 검증을 적용하며, ORIN runtime의 동시 제어 지원 버전이 필요합니다. 정상 실행 후 팔은 자세를 유지하지만 런타임 종료 시에는 팔·바퀴 토크를 해제합니다.

### Action 규칙

MCP 도구는 `observe`, `execute` 두 개이며, execute의 action은 `move`, `wait` 두 개입니다.
`move`는 베이스 속도(x/y/theta, 생략 시 0)와 팔 관절 목표(joints, 생략 시 팔 유지)를 선택적으로 받습니다. 둘 다 지정하면 동시에 움직입니다. `wait`에는 이동 목표를 넣지 않습니다.

`observe`와 `execute` 응답에는 `observation.imu`도 포함됩니다. 가속도·각속도·자세 및 초기 위치 기준 추정 변위(m)를 반환합니다. `available`, `estimate_valid`, `age_s`, `origin_id`를 확인하고, 동일 원점의 유효한 추정값끼리만 비교합니다. IMU 적분 변위는 누적 오차가 있는 추정치입니다. 도구와 action은 추가되지 않습니다.
## XYZ 팔 제어

GUI의 **팔 끝 XYZ 테스트 → 계산만 보기**에서 XYZ를 직접 입력할 수 있습니다. `URDF 예시 자세`는 로봇·모델 API·매핑 설정 없이 작동합니다. `Z=0.01`, `현재 위치 + 이동량`으로 **XYZ 미리보기 계산**을 누르면 1cm 위 목표의 관절 해와 오차를 표시합니다. **SO-101 URDF와 STL 외형 메시를 읽는 3D 화면**에서 현재·목표·겹쳐 보기를 선택하고, 드래그로 회전·휠로 확대할 수 있습니다. IK와 표시 모델은 같은 URDF를 사용하며 브라우저에서도 팔 끝 FK 일치를 검사합니다. 집게 벌림은 모델 기본 자세입니다. 입력은 m, 예시 관절각은 URDF 기준 °입니다. 저장 실행을 선택하면 해당 기록의 관절값과 제한으로 계산하며 로봇에는 전송하지 않습니다.

**실제 팔 1회 이동** 탭은 에이전트/모델 없이 작은 XYZ 시험을 수행합니다.

1. 처음에는 `X=0, Y=0, Z=5` **mm**로 두고 **① 로봇 상태 읽고 계산**을 누릅니다. 이 단계는 새 관절·카메라 관찰을 읽고 IK만 계산합니다.
2. URDF 현재 자세를 실물과 비교하고, 목표 자세와 관절 변화량을 확인합니다. 방향·영점 매핑이 미검증인 경우 그 일치 여부를 이번 1회에 한해 확인해야 합니다. 다른 자세로 보이면 방향·영점 입력을 수정하고 다시 계산하세요.
3. 이동 공간을 확인한 뒤 **② 실제 팔 한 번 이동**을 누릅니다. 1초간, 2mm 초과~10mm 이하, 관절별 최대 3° 변화만 허용합니다. 베이스와 집게 벌림은 유지합니다.

계산은 최대 30초 후 만료되고 전송 토큰은 한 번만 사용할 수 있습니다. 기존 guarded runtime 경로가 관찰 ID·실측 상태·보정 범위·속도·도달 피드백을 검사합니다. 응답 미확정 시 자동 재전송하지 않습니다. 실행 기록에 확인한 매핑, 목표, 관찰, 결과가 저장됩니다. 확인란은 파일을 바꾸거나 에이전트의 XYZ 실행 권한을 영구 활성화하지 않습니다. 에이전트와 수동 이동은 동시에 실행할 수 없습니다. 종료 전에 준비를 취소할 수 있지만 이미 전송한 명령은 취소할 수 없습니다. GUI 폴링은 로봇 관찰을 요청하지 않습니다.

FK와 해석적 Jacobian은 **Pinocchio (`pin`)**, 관절 제한 안의 미분 IK 최적화는 **OSQP**를 사용합니다. ffw_codex가 명령을 보내는 Cyclo와 같은 핵심 라이브러리이며, Cyclo의 C++ MoveL 제어기·충돌 회피 전체를 이식한 것은 아닙니다. 실행 중인 환경에서 `uv sync` 후 GUI/harness를 다시 시작해야 새 라이브러리가 적용됩니다.

**현재 로봇 자세 읽고 기본 자세와 비교**는 관절·카메라를 한 번 읽어 기록합니다. 측정 관절값을 IK 초기값으로 사용해 현재 FK→IK, 현재→GUI 기본 XYZ, GUI 기본→현재 XYZ를 검사하고 좌표·관절 차이를 보여줍니다. GUI 기본은 `(0, -15, 35, -20, 0)°` 예시이며 실제 홈 자세가 아닙니다. 모든 관절이 0°인 자세도 별도 표시합니다. 이 비교는 전체 보정 범위 안의 기하학 계산이며 실행 명령을 만들지 않습니다. 한 자세의 FK/IK 대응만으로 실물 영점·부호 또는 큐브 중심 위치가 검증되지는 않습니다.

기록 기반 IK 미리보기의 기본 이동 시간은 3초입니다. 2·3·5·10·20초를 선택할 수 있으며 관절 속도 제한은 유지됩니다. 짧은 시간으로 해를 찾지 못하면 더 긴 시간 조건을 별도 계산해 안내합니다. 버튼으로 시간을 바꿔 다시 미리볼 수 있고 자동 전송하지 않습니다. 작은 **실제 팔 1회 이동** 탭은 기존 1초·최대 3° 제한을 유지합니다.

**에이전트 추론**은 왼쪽 작업 지시와 **로봇 에이전트 실행**으로 시작합니다. `지금의 판단`과 타임라인에 모델이 출력한 계획·근거·피드백이 나타납니다. XYZ 미리보기와 수동 1회 이동은 모델을 호출하지 않습니다.

3D 파일은 로컬에서 제공하며 CDN이 필요하지 않습니다. 메시 출처·고정 리비전·해시는 `src/harness/data/mesh-sources.json`, 라이선스는 `SO-ARM100-LICENSE`에 있습니다. 표시 라이브러리는 Three.js 0.177.0과 urdf-loader 0.12.6이며 버전·출처·라이선스는 `src/harness/static/vendor/sources.json`과 각 `LICENSE`를 참조하세요.

업데이트 전 GUI 프로세스는 **GUI 터미널만** Ctrl+C로 종료하고 같은 `harness-gui` 명령으로 다시 실행한 뒤 브라우저를 새로고침하세요. 실행 중인 에이전트가 있다면 종료된 뒤 재시작하세요. 서버가 이전 버전이면 미리보기 패널에 재시작 안내가 나옵니다.

에이전트 루프에서 `cartesian` 목표(arm_base 기준 XYZ, m)를 SO-101 IK로 관절각으로 변환해 기존 guarded 경로로 실행할 수 있습니다. GUI에는 현재 FK, 목표·계산된 관절각·오차와 미리보기/실행 준비 상태가 표시됩니다. `--kinematics-config 로봇별설정.json`으로 실물 부호·영점 매핑과 보정 ID를 지정하며, 미검증 또는 보정 파일이 바뀐 상태는 실행하지 않습니다. 설정 예제는 `src/harness/data/cartesian.example.json`입니다. [설정·명령·오프라인 미리보기](../docs/agent-loop.md#xyz-팔-제어)

## Plan-Act 루프 (Planning → Action Loop → Verify/재계획)

`--skills-config`를 주면 기본 루프가 plan-act가 됩니다(`--loop step`으로 예전 방식). 한 번에 작은 명령 하나를 고르는 대신 다음 구조로 동작합니다.

1. **Planning**(높은 reasoning): 작업·현재 영상·이전 시도·**로봇 기억**(아래)을 보고 전체 계획(최대 10단계)을 세우고, 기댈 교훈의 ID를 `lessons_applied`로 밝힙니다.
2. **Action Loop**(짧은 reasoning): 한 턴에 큰 스킬 하나를 고릅니다. `locate`(front 영상의 상자 → 팔 기준 XYZ), `pick`/`place`(XYZ에서 집기·놓기), `move_to`, `home`, `gripper`, `drive`, `observe`. 루프가 스스로 `done` 또는 `failed`를 선언할 때까지 반복합니다.
3. `failed` → 기록 후 다시 Planning. `done` → 새 관찰로 **Verify** 턴이 독립적으로 확인(confirm)하거나 재계획을 요청합니다.
4. 끝나면 로봇 기억을 갱신합니다. 원시 기록은 계속 `runs/experience.jsonl`에 남습니다.

### 로봇 기억 (`runs/memory/`, 리셋할 수 없는 실제 로봇용)

시뮬레이션처럼 같은 장면을 다시 돌려 검증할 수 없으므로, 기억은 **이후 실제 에피소드의 결과로 검증**됩니다.

- **교훈** (`lessons-<robot>.json`): 모델이 남긴 교훈은 처음엔 **가설**입니다. 에피소드마다 최대 4개(결론·실패 우선, 비슷한 것은 합침), 그 교훈이 본 직전 결과의 스킬·실패 유형으로 분류합니다. 이후 에피소드가 그 교훈을 `lessons_applied`로 밝히고 적용했을 때 에피소드가 성공하면 +1, 그 교훈을 적용한 행동이 실패하거나 에피소드가 실패하면 −1입니다. +2 이상이고 −의 2배 이상이면 **검증됨**, −2 이상이고 +보다 많으면 **폐기**. 교훈을 쓴 에피소드는 스스로 검증하지 못하고, 통신·중단 같은 인프라 정지는 점수에 반영하지 않습니다. Planning에는 검증됨이 먼저, 가설은 "확인 안 됨"으로 전달됩니다.
- **몸 보정** (`embodiment-<robot>.json`, 모델 판단 없이 호스트가 측정):
  - 주행 계수 = `locate` → `drive` → `locate`(사이에 팔이 물체를 건드리지 않음, 회전 없음)에서 물체가 실제로 다가온 거리 / 명령한 속도×시간. 0.5 m보다 먼 측정·3 cm 미만 이동은 제외. Planning·Action에 `body.drive.gain`으로 전달.
  - 위치 보정 = `place`로 놓은 위치(팔 FK) − 이후 같은 물체를 `locate`한 위치. 같은 카메라 보정에서 3회 이상 일관되면(편차 ≤ 6 mm, 크기 ≤ 3 cm) 새 `locate` 결과에 자동 적용(`xyz_m_raw`에 원래 값).
- GUI 왼쪽 **로봇 기억** 카드에 보정값과 교훈(검증됨/가설)이, 판단 카드에 적용한 교훈·주행 측정·기억 갱신이 나옵니다.

```bash
# 기존 experience.jsonl에서 가설·주행 측정값 가져오기(검증 상태는 0부터)
harness/.venv/bin/python -m harness.memory import-experience runs/experience.jsonl --memory-dir runs/memory
harness/.venv/bin/python -m harness.memory show --memory-dir runs/memory
```

스킬은 호스트가 실측 관절값으로 계산한 명령을 기존 `EpisodeRunner.execute`로 하나씩 보냅니다. runtime의 신선도·속도·보정·경고·재조정 검사는 그대로입니다. XYZ 목표 하나는 IK 명령 하나(최대 20초, 관절 거리로 시간 자동 결정)이며, IK는 wrist_roll을 현재 값에 고정한 해를 먼저 찾습니다(roll은 접근축 회전이라 위치에만 약간 영향).

```bash
cd /home/jwshin/Lekiwi
uv run --project harness harness-gui --loop plan-act \
  --kinematics-config outputs/kinematics/cartesian-kiwi-01.json \
  --skills-config outputs/kinematics/skills-kiwi-01.json \
  --camera-config outputs/kinematics/camera-front-kiwi-01.json
```

`harness-run --execute ...`도 같은 옵션을 받습니다. plan-act의 기본 예산은 행동 120·판단 60·1200초입니다.

## policy.py 호스트 스킬 (locate / pick / place / home_arm / drive)

GUI의 policy 탭과 `harness-campaign run`에서 `policy.py`가 모델 판단 없이 직접 부르는 결정적 스킬입니다. 호출 하나가 단계 하나(`stages/NNNN_LYYY_<스킬>/`)이며, 모든 모터 명령은 `EpisodeRunner.execute`의 기존 검사를 그대로 거칩니다.

| 함수 | 동작 |
|---|---|
| `locate(label)` | 읽기 전용. 비전 모델이 front 영상에 상자만 표시하고, 보정된 카메라가 바닥 평면과 교차해 팔 기준 XYZ와 집기 가능 여부(`reachable`)를 계산합니다. `--camera-config` 필요 |
| `pick(xyz_m)` / `place(xyz_m)` | 열기 → 위로 접근 → 하강 → 닫기 → 들기 (놓기는 반대). 단계마다 IK 명령 하나(최대 20초), 들어 올린 뒤 그리퍼가 비면 실패 |
| `home_arm()` | 기록된 홈 자세로 복귀(front 시야 확보) |
| `drive(forward_m, left_m, turn_deg)` | 0.1 m/s·30°/s, 1초 이하 명령으로 나눠 이동. 한 번에 ±0.5 m, ±90° 이내. 이동 뒤 이전 XYZ는 무효이므로 다시 `locate` |

`--skills-config`가 있으면 `act()`의 안쪽 루프도 기본으로 plan-act(위 스킬을 쓰는 Planning → Action → Verify)가 됩니다(`--act-loop step`으로 기존 방식 유지). 설정이 없으면 스킬은 `skills_unavailable`을 돌려줍니다. GUI의 **Skill Template** 버튼이 찾기 → 다가가기 → 집기 예시를 넣습니다.

```bash
cd /home/jwshin/Lekiwi
uv run --project harness harness-gui \
  --kinematics-config outputs/kinematics/cartesian-kiwi-01.json \
  --skills-config outputs/kinematics/skills-kiwi-01.json \
  --camera-config outputs/kinematics/camera-front-kiwi-01.json
```

## front 카메라 위치 보정 (locate용)

1. 렌즈 보정: `lekiwi_runtime/tests/SLAM/calibration_result_20261113_48views/calibration.json`(OV5648, 640×480).
2. 체커보드(8×6 내부 코너, 24 mm)를 front 카메라에 전체가 보이게 바닥에 놓고, 팔을 홈에 둔 채 촬영합니다.
   `python -m harness.front_camera_capture OUT_DIR --board-only`
3. 보드를 치우고 큐브를 그리퍼에 물린 뒤 실행합니다. 큐브를 바닥 6곳에 놓고 홈으로 물러나 촬영한 다음 다시 집습니다(베이스는 움직이지 않음).
   `python -m harness.front_camera_capture OUT_DIR`
4. OpenCV가 있는 환경에서 계산합니다. 보드로 카메라 기울기·높이를, 큐브 점으로 팔 기준 위치·방향(yaw)을 맞추고 leave-one-out 오차를 보고합니다.
   ```bash
   lekiwi_runtime/.venv/bin/python harness/util/solve_front_camera.py OUT_DIR \
     --intrinsics lekiwi_runtime/tests/SLAM/calibration_result_20261113_48views/calibration.json \
     --output outputs/kinematics/camera-front-kiwi-01.json
   ```
`locate`의 XYZ는 바닥에 놓인 물체의 집기 점 추정치입니다. 멀수록 픽셀당 오차가 커지므로(`xy_change_per_3px_m`) 가까이 이동한 뒤 다시 locate합니다. 팔 보정을 다시 하면 `python -m harness.calibration_shift`로 XYZ 매핑과 홈 자세의 0점을 옮깁니다(기본은 미리보기).
