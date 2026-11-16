# Lekiwi Runtime

ORIN에서 카메라 2대, AWR2944 레이더, BMI160 IMU, STS3215 팔 관절을 읽고 ZeroMQ DEALER로 서버 요청에 응답합니다. 하드웨어 코드와 기본 설정을 이 패키지에 포함하므로 외부 ORIN 폴더가 필요하지 않습니다.

관찰 기반 에이전트 실행을 위한 선택적 guarded 명령도 지원합니다. 기존 `move`/`wait` 명령은 유지합니다. [에이전트 루프와 복구 안내](../docs/agent-loop.md)

## 설치와 실행

저장소 루트에서 실행합니다. Python 3.10 이상을 사용합니다.

```bash
uv sync --project lekiwi_runtime
uv run --project lekiwi_runtime lekiwi-runtime \
  --server tcp://SERVER_IP:5555 --robot-id kiwi-01
```

ORIN에 `lekiwi_runtime/`만 복사한 경우 해당 폴더에서 `uv sync` 후 `uv run lekiwi-runtime ...`을 실행하면 됩니다. pip 환경에서는 `pip install ./lekiwi_runtime`으로 설치합니다. OpenCV(headless), numpy, pyserial, Feetech SDK, pyzmq는 패키지 의존성으로 설치됩니다.

하드웨어 없이 실행하려면 같은 명령에 `--mock`을 붙입니다. 모의 카메라·레이더·관절 데이터에는 `simulated: true`가 표시됩니다.

## 장비 설정

### 팔 최소·최대 가동범위 캘리브레이션

ORIN에서 runtime과 teleop을 종료하고 팔을 손으로 지지한 뒤 실행합니다. 처음 Enter를 누르면 팔 ID 1–6의 토크를 해제합니다. 바퀴 명령·자동 팔 이동·EEPROM 쓰기는 하지 않습니다.

```bash
uv run --project lekiwi_runtime lekiwi-runtime calibrate --port /dev/ttyACM0
```

각 관절을 손으로 사용할 최소·최대 위치까지 천천히 움직이고, 양 끝에서 잠시 유지한 뒤 Enter로 다음 관절로 넘어갑니다. 현재 raw 값과 측정 범위를 표시합니다. `r`+Enter는 현재 관절 재측정, `q`+Enter 또는 Ctrl+C는 취소입니다. 마지막 범위 비교표를 검토하고 `save`를 입력해야 저장합니다. 끝을 억지로 밀지 마세요. 종료 후에도 토크는 꺼져 있습니다.

기본적으로 runtime 설정의 `arm.calib_path` 파일을 갱신하고, 덮어쓰기 전 원본 그대로 `*.backup-시각-ID`를 만듭니다. `--config`를 쓰는 runtime이라면 여기에도 같은 설정 파일을 지정하세요. `--output 새파일.json`은 별도 저장하며 이후 runtime 설정에 그 경로를 지정해야 합니다. 범위에 여유를 두려면 `--margin-ticks 10`을 사용합니다. 팔꿈치만 다시 측정할 수도 있습니다.

```bash
uv run --project lekiwi_runtime lekiwi-runtime calibrate \
  --port /dev/ttyACM0 --joints elbow_flex --margin-ticks 10
```

`lekiwi-calibrate` 명령이나 `python -m lekiwi_runtime.calibrate`로도 동일하게 실행합니다. `lekiwi_runtime/` 폴더 안에서는 `uv run lekiwi-runtime calibrate ...`를 사용합니다. 별도 설치 패키지 환경에서는 업데이트 후 실행하세요.

3개 샘플의 중앙값으로 순간 튐을 줄이며, 샘플 부족·32 ticks 미만의 범위·0/4095 encoder 경계 통과·통신 오류는 저장을 막습니다. 현재 모터에서 읽은 `homing_offset`과 기존 `drive_mode`를 저장하고 하드웨어 영점은 바꾸지 않습니다. 이는 JSON 기반 소프트웨어 가동범위 설정이며 모터 자체의 EEPROM 한계값 변경은 아닙니다. 저장한 뒤 팔을 새 범위 안의 자세로 놓고 runtime을 재시작하세요.

현재 각도 계산은 범위의 중간값을 0도로 삼으므로, 범위를 바꾸면 기존 각도 목표·저장 자세·XYZ 매핑도 재확인해야 합니다. 관찰의 `ego.calibration_id`와 저장 시 출력하는 ID가 보정 내용을 식별합니다. XYZ 설정이 이전 ID를 가리키면 실행을 허용하지 않습니다. 범위만 되돌리려면 해당 백업 파일을 원래 경로로 복사하고 runtime을 재시작합니다.

기본 설정은 `src/lekiwi_runtime/data/robot.json`입니다.

- 카메라: `/dev/video0`, `/dev/video2`, 640×480, 30 FPS, JPEG.
- 레이더: XDS110의 `if00`(CLI), `if03`(Data) 포트를 탐색합니다. 여러 레이더가 있거나 해당 장치 경로가 없으면 `cli_port`와 `port`를 지정합니다.
- 팔: WCH/FTDI/CP210x USB 포트에서 ID 1–6이 응답하는 보드를 찾습니다. 후보가 여러 개이거나 다른 USB 어댑터를 사용하면 `arm.port`를 지정합니다. 관측은 위치 읽기만 수행합니다. move 실행 시 같은 버스의 바퀴 ID 7–9를 제어하므로, 바퀴 전용 보드는 `arm.port`를 지정하고 `arm.enabled`를 false로 설정합니다.

기본 `data/radar.cfg`는 제공된 `last_sent.cfg`의 **AWR2944 TDM 3GHz 프로필**이며 TDM 펌웨어용입니다. `data/follower.json`은 제공된 follower 로봇의 보정값입니다. 다른 펌웨어나 로봇에는 해당 장비의 프로필·보정값을 지정해야 합니다.

장비별 JSON 파일로 필요한 설정만 덮어쓸 수 있습니다. 파일 경로는 이 JSON 파일이 있는 폴더를 기준으로 해석합니다.

```json
{
  "cameras": {"front": {"device": "/dev/video0", "rotation": 180}},
  "radar": {"cfg_path": "radar.cfg", "auto_init": true},
  "arm": {"port": "/dev/ttyUSB0", "calib_path": "follower.json"}
}
```

```bash
uv run --project lekiwi_runtime lekiwi-runtime \
  --server tcp://SERVER_IP:5555 --robot-id kiwi-01 \
  --config /etc/lekiwi/robot.json
```

레이더는 프레임이 들어오지 않으면 프로필과 `sensorStart`를 전송합니다. 이미 별도로 초기화한 레이더의 설정을 유지하려면 `radar.auto_init`을 `false`로 지정합니다. 센서는 각 설정의 `enabled: false`로 끌 수 있습니다. 센서 장애는 observation의 `available`과 `error`로 전달합니다.

## 코드 구성

| 파일 | 역할 |
| --- | --- |
| `client.py` | ZeroMQ 요청·응답, 오류 응답 |
| `orin.py` | 센서 시작·종료, observation 조합, move/wait 실행 |
| `config.py`, `data/` | 기본 설정과 장비별 설정 로딩 |
| `drivers/camera.py` | 최신 프레임 캡처와 JPEG 인코딩 |
| `drivers/radar.py` | 레이더 UART 수신과 초기화 |
| `drivers/radar_protocol.py` | 포인트·SNR 패킷 파싱 |
| `drivers/arm.py` | 팔 관측, 보정 변환, 목표 보간·피드백 검증 |
| `drivers/servo.py` | 팔과 바퀴가 공유하는 시리얼 연결 |
| `drivers/wheels.py` | 차체 속도 변환, 바퀴 실행과 정지 |

참고 코드 `TEMP/Lekiwi`에서 관측에 필요한 로직만 옮겼습니다. 키보드 teleop의 바퀴 속도 변환·설정 로직을 사용합니다. 키보드 UI, 리더 암, 녹화, GUI, 펌웨어 바이너리는 포함하지 않습니다. `TEMP`는 설치·실행에 필요하지 않습니다.

## 요청과 검증

- 관측: `{"request_id":"1","op":"observe"}` → `status: ok`
- 실행: `{"request_id":"2","op":"execute","action":"wait","duration_ms":100}` → `status: completed`와 실행 후 observation
- `wait`의 `duration_ms`는 0–1000 정수입니다.
- 주행: `{"request_id":"3","op":"execute","action":"move","x":0.2,"y":0.0,"theta":0.0,"duration_ms":500}`. x 전진/y 왼쪽(m/s), theta 반시계(deg/s). 평행이동 속도는 0.4 m/s 이하, 회전은 ±150 deg/s, 시간은 1–1000 ms입니다.
- move는 바퀴 속도를 동시 전송하고 지정 시간 후 속도 0·토크 해제를 시도합니다. 오류·Ctrl+C·SIGTERM에도 정리 경로를 실행합니다. 프로세스 강제 종료나 USB 단절 때 물리적 정지를 보장하는 하드웨어 watchdog은 아닙니다.
- 미지원·범위 밖 요청은 `rejected`, 통신/정지 실패는 `error`입니다. 같은 로봇의 실행 중 추가 요청은 서버에서 busy로 거절합니다.

```bash
uv run --project lekiwi_runtime python -m unittest discover -s lekiwi_runtime/tests -v
```

테스트는 모의 센서, 레이더 패킷, 서보 오류·자원 정리, 설정 로딩과 ZeroMQ 응답을 검증합니다. 실물 ORIN의 카메라·UART 통신은 장비에서 별도로 확인해야 합니다.

## 폴더에서 직접 실행

로봇 클라이언트를 시작합니다. 기본 서버는 `tcp://127.0.0.1:5555`이며, ORIN에서 원격 서버를 사용할 때는 `--server tcp://SERVER_IP:5555`를 붙입니다.

```bash
cd lekiwi_runtime
uv run python main.py
```

`uv run`이 의존성과 프로젝트 설치를 준비합니다. 이미 설치된 Python 환경에서는 `python main.py`로 실행할 수 있습니다. 기존 CLI 옵션도 그대로 사용할 수 있습니다.

## 관찰과 연결된 실행

`observe` 응답은 `capabilities.guarded_execution: 1`, `observation.observation_id`, `observation.runtime_id`, `action_context`를 추가합니다. guarded `execute`는 기존 명령에 `observation_id`, `runtime_id`, `action_id`, `reason`을 모두 전달합니다. 일부 필드만 보내면 거부합니다.

- 관찰 유효기간은 기본 60초이며 runtime의 monotonic 시계로 검사합니다. 이전 관찰, runtime 재시작 전 관찰, 바뀐 관절 상태로 실행하지 않습니다.
- 베이스 이동은 최신 전방 이미지, 팔 동작은 최신 손목 이미지와 관절 피드백을 검사합니다. 레이더·IMU를 정확한 위치나 장애물 회피 보장으로 해석하지 않습니다.
- 같은 행동 ID와 같은 명령은 저장된 결과를 재전송합니다. 다른 명령에 같은 ID를 쓰면 거부합니다. 프로세스당 기본 128개 ID 또는 인코딩된 결과 32MiB까지 저장합니다. 공간이 차면 이전 ID를 지우고 재실행 가능하게 만들지 않고 guarded 실행을 차단합니다.
- 응답 `execution`은 명령 전송, 팔 도달, 베이스 정지 명령 전송을 구분합니다. 베이스의 실제 정지·거리 도달을 확인했다고 표시하지 않습니다.
- 관련 카메라가 동작 종료 이후 프레임을 제공할 때까지 제한된 시간 동안 기다립니다. 동작은 끝났지만 관찰이 없으면 `completed_without_observation`으로 알립니다. 실행 여부가 불명확한 오류는 해당 runtime의 guarded 실행을 중단합니다.
- 팔의 목표·토크 기록과 베이스 시작 전에 확인된 검증 오류는 `rejected`, `command_sent: false`, `error_code`, `error_details`로 반환합니다. 현재 보정 범위 위반은 `arm_current_out_of_calibration`이며 관절명·현재 raw/각도·허용 범위를 포함합니다. 재관찰 후 수정한 새 명령을 받을 수 있습니다. 일반 예외나 전송 이후 실패를 전송 전 거부로 추정하지 않습니다.

guard 필드가 없는 기존 호출은 이전 인터페이스를 사용합니다. 이 호출도 이전 guarded 관찰을 무효화하므로 다른 판단 루프가 오래된 관찰로 이어서 움직일 수 없습니다. 프로세스 재시작과 복구 시 과거 행동은 자동 재생하지 않습니다.

## 팔 액션

`{"op":"execute","request_id":"arm-1","action":"move","joints":{"wrist_roll":10},"duration_ms":1000}`

- 절대 관절 각도(deg), 그리퍼는 0–100%입니다. 부분 목표를 허용하며 생략한 관절에는 쓰지 않습니다.
- `ego.limits`로 보정된 범위와 속도 제한을 제공합니다. 현재 위치·위치 모드·온도·하드웨어 homing offset을 검사한 후 현재 위치를 목표로 기록하고 토크를 켭니다.
- `ego.calibration_status`로 현재 raw 위치의 보정 범위 위반을 표시합니다. `ego.available`은 센서 읽기 가능 여부이며 모든 관절의 이동 허가를 뜻하지 않습니다. 범위를 벗어난 관절은 현재 위치가 정상 범위로 돌아왔는지 확인하기 전까지 명령 대상에서 제외해야 합니다.
- 50Hz 목표 보간과 서보 속도 제한을 적용합니다. 관절 30deg/s, 그리퍼 50%/s입니다. 팔 단독 이동(`joints`가 비어 있지 않고 `x=y=theta=0`)은 100–10000ms를 지정할 수 있습니다. 더 긴 시간은 같은 목표까지 천천히 보간하며 속도·보정 범위를 늘리지 않습니다. 베이스가 함께 움직이면 최대 1000ms입니다.
- 보간 전 이동 시간·속도 설정(레지스터 44·46)의 응답이 timeout 또는 손상으로 누락되면, 최대 두 번 읽어 요청값이 실제로 저장됐는지 확인합니다. 정확히 일치할 때만 기존 동작을 이어가며 설정 쓰기나 행동 전체를 재전송하지 않습니다. 확인 내역은 `execution.arm_motion.communication_recoveries`와 `ego.last_motion.communication_recoveries`에 남습니다. 값 불일치·읽기 실패·서보 오류 및 목표 위치·토크 쓰기 실패는 이 복구 대상이 아닙니다.
- 보간이 끝나면 마지막 목표를 유지한 채 최대 2초 동안 관절 피드백만 읽습니다. 이 대기 중 목표·토크·설정 쓰기를 반복하지 않습니다. 허용오차는 관절 2도, 그리퍼 5%p 그대로이며 미도달은 error입니다. 이전 500ms보다 늦게 안정되는 서보를 기다릴 여유를 주지만, 부하·접촉·정지 문제의 해결을 보장하지 않습니다.
- 도달 실패 시 `ego.last_motion`에 `reached: false`, 실패한 관절, 목표·측정 raw 및 변환값, 잔차·허용오차, 보간·피드백 대기 시간을 남깁니다. 이 측정은 오류 처리의 hold 이전 값이며 이후 관찰에서도 확인할 수 있습니다. 오류 메시지에도 해당 관절의 목표·측정·잔차가 포함됩니다. 통신 오류를 도달 실패로 바꾸지 않으며, 기존 실행 미확정 차단과 자동 재시도 금지는 유지합니다.
- 적용 후 관찰의 `ego.control.revision`이 `bounded-settling-v2`, `feedback_timeout_s`가 `2.0`인지 확인합니다. 이 필드가 없으면 이전 프로세스 또는 다른 설치 경로가 실행 중입니다. 파일 교체만으로 실행 중인 Python 프로세스가 갱신되지는 않습니다. 재시작 시 팔 토크가 해제되므로 팔을 지지한 상태에서 작업합니다.
- 정상 완료 후 목표 자세를 유지합니다. 중단·오류에는 측정 위치를 hold하도록 시도하고 자동으로 팔 토크를 끄지 않습니다. 이는 충돌 회피나 물체 파지 성공을 보장하지 않습니다.
- 팔 기능을 사용하려면 ORIN에 변경된 runtime을 설치하고 재시작해야 합니다. 단순 관측과 베이스 주행은 팔 토크를 활성화하지 않습니다.

## 베이스·팔 동시 제어와 종료

`move`는 베이스 속도와 팔 목표를 같은 시간 동안 실행합니다. 베이스 속도가 하나라도 0이 아니면 팔 목표가 있어도 최대 1000ms이며, `wait`도 기존 0–1000ms 범위를 유지합니다.

```json
{"action":"move","x":0.05,"y":0,"theta":0,"joints":{"wrist_roll":10},"duration_ms":1000}
```

팔 보정·속도·모드 검사를 통과한 후 바퀴 속도를 켜고 팔 목표를 보간합니다. 한 스레드에서 공유 버스를 사용하며 팔 이동 구간이 끝나면 바퀴를 정지한 뒤 팔 도달 여부를 확인합니다. 오류나 인터럽트에도 바퀴 정지를 시도합니다.

정상 종료·Ctrl+C·SIGTERM에는 바퀴 ID 7–9를 정지하고 토크를 해제한 뒤 팔 ID 1–6의 토크도 해제하고 포트를 닫습니다. 한 모터의 통신 실패가 다른 모터의 해제 시도를 막지 않습니다. 토크가 해제되면 팔은 자세를 유지하지 않습니다. SIGKILL·전원 차단·통신 단절 시에는 소프트웨어 정리가 보장되지 않습니다.

## IMU와 추정 변위

기존 IMU 테스트 코드의 BMI160 설정을 사용합니다. 기본 I²C 버스는 7, 주소는 105(`0x69`)이며 `imu: {"enabled": true, "bus": 7, "address": 105}`로 변경할 수 있습니다. `enabled: false`이면 하드웨어에 접근하지 않습니다. 드라이버는 런타임에 포함되어 TEMP 폴더나 추가 패키지가 필요하지 않습니다. 실행 계정에 `/dev/i2c-7` 접근 권한이 필요합니다.

시작 후 약 1초간 로봇을 정지시켜 초기 자세·자이로 편향·중력 방향 가속도 편향을 추정합니다. 움직임이 감지되면 보정을 실패 처리합니다. 균일한 속도의 이동은 IMU만으로 정지와 구별할 수 없으므로 실제 초기 정지가 필요합니다. 샘플은 약 100 Hz로 연속 수집하며 observe 호출 빈도와 무관하게 적분합니다.

`observation.imu` 필드:

| 필드 | 의미 |
| --- | --- |
| `available`, `timestamp`, `age_s`, `error` | 센서 상태와 최신 샘플 시각·경과 시간 |
| `accel_m_s2`, `gyro_deg_s`, `temperature_c` | 센서 좌표계 가속도(중력 포함), 각속도, 온도 |
| `orientation_deg` | 시작 시 yaw=0인 상대 roll/pitch/yaw |
| `linear_accel_m_s2` | 자세를 반영해 좌표를 변환하고 중력을 제거한 가속도 |
| `velocity_m_s`, `displacement_m` | 초기 정지·원점으로부터 적분한 XYZ 추정 속도·변위 |
| `estimate_valid`, `estimate_error` | 적분 연속성·포화 여부에 따른 사용 가능 상태 |
| `origin_id`, `origin_timestamp`, `frame` | 원점 식별자, 보정 시각, 시작 시 중력 정렬 센서 좌표계 |

변위는 실제 위치 측정값이 아닙니다. 시작 시 센서 수평 방향을 기준으로 하며 로봇 전방 축과 동일하다고 가정하지 않습니다. 초기 정지 보정 이후에는 자이로로 자세를 적분합니다. 단일 자세 보정은 가속도계의 모든 축 편향을 분리할 수 없으며 자이로·가속도 편향, 진동, 온도, 기울기 오차가 변위에 누적됩니다. 별도 엔코더·영상 보정이나 저가속도 기반 속도 강제 초기화는 하지 않습니다. `estimate_valid: true`는 연속 적분이 가능하다는 뜻이지 정확도가 보장된다는 뜻이 아닙니다.

샘플 간격이 100ms를 초과하거나 포화되면 추정 변위·속도는 null이 되고 재시작 전까지 무효 상태를 유지합니다. 마지막 샘플이 250ms 이상 오래되면 센서도 unavailable로 표시합니다. 센서 연결 실패 시 다른 센서는 계속 사용 가능합니다. 재시작하면 원점이 바뀌므로 전후 차이는 같은 origin_id끼리만 비교합니다. `--mock` IMU는 정지된 합성 데이터이며 실제 움직임을 나타내지 않습니다.

레지스터·단위 참고: [Bosch BMI160 데이터시트](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi160-ds000.pdf).
