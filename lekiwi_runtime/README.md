# Lekiwi Runtime

ORIN에서 카메라 2대, AWR2944 레이더, STS3215 팔 관절을 읽고 ZeroMQ DEALER로 서버 요청에 응답합니다. 하드웨어 코드와 기본 설정을 이 패키지에 포함하므로 외부 ORIN 폴더가 필요하지 않습니다.

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
| `orin.py` | 센서 시작·종료, observation 조합, wait/move 실행 |
| `config.py`, `data/` | 기본 설정과 장비별 설정 로딩 |
| `drivers/camera.py` | 최신 프레임 캡처와 JPEG 인코딩 |
| `drivers/radar.py` | 레이더 UART 수신과 초기화 |
| `drivers/radar_protocol.py` | 포인트·SNR 패킷 파싱 |
| `drivers/arm.py` | 팔 위치 읽기와 보정값 변환 |
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
