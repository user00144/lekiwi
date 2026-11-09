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
- 팔: WCH/FTDI/CP210x USB 포트에서 ID 1–6이 응답하는 보드를 찾습니다. 후보가 여러 개이거나 다른 USB 어댑터를 사용하면 `arm.port`를 지정합니다. 관측은 위치 읽기만 수행하며 토크·이동 명령은 보내지 않습니다.

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
| `orin.py` | 센서 시작·종료, observation 조합, wait 실행 |
| `config.py`, `data/` | 기본 설정과 장비별 설정 로딩 |
| `drivers/camera.py` | 최신 프레임 캡처와 JPEG 인코딩 |
| `drivers/radar.py` | 레이더 UART 수신과 초기화 |
| `drivers/radar_protocol.py` | 포인트·SNR 패킷 파싱 |
| `drivers/arm.py` | 서보 연결, 위치 읽기와 보정값 변환 |

참고 코드 `TEMP/Lekiwi`에서 관측에 필요한 로직만 옮겼습니다. teleop, 리더 암, 바퀴 제어, 녹화, GUI, 펌웨어 바이너리는 포함하지 않습니다. `TEMP`는 설치·실행에 필요하지 않습니다.

## 요청과 검증

- 관측: `{"request_id":"1","op":"observe"}` → `status: ok`
- 실행: `{"request_id":"2","op":"execute","action":"wait","duration_ms":100}` → `status: completed`와 실행 후 observation
- 지원 액션은 `wait`이며 `duration_ms`는 0–1000 정수입니다. 미지원 요청은 `rejected`, 요청 처리 예외는 `error`입니다.

```bash
uv run --project lekiwi_runtime python -m unittest discover -s lekiwi_runtime/tests -v
```

테스트는 모의 센서, 레이더 패킷, 서보 오류·자원 정리, 설정 로딩과 ZeroMQ 응답을 검증합니다. 실물 ORIN의 카메라·UART 통신은 장비에서 별도로 확인해야 합니다.
