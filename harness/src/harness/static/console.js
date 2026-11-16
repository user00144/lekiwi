"use strict";

(() => {
  const $ = (id) => document.getElementById(id);
  const state = {
    server: null, currentRun: null, selectedRun: null, selectedId: null,
    started: false, busy: false, stoppingId: null, connected: false,
    timelineKey: null, historyKey: null, imageView: "current", announcement: null,
  };
  const manual = {view: null, busy: false, dirty: false, key: null, lastKey: null, lastRun: null, visualReady: false};
  const preview = {data: null, busy: false, capturing: false, initialized: false, runsKey: null};
  const statusLabels = {
    idle: "대기", reference_saved: "기준 자세 저장", running: "실행 중", starting: "준비 중", stopping: "중지 중",
    interrupted: "중지됨", stopped: "중지됨", completion_claim: "완료 주장",
    blocked: "작업 보류", budget_actions: "행동 한도", budget_steps: "판단 한도",
    budget_seconds: "시간 한도", uncertain_execution: "실행 미확정",
    incomplete: "목표 미도달 · 재계획 필요", recovery_exhausted: "상태 재확인 한도",
    reconciliation_failed: "상태 재확인 실패",
    post_motion_observation_failed: "재관찰 실패", observation_failed: "관찰 실패",
    observation_invalid: "관찰 확인 필요", guarded_execution_unavailable: "실행 미지원",
    recorded_observation: "기록 실행 불가", runtime_changed: "로봇 연결 변경",
    invalid_decision: "판단 확인 필요", repeated_failed_action: "반복 행동 중단",
    host_error: "실행 오류", gui_error: "GUI 실행 오류", startup_error: "시작 실패", error: "오류", cancelled: "취소됨",
    interrupted_unknown: "종료 상태 미확정", preparing: "상태 확인 중", completed: "이동 응답 확인", expired: "계산 만료", prepared: "이동 준비", rejected: "이동 거부", executing: "이동 중",
  };
  const toolLabels = {observe: "재관찰", execute: "행동 실행", task_complete: "완료 주장", blocked: "작업 보류"};
  const feedbackLabels = {initial: "첫 판단", progress: "진전 확인", stalled: "진전 없음", uncertain: "판단 유보"};
  const eventLabels = {
    startup: "작업 준비", startup_started: "작업 준비", startup_finished: "준비 완료", demo_started: "데모 시작",
    planner_started: "에이전트 연결", observe_started: "새 관찰 요청", observe_finished: "관찰 수신",
    observation: "관찰 저장", pose_reference: "현재 ↔ 기본 자세 비교", kinematics_state: "현재 팔 위치 계산", ik_result: "XYZ 목표 변환 결과", planning_started: "다음 행동 판단 중", planning_finished: "판단 수신",
    decision: "에이전트 판단", decision_rejected: "판단 보류", action_prepared: "명령 준비",
    action_started: "행동 실행", action_finished: "행동 응답", action_result: "실행 결과",
    motion_feedback: "실행 전후 비교", rejection_feedback: "명령 거부 · 재관찰 후 수정", completion_candidate: "완료 주장 · 재관찰 요청",
    motion_incomplete: "목표 미도달 · 자세 유지 확인", reconcile_started: "상태 재확인",
    reconcile_finished: "상태 재확인 응답", reconcile_result: "상태 재확인 결과",
    motion_failure_feedback: "미도달 원인과 현재 관절", motion_replanning: "새 경로 재계획",
    motion_recovery_completed: "수정한 팔 목표 도달",
    episode_finished: "작업 종료", stopping: "루프 중지 요청", stop_requested: "루프 중지 요청",
    manual_operator_confirmation: "사용자 1회 이동 확인", error: "오류",
  };
  const jointLabels = {shoulder_pan: "어깨 회전", shoulder_lift: "어깨 높이", elbow_flex: "팔꿈치", wrist_flex: "손목 굽힘", wrist_roll: "손목 회전", gripper: "그리퍼"};
  const armJoints = Object.keys(jointLabels).filter(name => name !== "gripper");
  const rejectionLabels = {
    arm_current_out_of_calibration: "현재 관절 위치가 보정 범위를 벗어났습니다.",
    arm_target_out_of_range: "목표 관절값이 허용 범위를 벗어났습니다.",
    arm_speed_limit: "요청한 관절 변화가 속도 제한을 넘습니다.",
    arm_state_changed: "판단 이후 관절 위치가 바뀌었습니다.",
    observation_expired: "판단에 사용한 관찰의 유효기간이 지났습니다.",
    observation_mismatch: "명령과 현재 관찰이 일치하지 않습니다.",
    arm_feedback_unavailable_or_stale: "최신 관절 피드백을 확인하지 못했습니다.",
    wrist_camera_unavailable_or_stale: "최신 손목 카메라 관찰이 필요합니다.",
    front_camera_unavailable_or_stale: "최신 전방 카메라 관찰이 필요합니다.",
    no_fresh_camera: "사용할 수 있는 최신 카메라 관찰이 없습니다.",
    imu_stale: "IMU 관찰이 오래되어 베이스 명령을 보내지 않았습니다.",
    repeated_failed_action: "거부된 명령이 수정 없이 반복되었습니다.",
    arm_recovery_constraints: "재확인한 현재 관절값에 맞게 작은 팔 동작으로 수정해야 합니다.",
  };

  function node(tag, className, text) {
    const element = document.createElement(tag);
    if (className) element.className = className;
    if (text !== undefined && text !== null) element.textContent = String(text);
    return element;
  }
  function text(id, value) { $(id).textContent = value == null || value === "" ? "—" : String(value); }
  function numeric(value, fallback = 0) { return typeof value === "number" && Number.isFinite(value) ? value : fallback; }
  function shortId(value) { return typeof value === "string" && value ? value.slice(0, 8) : "—"; }
  function clock(value) {
    const seconds = Math.max(0, Math.floor(numeric(value)));
    return `${Math.floor(seconds / 60).toString().padStart(2, "0")}:${(seconds % 60).toString().padStart(2, "0")}`;
  }
  function date(value) {
    if (!value) return null;
    const parsed = new Date(typeof value === "number" ? value * 1000 : value);
    return Number.isNaN(parsed.getTime()) ? null : parsed;
  }
  function timeLabel(value) { return date(value)?.toLocaleTimeString("ko-KR", {hour12: false, hour: "2-digit", minute: "2-digit", second: "2-digit"}) || "—"; }
  function historyDate(value) { return date(value)?.toLocaleString("ko-KR", {month: "short", day: "numeric", hour: "2-digit", minute: "2-digit", hour12: false}) || "시각 정보 없음"; }
  function label(status) { return statusLabels[status] || status || "대기"; }
  function statusTone(status) {
    if (["running", "starting", "completion_claim"].includes(status)) return "good";
    if (["host_error", "gui_error", "startup_error", "error", "uncertain_execution", "post_motion_observation_failed", "observation_failed", "reconciliation_failed"].includes(status)) return "error";
    if (["blocked", "stopping", "budget_actions", "budget_steps", "budget_seconds", "repeated_failed_action", "incomplete", "recovery_exhausted"].includes(status)) return "warning";
    return "neutral";
  }
  function badge(id, value, tone = "neutral") { text(id, value); $(id).className = `badge ${tone}`; }
  function announce(message) {
    if (state.announcement === message) return;
    state.announcement = message;
    text("announcer", message);
  }
  function notice(message) { $("notice").hidden = !message; text("notice", message); }

  async function api(path, options = {}) {
    const response = await fetch(path, {
      cache: "no-store", credentials: "same-origin", signal: AbortSignal.timeout(10000),
      ...options,
      headers: {Accept: "application/json", ...(options.body ? {"Content-Type": "application/json"} : {}), ...options.headers},
    });
    let payload;
    try { payload = await response.json(); } catch { throw new Error(`서버 응답을 읽을 수 없습니다 (${response.status}).`); }
    if (!response.ok) throw new Error(payload.error || `요청을 처리하지 못했습니다 (${response.status}).`);
    return payload;
  }

  function setConnection(connected) {
    state.connected = connected;
    const target = $("connection");
    target.className = `connection ${connected ? "connected" : "disconnected"}`;
    target.replaceChildren(node("i"), document.createTextNode(connected ? "GUI 연결됨" : "GUI 연결 끊김"));
    target.firstChild.setAttribute("aria-hidden", "true");
    updateControls();
    previewControls();
    manualControls();
  }

  function initialize(config) {
    if (state.started) return;
    state.started = true;
    if (config.robot_id) $("robot-id").value = config.robot_id;
    $("model").value = config.model || "";
    for (const [field, id] of [["max_actions", "max-actions"], ["max_steps", "max-steps"], ["max_seconds", "max-seconds"]]) {
      if (config.limits?.[field]) $(id).value = config.limits[field];
    }
  }

  function updateControls() {
    const activeId = state.server?.active_run_id;
    const readingOther = state.selectedId && state.selectedId !== activeId;
    const active = Boolean(activeId);
    $("start").disabled = !state.connected || active || state.busy || manual.busy || preview.capturing || state.server?.pose_capture_busy || Boolean(state.server?.manual_xyz?.busy || state.server?.manual_xyz?.prepared);
    $("stop").disabled = !state.connected || !active || Boolean(readingOther) || state.stoppingId === activeId || state.busy;
    const mode = state.server?.mode;
    text("start-label", state.busy ? "작업 시작 중…" : mode === "demo" ? "데모 루프 실행" : "로봇 에이전트 실행");
    $("demo-note").hidden = mode !== "demo";
    $("live-note").hidden = mode !== "live";
    badge("mode-badge", mode === "demo" ? "DEMO SESSION" : mode === "live" ? "LIVE SESSION" : "LOCAL", mode === "demo" ? "demo" : mode === "live" ? "live" : "neutral");
    text("stop-note", state.stoppingId === activeId && active
      ? "중지를 요청했습니다. 이미 전송된 동작의 결과가 미확정일 수 있습니다. 다음 명령은 보내지 않습니다."
      : readingOther && active ? "다른 작업이 실행 중입니다. 현재 실행으로 돌아가면 루프를 중지할 수 있습니다."
        : "루프 중지는 다음 명령을 막습니다. 이미 전송된 동작의 즉시 정지를 보장하지 않습니다.");
    if (state.server?.config?.server) text("endpoint-label", state.server.config.server);
  }

  function phaseInfo(run) {
    const phase = String(run?.phase || "").toLowerCase();
    if (phase === "prepared") return ["reviewing", "수동 XYZ 목표 확인을 기다립니다"];
    if (phase === "preparing") return ["observing", "수동 XYZ 이동을 위한 관찰과 계산 중입니다"];
    if (phase === "reconciling") return ["observing", "목표 미도달 후 관절과 자세 유지를 다시 확인합니다"];
    if (phase === "replanning") return ["thinking", "현재 관절값으로 새 경로를 재계획합니다"];
    if (run?.status === "stopping" || state.stoppingId === run?.id) return ["reviewing", "다음 행동을 중지하고 있습니다"];
    if (/action|execut|mov/.test(phase)) return ["executing", "로봇이 한 행동을 실행하고 있습니다"];
    if (/plan|thinking|decid|decision/.test(phase)) return ["thinking", "관찰을 바탕으로 다음 행동을 판단합니다"];
    if (/review|verif|feedback|confirm|reject|recover/.test(phase)) return ["reviewing", "실행 결과와 새 관찰을 확인합니다"];
    if (/observ|captur/.test(phase)) return ["observing", "로봇의 현재 상태를 관찰하고 있습니다"];
    return ["observing", "작업을 준비하고 있습니다"];
  }

  function latestEvent(run, kind) { return [...(run?.events || [])].reverse().find((event) => event.event === kind); }
  function runElapsed(run) {
    if (!run) return 0;
    if (!run.active && typeof run.result?.elapsed_s === "number") return run.result.elapsed_s;
    const events = run.events || [];
    const last = numeric(events[events.length - 1]?.elapsed_s);
    const start = date(run.started_at || events[0]?.at);
    return run.active && start ? Math.max(last, (Date.now() - start.getTime()) / 1000) : last;
  }

  function renderRun(run) {
    const events = run?.events || [];
    const decision = latestEvent(run, "decision")?.data;
    const result = run?.result;
    const status = run?.status || (run?.active ? "running" : "idle");
    const limits = run?.request?.limits || state.server?.config?.limits || {max_actions: 20, max_steps: 40};
    const actions = Math.max(numeric(result?.action_attempts), events.filter((event) => event.event === "action_prepared").length,
      ...events.map((event) => numeric(event.data?.action_attempts)));
    const decisions = Math.max(numeric(result?.steps), events.filter((event) => event.event === "decision").length,
      ...events.map((event) => Math.max(numeric(event.data?.steps), event.event === "planning_started" ? numeric(event.data?.step) : 0)));
    const [phase, headline] = phaseInfo(run);
    text("status-title", !run ? "작업을 기다리고 있습니다" : run.active ? headline : status === "completion_claim" ? "재관찰 후 완료 주장을 확인했습니다" : label(status));
    text("run-label", run ? `${state.selectedId ? "SAVED RUN" : "CURRENT RUN"} / ${shortId(run.id).toUpperCase()}` : "READY FOR YOUR NEXT TASK");
    $("run-task").hidden = !run;
    text("run-instruction", run?.request?.instruction);
    text("run-context", run ? `${run.request?.robot_id || "로봇 미지정"} · ${run.mode === "demo" ? "고정 시연 스크립트" : run.request?.model || "기본 모델"}` : "");
    text("elapsed", clock(runElapsed(run)));
    text("action-count", actions); text("action-limit", ` / ${limits.max_actions ?? 20}`);
    text("step-count", decisions); text("step-limit", ` / ${limits.max_steps ?? 40}`);
    text("observation-id", shortId(run?.current_observation?.observation_id));
    $("observation-id").title = run?.current_observation?.observation_id || "";
    badge("result-badge", label(status), statusTone(status));
    $("status-light").className = `status-light ${run?.active ? "running" : statusTone(status) === "error" ? "error" : ""}`;
    $("run-mode").hidden = !run;
    if (run) badge("run-mode", run.mode === "demo" ? "데모 기록" : run.mode === "reference" ? "관찰만 저장" : run.mode === "live" ? "로봇 실행" : "실행 기록", run.mode === "demo" ? "demo" : "neutral");
    document.querySelectorAll(".phase-track li").forEach((element) => {
      const selected = Boolean(run?.active && element.dataset.phase === phase);
      element.classList.toggle("active", selected);
      if (selected) element.setAttribute("aria-current", "step"); else element.removeAttribute("aria-current");
    });
    let summary = result?.detail || "";
    if (status === "completion_claim") summary = `${result?.completion_claim?.evidence || result?.detail || "에이전트가 완료를 보고했습니다."} · 물리적 성공은 미확정입니다.`;
    if (run?.mode === "demo" && !run.active) summary += `${summary ? " " : ""}입력 지시와 무관한 고정 데모 결과입니다.`;
    $("run-summary").hidden = !summary; text("run-summary", summary);
    if (run) announce(`${label(status)}. ${run.active ? headline : ""}`);
    renderObservations(run);
    renderDecision(decision, latestEvent(run, "motion_feedback")?.data);
    renderRejection(run);
    renderMotionFailure(run);
    renderKinematics(run);
    renderTimeline(run);
  }

  function safeImageUrl(value) {
    if (typeof value !== "string") return null;
    try {
      const url = new URL(value, location.origin);
      return url.origin === location.origin && url.pathname.startsWith("/api/runs/") ? url.href : null;
    } catch { return null; }
  }
  function setImage(id, value) {
    const img = $(id);
    const frame = img.parentElement;
    const url = safeImageUrl(value);
    if (!url) {
      img.hidden = true;
      img.removeAttribute("src");
      frame.classList.remove("has-image", "failed");
      frame.querySelector(".camera-empty p").textContent = id.endsWith("-before") ? "동작 전 관찰이 없습니다" : "카메라 관찰을 기다리고 있습니다";
      return;
    }
    if (img.getAttribute("src") !== url) {
      img.hidden = true;
      frame.classList.remove("failed", "has-image");
      frame.querySelector(".camera-empty p").textContent = "관찰 이미지를 불러오는 중입니다";
      img.onload = () => { img.hidden = false; frame.classList.add("has-image"); frame.classList.remove("failed"); };
      img.onerror = () => { img.hidden = true; frame.classList.add("failed"); frame.classList.remove("has-image"); frame.querySelector(".camera-empty p").textContent = "이미지를 불러올 수 없습니다"; };
      img.src = url;
    }
  }
  function cameraImage(observation, camera) {
    return observation?.images?.find((image) => String(image.label).toLowerCase().includes(camera))?.url;
  }
  function renderObservations(run) {
    const current = run?.current_observation;
    const before = run?.before_observation;
    const observation = current?.packet?.observation || {};
    const beforeMotion = before && latestEvent(run, "motion_feedback")?.data?.before_observation_id === before.observation_id;
    document.querySelectorAll(".before-frame figcaption").forEach((caption) => { caption.textContent = beforeMotion ? "직전 동작 전" : "이전 관찰"; });
    for (const name of ["front", "wrist"]) {
      setImage(`${name}-current`, cameraImage(current, name));
      setImage(`${name}-before`, cameraImage(before, name));
      text(`${name}-time`, timeLabel(observation.cameras?.[name]?.timestamp || current?.at));
    }
    const mode = run?.mode === "demo" || observation.simulated ? "합성 관찰" : state.selectedId ? "저장된 관찰" : "로봇 관찰";
    text("camera-source", current ? `${mode} · 관찰 ${String(current.sequence).padStart(2, "0")} · ${timeLabel(current.at)} 수신` : "관찰 대기 · 실시간 영상 스트림이 아닙니다");
    badge("sensor-badge", current ? observation.simulated ? "SIMULATED" : state.selectedId ? "RECORDED" : "OBSERVATION" : "센서 대기", observation.simulated ? "demo" : "subtle");
    const joints = observation.ego?.joints;
    $("joint-values").hidden = !joints || !Object.keys(joints).length;
    $("joint-values").replaceChildren();
    for (const [name, value] of Object.entries(joints || {})) {
      if (typeof value !== "number" || !Number.isFinite(value)) continue;
      const item = node("span", "joint-chip", jointLabels[name] || name);
      item.append(node("b", "", `${value.toFixed(1)}${name === "gripper" ? "%" : "°"}`));
      $("joint-values").append(item);
    }
    renderCalibration(observation.ego || {});
    renderCalibrationId(observation.ego?.calibration_id);
  }

  function calibrationLine(detail) {
    const name = jointLabels[detail.joint] || detail.joint || "관절";
    const values = [detail.current, detail.min, detail.max];
    const unit = detail.unit === "%" || detail.joint === "gripper" ? "%" : "°";
    let line = values.every((value) => typeof value === "number" && Number.isFinite(value))
      ? `${name} ${values[0].toFixed(3)}${unit} / 허용 ${values[1].toFixed(3)}~${values[2].toFixed(3)}${unit}` : name;
    if (detail.joint === "gripper" && [detail.raw, detail.raw_min, detail.raw_max].every(Number.isFinite)) {
      line += ` · 센서 원시값 ${detail.raw} / 보정 ${detail.raw_min}~${detail.raw_max}`;
    }
    return `${line} · 해당 관절 이동 불가`;
  }
  function renderCalibration(ego) {
    let panel = $("calibration-diagnostics");
    if (!panel) {
      panel = node("div", "calibration-diagnostics");
      panel.id = "calibration-diagnostics";
      panel.setAttribute("aria-label", "관절 범위 진단");
      $("joint-values").after(panel);
    }
    // Older saved observations have joint limits but no structured diagnosis.
    const issues = Array.isArray(ego.calibration_status?.out_of_range) ? ego.calibration_status.out_of_range
      : Object.entries(ego.joints || {}).flatMap(([joint, current]) => {
        const range = ego.limits?.[joint];
        return range && [current, range.min, range.max].every(Number.isFinite) && (current < range.min - .001 || current > range.max + .001)
          ? [{joint, current, min: range.min, max: range.max, unit: range.unit}] : [];
      });
    panel.hidden = issues.length === 0;
    panel.replaceChildren();
    if (!issues.length) return;
    panel.append(node("strong", "", "현재 관절 범위 확인"));
    issues.forEach((detail) => panel.append(node("p", "", calibrationLine(detail))));
  }

  function renderCalibrationId(value) {
    let panel = $("calibration-identity");
    if (!panel) {
      panel = node("div", "calibration-identity");
      panel.id = "calibration-identity";
      const title = node("label", "", "보정 식별자 · XYZ 매핑 설정에 사용");
      title.htmlFor = "calibration-id-value";
      const input = node("input");
      input.id = "calibration-id-value";
      input.readOnly = true;
      input.setAttribute("spellcheck", "false");
      input.addEventListener("click", () => input.select());
      panel.append(title, input, node("small", "", "값을 선택해 복사할 수 있습니다. 보정 범위를 바꾸면 식별자도 바뀝니다."));
      $("calibration-diagnostics").after(panel);
    }
    panel.hidden = typeof value !== "string" || !/^[a-f0-9]{64}$/i.test(value);
    // Preserve an active text selection while the GUI polls the same observation.
    if ($("calibration-id-value").value !== value) $("calibration-id-value").value = panel.hidden ? "" : value;
  }

  function vectorLabel(value) {
    return Array.isArray(value) && value.length === 3 && value.every(Number.isFinite)
      ? value.map((part, index) => `${["X", "Y", "Z"][index]} ${part.toFixed(4)}`).join(" · ") + " m" : null;
  }
  function solutionLabel(data) {
    const runtime = data.solution_runtime_joints_deg || data.joints;
    const solution = runtime || data.solution_urdf_joints_deg || data.solution_joints_deg;
    const parts = Object.entries(solution || {}).filter(([, value]) => Number.isFinite(value));
    return parts.length ? {label: runtime ? "관절 해 · runtime" : "관절 해 · URDF",
      text: parts.map(([name, value]) => `${jointLabels[name] || name} ${value.toFixed(2)}°`).join(" · ")} : null;
  }
  function ikStatus(data) {
    return data.success === true && data.executable === true
      ? ["해 계산 성공 · 전송 전 검사 가능", "good"]
      : data.success === true ? ["해 계산 성공 · 미리보기", "warning"] : ["해 미확정 · 전송 안 함", "warning"];
  }
  function kinematicReasons(parent, data) {
    const reasons = Array.isArray(data.reasons) ? data.reasons.filter((reason) => typeof reason === "string" && reason) : [];
    for (const reason of reasons) parent.append(node("p", "kinematics-reason", reason));
  }
  function ikLines(parent, data) {
    if (data.request?.mode === "relative") eventLine(parent, "요청 이동량", vectorLabel(data.request.xyz_m));
    eventLine(parent, "목표 XYZ", vectorLabel(data.target_xyz_m || (data.request?.mode === "absolute" ? data.request.xyz_m : null)));
    eventLine(parent, "계산 끝점", vectorLabel(data.achieved_xyz_m));
    const solution = solutionLabel(data);
    if (solution) eventLine(parent, solution.label, solution.text);
    if (Number.isFinite(data.residual_m)) {
      const tolerance = Number.isFinite(data.tolerance_m) ? ` / 허용 ${(data.tolerance_m * 1000).toFixed(2)} mm` : "";
      eventLine(parent, "위치 오차", `${(data.residual_m * 1000).toFixed(2)} mm${tolerance}`);
    }
    if (data.orientation_checked === true && Number.isFinite(data.orientation_error_deg)) {
      eventLine(parent, "접근축 오차", `${data.orientation_error_deg.toFixed(2)}°`);
    }
    eventLine(parent, "명령 전송", data.success === true && data.executable === true
      ? "IK 검사 통과 · 실제 전송과 도달 여부는 실행 결과에서 확인" : "이 XYZ 요청은 미전송 · 미리보기 또는 제한 조건 확인 필요");
    kinematicReasons(parent, data);
  }

  let viewerModule;
  const renderVersions = new Map();
  async function drawRobot(prefix, data) {
    const element = $(`${prefix}-plot`), version = (renderVersions.get(prefix) || 0) + 1;
    renderVersions.set(prefix, version);
    element.dataset.fkMatch = "false";
    if (prefix === "manual") manual.visualReady = false;
    for (const option of $(`${prefix}-display`).options) option.disabled = option.value !== "current" && !data?.result?.success;
    if (!data?.result?.success) $(`${prefix}-display`).value = "current";
    const target = data?.result?.target_xyz_m, current = data?.state?.current_xyz_m;
    text(`${prefix}-movement`, target && current
      ? "팔 끝 목표 이동: " + target.map((v, i) => `${["X", "Y", "Z"][i]} ${(1000 * (v - current[i])).toFixed(1)} mm`).join(" · ")
      : "현재 자세 · 목표 계산 전");
    try {
      viewerModule ||= import("/static/urdf-viewer.js");
      const module = await viewerModule;
      if (renderVersions.get(prefix) !== version) return;
      await module.updateViewer(element, data, $(`${prefix}-view`).value, $(`${prefix}-display`).value);
      if (renderVersions.get(prefix) !== version) return;
      if (prefix === "manual") manual.visualReady = element.dataset.fkMatch === "true";
    } catch (error) {
      element.dataset.fkMatch = "false";
      element.replaceChildren(node("p", "kinematics-reason", `3D 표시 오류: ${error.message}`));
    } finally { if (prefix === "manual") manualControls(); }
  }
  function drawPreview() { drawRobot("xyz", preview.data); }

  function previewControls() {
    const enabled = state.server?.capabilities?.xyz_preview === 1 && state.connected;
    for (const element of $("xyz-form").querySelectorAll("input, select, button")) element.disabled = preview.busy || !enabled;
    $("xyz-duration").disabled ||= $("xyz-source").value === "model";
    $("xyz-solve").textContent = preview.busy ? "계산 중…" : "XYZ 미리보기 계산";
    $("xyz-joint-editor").hidden = $("xyz-source").value !== "model";
    $("xyz-capture").disabled = !enabled || state.server?.capabilities?.pose_capture !== 1 || preview.busy || preview.capturing || state.server?.pose_capture_busy || Boolean(state.server?.active_run_id || state.server?.manual_xyz?.busy || state.server?.manual_xyz?.prepared);
    $("xyz-capture").textContent = preview.capturing ? "현재 관절을 읽어 비교하는 중…" : "현재 로봇 자세 읽고 기본 자세와 비교";
    $("xyz-availability").hidden = enabled;
    text("xyz-availability", state.connected
      ? "실행 중인 GUI 서버가 이전 버전입니다. GUI 터미널에서 Ctrl+C 후 같은 harness-gui 명령으로 다시 실행해주세요."
      : "GUI 연결을 기다리고 있습니다.");
  }

  function updatePreviewSources() {
    const runs = state.server?.runs || [];
    const key = JSON.stringify(runs.map(run => [run.id, run.instruction]));
    if (key !== preview.runsKey) {
      preview.runsKey = key;
      const select = $("xyz-source"), selected = select.value;
      select.replaceChildren(new Option("URDF 예시 자세 · 로봇 연결 불필요", "model"));
      for (const run of runs) select.add(new Option(`기록 ${run.id} · ${(run.instruction || "작업").slice(0, 30)}`, run.id));
      if ([...select.options].some(option => option.value === selected)) select.value = selected;
    }
    previewControls();
    if (!preview.initialized && state.server?.capabilities?.xyz_preview === 1 && state.connected) {
      preview.initialized = true;
      requestPreview(false);
    }
  }

  function renderPreview() {
    const data = preview.data, output = $("xyz-output");
    output.replaceChildren();
    renderPoseReference(data?.pose_reference);
    if (!data) { output.append(node("p", "kinematics-note", "자세를 불러와 현재 XYZ를 확인하세요.")); drawPreview(); return; }
    text("xyz-provenance", data.source.kind === "model"
      ? "입력 기준: URDF 예시 관절각 · 로봇에 연결하지 않은 모델 계산"
      : `입력 기준: 저장 관찰 ${shortId(data.source.observation_id)} · ${historyDate(data.source.observed_at || data.source.at)} · 선택한 기록의 최신 관찰은 ‘자세 불러오기’로 갱신하세요.`);
    eventLine(output, "시작 XYZ", vectorLabel(data.state.current_xyz_m) || "관절 정보를 읽을 수 없습니다.");
    if (data.state.fk?.kinematics_library) eventLine(output, "계산 라이브러리", `Pinocchio ${data.state.fk.kinematics_version} · IK: OSQP`);
    if (data.result) {
      const [summary, tone] = ikStatus(data.result);
      output.append(node("span", `badge ${tone}`, summary));
      if (data.result.duration_check?.success) {
        const check = data.result.duration_check;
        const retry = node("button", "button xyz-secondary", `${check.checked_duration_ms / 1000}초로 바꿔 다시 계산 · 로봇 전송 없음`);
        retry.type = "button";
        retry.addEventListener("click", () => { $("xyz-duration").value = String(check.checked_duration_ms); requestPreview(true); });
        output.append(node("p", "kinematics-reason", `${check.requested_duration_ms}ms 제한에서는 오차 ${(data.result.residual_m * 1000).toFixed(2)}mm입니다. ${check.checked_duration_ms / 1000}초로 계산하면 허용 오차 안의 해를 찾습니다.`), retry);
      }
      ikLines(output, data.result);
    } else kinematicReasons(output, data.state);
    const violations = data.state.fk?.urdf_limit_violations || {};
    for (const [name, range] of Object.entries(violations)) {
      output.append(node("p", "kinematics-reason", `${jointLabels[name] || name}: ${range.value_deg.toFixed(2)}° / URDF 범위 ${range.lower_deg.toFixed(2)} ~ ${range.upper_deg.toFixed(2)}°. 범위를 벗어난 시작 자세는 IK를 계산하지 않습니다.`));
    }
    drawPreview();
  }

  function renderPoseReference(reference) {
    const panel = $("xyz-reference");
    panel.replaceChildren(); panel.hidden = !reference;
    if (!reference) return;
    panel.append(node("strong", "", "측정한 현재 자세 ↔ 기본 자세"));
    panel.append(node("p", "kinematics-note", `관찰 ${shortId(reference.provenance?.observation_id)} · ${historyDate(reference.provenance?.timestamp)} · 팔 장착부 기준, 단위 m`));
    const table = node("table", "pose-reference-table");
    const header = node("tr");
    for (const title of ["자세", "X", "Y", "Z"]) header.append(node("th", "", title));
    const head = node("thead"), body = node("tbody"); head.append(header);
    for (const [key, label] of [["current", "측정 자세"], ["default", "GUI 기본"], ["zero", "모두 0°"]]) {
      const pose = reference.poses?.[key], row = node("tr");
      row.append(node("th", "", label));
      for (let i = 0; i < 3; i++) row.append(node("td", "", Number.isFinite(pose?.xyz_m?.[i]) ? pose.xyz_m[i].toFixed(5) : "—"));
      body.append(row);
    }
    table.append(head, body); panel.append(table);
    eventLine(panel, "현재 − 기본", vectorLabel(reference.delta?.current_minus_default_xyz_m));
    const joints = reference.provenance?.joints || {};
    eventLine(panel, "측정 관절 (°)", armJoints.map(name => `${jointLabels[name]} ${Number.isFinite(joints[name]) ? joints[name].toFixed(3) : "—"}`).join(" · "));
    for (const [key, label] of [["current_round_trip", "현재 자세 재계산"], ["current_to_default", "현재 관절 → 기본 XYZ"], ["default_to_current", "기본 관절 → 현재 XYZ"]]) {
      const check = reference.checks?.[key];
      eventLine(panel, label, check?.success ? `IK 성공 · 오차 ${(check.residual_m * 1000).toFixed(4)} mm` : "IK 확인 불가 · 이벤트 데이터에서 원인 확인");
    }
    panel.append(node("p", "kinematics-note", "위 비교는 관절 가동범위 안에서의 기하학 계산입니다. 이동 시간·속도를 적용한 실제 명령이 아닙니다. GUI 기본은 (0°, −15°, 35°, −20°, 0°) 예시이며 로봇 홈 자세가 아닙니다."));
    panel.append(node("p", "kinematics-reason", "이 값은 URDF가 계산한 팔 끝 좌표입니다. 큐브 중심의 실측 좌표가 아니며, 한 자세의 비교만으로 실물 영점·회전 방향을 보정할 수 없습니다."));
    if (!reference.reference_valid) kinematicReasons(panel, reference);
    const useDefault = node("button", "button xyz-secondary", "기본 자세 XYZ를 목표에 넣기 · 계산만");
    useDefault.type = "button";
    useDefault.disabled = !reference.poses?.default?.xyz_m;
    useDefault.addEventListener("click", () => {
      $("xyz-mode").value = "absolute";
      ["x", "y", "z"].forEach((axis, index) => { $(`xyz-${axis}`).value = reference.poses.default.xyz_m[index]; });
      requestPreview(true);
    });
    panel.append(useDefault);
  }

  async function capturePose() {
    if ($("xyz-capture").disabled) return;
    preview.capturing = true; preview.busy = true; previewControls(); updateControls();
    try {
      const saved = await api("/api/kinematics/capture", {method: "POST", body: "{}", signal: AbortSignal.timeout(30000)});
      state.server = await api("/api/state"); updatePreviewSources();
      $("xyz-source").value = saved.run_id;
      preview.data = await api("/api/kinematics/preview", {method: "POST", body: JSON.stringify({source: "recorded", run_id: saved.run_id})});
      renderPreview(); notice("");
    } catch (error) { notice(error.message); }
    finally { preview.capturing = false; preview.busy = false; previewControls(); updateControls(); }
  }

  async function requestPreview(solve) {
    if (preview.busy || state.server?.capabilities?.xyz_preview !== 1 || !state.connected) return;
    if (!$("xyz-form").reportValidity()) return;
    const selected = $("xyz-source").value, model = selected === "model";
    const payload = {source: model ? "model" : "recorded", duration_ms: Number($("xyz-duration").value)};
    if (model) payload.joints_deg = Object.fromEntries([...$("xyz-joint-inputs").querySelectorAll("input")].map(input => [input.dataset.joint, Number(input.value)]));
    else {
      payload.run_id = selected;
      if (solve && preview.data?.source.run_id === selected) payload.observation_id = preview.data.source.observation_id;
    }
    if (solve) payload.target = {frame: "arm_base", mode: $("xyz-mode").value, xyz_m: ["x", "y", "z"].map(axis => Number($(`xyz-${axis}`).value)), approach_axis: null};
    preview.busy = true; previewControls();
    try {
      preview.data = await api("/api/kinematics/preview", {method: "POST", body: JSON.stringify(payload)});
      renderPreview();
    } catch (error) {
      preview.data = null; renderPreview();
      text("xyz-provenance", "미리보기 계산을 완료하지 못했습니다.");
      $("xyz-output").replaceChildren(node("p", "kinematics-reason", error.message));
    } finally { preview.busy = false; previewControls(); }
  }
  function renderKinematics(run) {
    let current = $("kinematics-current");
    if (!current) {
      current = node("section", "kinematics-panel");
      current.id = "kinematics-current";
      current.setAttribute("aria-label", "현재 관찰의 팔 끝 위치 계산");
      $("calibration-identity").after(current);
    }
    const kinematics = run?.kinematics || latestEvent(run, "kinematics_state")?.data;
    const currentId = run?.current_observation?.observation_id;
    const matching = Boolean(currentId && kinematics?.observation_id === currentId);
    current.hidden = !kinematics;
    current.replaceChildren();
    if (kinematics) {
      const head = node("div", "kinematics-heading");
      const ready = matching && kinematics.executable === true;
      head.append(node("strong", "", "현재 팔 끝 위치 · FK"), node("span", `badge ${ready ? "good" : "neutral"}`,
        !matching ? "계산 대기" : ready ? "매핑 확인됨" : "미리보기"));
      current.append(head);
      if (matching) {
        eventLine(current, "현재 XYZ", vectorLabel(kinematics.current_xyz_m) || "계산값 없음");
        eventLine(current, "좌표 기준", `${kinematics.frame || "arm_base"} → ${kinematics.tip_frame || "gripper_frame_link"}`);
        current.append(node("p", "kinematics-note", "관절 측정값으로 계산한 팔 끝 위치입니다. 카메라·세계 좌표에서 측정한 위치가 아닙니다."));
        kinematicReasons(current, kinematics);
      } else current.append(node("p", "kinematics-note", "현재 관찰에 대응하는 위치 계산을 기다리고 있습니다."));
    }
    let result = $("kinematics-result");
    if (!result) {
      result = node("section", "kinematics-panel");
      result.id = "kinematics-result";
      result.setAttribute("aria-label", "최근 XYZ 요청의 역기구학 계산 결과");
      $("command-details").before(result);
    }
    const event = latestEvent(run, "ik_result");
    result.hidden = !event;
    result.replaceChildren();
    if (!event) return;
    const data = event.data || {};
    const [summary, tone] = ikStatus(data);
    const head = node("div", "kinematics-heading");
    head.append(node("strong", "", "최근 XYZ 변환 · IK"), node("span", `badge ${tone}`, summary));
    result.append(head, node("p", "kinematics-note", `${timeLabel(event.at)} · 관찰 ${shortId(data.observation_id)} 기준 · ${data.frame || data.request?.frame || "arm_base"}`));
    ikLines(result, data);
    result.append(node("p", "kinematics-note", "끝점 관절 해를 계산합니다. 직선 이동·충돌 회피·전체 6D 자세는 검증하지 않습니다."));
  }

  function safeRejection(packet) {
    return packet?.status === "rejected" && packet.execution?.outcome === "rejected" && packet.execution?.command_sent === false;
  }
  function verifiedIncomplete(packet) {
    return packet?.status === "incomplete" && packet.execution?.outcome === "incomplete"
      && packet.execution.command_sent === true && packet.execution.arm_reached === false
      && packet.execution.arm_hold_verified === true && packet.execution.arm_motion?.hold?.verified === true
      && packet.capabilities?.arm_reconciliation === 1;
  }
  function armMotionLines(parent, motion) {
    for (const [name, target] of Object.entries(motion?.targets || {})) {
      const measured = motion?.measured?.[name];
      if (!Number.isFinite(target) || !Number.isFinite(measured)) continue;
      const unit = name === "gripper" ? "%" : "°";
      eventLine(parent, jointLabels[name] || name, `목표 ${target.toFixed(2)}${unit} · 실제 ${measured.toFixed(2)}${unit} · 차이 ${(measured - target).toFixed(2)}${unit}`);
    }
  }
  function renderMotionFailure(run) {
    let card = $("motion-failure-card");
    if (!card) {
      card = node("section", "rejection-card"); card.id = "motion-failure-card";
      card.setAttribute("aria-label", "목표 미도달과 새 경로 재계획");
      $("command-details").before(card);
    }
    const event = [latestEvent(run, "motion_failure_feedback"), latestEvent(run, "motion_incomplete")]
      .filter(Boolean).sort((left, right) => right.seq - left.seq)[0];
    const resolved = latestEvent(run, "motion_recovery_completed");
    card.hidden = !event || Boolean(resolved && resolved.seq > event.seq);
    card.replaceChildren();
    if (card.hidden) return;
    card.append(node("strong", "", "목표 미도달 · 자세 유지 확인"));
    armMotionLines(card, event.data?.arm_motion);
    const constraints = event.data?.recovery_constraints;
    card.append(node("p", "", constraints
      ? `상태 재확인 후 새 경로를 판단합니다. 현재 관절에서 최대 ${constraints.max_joint_delta_deg}°씩, ${constraints.min_duration_ms / 1000}초 이상으로 수정합니다.`
      : "일부 이동했지만 목표에 도달하지 못했습니다. 새 관찰과 재계획이 필요합니다."));
    card.append(node("small", "", "이전 명령을 그대로 다시 보내지 않으며, 부분 이동을 작업 완료로 처리하지 않습니다."));
  }
  function rejectionReason(data) {
    const code = data.error_code || data.error;
    return rejectionLabels[code] || data.error || "실행 전 검사에서 명령을 거부했습니다.";
  }
  function rejectionGuidance(data) {
    const code = data.error_code || data.error;
    if (code === "arm_current_out_of_calibration") return "해당 관절의 목표만 바꿔서는 해결되지 않습니다. 새 관찰을 확인하고 제한된 관절을 제외한 계획을 검토합니다.";
    if (code === "arm_target_out_of_range") return "새 관찰의 허용 범위 안으로 목표값을 수정합니다.";
    if (code === "arm_speed_limit") return "이동량을 줄이거나 허용 시간 안에서 동작 시간을 늘려 다시 판단합니다.";
    return data.source === "host" ? "이미 거부된 명령은 다시 전송하지 않았습니다. 새 관찰로 가능한 다른 행동을 판단합니다."
      : "명령을 다시 보내기 전에 새 관찰로 원인을 확인하고 계획을 수정합니다.";
  }
  function renderRejection(run) {
    let card = $("rejection-card");
    if (!card) {
      card = node("section", "rejection-card");
      card.id = "rejection-card";
      card.setAttribute("aria-label", "전송 전 명령 거부와 수정 지침");
      $("command-details").before(card);
    }
    const resultEvent = latestEvent(run, "action_result");
    const feedbackEvent = latestEvent(run, "rejection_feedback");
    const feedbackSafe = feedbackEvent?.data?.command_sent === false;
    const latestSafe = safeRejection(resultEvent?.data?.packet);
    const event = feedbackSafe && (!resultEvent || feedbackEvent.seq >= resultEvent.seq) ? feedbackEvent : latestSafe ? resultEvent : null;
    card.hidden = !event;
    card.replaceChildren();
    if (!event) return;
    const data = event.event === "action_result" ? event.data.packet : event.data;
    const head = node("div", "rejection-heading");
    head.append(node("strong", "", "명령 거부 · 재관찰 후 수정"), node("span", "badge warning", "미전송 확인"));
    card.append(head, node("p", "rejection-reason", rejectionReason(data)));
    if (data.error_code === "arm_current_out_of_calibration" && data.error_details?.joint) card.append(node("p", "rejection-detail", calibrationLine(data.error_details)));
    card.append(node("p", "", rejectionGuidance(data)));
    const context = run?.active ? "이 명령은 전송되지 않았으며, 에이전트가 다음 판단을 진행합니다." : "이전에 거부된 명령의 기록입니다. 현재 작업 상태는 위에 표시됩니다.";
    card.append(node("small", "", context));
  }

  function renderDecision(decision, feedback) {
    text("decision-reason", decision?.reason || "관찰을 읽고 다음 행동을 정하면, 그 근거가 여기에 표시됩니다.");
    $("decision-reason").classList.toggle("empty-text", !decision);
    text("decision-expected", decision?.expected || "—");
    badge("decision-tool", toolLabels[decision?.tool] || "판단 대기", decision?.tool === "execute" ? "good" : "neutral");
    $("decision-plan").replaceChildren();
    if (decision?.plan?.length) decision.plan.forEach((step) => $("decision-plan").append(node("li", "", step)));
    else $("decision-plan").append(node("li", "empty-text", "아직 작성된 계획이 없습니다."));
    badge("decision-feedback", feedbackLabels[decision?.feedback] || "대기", decision?.feedback === "progress" ? "good" : decision?.feedback === "stalled" ? "warning" : "neutral");
    text("decision-evidence", decision?.evidence || "실행 후 관찰과 측정값을 바탕으로 확인합니다.");
    const commandData = {};
    if (decision?.command) commandData.command = decision.command;
    if (feedback) commandData.feedback = feedback;
    $("command-details").hidden = !Object.keys(commandData).length;
    text("command-json", JSON.stringify(commandData, null, 2));
  }

  function eventLine(parent, title, value) {
    if (value == null || value === "") return;
    const row = node("div", "event-line");
    row.append(node("span", "", title), node("p", "", value));
    parent.append(row);
  }
  function commandLabel(command) {
    if (!command) return "";
    if (command.action === "wait") return `대기 ${command.duration_ms}ms`;
    const parts = [];
    if (command.cartesian) {
      const xyz = vectorLabel(command.cartesian.xyz_m);
      if (xyz) parts.push(`팔 ${command.cartesian.mode === "relative" ? "상대 이동" : "목표"} ${xyz} · ${command.cartesian.frame || "arm_base"}`);
    }
    if (command.joints) {
      for (const [name, value] of Object.entries(command.joints)) {
        if (typeof value === "number") parts.push(`${jointLabels[name] || name} ${value}${name === "gripper" ? "%" : "°"}`);
      }
    }
    if (command.x || command.y || command.theta) parts.push(`베이스 x ${command.x || 0}, y ${command.y || 0}m/s · 회전 ${command.theta || 0}°/s`);
    if (command.duration_ms !== undefined) parts.push(`${command.duration_ms}ms`);
    return parts.join(" · ") || "동작 명령";
  }
  function renderEvent(event) {
    const kind = event.event;
    const data = event.data || {};
    const finishedStatus = kind === "action_finished" && !("transport_status" in data) && data.status === "completed" && data.result_status ? data.result_status : data.status;
    const rejected = kind === "rejection_feedback" || kind === "action_result" && safeRejection(data.packet);
    const failedFinish = kind.endsWith("_finished") && ["error", "uncertain_execution", "completed_without_observation"].includes(finishedStatus);
    const category = rejected ? "rejection" : failedFinish ? "error" : kind === "decision" ? "decision" : kind.startsWith("action") ? "action" : kind === "error" || kind === "decision_rejected" ? "error" : "";
    const item = node("article", `timeline-event ${category}`);
    const head = node("div", "event-head");
    const finishedPhase = {observe_finished: "관찰", planning_finished: "판단", action_finished: "행동 실행", startup_finished: "작업 준비"}[kind];
    const interruptedLabel = {cancelled: "취소", error: "실패", uncertain_execution: "결과 미확정", incomplete: "목표 미도달", rejected: "명령 거부", completed_without_observation: "완료 · 재관찰 실패"}[finishedStatus];
    head.append(node("strong", "", finishedPhase && interruptedLabel ? `${finishedPhase} ${interruptedLabel}` : eventLabels[kind] || kind));
    if (kind === "decision") head.append(node("span", "event-tag", toolLabels[data.tool] || data.tool));
    const time = node("time", "", timeLabel(event.at));
    if (date(event.at)) time.dateTime = date(event.at).toISOString();
    head.append(time); item.append(head);
    if (kind === "decision") {
      item.append(node("p", "event-body", data.reason));
      if (Array.isArray(data.plan) && data.plan.length) {
        const plan = node("ol", "event-plan");
        data.plan.forEach((step) => plan.append(node("li", "", step)));
        item.append(plan);
      }
      eventLine(item, "예상 변화", data.expected);
      eventLine(item, "피드백", feedbackLabels[data.feedback] || data.feedback);
      eventLine(item, "관찰 근거", data.evidence);
      eventLine(item, "실행 명령", commandLabel(data.command));
    } else if (kind === "action_prepared") {
      item.append(node("p", "event-body", commandLabel(data.command || data)));
      eventLine(item, "판단 근거", data.reason);
    } else if (kind === "kinematics_state") {
      eventLine(item, "현재 XYZ", vectorLabel(data.current_xyz_m) || "계산값 없음");
      eventLine(item, "XYZ 준비", data.executable === true ? "매핑 확인됨 · 목표별 IK 검사 필요" : "미리보기 · 실행 조건 미충족");
      kinematicReasons(item, data);
    } else if (kind === "ik_result") {
      item.append(node("p", "event-body", ikStatus(data)[0]));
      ikLines(item, data);
    } else if (kind === "action_result") {
      const packet = data.packet || data;
      const execution = packet.execution || {};
      const recordedRecoveries = execution.arm_motion?.communication_recoveries;
      const recoveries = packet.status === "completed" && execution.outcome === "completed" && Array.isArray(recordedRecoveries)
        ? recordedRecoveries.filter((recovery) => recovery?.recovery === "verified_readback" && recovery.requested === recovery.read_value
          && [recovery.motor_id, recovery.register, recovery.requested, recovery.read_value].every(Number.isFinite)) : [];
      const message = safeRejection(packet) ? "명령 거부 · 재관찰 후 수정. 이 명령은 로봇에 전송되지 않았습니다."
        : verifiedIncomplete(packet) ? "목표 미도달 · 측정한 자세 유지를 확인했습니다. 상태 재확인 후 새 요청이 필요합니다."
        : execution.outcome === "uncertain_execution" ? "실행 여부 미확정 · 자동 재실행을 중단했습니다."
          : packet.status === "completed" ? recoveries.length ? "통신 응답 오류 · 설정값 재확인 후 완료" : "동작 응답 수신 · 새 관찰로 결과를 확인합니다."
            : packet.error || packet.observation_error || label(packet.status);
      item.append(node("p", "event-body", message));
      if (safeRejection(packet)) eventLine(item, "거부 원인", rejectionReason(packet));
      else if (execution.outcome === "uncertain_execution") eventLine(item, "오류", packet.error);
      if (execution.arm_motion?.reached === false) armMotionLines(item, execution.arm_motion);
      for (const recovery of recoveries) {
        const attempts = Number.isInteger(recovery.read_attempts) ? ` · 읽기 ${recovery.read_attempts}회` : "";
        eventLine(item, "설정 확인", `모터 ${recovery.motor_id} · 레지스터 ${recovery.register} · 요청 ${recovery.requested} / 확인 ${recovery.read_value}${attempts}`);
      }
      if (typeof execution.arm_reached === "boolean") eventLine(item, "팔 목표", execution.arm_reached ? "엔코더 허용 오차 내 도달" : "도달 미확인");
      if (execution.base_stop_verified === false) eventLine(item, "베이스", "정지 명령 처리 · 실제 이동량·정지 상태는 미측정");
    } else if (kind === "rejection_feedback") {
      item.append(node("p", "event-body", rejectionReason(data)));
      eventLine(item, "전송 상태", data.command_sent === false ? "미전송 · 새 관찰 후 수정 판단" : "확인 필요");
      if (data.error_code === "arm_current_out_of_calibration" && data.error_details?.joint) eventLine(item, "관절 상태", calibrationLine(data.error_details));
      eventLine(item, "수정 지침", rejectionGuidance(data));
      eventLine(item, "거부 명령", commandLabel(data.command));
    } else if (kind === "motion_incomplete" || kind === "motion_failure_feedback") {
      item.append(node("p", "event-body", "목표에 도달하지 못했으며, 현재 관절값과 자세 유지 결과를 다음 판단에 전달합니다."));
      armMotionLines(item, data.arm_motion);
      eventLine(item, "실패한 요청", commandLabel(data.failed_command));
      if (data.recovery_constraints) eventLine(item, "수정 범위", `현재 관절에서 ${data.recovery_constraints.max_joint_delta_deg}° 이내 · ${data.recovery_constraints.min_duration_ms / 1000}초 이상 · 그리퍼와 베이스 제외`);
    } else if (kind === "motion_replanning") {
      item.append(node("p", "event-body", "새 관찰을 기준으로 작은 팔 이동을 다시 계획합니다. 새 판단마다 새 행동 ID를 사용합니다."));
    } else if (kind === "motion_recovery_completed") {
      item.append(node("p", "event-body", "수정한 팔 목표에 도달했고 새 관찰을 받았습니다. 물체 파지와 전체 작업 완료는 별도로 확인합니다."));
    } else if (kind === "motion_feedback") {
      const deltas = Object.entries(data.joint_delta || {}).filter(([, value]) => typeof value === "number");
      item.append(node("p", "event-body", deltas.length ? deltas.map(([name, value]) => `${jointLabels[name] || name} ${value >= 0 ? "+" : ""}${value.toFixed(1)}${name === "gripper" ? "%p" : "°"}`).join(" · ") : "동작 전후 관찰이 다음 판단에 전달됩니다."));
      if (data.no_joint_progress) eventLine(item, "피드백", "측정된 관절 변화 없음");
    } else if (kind === "observation") {
      item.append(node("p", "event-body", `관찰 ${shortId(data.observation_id)} 저장`));
    } else if (kind === "episode_finished") {
      item.append(node("p", "event-body", `${label(data.status)}${data.detail ? ` · ${data.detail}` : ""}`));
    } else if (kind === "completion_candidate") {
      item.append(node("p", "event-body", data.evidence || data.reason || "새 관찰을 받아 완료 주장을 다시 확인합니다."));
    } else if (data.reason || data.message || data.error || data.result_error || data.detail) {
      item.append(node("p", "event-body", data.reason || data.message || data.error || data.result_error || data.detail));
    }
    if (Object.keys(data).length && !["observe_started", "planning_started", "startup"].includes(kind)) {
      const details = node("details", "event-details");
      details.append(node("summary", "", "이벤트 데이터"), node("pre", "", JSON.stringify(data, null, 2)));
      item.append(details);
    }
    return item;
  }
  function renderTimeline(run) {
    const events = run?.events || [];
    const key = `${run?.id || "idle"}:${events.length}:${events.at(-1)?.seq ?? ""}`;
    if (state.timelineKey === key) return;
    state.timelineKey = key;
    text("event-count", `${events.length} EVENTS`);
    const timeline = $("timeline");
    const atBottom = timeline.scrollHeight - timeline.scrollTop - timeline.clientHeight < 70;
    const opened = [...timeline.querySelectorAll("details[open]")].map((element) => element.parentElement.dataset.seq);
    timeline.replaceChildren();
    if (!events.length) {
      const empty = node("div", "timeline-empty");
      const copy = node("p", "", "첫 작업을 시작해보세요.");
      copy.append(node("small", "", "이곳에서 전체 실행 흐름을 확인할 수 있습니다."));
      empty.append(node("span", "", "↳"), copy); timeline.append(empty); return;
    }
    const fragment = document.createDocumentFragment();
    for (const [index, event] of events.entries()) {
      const item = renderEvent(event);
      item.dataset.seq = String(event.seq ?? index);
      if (opened.includes(item.dataset.seq) && item.querySelector("details")) item.querySelector("details").open = true;
      fragment.append(item);
    }
    timeline.append(fragment);
    if (atBottom && run.active) timeline.scrollTop = timeline.scrollHeight;
  }

  function renderHistory(runs) {
    const key = JSON.stringify([runs, state.selectedId]);
    if (state.historyKey === key) return;
    state.historyKey = key;
    $("history-list").replaceChildren();
    if (!runs?.length) { $("history-list").append(node("p", "empty-small", "아직 저장된 실행 기록이 없습니다.")); return; }
    for (const run of runs) {
      const button = node("button", `history-item${run.id === state.selectedId ? " selected" : ""}`);
      button.type = "button";
      button.setAttribute("aria-label", `${run.instruction || "제목 없는 작업"}, ${label(run.status)} 기록 보기`);
      button.append(node("strong", "", run.instruction || "제목 없는 작업"));
      const line = node("span");
      line.append(node("span", "", historyDate(run.started_at)), node("em", run.mode === "demo" ? "demo-text" : "", `${run.mode === "demo" ? "데모 · " : ""}${label(run.status)}`));
      button.append(line);
      button.addEventListener("click", () => selectRun(run.id));
      $("history-list").append(button);
    }
  }

  async function selectRun(id) {
    try {
      notice("");
      const run = await api(`/api/runs/${encodeURIComponent(id)}`);
      if (id === state.server?.active_run_id) {
        state.currentRun = run; state.selectedId = null; state.selectedRun = null;
      } else { state.selectedId = id; state.selectedRun = run; }
      state.timelineKey = null;
      render();
    } catch (error) { notice(error.message); }
  }
  function render() {
    $("history-notice").hidden = !state.selectedId;
    renderRun(state.selectedId ? state.selectedRun : state.currentRun);
    renderHistory(state.server?.runs || []);
    updateControls();
    updatePreviewSources();
    text("xyz-agent-state", state.server?.active_run_id ? "에이전트 실행 중 · 관찰 → 판단 → 행동 → 결과 확인" : "에이전트 대기 · 현재 실행 중인 추론 작업이 없습니다.");
    syncManual();
  }

  async function poll() {
    try {
      const snapshot = await api("/api/state");
      const wasDisconnected = !state.connected;
      state.server = snapshot;
      initialize(snapshot.config || {});
      setConnection(true);
      if (wasDisconnected && $("notice").dataset.connectionError === "true") { notice(""); delete $("notice").dataset.connectionError; }
      if (state.selectedId && state.selectedRun?.active) state.selectedRun = await api(`/api/runs/${encodeURIComponent(state.selectedId)}`);
      if (snapshot.active) state.currentRun = snapshot.active;
      else if (state.currentRun?.active) state.currentRun = await api(`/api/runs/${encodeURIComponent(state.currentRun.id)}`);
      if (!snapshot.active_run_id) state.stoppingId = null;
      render();
    } catch (error) {
      setConnection(false);
      notice(`서버 상태를 가져오지 못했습니다. 마지막 표시는 현재 상태를 보장하지 않습니다. ${error.message}`);
      $("notice").dataset.connectionError = "true";
    } finally { window.setTimeout(poll, 800); }
  }

  $("task-form").addEventListener("submit", async (event) => {
    event.preventDefault();
    if (state.busy || state.server?.active_run_id || !state.connected || !$("task-form").reportValidity()) return;
    const instruction = $("instruction").value.trim();
    const robotId = $("robot-id").value.trim();
    if (!instruction || !robotId) { notice("작업 지시와 로봇 ID를 입력해주세요."); return; }
    state.busy = true; updateControls(); notice("");
    try {
      const run = await api("/api/runs", {method: "POST", body: JSON.stringify({
        instruction, robot_id: robotId, model: $("model").value.trim() || null,
        limits: {max_actions: Number($("max-actions").value), max_steps: Number($("max-steps").value), max_seconds: Number($("max-seconds").value)},
        execute: true,
      })});
      state.currentRun = run; state.selectedId = null; state.selectedRun = null; state.timelineKey = null;
      state.server.active_run_id = run.id; state.server.active = run;
      announce("새 에이전트 작업을 시작했습니다."); render();
    } catch (error) { notice(error.message); }
    finally { state.busy = false; updateControls(); }
  });

  for (const [name, value] of Object.entries({shoulder_pan: 0, shoulder_lift: -15, elbow_flex: 35, wrist_flex: -20, wrist_roll: 0})) {
    const field = node("div", "field"), title = node("label", "", jointLabels[name]), input = node("input");
    input.id = `xyz-joint-${name}`; input.dataset.joint = name; input.type = "number"; input.step = "any"; input.value = value; input.required = true;
    title.htmlFor = input.id; field.append(title, input); $("xyz-joint-inputs").append(field);
  }
  $("xyz-form").addEventListener("submit", event => { event.preventDefault(); requestPreview(true); });
  $("xyz-load").addEventListener("click", () => requestPreview(false));
  $("xyz-source").addEventListener("change", () => { preview.data = null; renderPreview(); requestPreview(false); });
  $("xyz-capture").addEventListener("click", capturePose);
  $("xyz-view").addEventListener("change", drawPreview);
  $("xyz-display").addEventListener("change", drawPreview);
  $("xyz-form").addEventListener("input", event => {
    if (event.target.id === "xyz-source") return;
    if (preview.data) {
      if (event.target.dataset.joint) preview.data = null;
      else { preview.data.result = null; preview.data.solution_points = []; }
      renderPreview();
      text("xyz-provenance", "입력값이 바뀌었습니다. 미리보기 계산을 눌러 결과를 갱신하세요.");
    }
  });

  function manualControls() {
    const remote = state.server?.manual_xyz, busy = manual.busy || remote?.busy || preview.capturing || state.server?.pose_capture_busy;
    const enabled = state.connected && state.server?.capabilities?.manual_xyz === 1;
    const agentActive = Boolean(state.server?.active_run_id);
    text("manual-availability", !state.connected ? "GUI 연결을 기다리고 있습니다." : !enabled
      ? state.server?.mode === "demo" ? "데모 화면에서는 실제 팔을 움직일 수 없습니다." : "GUI 서버를 다시 실행하면 수동 XYZ 이동을 사용할 수 있습니다."
      : agentActive ? "에이전트 작업이 끝난 뒤 수동 실험을 시작하세요." : "이동량은 팔 장착부의 X·Y·Z 축 기준입니다. 먼저 Z +5mm로 계산해 보세요.");
    for (const input of $("manual-form").querySelectorAll("input, select, button")) input.disabled = !enabled || busy || agentActive;
    for (const input of $("manual-mapping-inputs").querySelectorAll("input, select")) input.disabled ||= !$("manual-custom-mapping").checked;
    text("manual-prepare", busy ? remote?.phase === "executing" ? "이동 결과를 기다리는 중…" : "현재 상태 확인 중…" : "① 로봇 상태 읽고 계산 · 아직 이동 안 함");
    const view = manual.view, valid = view?.ready && view.token && view.expires_at * 1000 > Date.now();
    $("manual-execute").disabled = !enabled || busy || agentActive || !valid || manual.dirty || !manual.visualReady
      || !$("manual-clearance").checked || (view?.requires_alignment_confirmation && !$("manual-alignment").checked);
    $("manual-cancel").disabled = busy || !state.connected || !remote?.prepared;
    $("manual-alignment").disabled = busy || !valid;
    $("manual-clearance").disabled = busy || !valid;
    text("manual-expiry", manual.dirty ? "입력값이 바뀌었습니다. ① 버튼으로 다시 계산하세요." : valid
      ? `계산 유효시간 ${Math.max(0, Math.ceil(view.expires_at - Date.now() / 1000))}초 · 실행하면 이 계산은 다시 사용할 수 없습니다.`
      : view ? "이 계산은 전송할 수 없습니다. ① 버튼으로 상태를 다시 읽으세요." : "");
    updateControls();
  }

  function showManual(view) {
    manual.view = view; manual.key = view?.token || view?.run_id; manual.dirty = false;
    $("manual-alignment").checked = false; $("manual-clearance").checked = false;
    $("manual-review").hidden = !view;
    if (!view) { manualControls(); return; }
    $("manual-alignment-label").hidden = !view.requires_alignment_confirmation;
    $("manual-cameras").replaceChildren();
    for (const camera of view.images || []) {
      const figure = node("figure"), img = node("img"), caption = node("figcaption", "", camera.label);
      img.src = camera.url; img.alt = `${camera.label} · 계산 직전 관찰`; figure.append(img, caption); $("manual-cameras").append(figure);
    }
    const output = $("manual-targets"); output.replaceChildren();
    eventLine(output, "대상 로봇", view.robot_id);
    eventLine(output, "현재 XYZ", vectorLabel(view.state?.current_xyz_m));
    eventLine(output, "목표 XYZ", vectorLabel(view.result?.target_xyz_m));
    eventLine(output, "동작", "1초 · 베이스와 집게 벌림 유지");
    const table = node("table", "manual-joints"), head = node("tr");
    for (const title of ["관절", "현재", "목표", "변화량"]) head.append(node("th", "", title));
    table.append(head);
    for (const [name, delta] of Object.entries(view.joint_delta_deg || {})) {
      const row = node("tr");
      for (const value of [jointLabels[name], `${view.state.current_runtime_joints_deg[name].toFixed(2)}°`, `${view.result.solution_runtime_joints_deg[name].toFixed(2)}°`, `${delta >= 0 ? "+" : ""}${delta.toFixed(2)}°`]) row.append(node("td", "", value));
      table.append(row);
    }
    output.append(table);
    if (!view.ready) kinematicReasons(output, view);
    else if (view.requires_alignment_confirmation) output.append(node("p", "kinematics-reason", "관절 방향·영점이 아직 검증되지 않았습니다. 3D 현재 자세를 실물과 비교하고, 맞는 경우에만 아래 확인란을 선택하세요."));
    drawRobot("manual", view); manualControls();
  }

  function syncManual() {
    const remote = state.server?.manual_xyz;
    if (remote?.prepared && remote.prepared.token !== manual.key && !manual.busy) showManual(remote.prepared);
    if (!remote?.prepared && !manual.busy && manual.view?.token && remote?.last?.status !== "prepared") manual.view.token = null;
    const key = JSON.stringify(remote?.last);
    if (remote?.last && key !== manual.lastKey && !manual.busy) {
      manual.lastKey = key; manual.lastRun = remote.last.run_id;
      text("manual-status", `${label(remote.last.status)} · ${remote.last.detail || ""}`);
      $("manual-history").hidden = !manual.lastRun;
    }
    manualControls();
  }

  $("manual-form").addEventListener("submit", async event => {
    event.preventDefault();
    if (manual.busy || !$("manual-form").reportValidity() || $("manual-prepare").disabled) return;
    const payload = {robot_id: $("robot-id").value.trim(), delta_m: ["x", "y", "z"].map(axis => Number($(`manual-${axis}`).value) / 1000), duration_ms: 1000};
    if ($("manual-custom-mapping").checked) payload.joint_mapping = Object.fromEntries([...$("manual-mapping-inputs").querySelectorAll("[data-joint]")].map(row => [row.dataset.joint, {sign: Number(row.querySelector("select").value), offset_deg: Number(row.querySelector("input").value)}]));
    manual.busy = true; manualControls(); text("manual-status", "현재 로봇 상태를 읽고 목표를 계산합니다. 아직 이동하지 않습니다.");
    try {
      const view = await api("/api/manual-xyz/prepare", {method: "POST", body: JSON.stringify(payload), signal: AbortSignal.timeout(30000)});
      showManual(view); manual.lastRun = view.run_id;
      if (state.server.manual_xyz) state.server.manual_xyz.prepared = view.ready ? view : null;
      text("manual-status", view.ready ? "목표 계산 완료 · 아직 명령을 보내지 않았습니다. 3D 모델과 실물을 확인하세요." : "이 목표는 전송할 수 없습니다. 위 제한 사유를 확인하세요.");
      $("manual-history").hidden = false;
    } catch (error) { text("manual-status", error.message); }
    finally { manual.busy = false; manualControls(); }
  });
  $("manual-execute").addEventListener("click", async () => {
    manualControls();
    if ($("manual-execute").disabled) return;
    const token = manual.view.token;
    manual.view.token = null; manual.busy = true; manualControls();
    text("manual-status", "한 번 이동을 요청했습니다. 결과를 기다립니다.");
    try {
      const result = await api("/api/manual-xyz/execute", {method: "POST", body: JSON.stringify({token, execute: true, alignment_confirmed: $("manual-alignment").checked, clearance_confirmed: true})});
      manual.lastRun = result.run_id;
      if (state.server.manual_xyz) { state.server.manual_xyz.prepared = null; state.server.manual_xyz.busy = true; state.server.manual_xyz.phase = "executing"; }
      text("manual-status", result.detail);
    } catch (error) { text("manual-status", `요청 결과 확인 필요 · ${error.message} 자동으로 다시 보내지 않습니다.`); }
    finally { manual.busy = false; manualControls(); }
  });
  $("manual-cancel").addEventListener("click", async () => {
    if (manual.busy) return;
    manual.busy = true; manualControls();
    try {
      state.server.manual_xyz = await api("/api/manual-xyz/cancel", {method: "POST", body: "{}"});
      showManual(null); text("manual-status", "준비된 계산을 취소했습니다. 명령을 보내지 않았습니다.");
    } catch (error) { text("manual-status", error.message); }
    finally { manual.busy = false; manualControls(); }
  });
  $("manual-form").addEventListener("input", () => { manual.dirty = true; manualControls(); });
  $("robot-id").addEventListener("input", () => { manual.dirty = true; manualControls(); });
  for (const id of ["manual-alignment", "manual-clearance"]) $(id).addEventListener("change", manualControls);
  for (const id of ["manual-view", "manual-display"]) $(id).addEventListener("change", () => drawRobot("manual", manual.view));
  $("manual-history").addEventListener("click", () => { if (manual.lastRun) selectRun(manual.lastRun); });
  for (const mode of ["preview", "manual"]) $(`xyz-${mode}-tab`).addEventListener("click", () => {
    for (const name of ["preview", "manual"]) {
      $(`xyz-${name}-content`).hidden = name !== mode;
      $(`xyz-${name}-tab`).classList.toggle("selected", name === mode);
      $(`xyz-${name}-tab`).setAttribute("aria-pressed", String(name === mode));
    }
    badge("xyz-mode-badge", mode === "manual" ? "실제 팔 · 수동 1회" : "계산 전용", mode === "manual" ? "warning" : "neutral");
  });
  for (const name of Object.keys(jointLabels).filter(name => name !== "gripper")) {
    const row = node("div", "manual-mapping-row"); row.dataset.joint = name;
    const title = node("label", "", jointLabels[name]), sign = node("select"), offset = node("input");
    sign.add(new Option("같은 방향 (+1)", "1")); sign.add(new Option("반대 방향 (−1)", "-1"));
    sign.setAttribute("aria-label", `${jointLabels[name]} 방향`);
    offset.type = "number"; offset.step = "any"; offset.min = -180; offset.max = 180; offset.value = 0; offset.required = true;
    offset.setAttribute("aria-label", `${jointLabels[name]} 영점 (도)`);
    row.append(title, sign, offset); $("manual-mapping-inputs").append(row);
  }

  $("stop").addEventListener("click", async () => {
    const id = state.server?.active_run_id;
    if (!id || state.stoppingId === id || state.selectedId && state.selectedId !== id) return;
    state.stoppingId = id; updateControls(); notice("");
    try {
      await api(`/api/runs/${encodeURIComponent(id)}/stop`, {method: "POST", body: "{}"});
      announce("루프 중지를 요청했습니다."); render();
    } catch (error) { state.stoppingId = null; notice(error.message); updateControls(); }
  });
  $("back-live").addEventListener("click", () => { state.selectedId = null; state.selectedRun = null; state.timelineKey = null; render(); });
  $("refresh-history").addEventListener("click", async () => {
    try { const snapshot = await api("/api/state"); state.server = snapshot; state.historyKey = null; renderHistory(snapshot.runs || []); updateControls(); }
    catch (error) { notice(error.message); }
  });
  document.querySelectorAll("[data-view]").forEach((button) => button.addEventListener("click", () => {
    state.imageView = button.dataset.view;
    document.querySelectorAll("[data-view]").forEach((item) => { item.classList.toggle("selected", item === button); item.setAttribute("aria-pressed", String(item === button)); });
    $("camera-grid").classList.toggle("compare", state.imageView === "compare");
    document.querySelectorAll(".before-frame").forEach((frame) => { frame.hidden = state.imageView !== "compare"; });
  }));
  window.setInterval(() => {
    manualControls();
    const run = state.selectedId ? state.selectedRun : state.currentRun;
    if (state.connected && run?.active) text("elapsed", clock(runElapsed(run)));
  }, 1000);
  poll();
})();
