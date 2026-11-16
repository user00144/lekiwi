"use strict";

(() => {
  const $ = (id) => document.getElementById(id);
  const state = {
    server: null, currentRun: null, selectedRun: null, selectedId: null,
    started: false, busy: false, stoppingId: null, connected: false,
    timelineKey: null, historyKey: null, imageView: "current", announcement: null,
  };
  const statusLabels = {
    idle: "대기", running: "실행 중", starting: "준비 중", stopping: "중지 중",
    interrupted: "중지됨", stopped: "중지됨", completion_claim: "완료 주장",
    blocked: "작업 보류", budget_actions: "행동 한도", budget_steps: "판단 한도",
    budget_seconds: "시간 한도", uncertain_execution: "실행 미확정",
    post_motion_observation_failed: "재관찰 실패", observation_failed: "관찰 실패",
    observation_invalid: "관찰 확인 필요", guarded_execution_unavailable: "실행 미지원",
    recorded_observation: "기록 실행 불가", runtime_changed: "로봇 연결 변경",
    invalid_decision: "판단 확인 필요", repeated_failed_action: "반복 행동 중단",
    host_error: "실행 오류", gui_error: "GUI 실행 오류", startup_error: "시작 실패", error: "오류", cancelled: "취소됨",
    interrupted_unknown: "종료 상태 미확정",
  };
  const toolLabels = {observe: "재관찰", execute: "행동 실행", task_complete: "완료 주장", blocked: "작업 보류"};
  const feedbackLabels = {initial: "첫 판단", progress: "진전 확인", stalled: "진전 없음", uncertain: "판단 유보"};
  const eventLabels = {
    startup: "작업 준비", startup_started: "작업 준비", startup_finished: "준비 완료", demo_started: "데모 시작",
    planner_started: "에이전트 연결", observe_started: "새 관찰 요청", observe_finished: "관찰 수신",
    observation: "관찰 저장", planning_started: "다음 행동 판단 중", planning_finished: "판단 수신",
    decision: "에이전트 판단", decision_rejected: "판단 보류", action_prepared: "명령 준비",
    action_started: "행동 실행", action_finished: "행동 응답", action_result: "실행 결과",
    motion_feedback: "실행 전후 비교", rejection_feedback: "명령 거부 · 재관찰 후 수정", completion_candidate: "완료 주장 · 재관찰 요청",
    episode_finished: "작업 종료", stopping: "루프 중지 요청", stop_requested: "루프 중지 요청",
    error: "오류",
  };
  const jointLabels = {shoulder_pan: "어깨 회전", shoulder_lift: "어깨 높이", elbow_flex: "팔꿈치", wrist_flex: "손목 굽힘", wrist_roll: "손목 회전", gripper: "그리퍼"};
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
    if (["host_error", "gui_error", "startup_error", "error", "uncertain_execution", "post_motion_observation_failed", "observation_failed"].includes(status)) return "error";
    if (["blocked", "stopping", "budget_actions", "budget_steps", "budget_seconds", "repeated_failed_action"].includes(status)) return "warning";
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
    $("start").disabled = !state.connected || active || state.busy;
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
    if (run) badge("run-mode", run.mode === "demo" ? "데모 기록" : run.mode === "live" ? "로봇 실행" : "실행 기록", run.mode === "demo" ? "demo" : "neutral");
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

  function safeRejection(packet) {
    return packet?.status === "rejected" && packet.execution?.outcome === "rejected" && packet.execution?.command_sent === false;
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
    const rejected = kind === "rejection_feedback" || kind === "action_result" && safeRejection(data.packet);
    const category = rejected ? "rejection" : kind === "decision" ? "decision" : kind.startsWith("action") ? "action" : kind === "error" || kind === "decision_rejected" ? "error" : "";
    const item = node("article", `timeline-event ${category}`);
    const head = node("div", "event-head");
    const finishedPhase = {observe_finished: "관찰", planning_finished: "판단", action_finished: "행동 실행", startup_finished: "작업 준비"}[kind];
    const interruptedLabel = data.status === "cancelled" ? "취소" : data.status === "error" ? "실패" : null;
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
    } else if (kind === "action_result") {
      const packet = data.packet || data;
      const execution = packet.execution || {};
      const message = safeRejection(packet) ? "명령 거부 · 재관찰 후 수정. 이 명령은 로봇에 전송되지 않았습니다."
        : execution.outcome === "uncertain_execution" ? "실행 여부 미확정 · 자동 재실행을 중단했습니다."
          : packet.status === "completed" ? "동작 응답 수신 · 새 관찰로 결과를 확인합니다." : packet.error || packet.observation_error || label(packet.status);
      item.append(node("p", "event-body", message));
      if (safeRejection(packet)) eventLine(item, "거부 원인", rejectionReason(packet));
      else if (execution.outcome === "uncertain_execution") eventLine(item, "오류", packet.error);
      if (typeof execution.arm_reached === "boolean") eventLine(item, "팔 목표", execution.arm_reached ? "엔코더 허용 오차 내 도달" : "도달 미확인");
      if (execution.base_stop_verified === false) eventLine(item, "베이스", "정지 명령 처리 · 실제 이동량·정지 상태는 미측정");
    } else if (kind === "rejection_feedback") {
      item.append(node("p", "event-body", rejectionReason(data)));
      eventLine(item, "전송 상태", data.command_sent === false ? "미전송 · 새 관찰 후 수정 판단" : "확인 필요");
      if (data.error_code === "arm_current_out_of_calibration" && data.error_details?.joint) eventLine(item, "관절 상태", calibrationLine(data.error_details));
      eventLine(item, "수정 지침", rejectionGuidance(data));
      eventLine(item, "거부 명령", commandLabel(data.command));
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
    } else if (data.reason || data.message || data.error || data.detail) {
      item.append(node("p", "event-body", data.reason || data.message || data.error || data.detail));
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
  }

  async function poll() {
    try {
      const snapshot = await api("/api/state");
      const wasDisconnected = !state.connected;
      state.server = snapshot;
      initialize(snapshot.config || {});
      setConnection(true);
      if (wasDisconnected && $("notice").dataset.connectionError === "true") { notice(""); delete $("notice").dataset.connectionError; }
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
    const run = state.selectedId ? state.selectedRun : state.currentRun;
    if (state.connected && run?.active) text("elapsed", clock(runElapsed(run)));
  }, 1000);
  poll();
})();
