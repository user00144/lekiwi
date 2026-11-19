(() => {
  'use strict';

  const VISUAL_POLICY_TEMPLATE = [
    'report = {"stage": "start"}',
    '',
    '# Code block 1: check localization mode (SLAM is optional; confidence=0 means visual-only)',
    'pose = get_pose()',
    'report["slam_active"] = pose["confidence"] > 0.0',
    'obs = inspect_scene("Check front and wrist cameras and locate the target object.")',
    'report["inspect_ok"] = obs["ok"]',
    'print("inspect:", obs["summary"])',
    '',
    '# Code block 2: approach visually with holonomic base + 5-DoF arm and verify grasp',
    'if report["inspect_ok"]:',
    '    grasp = act("Drive smoothly into arm reach, grasp the target, and lift slightly to confirm hold.")',
    '    report["grasp_ok"] = grasp["ok"]',
    '    if grasp["ok"]:',
    '        result = report',
    '        finish("Target grasped and verified visually without SLAM")',
    '',
    'result = report',
  ].join('\n');

  const SKILL_POLICY_TEMPLATE = [
    '# Host skills: locate() = calibrated front-camera XYZ, pick()/place() = one IK move per phase',
    'report = {"stage": "start", "cfg": {"target": "silver cube", "reach_x_m": 0.27, "max_drives": 4}}',
    'cfg = report["cfg"]',
    '',
    '# Code block 1: arm home (front view clear), locate the target, short turn scan if not seen',
    'home = home_arm()',
    'loc = locate(cfg["target"])',
    'for scan_idx in range(3):',
    '    if loc["ok"]:',
    '        break',
    '    drive(turn_deg=30.0)',
    '    loc = locate(cfg["target"])',
    'print("locate:", loc["summary"])',
    '',
    '# Code block 2: drive closer until the arm can reach it; every drive makes the old XYZ stale',
    'for drive_idx in range(cfg["max_drives"]):',
    '    if not loc["ok"] or loc["reachable"]:',
    '        break',
    '    forward = max(-0.3, min(0.3, loc["xyz_m"][0] - cfg["reach_x_m"]))',
    '    left = max(-0.3, min(0.3, loc["xyz_m"][1]))',
    '    drive(forward_m=forward, left_m=left)',
    '    loc = locate(cfg["target"])',
    '    print("re-locate:", loc["summary"])',
    'report["located"] = loc["ok"]',
    'report["reachable"] = loc.get("reachable")',
    '',
    '# Code block 3: pick at the located XYZ; the skill re-checks the gripper after lifting',
    'if loc["ok"] and loc["reachable"]:',
    '    grasp = pick(loc["xyz_m"])',
    '    report["grasp_ok"] = grasp["ok"]',
    '    print("pick:", grasp["summary"])',
    '    if grasp["ok"] and grasp.get("object_between_jaws_likely"):',
    '        result = report',
    '        finish("Picked the target at the calibrated XYZ", xyz_m=loc["xyz_m"])',
    '',
    'result = report',
  ].join('\n');

  const MEMORY_POLICY_TEMPLATE = [
    'import math',
    '',
    'report = {"stage": "start"}',
    '',
    '# Code block 1: query spatial memory; fall back to visual scan when SLAM is off',
    'pose = get_pose()',
    'if pose["confidence"] > 0.0 and not pose["relocalized"]:',
    '    pose = relocalize()',
    'mem = search_memory("red ceramic mug", top_k=3)',
    'report["hits"] = mem["count"]',
    '',
    '# Code block 2: route via SLAM if available, otherwise operate directly from cameras',
    'if pose["confidence"] > 0.0 and mem["count"] > 0:',
    '    top = mem["hits"][0]',
    '    nav = navigate_to(top["floor"], top["x_m"], top["y_m"], top["yaw_deg"])',
    '    report["nav_ok"] = nav["ok"]',
    'obs = inspect_scene("Confirm whether the target mug is visible ahead.")',
    'if obs["ok"]:',
    '    grasp = act("Approach locally if needed, grasp the mug, and lift 5 cm to verify.")',
    '    report["grasp_ok"] = grasp["ok"]',
    '    if grasp["ok"]:',
    '        result = report',
    '        finish("Completed memory/visual fetch")',
    '',
    'result = report',
  ].join('\n');

  const $ = (id) => document.getElementById(id);

  const ui = {
    tab: 'direct',
    camMode: 'current',
    pinnedRunId: null,
    selectedStage: null,      // null = follow latest stage
    selectedStepIdx: null,    // null = follow latest decision step
    traceFilter: 'all',       // 'all' | 'reasoning' | 'tools' | 'alerts'
    openTraceKeys: new Set(), // preserve expanded <details> across polls
    openTimelineKeys: new Set(),
    showAllPlans: false,
    timelineSig: '',          // skip re-rendering when nothing changed (keeps scroll and text selection)
    memorySig: '',
    timelineScope: '',
    traceSig: '',
    lastRunKey: '',
    serverState: null,
    selectedRun: null,
    robotSeeded: false,
    pollTimer: null,
  };

  function showAlert(msg) {
    const el = $('alert-banner');
    if (!msg) {
      el.hidden = true;
      el.textContent = '';
      return;
    }
    el.textContent = msg;
    el.hidden = false;
  }

  async function apiPost(url, payload) {
    const res = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) {
      throw new Error(data.error || `HTTP ${res.status}`);
    }
    return data;
  }

  async function apiGet(url) {
    const res = await fetch(url);
    const data = await res.json().catch(() => ({}));
    if (!res.ok) {
      throw new Error(data.error || `HTTP ${res.status}`);
    }
    return data;
  }

  function formatDuration(sec) {
    if (!Number.isFinite(sec) || sec < 0) return '00:00';
    const total = Math.floor(sec);
    const m = String(Math.floor(total / 60)).padStart(2, '0');
    const s = String(total % 60).padStart(2, '0');
    return `${m}:${s}`;
  }

  function formatTimeShort(iso) {
    if (!iso) return '—';
    const d = new Date(iso);
    if (Number.isNaN(d.getTime())) return String(iso).slice(0, 19);
    return d.toLocaleTimeString([], { hour12: false });
  }

  function statusClass(status) {
    if (!status) return 'neutral';
    if (status === 'completed' || status === 'completion_claim' || status === 'reference_saved') return 'ok';
    if (status === 'running' || status === 'starting' || status === 'stopping') return 'teal';
    if (status === 'uncertain_execution' || status === 'gui_error' || status === 'host_error') return 'danger';
    if (status === 'blocked' || status.startsWith('budget_') || status.startsWith('interrupted')) return 'warn';
    return 'neutral';
  }

  function setTab(nextTab) {
    ui.tab = nextTab;
    const isDirect = nextTab === 'direct';
    $('tab-direct').classList.toggle('active', isDirect);
    $('tab-direct').setAttribute('aria-selected', String(isDirect));
    $('tab-policy').classList.toggle('active', !isDirect);
    $('tab-policy').setAttribute('aria-selected', String(!isDirect));
    $('direct-form').hidden = !isDirect;
    $('policy-form').hidden = isDirect;
  }

  function setCamMode(mode) {
    ui.camMode = mode;
    const compare = mode === 'compare';
    $('cam-mode-current').classList.toggle('active', !compare);
    $('cam-mode-compare').classList.toggle('active', compare);
    $('front-frames').classList.toggle('compare-mode', compare);
    $('wrist-frames').classList.toggle('compare-mode', compare);
    $('front-before-slot').hidden = !compare;
    $('wrist-before-slot').hidden = !compare;
    $('front-current-cap').hidden = !compare;
    $('wrist-current-cap').hidden = !compare;
  }

  function updateBudgetSummary() {
    const a = $('max-actions').value || 20;
    const s = $('max-steps').value || 40;
    const t = $('max-seconds').value || 300;
    $('budget-summary').textContent = `${a} actions · ${s} steps · ${t}s`;
  }

  function renderHeader(state) {
    $('conn-dot').className = 'dot online';
    $('conn-text').textContent = 'Connected';

    const isDemo = state.mode === 'demo';
    const modeChip = $('mode-chip');
    modeChip.textContent = isDemo ? 'DEMO' : 'LIVE';
    modeChip.className = `chip ${isDemo ? 'warn' : 'ok'}`;

    const cfg = state.config || {};
    const navChip = $('nav-chip');
    if (isDemo) {
      navChip.textContent = 'SYNTHETIC DEMO';
      navChip.className = 'chip warn';
    } else if (cfg.slam_configured) {
      navChip.textContent = 'SLAM POSE ACTIVE';
      navChip.className = 'chip ok';
    } else {
      navChip.textContent = 'SLAM OFF · Visual-Only';
      navChip.className = 'chip teal';
    }

    const ikChip = $('ik-chip');
    if (cfg.skills_configured) {
      ikChip.textContent = `IK + Skills${cfg.camera_configured ? ' + Locate' : ''} · act ${cfg.act_loop}`;
      ikChip.className = 'chip ok';
    } else if (cfg.kinematics_configured) {
      ikChip.textContent = '5-DoF IK Ready';
      ikChip.className = 'chip ok';
    } else {
      ikChip.textContent = 'Joint / URDF FK';
      ikChip.className = 'chip subtle';
    }

    if (!ui.robotSeeded && cfg.robot_id) {
      $('direct-robot-id').value = cfg.robot_id;
      $('policy-robot-id').value = cfg.robot_id;
      if (cfg.model) $('direct-model').value = cfg.model;
      if (isDemo) $('policy-backend').value = 'sim';
      if (cfg.limits) {
        $('max-actions').value = cfg.limits.max_actions;
        $('max-steps').value = cfg.limits.max_steps;
        $('max-seconds').value = cfg.limits.max_seconds;
        updateBudgetSummary();
      }
      if (state.capabilities?.host_skills) $('policy-source').value = SKILL_POLICY_TEMPLATE;
      ui.robotSeeded = true;
    }

    const busy = Boolean(state.active_run_id) || Boolean(state.pose_capture_busy);
    $('observe-once-btn').disabled = isDemo || busy;
    $('direct-start-btn').disabled = busy;
    $('policy-start-btn').disabled = busy;
    $('direct-stop-btn').disabled = !state.active_run_id;
    $('policy-stop-btn').disabled = !state.active_run_id;
  }

  const ENTRY_KIND_KO = { workflow: '검증된 절차', repair: '검증된 수리' };
  const LEDGER_STATUS_KO = { eliminated: '배제', blocked: '막힘', tried: '시도함', resolved: '해결됨' };
  const TOPIC_KO = {
    'episodic-workflows.md': '작업 절차', 'mobile-grasp.md': '집기·놓기', 'search-and-memory.md': '위치 찾기',
    'navigation.md': '주행·접근', 'lekiwi-operations.md': '운용·복구',
  };

  function entryChip(entry) {
    const chip = el('span', `lesson-chip ${entry.stale ? 'retired' : 'verified'}`,
      `${entry.id} ${ENTRY_KIND_KO[entry.kind] || entry.kind}`);
    chip.title = entry.trigger || '';
    return chip;
  }

  function renderMemory(memory) {
    const body = $('memory-body');
    const sig = JSON.stringify(memory || {});
    if (sig === ui.memorySig) return;
    ui.memorySig = sig;
    body.innerHTML = '';
    if (!memory || memory.error) {
      body.appendChild(el('p', 'empty-msg', memory?.error ? `기억을 읽지 못함: ${memory.error}` : '아직 기억이 없습니다.'));
      return;
    }
    const drive = memory.body?.drive || {};
    const offset = memory.body?.locate_offset || {};
    const rows = el('div', 'memory-rows');
    const addRow = (label, value, ready, title) => {
      const row = el('div', 'memory-row');
      row.title = title || '';
      row.appendChild(el('span', 'memory-label', label));
      row.appendChild(el('span', `memory-value${ready ? ' ready' : ''}`, value));
      rows.appendChild(row);
    };
    addRow('주행 보정',
      drive.gain != null ? `×${fmtNum(drive.gain, 3)} (${drive.samples}회 측정${drive.ready ? '' : ', 3회부터 사용'})` : '측정 없음',
      drive.ready, '다시 찾은 물체 위치로 잰 실제 이동 / 명령한 속도×시간');
    addRow('위치 보정',
      offset.offset_m ? `[${offset.offset_m.map((v) => fmtNum(v * 1000, 1)).join(', ')}] mm (${offset.samples}회${offset.ready ? ', 적용 중' : ''})`
        : '측정 없음 (놓은 뒤 다시 찾을 때 측정)',
      offset.ready, '놓은 위치(팔 FK) - 이후 locate로 본 위치');
    const counts = memory.counts || {};
    const total = Object.values(counts).reduce((a, b) => a + b, 0);
    addRow('스킬 라이브러리',
      total ? Object.entries(counts).filter(([, n]) => n).map(([t, n]) => `${TOPIC_KO[t] || t} ${n}`).join(' · ')
        : '항목 없음 (검증된 성공 실행에서만 추가)',
      total > 0, memory.library_dir || '');
    const lc = memory.ledger_counts || {};
    const unresolved = (lc.eliminated || 0) + (lc.blocked || 0) + (lc.tried || 0);
    addRow('실패 장부',
      (unresolved + (lc.resolved || 0))
        ? `배제 ${lc.eliminated || 0} · 막힘 ${lc.blocked || 0} · 시도 ${lc.tried || 0} · 해결 ${lc.resolved || 0}`
        : '기록 없음',
      false, '실패한 접근(스킬 순서 + 마지막 팔 목표). 계획 때 반복하지 말 것으로 전달');
    if (memory.frozen) addRow('', '동결됨: 읽기만, 새 항목 추가 안 함', false);
    body.appendChild(rows);

    const entries = memory.entries || [];
    if (entries.length) {
      const list = el('ul', 'memory-lessons');
      for (const entry of entries.slice(0, 10)) {
        const li = el('li');
        const head = el('div', 'lesson-head');
        head.appendChild(entryChip(entry));
        head.appendChild(el('span', 'lesson-when', TOPIC_KO[entry.topic] || entry.topic));
        head.appendChild(el('span', 'lesson-score',
          `재확인 ${entry.confirmed} · 재사용 실패 ${entry.failed_reuse}${entry.stale ? ' · 오래됨' : ''}`));
        li.appendChild(head);
        li.appendChild(el('p', 'lesson-text', entry.trigger));
        if (entry.contrast) li.appendChild(el('p', 'lesson-text muted', entry.contrast));
        const details = el('details', 'tl-plan');
        details.appendChild(el('summary', null, `실행된 스킬 ${entry.snippet.length}개`));
        details.appendChild(el('pre', 'code-block', entry.snippet.join('\n')));
        li.appendChild(details);
        list.appendChild(li);
      }
      body.appendChild(list);
    }

    const ledger = memory.ledger || [];
    if (ledger.length) {
      body.appendChild(el('p', 'subhead mt-sm', '실패 장부 (해봤는데 안 된 접근)'));
      const list = el('ul', 'memory-lessons');
      for (const record of ledger.slice(0, 10)) {
        const li = el('li');
        const head = el('div', 'lesson-head');
        const cls = record.status === 'resolved' ? 'verified' : record.status === 'tried' ? 'hypothesis' : 'retired';
        head.appendChild(el('span', `lesson-chip ${cls}`, `${record.id} ${LEDGER_STATUS_KO[record.status] || record.status}`));
        head.appendChild(el('span', 'lesson-when', record.failure));
        head.appendChild(el('span', 'lesson-score', `실패 ${record.times_failed}회`));
        li.appendChild(head);
        li.appendChild(el('p', 'lesson-text', `${record.approach}${record.last_arm_target ? ` · 마지막 팔 목표 ${fmtXyz(record.last_arm_target)}` : ''}`));
        if (record.hypothesis) {
          li.appendChild(el('p', 'lesson-text', `가설 (${FAILURE_TYPE_KO[record.failure_type] || record.failure_type}, `
            + `${HYP_STATUS_KO[record.hypothesis_status] || record.hypothesis_status}): ${record.hypothesis}`));
          if (record.next_try) li.appendChild(el('p', 'lesson-text muted', `다음에 바꿀 것: ${record.next_try}`));
        } else {
          li.appendChild(el('p', 'lesson-text muted', record.last_reason));
        }
        list.appendChild(li);
      }
      body.appendChild(list);
    }
  }

  function renderHistory(state, currentId) {
    const container = $('history-list');
    const runs = state.runs || [];
    if (!runs.length) {
      container.innerHTML = '<p class="empty-msg">No recorded runs yet.</p>';
      return;
    }
    container.innerHTML = '';
    for (const item of runs.slice(0, 40)) {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'history-item' + (item.id === currentId ? ' selected' : '');
      const tag = item.mode === 'policy'
        ? (item.backend === 'sim' ? 'POLICY·SIM' : 'POLICY')
        : item.mode === 'reference'
          ? 'SNAPSHOT'
          : item.mode === 'demo'
            ? 'DEMO'
            : 'TASK';
      const cls = statusClass(item.status);
      btn.innerHTML = `
        <div class="history-top">
          <span class="history-tag">${tag}</span>
          <span class="chip ${cls}">${item.status}</span>
        </div>
        <div class="history-goal"></div>
        <div class="history-time">${formatTimeShort(item.started_at)} · ${item.id.slice(-8)}</div>
      `;
      btn.querySelector('.history-goal').textContent = item.instruction || item.id;
      btn.addEventListener('click', () => {
        ui.pinnedRunId = item.id === state.active_run_id ? null : item.id;
        ui.selectedStage = null;
        ui.selectedStepIdx = null;
        refresh();
      });
      container.appendChild(btn);
    }
  }

  function findCameraImage(obs, name) {
    if (!obs || !Array.isArray(obs.images)) return null;
    return obs.images.find((img) => String(img.label).toLowerCase().includes(name)) || null;
  }

  function setImgSlot(imgId, emptyId, entry) {
    const img = $(imgId);
    const empty = $(emptyId);
    if (entry && entry.url) {
      if (img.getAttribute('src') !== entry.url) {
        img.src = entry.url;
      }
      img.hidden = false;
      empty.hidden = true;
    } else {
      img.removeAttribute('src');
      img.hidden = true;
      empty.hidden = false;
    }
  }

  function renderCamerasAndJoints(run) {
    const cur = run ? run.current_observation : null;
    const bef = run ? run.before_observation : null;

    setImgSlot('front-current-img', 'front-current-empty', findCameraImage(cur, 'front'));
    setImgSlot('wrist-current-img', 'wrist-current-empty', findCameraImage(cur, 'wrist'));
    setImgSlot('front-before-img', 'front-before-empty', findCameraImage(bef, 'front'));
    setImgSlot('wrist-before-img', 'wrist-before-empty', findCameraImage(bef, 'wrist'));

    const ts = cur ? formatTimeShort(cur.at) : '—';
    $('front-time').textContent = ts;
    $('wrist-time').textContent = ts;

    const strip = $('joint-strip');
    const joints = cur?.packet?.observation?.ego?.joints;
    if (!joints || typeof joints !== 'object') {
      strip.innerHTML = '<span class="empty-inline">Joint telemetry appears when an observation is captured.</span>';
      return;
    }

    const labels = [
      ['shoulder_pan', 'pan', '°'],
      ['shoulder_lift', 'lift', '°'],
      ['elbow_flex', 'elbow', '°'],
      ['wrist_flex', 'w_flex', '°'],
      ['wrist_roll', 'w_roll', '°'],
      ['gripper', 'gripper', '%'],
    ];
    const pills = [];
    for (const [key, shortName, unit] of labels) {
      if (typeof joints[key] === 'number') {
        pills.push(`<span class="joint-pill"><b>${shortName}</b>${joints[key].toFixed(1)}${unit}</span>`);
      }
    }
    const xyz = run?.kinematics?.current_xyz_m;
    if (Array.isArray(xyz) && xyz.length === 3) {
      pills.push(
        `<span class="joint-pill"><b>FK xyz</b>[${xyz.map((v) => Number(v).toFixed(3)).join(', ')}] m</span>`
      );
    }
    strip.innerHTML = pills.length
      ? pills.join('')
      : '<span class="empty-inline">No numeric joint values in packet.</span>';
  }

  function classifyTraceEvent(evName) {
    if (evName === 'decision' || evName === 'completion_candidate') return 'reasoning';
    if (
      evName === 'decision_rejected' ||
      evName === 'motion_warning' ||
      evName === 'monitor_stop' ||
      evName === 'motion_incomplete' ||
      evName === 'motion_failure_feedback' ||
      evName === 'reconcile_started' ||
      evName === 'reconcile_finished' ||
      evName === 'planner_cleanup_error'
    ) {
      return 'alerts';
    }
    if (
      evName === 'action_prepared' ||
      evName === 'action_result' ||
      evName === 'ik_result' ||
      evName === 'observe_finished' ||
      evName === 'motion_feedback' ||
      evName === 'skill_result' ||
      evName === 'locate_answer'
    ) {
      return 'tools';
    }
    return 'other';
  }

  function summarizeEventOneLine(ev) {
    const kind = ev.event;
    const d = ev.data || {};
    if (kind === 'decision') {
      const cmdPart = describeCommand(d.command);  // step-loop command or plan-act skill arguments
      return `[${d.tool}] ${cmdPart ? cmdPart + ' — ' : ''}${d.reason || ''}`;
    }
    if (kind === 'action_prepared') {
      const j = d.joints ? Object.entries(d.joints).map(([k, v]) => `${k}=${Number(v).toFixed(1)}`).join(', ') : 'joints=unchanged';
      return `move (x=${d.x}, y=${d.y}, th=${d.theta}, ${d.duration_ms}ms) · ${j}`;
    }
    if (kind === 'action_result') {
      const p = d.packet || {};
      const ex = p.execution || {};
      return `status=${p.status} · sent=${ex.command_sent} · arm_reached=${ex.arm_reached}${p.error ? ' · err=' + p.error : ''}`;
    }
    if (kind === 'ik_result') {
      return `IK success=${d.success} · residual=${d.residual_m ?? '—'}m ${Array.isArray(d.reason_codes) && d.reason_codes.length ? '[' + d.reason_codes.join(',') + ']' : ''}`;
    }
    if (kind === 'observation') {
      return `observation_id=${d.observation_id} (${d.path || ''})`;
    }
    if (kind === 'kinematics_state') {
      const codes = Array.isArray(d.reason_codes) && d.reason_codes.length ? ` · ${d.reason_codes.join(', ')}` : '';
      return `팔 끝 FK ${fmtXyz(d.current_xyz_m)} m · XYZ 제어 ${d.executable ? '가능' : '불가'}${codes}`;
    }
    if (kind === 'motion_feedback') {
      const ex = d.execution || {};
      const moved = Object.entries(d.joint_delta || {}).filter(([, v]) => Math.abs(v) >= 0.5)
        .map(([k, v]) => `${JOINT_SHORT[k] || k} ${v > 0 ? '+' : ''}${fmtNum(v, 1)}°`);
      return `실행 ${ex.outcome || '?'}${ex.arm_reached === true ? ' · 목표 도달' : ex.arm_reached === false ? ' · 목표 미도달' : ''}`
        + `${moved.length ? ` · 관절 변화 ${moved.join(', ')}` : ''}${d.no_joint_progress ? ' · 관절 변화 없음' : ''}`;
    }
    if (['motion_warning', 'monitor_stop', 'hypothesis_test', 'skill_result', 'locate_answer', 'decision_rejected', 'rejection_feedback',
         'memory_update', 'skill_finished'].includes(kind)) {
      const outcome = describeOutcome(ev, new Map());
      if (outcome) return outcome.text;
    }
    return d.reason || d.detail || d.status || JSON.stringify(d);
  }

  function renderStagePills(policy) {
    const wrap = $('stage-pills');
    const stages = policy.stages || [];
    wrap.innerHTML = '';
    if (!stages.length) {
      wrap.innerHTML = '<span class="muted-sm">No robot stages invoked yet.</span>';
      return;
    }

    const latestBtn = document.createElement('button');
    latestBtn.type = 'button';
    latestBtn.className = 'pill-btn' + (ui.selectedStage === null ? ' active' : '');
    latestBtn.textContent = 'Auto (Latest)';
    latestBtn.addEventListener('click', () => {
      ui.selectedStage = null;
      ui.selectedStepIdx = null;
      refresh();
    });
    wrap.appendChild(latestBtn);

    for (const s of stages) {
      const btn = document.createElement('button');
      btn.type = 'button';
      const isSelected = ui.selectedStage
        ? ui.selectedStage === s.name
        : policy.selected_stage === s.name && stages.length === 1;
      const match = /^(\d{4})_L(\d{3})_(.+)$/.exec(s.name);
      const tool = match ? match[3] : s.name;
      const ok = s.status === 'completed' || s.status === 'completion_claim';
      const running = s.status === 'running';
      const mark = ok ? '✓' : running ? '…' : '✗';
      btn.className = 'pill-btn' + (ui.selectedStage === s.name ? ' active' : '') + (ok ? ' pill-ok' : running ? '' : ' pill-bad');
      btn.textContent = match
        ? `${Number(match[1])}. L${Number(match[2])} ${TOOL_KO[tool] || tool} ${mark}`
        : `${s.name} ${mark}`;
      btn.title = `${s.name} · ${s.status}${s.summary ? `\n${s.summary}` : ''}`;
      btn.addEventListener('click', () => {
        ui.selectedStage = s.name;
        ui.selectedStepIdx = null;
        refresh();
      });
      wrap.appendChild(btn);
    }
  }

  // ---- Decision timeline -------------------------------------------------------------
  const TOOL_KO = {
    observe: '재관찰', execute: '명령 실행', task_complete: '완료 주장', blocked: '중단',
    plan: '계획 수립', locate: '위치 찾기', pick: '집기', place: '놓기', move_to: 'XYZ 이동',
    home: '홈 복귀', home_arm: '홈 복귀', gripper: '그리퍼', drive: '주행', done: '루프 완료 선언',
    failed: '루프 실패 선언', confirm: '완료 확인', replan: '재계획 요청',
    navigate_to: '경로 이동', inspect_scene: '장면 확인', act: '행동(act)', review: '실패 원인 가설',
  };
  const FAILURE_TYPE_KO = {
    locate_wrong: '위치 측정 틀림', out_of_reach: '팔이 닿지 않음', grasp_missed: '집기 실패',
    grip_lost: '옮기다 놓침', release_misaligned: '놓는 위치 어긋남', scene_disturbed: '다른 물체를 건드림',
    base_motion_error: '주행 오차', wrong_plan: '계획 자체가 틀림', robot_fault: '로봇 실행 불가', other: '기타',
  };
  const LEVEL_KO = { execution: '실행 실패', planning: '계획 실패', environment: '환경 변화', unknown: '불명' };
  const HYP_STATUS_KO = { untested: '아직 검증 안 함', not_supported: '바꿔봤지만 또 실패', inconclusive: '검증 중 중단', supported: '바꿔서 성공' };
  const MET_MARK = { yes: '✓', no: '✗', unclear: '?' };
  const PHASE_KO = { planning: '계획', action: '행동', verify: '검증', review: '실패 분석', skill: '호스트 스킬' };
  const FEEDBACK_KO = { initial: '첫 판단', progress: '진전 있음', stalled: '진전 없음', uncertain: '판단 유보' };
  const JOINT_SHORT = {
    shoulder_pan: 'pan', shoulder_lift: 'lift', elbow_flex: 'elbow', wrist_flex: 'w_flex', wrist_roll: 'w_roll',
    gripper: 'gripper',
  };

  function el(tag, cls, text) {
    const node = document.createElement(tag);
    if (cls) node.className = cls;
    if (text !== undefined && text !== null) node.textContent = text;
    return node;
  }

  function fmtNum(value, digits = 3) {
    return typeof value === 'number' && Number.isFinite(value) ? String(Number(value.toFixed(digits))) : String(value);
  }

  function fmtXyz(xyz) {
    return Array.isArray(xyz) ? `[${xyz.map((v) => fmtNum(v, 3)).join(', ')}]` : '—';
  }

  function describeCommand(cmd) {
    if (!cmd || typeof cmd !== 'object') return '';
    if ('duration_ms' in cmd && ('action' in cmd || 'joints' in cmd || 'cartesian' in cmd)) {
      const parts = [];
      if (cmd.cartesian) {
        parts.push(`XYZ ${cmd.cartesian.mode === 'relative' ? '상대' : '절대'} ${fmtXyz(cmd.cartesian.xyz_m)} m`);
      }
      const joints = cmd.joints ? Object.entries(cmd.joints).filter(([, v]) => v !== null && v !== undefined) : [];
      if (joints.length && !cmd.cartesian) {
        parts.push(joints.map(([k, v]) => `${JOINT_SHORT[k] || k} ${fmtNum(v, 1)}`).join(' · '));
      }
      if (cmd.x || cmd.y || cmd.theta) {
        parts.push(`베이스 x=${fmtNum(cmd.x, 2)} y=${fmtNum(cmd.y, 2)} m/s θ=${fmtNum(cmd.theta, 1)}°/s`);
      }
      if (!parts.length) parts.push('대기');
      parts.push(`${cmd.duration_ms} ms`);
      return parts.join(' · ');
    }
    const parts = [];
    if (cmd.label) parts.push(`"${cmd.label}"`);
    if (Array.isArray(cmd.bbox_px)) parts.push(`상자 [${cmd.bbox_px.map((v) => Math.round(v)).join(', ')}] px`);
    if (cmd.xyz_m) parts.push(`xyz ${fmtXyz(cmd.xyz_m)} m`);
    if (cmd.approach) parts.push(`접근 ${cmd.approach}`);
    if (typeof cmd.gripper_percent === 'number') parts.push(`그리퍼 ${fmtNum(cmd.gripper_percent, 1)}%`);
    if (cmd.drive) {
      parts.push(`x=${fmtNum(cmd.drive.x, 2)} y=${fmtNum(cmd.drive.y, 2)} m/s θ=${fmtNum(cmd.drive.theta, 1)}°/s · ${cmd.drive.duration_ms} ms`);
    }
    if ('forward_m' in cmd || 'turn_deg' in cmd) {
      parts.push(`앞 ${fmtNum(cmd.forward_m || 0, 2)} m · 왼쪽 ${fmtNum(cmd.left_m || 0, 2)} m · 회전 ${fmtNum(cmd.turn_deg || 0, 0)}°`);
    }
    return parts.length ? parts.join(' · ') : JSON.stringify(cmd);
  }

  function describeMonitorStop(stop) {
    if (!stop) return '감시 조건이 깨짐';
    const where = typeof stop.progress_fraction === 'number' ? `동작 ${Math.round(stop.progress_fraction * 100)}% 지점` : '동작 중';
    if (stop.condition === 'gripper_below_min') {
      return `${where}에서 그리퍼 ${fmtNum(stop.gripper_percent, 1)}% < 쥔 기준 ${fmtNum(stop.gripper_min_percent, 1)}% (물체 놓침 추정)`;
    }
    return `${where}에서 ${stop.condition}`;
  }

  function describeSkillResult(d) {
    const r = d.result || {};
    const bits = [];
    if (r.xyz_m) bits.push(`xyz ${fmtXyz(r.xyz_m)}`);
    if (r.reachable !== undefined) bits.push(r.reachable ? '팔이 닿음' : '팔이 닿지 않음');
    if (r.in_calibrated_region === false) bits.push('보정 영역 밖(오차 큼)');
    if (r.gripper_after_close !== undefined) bits.push(`닫은 뒤 그리퍼 ${fmtNum(r.gripper_after_close, 1)}%`);
    if (r.object_between_jaws_likely !== undefined) bits.push(r.object_between_jaws_likely ? '물체 잡힘 추정' : '빈 집게 추정');
    if (r.grasp_fk_xyz_m) bits.push(`집은 위치 ${fmtXyz(r.grasp_fk_xyz_m)}`);
    if (r.arm_xyz_m) bits.push(`팔 끝 ${fmtXyz(r.arm_xyz_m)}`);
    if (r.locate_offset_applied) bits.push(`위치 보정 적용 (원래 ${fmtXyz(r.xyz_m_raw)})`);
    if (r.monitor_stop) bits.push(`이동 중 감시 정지: ${describeMonitorStop(r.monitor_stop)}`);
    if (r.failed_step) bits.push(`실패 단계: ${r.failed_step}`);
    const dm = r.drive_measurement;
    if (dm && dm.accepted) {
      const cmd = Math.hypot(...dm.commanded_m);
      bits.push(`주행 측정: 명령 ${fmtNum(cmd, 3)} m → 실제 ${fmtNum(cmd * dm.gain, 3)} m (×${fmtNum(dm.gain, 2)})`);
    } else if (dm) {
      bits.push(`주행 측정 제외: ${dm.reason}`);
    }
    const lm = r.locate_measurement;
    if (lm && lm.accepted) bits.push(`위치 오차 측정 [${lm.offset_m.map((v) => fmtNum(v * 1000, 1)).join(', ')}] mm`);
    const ok = d.status === 'ok';
    return {
      level: ok ? 'ok' : 'bad',
      text: `${TOOL_KO[d.tool] || d.tool} ${ok ? '성공' : '실패'}${d.error ? ` — ${d.error}` : ''}${bits.length ? ` · ${bits.join(' · ')}` : ''}`,
    };
  }

  function describeOutcome(ev, prepared) {
    const d = ev.data || {};
    switch (ev.event) {
      case 'action_result': {
        const p = d.packet || {};
        const ex = p.execution || {};
        const what = describeCommand(prepared.get(d.action_id)) || '명령';
        if (p.status === 'rejected') {
          return { level: 'bad', text: `${what} → 실행 전 거부: ${p.error_code || ''} ${p.error || ''}`.trim() };
        }
        if (p.status === 'completed' || p.status === 'completed_without_observation') {
          return { level: 'ok', text: `${what} → 실행 완료${ex.arm_reached === true ? ' · 목표 도달' : ''}` };
        }
        if (String(p.status).startsWith('warning')) {
          return { level: 'warn', text: `${what} → 완료, 목표와 실측이 조금 다름` };
        }
        if (String(p.status).startsWith('stopped')) {
          return { level: 'bad', text: `${what} → 이동 중 감시 정지: ${describeMonitorStop(ex.monitor_stop)}` };
        }
        return { level: 'bad', text: `${what} → ${p.status}${p.error ? `: ${p.error}` : ''}` };
      }
      case 'decision_rejected':
        return { level: 'warn', text: `판단이 실행되지 않음: ${d.reason || ''}${d.detail ? ` — ${d.detail}` : ''}${d.error ? ` — ${d.error}` : ''}` };
      case 'rejection_feedback':
        return { level: 'bad', text: `실행 전 거부: ${d.error_code || ''} ${d.error || ''}`.trim() };
      case 'ik_result':
        return d.executable === false
          ? { level: 'bad', text: `IK 해 없음: ${(d.reason_codes || []).join(', ') || '이유 미기록'}` }
          : null;
      case 'motion_warning': {
        const w = d.warning || {};
        const amount = typeof w.measured_max_error_deg === 'number'
          ? ` 최대 ${w.measured_max_error_deg.toFixed(1)}° (허용 ${fmtNum(w.max_error_deg, 0)}°)` : '';
        return { level: 'warn', text: `동작 경고: 목표와 실측 관절 차이${amount} → 실측 자세에서 계속` };
      }
      case 'monitor_stop':
        return { level: 'bad', text: `이동 중 감시 정지: ${describeMonitorStop(d.monitor_stop)} → 팔을 그 자리에 세우고 실측 확인` };
      case 'motion_incomplete':
      case 'motion_failure_feedback':
        return { level: 'bad', text: '부분 동작: 일부 관절이 목표에 못 미침 → 재확인·수정 필요' };
      case 'motion_recovery_completed':
        return { level: 'ok', text: '복구 동작 완료' };
      case 'reconcile_result':
        return { level: 'info', text: `상태 재확인: ${d.status || ''}` };
      case 'completion_candidate':
        return { level: 'info', text: '완료 후보로 기록 — 새 관찰에서 한 번 더 확인해야 끝남' };
      case 'locate_answer':
        return d.found
          ? { level: 'ok', text: `상자 [${(d.bbox_px || []).map((v) => Math.round(v)).join(', ')}] px · 확신 ${d.confidence}${d.note ? ` · ${d.note}` : ''}` }
          : { level: 'warn', text: `찾지 못함${d.note ? `: ${d.note}` : ''}` };
      case 'skill_result':
        return describeSkillResult(d);
      case 'skill_finished':
        return { level: d.status === 'completed' ? 'ok' : 'bad', text: `스킬 종료: ${d.status}${d.detail ? ` — ${d.detail}` : ''}` };
      case 'hypothesis_test':
        return {
          level: d.result === 'supported' ? 'ok' : d.result === 'not_supported' ? 'bad' : 'warn',
          text: `가설 ${d.id} 검증: ${HYP_STATUS_KO[d.result] || d.result}${d.change ? ` — 바꾼 점: ${d.change}` : ''}`,
        };
      case 'memory_update': {
        if (d.error) return { level: 'warn', text: `기억 갱신 실패: ${d.error}` };
        const parts = [];
        for (const h of d.hypotheses || []) parts.push(`가설 ${h.id} ${HYP_STATUS_KO[h.status] || h.status}`);
        const v = d.verification || {};
        const failedChecks = Object.entries(v).filter(([k, val]) => val === false && k !== 'passed').map(([k]) => k);
        parts.push(v.passed ? '성공 검증 통과' : `성공 검증 실패${failedChecks.length ? ` (${failedChecks.join(', ')})` : ''}`);
        const lib = d.library || {};
        if (lib.added?.length) parts.push(`라이브러리 새 항목 ${lib.added.join(', ')}`);
        if (lib.confirmed?.length) parts.push(`재확인 ${lib.confirmed.join(', ')}`);
        if (lib.skipped && v.passed) parts.push(`추가 안 함: ${lib.skipped}`);
        const recorded = (d.ledger?.recorded || []).map(([id, status]) => `${id} ${LEDGER_STATUS_KO[status] || status}`);
        if (recorded.length) parts.push(`실패 장부 기록 ${recorded.join(', ')}`);
        if (d.ledger?.resolved?.length) parts.push(`실패 장부 해결 ${d.ledger.resolved.join(', ')}`);
        const reuse = Object.entries(d.reuse || {}).map(([id, r]) => `${id} ${r === 'confirmed' ? '재사용 성공' : '재사용 실패'}`);
        if (reuse.length) parts.push(reuse.join(', '));
        const gain = d.body?.drive?.gain;
        if (gain != null) parts.push(`주행 보정 ×${fmtNum(gain, 3)} (${d.body.drive.samples}회)`);
        return { level: 'info', text: `기억 갱신 — ${parts.join(' · ') || '변경 없음'}` };
      }
      case 'episode_finished':
        return {
          level: d.status === 'completion_claim' ? 'ok' : 'bad',
          text: `에피소드 종료: ${d.status}${d.detail ? ` — ${d.detail}` : ''}`,
          final: true,
        };
      default:
        return null;
    }
  }

  function buildTimeline(events) {
    const prepared = new Map();
    const groups = [];
    let current = null;
    let thinkS = null;
    for (const ev of events) {
      const d = ev.data || {};
      if (ev.event === 'planning_finished') {
        thinkS = d.duration_s;
        continue;
      }
      if (ev.event === 'action_prepared') {
        prepared.set(d.action_id, d);
        continue;
      }
      if (ev.event === 'decision') {
        current = { ev, outcomes: [], thinkS };
        thinkS = null;
        groups.push(current);
        continue;
      }
      if (ev.event === 'skill_started' && !groups.length) {
        current = { ev, skill: true, outcomes: [] };
        groups.push(current);
        continue;
      }
      if (!current) continue;
      const outcome = describeOutcome(ev, prepared);
      if (outcome) current.outcomes.push(outcome);
    }
    return groups;
  }

  function phaseOf(group) {
    const d = group.ev.data || {};
    if (group.skill) return 'skill';
    if (d.phase) return d.phase;
    if (d.tool === 'execute') return 'action';
    if (d.tool === 'task_complete') return 'verify';
    return 'observe';
  }

  function keepOpen(details, key) {
    details.open = ui.openTimelineKeys.has(key) || details.open;
    details.addEventListener('toggle', () => {
      if (details.open) ui.openTimelineKeys.add(key);
      else ui.openTimelineKeys.delete(key);
    });
  }

  function appendReview(body, r) {
    const rows = [
      ['처음 틀어진 단계', r.failed_step
        ? `${r.failed_step.index}번 ${TOOL_KO[r.failed_step.tool] || r.failed_step.tool}${r.failed_step.phase ? ` (${r.failed_step.phase})` : ''}`
        : '없음 — 모든 단계가 확인됐는데 과제 실패'],
      ['유형', `${FAILURE_TYPE_KO[r.failure_type] || r.failure_type} · ${LEVEL_KO[r.failure_level] || r.failure_level} · 확신 ${r.confidence}`
        + (r.agrees_with_host === false ? ` · 호스트 측정은 ${FAILURE_TYPE_KO[r.host_type_hint] || r.host_type_hint}` : '')],
      ['가설 ' + (r.id || ''), r.hypothesis],
      ['다음에 바꿀 것', r.next_try],
      ['맞다면 보일 것', r.prediction],
    ];
    for (const [label, text] of rows) {
      if (!text) continue;
      const row = el('div', 'tl-row');
      row.appendChild(el('span', 'tl-label', label));
      row.appendChild(el('span', 'tl-meta', text));
      body.appendChild(row);
    }
    if (Array.isArray(r.step_checks) && r.step_checks.length) {
      const details = el('details', 'tl-plan');
      details.appendChild(el('summary', null, `단계별 확인 ${r.step_checks.length}개`));
      const ol = el('ul', 'tl-plan-list');
      for (const c of r.step_checks) {
        ol.appendChild(el('li', null, `${MET_MARK[c.met] || c.met} ${c.index}번 · 예상: ${c.expected} · 실제: ${c.observed}`));
      }
      details.appendChild(ol);
      details.open = true;
      body.appendChild(details);
    }
    for (const note of r.host_notes || []) body.appendChild(el('p', 'tl-pending', `호스트 보정: ${note}`));
  }

  function renderDecisionTimeline(events, run) {
    const list = $('decision-timeline');
    const scope = `${run.id}|${run.policy?.selected_stage || ''}`;
    const last = events.length ? events[events.length - 1].seq : 0;
    const sig = `${scope}|${events.length}|${last}|${ui.showAllPlans}|${run.active}`;
    if (sig === ui.timelineSig) return;
    ui.timelineSig = sig;
    const newScope = ui.timelineScope !== scope;
    if (newScope) ui.openTimelineKeys.clear();
    ui.timelineScope = scope;
    const nearBottom = list.scrollHeight - list.scrollTop - list.clientHeight < 80;

    const groups = buildTimeline(events);
    const decisions = groups.filter((g) => !g.skill);
    const actions = events.filter((e) => e.event === 'action_prepared').length;
    $('decision-count').textContent = groups.length
      ? `판단 ${decisions.length}개 · 로봇 명령 ${actions}개`
      : '판단 기록 없음';

    list.innerHTML = '';
    if (!groups.length) {
      const stage = run.mode === 'policy'
        ? (run.policy?.stages || []).find((item) => item.name === run.policy?.selected_stage)
        : null;
      if (stage) {
        // Host skills and sim stages run without planner turns; show the stage result instead.
        const match = /^(\d{4})_L(\d{3})_(.+)$/.exec(stage.name);
        const tool = match ? match[3] : stage.name;
        const ok = stage.status === 'completed' || stage.status === 'completion_claim';
        const card = el('li', 'tl-card phase-skill');
        const head = el('div', 'tl-head');
        head.appendChild(el('span', 'tl-index', match ? `${Number(match[1])}.` : ''));
        head.appendChild(el('span', 'tl-phase phase-skill', '정책 단계'));
        head.appendChild(el('span', 'tl-tool', TOOL_KO[tool] || tool));
        if (match) head.appendChild(el('span', 'tl-attempt', `policy.py ${Number(match[2])}번째 줄`));
        card.appendChild(head);
        const body = el('div', 'tl-body');
        body.appendChild(el('p', 'tl-reason', '모델 판단 없이 실행된 단계입니다 (호스트 스킬 또는 sim).'));
        const row = el('div', 'tl-row');
        row.appendChild(el('span', 'tl-label', '결과'));
        const ul = el('ul', 'tl-outcomes');
        ul.appendChild(el('li', ok ? 'out-ok' : 'out-bad', `${stage.status}${stage.summary ? ` — ${stage.summary}` : ''}`));
        row.appendChild(ul);
        body.appendChild(row);
        card.appendChild(body);
        list.appendChild(card);
        $('decision-count').textContent = '모델 판단 없음';
        $('decision-tool-badge').textContent = TOOL_KO[tool] || tool;
        return;
      }
      const msg = run.mode === 'reference'
        ? '카메라·관절 스냅샷만 저장된 기록입니다 (모델 판단 없음).'
        : '아직 기록된 판단이 없습니다.';
      list.appendChild(el('li', 'empty-msg', msg));
      $('decision-tool-badge').textContent = run.status;
      return;
    }

    let prevPlan = '';
    groups.forEach((group, idx) => {
      const d = group.ev.data || {};
      const phase = phaseOf(group);
      const isLast = idx === groups.length - 1;
      const card = el('li', `tl-card phase-${phase}${isLast && run.active ? ' live' : ''}`);

      const head = el('div', 'tl-head');
      head.appendChild(el('span', 'tl-index', `#${idx + 1}`));
      head.appendChild(el('span', `tl-phase phase-${phase}`, PHASE_KO[phase] || '관찰'));
      const toolName = group.skill ? d.skill : d.tool;
      const tool = el('span', 'tl-tool', TOOL_KO[toolName] || toolName || '판단');
      tool.title = toolName || '';
      head.appendChild(tool);
      if (d.feedback && FEEDBACK_KO[d.feedback] && !d.phase) {
        head.appendChild(el('span', `tl-feedback fb-${d.feedback}`, FEEDBACK_KO[d.feedback]));
      }
      if (d.attempt) head.appendChild(el('span', 'tl-attempt', `시도 ${d.attempt}`));
      const timing = [`+${Number(group.ev.elapsed_s || 0).toFixed(1)}s`];
      if (typeof group.thinkS === 'number') timing.push(`생각 ${group.thinkS.toFixed(1)}s`);
      head.appendChild(el('span', 'tl-time', timing.join(' · ')));
      card.appendChild(head);

      const body = el('div', 'tl-body');
      if (group.skill) {
        body.appendChild(el('p', 'tl-reason', `policy.py가 호출한 결정적 스킬입니다 (모델 판단 없음).`));
      } else if (d.reason && !d.review) {  // a review shows its hypothesis in its own rows
        body.appendChild(el('p', 'tl-reason', d.reason));
      }
      const command = group.skill ? describeCommand(d.args) : describeCommand(d.command);
      if (command) {
        const row = el('div', 'tl-row');
        row.appendChild(el('span', 'tl-label', '명령'));
        row.appendChild(el('code', 'tl-cmd', command));
        body.appendChild(row);
      }
      if (d.expected) {
        const row = el('div', 'tl-row');
        row.appendChild(el('span', 'tl-label', '예상'));
        row.appendChild(el('span', 'tl-meta', d.expected));
        body.appendChild(row);
      }
      if (d.hypothesis_tested || d.hypothesis_change) {
        const row = el('div', 'tl-row');
        row.appendChild(el('span', 'tl-label', '검증할 가설'));
        row.appendChild(el('span', 'tl-meta', d.hypothesis_tested
          ? `${d.hypothesis_tested}${d.hypothesis_text ? ` — ${d.hypothesis_text}` : ''} → 바꾼 점: ${d.hypothesis_change || '(설명 없음)'}`
          : `검증 안 함 — ${d.hypothesis_change}`));
        body.appendChild(row);
      }
      if (d.review) appendReview(body, d.review);
      if (Array.isArray(d.skills_applied) && d.skills_applied.length) {
        const row = el('div', 'tl-row');
        row.appendChild(el('span', 'tl-label', '따른 항목'));
        const wrap = el('div', 'tl-lessons');
        for (const entry of d.skills_applied) {
          wrap.appendChild(entryChip(entry));
          wrap.appendChild(el('span', 'tl-lesson-text', `${entry.trigger || ''} (재확인 ${entry.confirmed ?? 0})`));
        }
        row.appendChild(wrap);
        body.appendChild(row);
      }
      if (d.evidence) {
        const row = el('div', 'tl-row');
        row.appendChild(el('span', 'tl-label', d.phase === 'action' ? '메모' : d.phase === 'planning' ? '라이브러리 활용' : '근거'));
        row.appendChild(el('span', 'tl-meta', d.evidence));
        body.appendChild(row);
      }
      if (group.outcomes.length) {
        const row = el('div', 'tl-row');
        row.appendChild(el('span', 'tl-label', '결과'));
        const ul = el('ul', 'tl-outcomes');
        for (const outcome of group.outcomes) ul.appendChild(el('li', `out-${outcome.level}`, outcome.text));
        row.appendChild(ul);
        body.appendChild(row);
      } else if (!group.skill && (d.tool === 'execute' || phase === 'action') && isLast && run.active) {
        body.appendChild(el('p', 'tl-pending', '실행 결과 기다리는 중…'));
      }

      // plan-act action/verify turns repeat the attempt's plan; it is shown once on the planning card.
      const plan = Array.isArray(d.plan) && d.phase !== 'action' && d.phase !== 'verify' ? d.plan.filter(Boolean) : [];
      if (plan.length) {
        const planText = plan.join('\n');
        const changed = planText !== prevPlan;
        prevPlan = planText;
        const details = el('details', 'tl-plan');
        const note = d.phase ? '' : changed ? (idx === 0 ? '' : ' · 바뀜') : ' · 이전과 같음';
        const summary = el('summary', null, `계획 ${plan.length}단계${note}`);
        details.appendChild(summary);
        const ol = el('ol', 'tl-plan-list');
        for (const step of plan) ol.appendChild(el('li', null, step));
        details.appendChild(ol);
        details.open = ui.showAllPlans || phase === 'planning' || (!d.phase && changed);
        keepOpen(details, `${idx}|plan`);
        body.appendChild(details);
      }

      const raw = el('details', 'tl-raw');
      raw.appendChild(el('summary', null, '원본 JSON'));
      raw.appendChild(el('pre', 'code-block', JSON.stringify(d, null, 2)));
      keepOpen(raw, `${idx}|raw`);
      body.appendChild(raw);
      card.appendChild(body);
      list.appendChild(card);
    });

    const lastGroup = groups[groups.length - 1];
    const lastTool = lastGroup.skill ? lastGroup.ev.data?.skill : lastGroup.ev.data?.tool;
    $('decision-tool-badge').textContent = `#${groups.length} ${TOOL_KO[lastTool] || lastTool || ''}`;
    if (newScope || nearBottom) list.scrollTop = list.scrollHeight;
  }

  function renderExpandableTrace(run) {
    const events = run.events || [];
    const traceLen = (run.policy?.trace || []).length;
    const sig = `${run.id}|${run.policy?.selected_stage || ''}|${events.length}|${events.length ? events[events.length - 1].seq : 0}|${traceLen}|${ui.traceFilter}`;
    if (sig === ui.traceSig) return;
    ui.traceSig = sig;
    const items = [];

    if (run.mode === 'policy' && Array.isArray(run.policy?.trace)) {
      for (const t of run.policy.trace) {
        const isAlert = t.output && (t.output.ok === false || t.output.status === 'uncertain_execution' || t.output.status === 'blocked');
        items.push({
          key: `pol-${t.seq}`,
          cat: isAlert ? 'alerts' : 'tools',
          t: `${Number(t.elapsed_s || 0).toFixed(1)}s`,
          kind: `policy L${t.lineno}:${t.tool}`,
          desc: `${t.line_text || t.tool} -> ${t.output?.summary || t.output?.status || JSON.stringify(t.output)}`,
          detailText: [
            t.highlighted_policy ? `# Active Policy Line (L${t.lineno})\n${t.highlighted_policy}` : '',
            `# Arguments\n${JSON.stringify(t.args, null, 2)}`,
            `# Return Value\n${JSON.stringify(t.output, null, 2)}`,
          ].filter(Boolean).join('\n\n'),
        });
      }
    }

    const stagePrefix = run.mode === 'policy' && run.policy?.selected_stage
      ? `[${run.policy.selected_stage}] `
      : '';

    for (const ev of events) {
      if (/_(started|finished)$/.test(ev.event) && ev.event !== 'episode_finished' && ev.event !== 'skill_finished') {
        continue; // phase bookkeeping; durations are shown in the decision timeline
      }
      const cat = classifyTraceEvent(ev.event);
      items.push({
        key: `ev-${stagePrefix}-${ev.seq}`,
        cat,
        t: `${Number(ev.elapsed_s || 0).toFixed(1)}s`,
        kind: `${stagePrefix}${ev.event}`,
        desc: summarizeEventOneLine(ev),
        detailText: JSON.stringify(ev.data, null, 2),
      });
    }

    const filtered = items.filter((item) => {
      if (ui.traceFilter === 'all') return true;
      if (ui.traceFilter === 'tools') return item.cat === 'tools';
      if (ui.traceFilter === 'reasoning') return item.cat === 'reasoning';
      if (ui.traceFilter === 'alerts') return item.cat === 'alerts';
      return true;
    });

    $('event-count').textContent = `${filtered.length} / ${items.length} items`;
    const logEl = $('event-log');
    if (!filtered.length) {
      logEl.innerHTML = '<p class="empty-msg">No trace items match the current filter.</p>';
      return;
    }

    logEl.innerHTML = '';
    for (const item of filtered.slice(-150).reverse()) {
      const details = document.createElement('details');
      details.className = 'trace-item';
      if (ui.openTraceKeys.has(item.key)) {
        details.open = true;
      }
      details.addEventListener('toggle', () => {
        if (details.open) ui.openTraceKeys.add(item.key);
        else ui.openTraceKeys.delete(item.key);
      });

      const summary = document.createElement('summary');
      summary.className = 'trace-summary';
      summary.innerHTML = `<span class="event-time"></span><span class="event-kind cat-${item.cat}"></span><span class="event-desc"></span>`;
      summary.children[0].textContent = item.t;
      summary.children[1].textContent = item.kind;
      summary.children[2].textContent = item.desc;

      const body = document.createElement('div');
      body.className = 'trace-body';
      body.textContent = item.detailText;

      details.appendChild(summary);
      details.appendChild(body);
      logEl.appendChild(details);
    }
  }

  function renderRunView(run, state) {
    const historyBanner = $('history-banner');
    if (ui.pinnedRunId && ui.pinnedRunId !== state.active_run_id) {
      $('history-banner-text').textContent = `Viewing saved run: ${ui.pinnedRunId}`;
      historyBanner.hidden = false;
    } else {
      historyBanner.hidden = true;
    }

    if (!run) {
      $('status-led').className = 'status-led';
      $('status-headline').textContent = 'Idle · Ready';
      $('status-subline').textContent = state.config?.slam_configured
        ? 'SLAM pose file configured. Ready for Direct Task or Policy.py.'
        : 'SLAM-free visual mode ready. Run a Direct Task, execute Policy.py, or click Observe Once.';
      $('run-type-chip').hidden = true;
      $('status-badge').textContent = 'idle';
      $('status-badge').className = 'chip neutral';
      $('uncertain-alert').hidden = true;
      $('metric-elapsed').textContent = '00:00';
      $('metric-actions-label').textContent = 'Actions';
      $('metric-actions').textContent = `0 / ${state.config?.limits?.max_actions ?? 20}`;
      $('metric-steps').textContent = `0 / ${state.config?.limits?.max_steps ?? 40}`;
      $('metric-obs').textContent = '—';
      renderCamerasAndJoints(null);
      $('policy-inspector').hidden = true;
      $('decision-tool-badge').textContent = 'idle';
      $('decision-count').textContent = '판단 기록 없음';
      $('decision-timeline').innerHTML = '<li class="empty-msg">실행을 시작하거나 History에서 기록을 고르면 판단이 여기에 시간순으로 나옵니다.</li>';
      ui.timelineSig = '';
      ui.traceSig = '';
      $('event-count').textContent = '0 items';
      $('event-log').innerHTML = '<p class="empty-msg">판단, 정책 호출, IK 검사, 모터 결과가 여기에 쌓입니다.</p>';
      return;
    }

    if (ui.lastRunKey !== run.id) {
      ui.lastRunKey = run.id;
      ui.openTraceKeys.clear();
    }

    const cls = statusClass(run.status);
    $('status-led').className = `status-led ${run.active ? 'active' : cls === 'teal' ? 'active' : cls}`;
    $('status-headline').textContent = `${run.phase || run.status} · ${run.request?.robot_id || 'robot'}`;
    $('status-subline').textContent = run.result?.detail || run.request?.instruction || run.id;

    const typeChip = $('run-type-chip');
    typeChip.hidden = false;
    typeChip.textContent = run.mode === 'policy'
      ? `POLICY (${(run.policy?.backend || 'live').toUpperCase()})`
      : run.mode.toUpperCase();

    const badge = $('status-badge');
    badge.textContent = run.status;
    badge.className = `chip ${cls}`;
    $('uncertain-alert').hidden = run.status !== 'uncertain_execution';

    const events = run.events || [];
    const lastElapsed = events.length ? events[events.length - 1].elapsed_s : (run.result?.elapsed_s ?? 0);
    $('metric-elapsed').textContent = formatDuration(Number(lastElapsed) || 0);

    if (run.mode === 'policy' && run.policy) {
      const stagesCount = (run.policy.stages || []).length;
      const callsCount = (run.policy.trace || []).length;
      $('metric-actions-label').textContent = 'Stages / Calls';
      $('metric-actions').textContent = `${stagesCount} stages · ${callsCount} calls`;
      const outcome = run.policy.outcome;
      $('metric-steps').textContent = outcome
        ? `task_success=${outcome.task_success ? 'yes' : 'no'}`
        : `${events.filter((e) => e.event === 'decision').length} stage steps`;
    } else {
      const limits = run.request?.limits || { max_actions: 20, max_steps: 40 };
      const actUsed = run.result?.action_attempts ?? events.filter((e) => e.event === 'action_prepared').length;
      const stepsUsed = run.result?.steps ?? events.filter((e) => e.event === 'decision').length;
      $('metric-actions-label').textContent = 'Actions';
      $('metric-actions').textContent = `${actUsed} / ${limits.max_actions}`;
      $('metric-steps').textContent = `${stepsUsed} / ${limits.max_steps}`;
    }
    $('metric-obs').textContent = run.current_observation?.observation_id || '—';

    renderCamerasAndJoints(run);

    if (run.mode === 'policy' && run.policy) {
      $('policy-inspector').hidden = false;
      renderStagePills(run.policy);
      $('policy-cursor-pre').textContent = run.policy.active_highlighted || run.policy.source || '—';
      const outParts = [];
      if (run.policy.stdout) outParts.push(`# stdout\n${run.policy.stdout.trim()}`);
      if (run.policy.report !== undefined && run.policy.report !== null) {
        outParts.push(`# report\n${JSON.stringify(run.policy.report, null, 2)}`);
      }
      if (run.policy.outcome) {
        outParts.push(`# outcome\n${JSON.stringify(run.policy.outcome, null, 2)}`);
      }
      $('policy-output-pre').textContent = outParts.length ? outParts.join('\n\n') : '—';
    } else {
      $('policy-inspector').hidden = true;
    }

    renderDecisionTimeline(events, run);
    renderExpandableTrace(run);
  }

  async function refresh() {
    try {
      const state = await apiGet('/api/state');
      ui.serverState = state;
      renderHeader(state);
      renderMemory(state.memory);

      const targetId = ui.pinnedRunId || state.active_run_id || (state.runs?.[0]?.id ?? null);
      let runView = null;
      if (targetId) {
        const query = ui.selectedStage ? `?stage=${encodeURIComponent(ui.selectedStage)}` : '';
        if (!ui.selectedStage && state.active && state.active.id === targetId) {
          runView = state.active;
        } else {
          runView = await apiGet(`/api/runs/${encodeURIComponent(targetId)}${query}`).catch(() => null);
        }
      }
      ui.selectedRun = runView;
      renderHistory(state, targetId);
      renderRunView(runView, state);
    } catch (err) {
      $('conn-dot').className = 'dot offline';
      $('conn-text').textContent = 'Offline';
    }
  }

  async function stopActiveRun() {
    const activeId = ui.serverState?.active_run_id;
    if (!activeId) return;
    showAlert('');
    try {
      await apiPost(`/api/runs/${encodeURIComponent(activeId)}/stop`, {});
      await refresh();
    } catch (err) {
      showAlert(err.message);
    }
  }

  function wireEvents() {
    $('policy-source').value = VISUAL_POLICY_TEMPLATE;

    $('tab-direct').addEventListener('click', () => setTab('direct'));
    $('tab-policy').addEventListener('click', () => setTab('policy'));

    $('cam-mode-current').addEventListener('click', () => setCamMode('current'));
    $('cam-mode-compare').addEventListener('click', () => setCamMode('compare'));

    document.querySelectorAll('[data-filter]').forEach((btn) => {
      btn.addEventListener('click', () => {
        ui.traceFilter = btn.dataset.filter;
        document.querySelectorAll('[data-filter]').forEach((b) => b.classList.toggle('active', b === btn));
        if (ui.selectedRun) renderExpandableTrace(ui.selectedRun);
      });
    });

    $('preset-visual-btn').addEventListener('click', () => {
      $('policy-source').value = VISUAL_POLICY_TEMPLATE;
    });
    $('preset-memory-btn').addEventListener('click', () => {
      $('policy-source').value = MEMORY_POLICY_TEMPLATE;
    });
    $('preset-skill-btn').addEventListener('click', () => {
      $('policy-source').value = SKILL_POLICY_TEMPLATE;
    });
    $('show-all-plans').addEventListener('change', (ev) => {
      ui.showAllPlans = ev.target.checked;
      if (ui.selectedRun) renderDecisionTimeline(ui.selectedRun.events || [], ui.selectedRun);
    });
    $('timeline-bottom-btn').addEventListener('click', () => {
      const list = $('decision-timeline');
      list.scrollTop = list.scrollHeight;
    });

    for (const id of ['max-actions', 'max-steps', 'max-seconds']) {
      $(id).addEventListener('input', updateBudgetSummary);
    }

    $('return-live-btn').addEventListener('click', () => {
      ui.pinnedRunId = null;
      ui.selectedStage = null;
      ui.selectedStepIdx = null;
      refresh();
    });

    $('refresh-btn').addEventListener('click', () => refresh());

    $('observe-once-btn').addEventListener('click', async () => {
      showAlert('');
      $('observe-once-btn').disabled = true;
      try {
        const snap = await apiPost('/api/kinematics/capture', {});
        ui.pinnedRunId = snap.run_id;
        ui.selectedStage = null;
        ui.selectedStepIdx = null;
        await refresh();
      } catch (err) {
        showAlert(err.message);
      } finally {
        await refresh();
      }
    });

    $('direct-form').addEventListener('submit', async (ev) => {
      ev.preventDefault();
      showAlert('');
      const modelVal = $('direct-model').value.trim();
      const payload = {
        instruction: $('direct-instruction').value.trim(),
        robot_id: $('direct-robot-id').value.trim(),
        model: modelVal || null,
        limits: {
          max_actions: Number($('max-actions').value),
          max_steps: Number($('max-steps').value),
          max_seconds: Number($('max-seconds').value),
        },
        execute: true,
      };
      try {
        ui.pinnedRunId = null;
        ui.selectedStage = null;
        ui.selectedStepIdx = null;
        await apiPost('/api/runs', payload);
        await refresh();
      } catch (err) {
        showAlert(err.message);
      }
    });

    $('policy-form').addEventListener('submit', async (ev) => {
      ev.preventDefault();
      showAlert('');
      const modelVal = $('direct-model').value.trim();
      const payload = {
        instruction: $('policy-instruction').value.trim(),
        policy_source: $('policy-source').value,
        robot_id: $('policy-robot-id').value.trim(),
        model: modelVal || null,
        backend: $('policy-backend').value,
        execute: true,
      };
      try {
        ui.pinnedRunId = null;
        ui.selectedStage = null;
        ui.selectedStepIdx = null;
        await apiPost('/api/policy/run', payload);
        await refresh();
      } catch (err) {
        showAlert(err.message);
      }
    });

    $('direct-stop-btn').addEventListener('click', stopActiveRun);
    $('policy-stop-btn').addEventListener('click', stopActiveRun);
  }

  document.addEventListener('DOMContentLoaded', () => {
    wireEvents();
    refresh();
    ui.pollTimer = window.setInterval(refresh, 1200);
  });
})();
