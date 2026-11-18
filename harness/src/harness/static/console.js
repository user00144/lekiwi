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
    if (cfg.kinematics_configured) {
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
      ui.robotSeeded = true;
    }

    const busy = Boolean(state.active_run_id) || Boolean(state.pose_capture_busy);
    $('observe-once-btn').disabled = isDemo || busy;
    $('direct-start-btn').disabled = busy;
    $('policy-start-btn').disabled = busy;
    $('direct-stop-btn').disabled = !state.active_run_id;
    $('policy-stop-btn').disabled = !state.active_run_id;
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
      evName === 'motion_feedback'
    ) {
      return 'tools';
    }
    return 'other';
  }

  function summarizeEventOneLine(ev) {
    const kind = ev.event;
    const d = ev.data || {};
    if (kind === 'decision') {
      const cmdPart = d.command
        ? d.command.cartesian
          ? `cartesian ${d.command.cartesian.mode} [${d.command.cartesian.xyz_m?.join(', ')}]`
          : `${d.command.action} (${d.command.duration_ms}ms)`
        : '';
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
      btn.className = 'pill-btn' + (ui.selectedStage === s.name ? ' active' : '');
      btn.textContent = `${s.name} (${s.status})`;
      btn.title = s.summary || s.name;
      btn.addEventListener('click', () => {
        ui.selectedStage = s.name;
        ui.selectedStepIdx = null;
        refresh();
      });
      wrap.appendChild(btn);
    }
  }

  function renderStepReasoning(events, run) {
    const decisions = events.filter((e) => e.event === 'decision');
    const navBar = $('step-nav-bar');
    const pillsEl = $('step-pills');

    if (!decisions.length) {
      navBar.hidden = true;
      $('decision-tool-badge').textContent = run.status;
      $('decision-feedback-tag').textContent = '';
      $('decision-reason').textContent = run.mode === 'reference'
        ? 'Read-only camera & joint snapshot captured (no planner invoked).'
        : 'No inner planner decision recorded for this stage/run yet.';
      $('decision-reason').classList.add('muted');
      $('decision-evidence').textContent = '—';
      $('decision-plan').innerHTML = '<li class="muted">—</li>';
      $('raw-cmd-details').hidden = true;
      return;
    }

    navBar.hidden = false;
    pillsEl.innerHTML = '';

    const latestBtn = document.createElement('button');
    latestBtn.type = 'button';
    latestBtn.className = 'pill-btn' + (ui.selectedStepIdx === null ? ' active' : '');
    latestBtn.textContent = `Latest (#${decisions.length})`;
    latestBtn.addEventListener('click', () => {
      ui.selectedStepIdx = null;
      renderStepReasoning(events, run);
    });
    pillsEl.appendChild(latestBtn);

    decisions.forEach((decEv, idx) => {
      const b = document.createElement('button');
      b.type = 'button';
      b.className = 'pill-btn' + (ui.selectedStepIdx === idx ? ' active' : '');
      b.textContent = `#${idx + 1} ${decEv.data?.tool || 'step'}`;
      b.addEventListener('click', () => {
        ui.selectedStepIdx = idx;
        renderStepReasoning(events, run);
      });
      pillsEl.appendChild(b);
    });

    const chosenIdx = ui.selectedStepIdx !== null && ui.selectedStepIdx < decisions.length
      ? ui.selectedStepIdx
      : decisions.length - 1;
    const dec = decisions[chosenIdx].data || {};

    $('decision-tool-badge').textContent = `Step #${chosenIdx + 1} · ${dec.tool || 'decided'}`;
    $('decision-feedback-tag').textContent = dec.feedback ? `feedback: ${dec.feedback} · obs: ${dec.observation_id || '—'}` : '';
    $('decision-reason').textContent = dec.reason || '—';
    $('decision-reason').classList.remove('muted');

    const evParts = [];
    if (dec.expected) evParts.push(`Expected: ${dec.expected}`);
    if (dec.evidence) evParts.push(`Evidence: ${dec.evidence}`);
    $('decision-evidence').textContent = evParts.length ? evParts.join('\n') : '—';
    $('decision-evidence').classList.remove('muted');

    const planItems = Array.isArray(dec.plan) && dec.plan.length ? dec.plan : ['—'];
    $('decision-plan').innerHTML = '';
    for (const stepText of planItems) {
      const li = document.createElement('li');
      li.textContent = stepText;
      $('decision-plan').appendChild(li);
    }

    $('raw-cmd-details').hidden = false;
    $('raw-cmd-pre').textContent = JSON.stringify(dec, null, 2);
  }

  function renderExpandableTrace(run) {
    const events = run.events || [];
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
      $('step-nav-bar').hidden = true;
      $('decision-tool-badge').textContent = 'idle';
      $('decision-feedback-tag').textContent = '';
      $('decision-reason').textContent = 'No planner decision recorded yet.';
      $('decision-evidence').textContent = '—';
      $('decision-plan').innerHTML = '<li class="muted">—</li>';
      $('raw-cmd-details').hidden = true;
      $('event-count').textContent = '0 items';
      $('event-log').innerHTML = '<p class="empty-msg">Reasoning turns, policy tool calls, IK checks, and motor results stream here.</p>';
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

    renderStepReasoning(events, run);
    renderExpandableTrace(run);
  }

  async function refresh() {
    try {
      const state = await apiGet('/api/state');
      ui.serverState = state;
      renderHeader(state);

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
