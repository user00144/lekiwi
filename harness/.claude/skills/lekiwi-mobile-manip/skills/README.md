# Shared LeKiwi Mobile Manipulation Skills

Empirical episodic workflows and failure-contrasted operational skills for the LeKiwi mobile
manipulator (3-wheel holonomic base + 5-DoF SO-101 arm + dual OV5648 USB cameras + TI AWR2944 radar +
BMI160 IMU). Campaign orchestration lives one directory up in [../SKILL.md](../SKILL.md) and
[../main-agent-prompt.md](../main-agent-prompt.md).

| Topic File | Purpose |
|---|---|
| [episodic-workflows.md](episodic-workflows.md) | Coarse task progression & multi-stage block structures (*"For task class X, sequence stages A -> B -> C"*) |
| [search-and-memory.md](search-and-memory.md) | ChromaDB spatial-semantic retrieval, `mark_absent` invalidation, visual confirmation, `save_memory` updates |
| [navigation.md](navigation.md) | `relocalize()` confidence checks, multi-floor `TopologicalMap` ramp routing, SLAM vs. visual-only branching |
| [mobile-grasp.md](mobile-grasp.md) | Coupled holonomic base + 5-DoF `arm_base` IK reach recovery, `motion_warning` / reconciliation, lift checks |
| [time-budget.md](time-budget.md) | Action, step, and wall-clock budgets for `inspect_scene` and `act`, plus block-replay cost control |
| [lekiwi-operations.md](lekiwi-operations.md) | Guarded `observation_id` leases, `arm_current_out_of_calibration`, camera views, `uncertain_execution` stops |

## How These Files Are Used in a Campaign

These six topic files define the **skeleton schema** of a campaign's `skill-library/`. At the start
of a campaign's first task, run:

```bash
uv run harness-campaign init-library --dest "$RUN_ROOT/lekiwi/skill-library"
```

That command extracts **only `#` and `##` headings** from the six topic files (ignoring code fences
and prose), seeding `$RUN_ROOT/lekiwi/skill-library/` with empty sections so zero pre-baked priors
reach Stage 1 or Stage 2.

During **Stage 1**, per-trial workers populate `skill-library/*.md` with empirical discoveries.
Before **Stage 2**, the coordinator snapshots `skill-library/` into `skill-library-frozen/` and
locks its SHA-256 manifest (`frozen-manifest.sha256`). Durable cross-campaign patterns are promoted
back into this directory **only between campaigns**, never while a campaign is running.

## Entry Schema

Every promoted entry must contain four elements:

1. **Trigger**: Observable symptom, task class, or return dictionary condition.
2. **Contrast**: What failed first (with measured status, error code, or camera observation) and why.
3. **Snippet**: Runnable `policy.py` code using only the public `harness-campaign` primitives
   (`import math` only, no `_`-prefixed identifiers, `result = report` at end of file).
4. **Provenance**: `task=<slug>, trial=<N>, attempt=<attempt_NNN>, date=<YYYY-MM-DD>`, plus whether
   the rule is a **transferable harness/embodiment invariant** or a **task-specific hypothesis**.

Reference notes shipped before empirical campaign promotion are explicitly marked
**unverified (reference prose)**.

