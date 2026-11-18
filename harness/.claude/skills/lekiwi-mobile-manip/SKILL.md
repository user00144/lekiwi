---
name: lekiwi-mobile-manip
description: "ASPIRE-style block-by-block policy growth and continual skill-library protocol for the LeKiwi mobile manipulator (holonomic 3-wheel base + 5-DoF SO-101 arm + dual OV5648 cameras). Stage 1 learns episodic workflows and contrastive operational skills into a shared Markdown library; Stage 2 evaluates unseen trials with the library frozen."
---

# Block-by-Block Policy Loop & Continual Skill Library on LeKiwi

Long-horizon mobile manipulation on LeKiwi cannot be authored as a single monolithic script up
front. At trial start, the target object may sit in another room or on another floor, a previously
indexed ChromaDB memory entry may be stale because someone moved the object, global SLAM may be
ambiguous or disabled, and the 5-DoF SO-101 arm's reachable workspace depends on where the
3-wheel holonomic base actually came to rest. Any block written past the latest observed physical
state is a guess.

Following the ASPIRE `behavior/` methodology, policies on LeKiwi are **grown block by block**, and
what transfers across trials and tasks is a **structured Markdown skill library** alongside a
persistent **spatial-semantic object map**, never a frozen monolithic policy:

1. Author one `# Code block N` section in `$TRIAL_DIR/policy.py`.
2. Execute `policy.py` via `harness-campaign run`, producing a structured attempt directory
   (`outcome.json`, `source/policy-result.json`, `logs/policy.stdout`, `trace/events.jsonl`, and
   per-stage `EpisodeRunner` observations under `stages/`).
3. Inspect what physically happened: current `(floor, x_m, y_m, yaw_deg)` or visual camera frames,
   ChromaDB hits, IK/swept-path rejections, `motion_warning` residuals, and gripper/lift evidence.
4. Write the next `# Code block N+1` against that observed state (or revise a faulty Stage 1 block)
   and run again until the trial succeeds or the attempt budget is exhausted.

## Two Complementary Memory Tiers

| Tier | Backing Store | Mutability in Stage 2 | What It Stores |
|---|---|---|---|
| **Spatial-Semantic Scene Map** | `ChromaDB` (`SemanticMemoryStore`) + `TopologicalMap` JSON | Live world state (`search_memory`, `save_memory`, `mark_absent`) | Where specific objects and navigable waypoints/ramps are (`query -> floor, x_m, y_m, yaw_deg`) |
| **Episodic & Skill Library** | Fixed topic `.md` files in `skill-library/` | **Frozen** (`chmod -R a-w` + `frozen-manifest.sha256`) | Reusable coarse stage workflows (`episodic-workflows.md`) and failure-contrasted operational skills (`navigation.md`, `mobile-grasp.md`, etc.) |

Keeping scene coordinates in ChromaDB/TopologicalMap and procedural know-how in topic Markdown
files avoids polluting the skill library with transient object locations while giving the coding
agent human-readable, provenance-tagged code patterns it can inspect and refine directly.

## Execution Modes: SLAM-Guided vs. SLAM-Free Visual Control

`harness-campaign run` supports three operating regimes under one uniform `policy.py` contract:

1. **Deterministic Campaign Simulation (`--mode sim`)**: Reset-and-replay multi-floor world for
   rapid block-by-block policy development, stale-memory recovery tests, and pipeline validation.
2. **SLAM-Localized Live Hardware (`--mode live --pose-file <path> --topology <path>`)**: Uses external
   pose estimates (`confidence > 0`) to plan multi-floor routes across `TopologicalMap` (`navigate_to`,
   `navigate_to_place`) before handing off to `EpisodeRunner` (`inspect_scene`, `act`).
3. **SLAM-Free Visual Live Hardware (`--mode live` without `--pose-file`)**: `get_pose()` returns
   `confidence: 0.0`. Policies skip `navigate_to` and perform visual search, local holonomic base
   approach, and 5-DoF `arm_base` Cartesian/joint grasping directly through `inspect_scene()` and
   `act()`, preserving full compatibility with direct camera-driven operation.

With `--skills-config` (and `--camera-config` for `locate`), live and sim runs also expose
deterministic **host skills** to `policy.py`: `locate(label)` (calibrated front-camera floor XYZ +
pick reachability), `pick(xyz_m)`, `place(xyz_m)`, `home_arm()` and `drive(forward_m, left_m,
turn_deg)`. They let a block state *where* to grasp in code (tunable via `report["cfg"]`) instead of
asking an inner model loop to guess metric depth from RGB; `act()` then defaults to the plan-act
inner loop built on the same skills.

## Campaign Directory Layout

```text
$RUN_ROOT/lekiwi/
├── skill-library/                   # Shared cumulative Markdown library (seeded headings-only)
├── semantic-db/                     # Optional persistent ChromaDB spatial object store
├── topology.json                    # Optional multi-floor TopologicalMap graph
└── <task_slug>/
    ├── campaign-state.md            # Coordinator ledger: trial, stage, attempts, outcome
    ├── skill-library-frozen/        # Read-only snapshot taken before Stage 2
    ├── frozen-manifest.sha256       # SHA-256 manifest verified before every Stage 2 trial
    ├── stage1/trial_01 … trial_10/
    │   ├── policy.py                # Grown block by block
    │   ├── attempts/attempt_001/ …  # Structured harness-campaign run directories
    │   └── trial-summary.md
    └── stage2/trial_01 … trial_25/  # Fresh agent per trial, frozen library, append-only
```

## Run Order

1. **Initialize empty library (first task of a campaign only)**:
   Run `uv run harness-campaign init-library --dest "$RUN_ROOT/lekiwi/skill-library"` from
   `REF/lekiwi/harness/`. This copies only `#` and `##` headings from [skills/](skills/) with
   empty bodies. Never copy reference prose or snippets from [skills/](skills/) into a campaign's
   active library; Stage 1 must earn every entry empirically.
2. **Stage 1 (Learning Trials `trial_01` .. `trial_10`)**:
   Dispatch one worker per trial using [subagent-prompt.md](subagent-prompt.md) with `STAGE: 1`.
   After each trial, fold generalizable episodic workflows and failure-contrasted skills into
   `$RUN_ROOT/lekiwi/skill-library/` with explicit provenance (`task`, `trial`, `attempt`, `date`),
   optionally testing contentious entries with [verifier-prompt.md](verifier-prompt.md).
3. **Freeze Gate**:
   Follow [main-agent-prompt.md](main-agent-prompt.md) to snapshot `skill-library/` into
   `<task_slug>/skill-library-frozen/`, strip write permissions, and record
   `frozen-manifest.sha256`.
4. **Stage 2 (Evaluation Trials `trial_01` .. `trial_25`)**:
   Dispatch one **fresh** worker per trial with `STAGE: 2`. Each worker starts with an empty
   `policy.py`, verifies `frozen-manifest.sha256`, authors **one block per attempt (append-only)**,
   and never reads another trial's directory.
5. **Score**:
   Run `./score_campaign.py "$RUN_ROOT/lekiwi"` to report the two exact final-policy metrics per
   task: **SUCCESS** and **NAVIGATION**.
6. **Reruns & Maintenance**:
   Follow [clean-task-slate.md](clean-task-slate.md) before any rerun. Promote verified campaign
   discoveries back into this folder's [skills/](skills/) reference files only between campaigns.

