# Time, Action, and Replay Budgets

## Sizing `inspect_scene` vs. `act` Budgets

**Trigger:** Choosing `max_actions`, `max_steps`, and `max_seconds` on coarse policy calls.

**Contrast:** Every `EpisodeRunner` completion requires **at least two observations and two
completion claims** after the final motion, plus extra observations if `rejection_feedback` or
`reconciliation` fires. Setting `max_steps` barely above `max_actions` causes premature
`budget_steps` termination right before the second confirmation observation.

| Primitive | Recommended `max_actions` | Recommended `max_steps` | Recommended `max_seconds` |
|---|---|---|---|
| `inspect_scene` (quick check) | `2` | `6` | `45.0` |
| `act` (local approach + grasp + lift) | `12–15` | `24–30` | `120.0–150.0` |
| `act` (place + release + retract) | `8–10` | `18–22` | `90.0–120.0` |

```python
obs = inspect_scene("Check whether target mug sits on pantry counter.", max_actions=2, max_steps=6, max_seconds=45.0)
```

**Provenance:** **unverified (reference prose)** — derived from `EpisodeLimits` and two-claim completion rule.

## Keeping Early Blocks Lean Across Block-Growth Attempts

**Trigger:** Growing `policy.py` past Block 3 in `--mode sim` or reset-replay campaigns.

**Contrast:** Because every attempt runs `policy.py` from top to bottom, redundant `inspect_scene`
sweeps in Block 1 multiply wall-clock cost on every later attempt. Guard expensive fallback searches
behind `if not report.get("found"):` so nominal paths execute straight through.

**Provenance:** **unverified (reference prose)** — block-by-block replay cost rule.

