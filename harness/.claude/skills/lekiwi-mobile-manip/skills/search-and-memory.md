# Search and Spatial-Semantic Memory (`ChromaDB`)

## Query Phrasing and Score Filtering

**Trigger:** Looking up an object's last observed `(floor, x_m, y_m, yaw_deg)` in
`SemanticMemoryStore`.

**Contrast:** Overly long imperative sentences dilute n-gram token overlap in
`LocalMultilingualEmbeddingFunction`; concise noun phrases with color/type/container tokens retrieve
more reliably.

```python
mem = search_memory("red ceramic mug pantry", top_k=3)
hits = mem["hits"]
best = hits[0] if len(hits) > 0 and hits[0]["score"] >= 0.15 else None
report["best_hit"] = best
```

**Provenance:** **unverified (reference prose)** — `SemanticMemoryStore` cosine-similarity contract.

## Invalidating Displaced Objects with `mark_absent`

**Trigger:** Robot navigates to `best["floor"], best["x_m"], best["y_m"]` and `inspect_scene` or
`act` confirms the object is no longer there.

```python
if best is not None and not obs["ok"]:
    mark_absent(best["record_id"], floor=best["floor"], note="absent_after_visual_inspection")
```

**Provenance:** **unverified (reference prose)** — prevents stale spatial memory loops.

## Recording Newly Discovered Landmarks and Objects

**Trigger:** `inspect_scene` or `act` visually identifies task-relevant objects while `get_pose()`
has valid localization (`confidence > 0.0`).

```python
pose = get_pose()
if pose["confidence"] > 0.0 and obs["ok"]:
    save_memory(
        "blue vitamin bottle on first-floor side table",
        floor=pose["floor"],
        x_m=pose["x_m"],
        y_m=pose["y_m"],
        yaw_deg=pose["yaw_deg"],
        tags=["bottle", "blue"],
    )
```

**Provenance:** **unverified (reference prose)** — spatial memory write-back contract.

## Visual Confirmation Before Committing Manipulation Budget

**Trigger:** Arriving at a coarse room or waypoint prior to running a 12-action `act()` stage.

**Contrast:** Launching a full 120 s `act()` stage on an empty counter burns steps turning and
probing; a 2-action `inspect_scene()` confirms presence cheaply first.

```python
obs = inspect_scene("Is the target object visible on the counter or floor ahead?", max_actions=2, max_steps=6)
report["visible_check"] = obs
```

**Provenance:** **unverified (reference prose)** — budget-preserving visual gate.

