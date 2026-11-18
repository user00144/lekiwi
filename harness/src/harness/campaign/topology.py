"""Multi-floor topological waypoint graph and SLAM localization contracts."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import heapq
import json
import math
from pathlib import Path
from typing import Any, Protocol

from ..episode import atomic_json


def _finite_float(value: float, label: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    return float(value)


@dataclass(frozen=True)
class PoseEstimate:
    floor: str
    x_m: float
    y_m: float
    yaw_deg: float = 0.0
    confidence: float = 1.0
    relocalized: bool = True

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


class LocalizationProvider(Protocol):
    def get_pose(self, robot_id: str) -> PoseEstimate | Any: ...
    def relocalize(self, robot_id: str) -> PoseEstimate | Any: ...


@dataclass(frozen=True)
class WaypointNode:
    node_id: str
    floor: str
    x_m: float
    y_m: float
    yaw_deg: float = 0.0
    kind: str = 'waypoint'
    aliases: tuple[str, ...] = ()


@dataclass(frozen=True)
class WaypointEdge:
    src: str
    dst: str
    via: str = 'drive'
    cost_m: float | None = None
    bidirectional: bool = True


@dataclass(frozen=True)
class RouteLeg:
    floor: str
    x_m: float
    y_m: float
    yaw_deg: float
    via: str
    node_id: str | None = None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class RoutePlan:
    ok: bool
    legs: tuple[RouteLeg, ...] = ()
    total_cost_m: float = 0.0
    floors_traversed: tuple[str, ...] = ()
    error: str | None = None

    def to_dict(self) -> dict[str, object]:
        return {
            'ok': self.ok,
            'legs': [leg.to_dict() for leg in self.legs],
            'total_cost_m': round(self.total_cost_m, 3),
            'floors_traversed': list(self.floors_traversed),
            'error': self.error,
        }


class TopologicalMap:
    """Multi-floor navigable waypoint graph separating geometry from ChromaDB."""

    def __init__(self):
        self.nodes: dict[str, WaypointNode] = {}
        self._adj: dict[str, list[tuple[str, str, float]]] = {}
        self._edges: list[WaypointEdge] = []

    def add_node(
        self,
        node_id: str,
        *,
        floor: str,
        x_m: float,
        y_m: float,
        yaw_deg: float = 0.0,
        kind: str = 'waypoint',
        aliases: tuple[str, ...] | list[str] = (),
    ) -> WaypointNode:
        node = WaypointNode(
            node_id=str(node_id),
            floor=str(floor),
            x_m=_finite_float(x_m, 'x_m'),
            y_m=_finite_float(y_m, 'y_m'),
            yaw_deg=_finite_float(yaw_deg, 'yaw_deg'),
            kind=str(kind),
            aliases=tuple(str(a).strip() for a in aliases if str(a).strip()),
        )
        self.nodes[node.node_id] = node
        self._adj.setdefault(node.node_id, [])
        return node

    def add_edge(
        self,
        src: str,
        dst: str,
        *,
        via: str = 'drive',
        cost_m: float | None = None,
        bidirectional: bool = True,
    ) -> None:
        if src not in self.nodes or dst not in self.nodes:
            raise KeyError(f'Both nodes must exist before linking: {src!r} -> {dst!r}')
        a, b = self.nodes[src], self.nodes[dst]
        if cost_m is None:
            base = math.hypot(b.x_m - a.x_m, b.y_m - a.y_m)
            cost = max(0.3, base) if a.floor == b.floor else max(5.0, base + 5.0)
        else:
            cost = _finite_float(cost_m, 'cost_m')
            if cost <= 0:
                raise ValueError('cost_m must be positive')
        edge = WaypointEdge(src=src, dst=dst, via=via, cost_m=cost, bidirectional=bidirectional)
        self._edges.append(edge)
        self._adj[src].append((dst, via, cost))
        if bidirectional:
            self._adj[dst].append((src, via, cost))

    def ingest_trajectory(
        self,
        floor: str,
        poses_xy_yaw: list[tuple[float, float, float]],
        *,
        spacing_m: float = 0.8,
        prefix: str | None = None,
    ) -> list[str]:
        if spacing_m <= 0:
            raise ValueError('spacing_m must be positive')
        tag = prefix or f'{floor}_traj'
        created: list[str] = []
        last_xy: tuple[float, float] | None = None
        for idx, (x_m, y_m, yaw_deg) in enumerate(poses_xy_yaw):
            if last_xy is not None and idx < len(poses_xy_yaw) - 1:
                if math.hypot(x_m - last_xy[0], y_m - last_xy[1]) < spacing_m:
                    continue
            nid = f'{tag}_{len(created):03d}'
            self.add_node(nid, floor=floor, x_m=x_m, y_m=y_m, yaw_deg=yaw_deg)
            if created:
                self.add_edge(created[-1], nid, via='drive')
            created.append(nid)
            last_xy = (x_m, y_m)
        return created

    def find_place(self, name: str, *, floor: str | None = None) -> WaypointNode | None:
        key = name.strip().lower()
        for node in self.nodes.values():
            if floor is not None and node.floor != floor:
                continue
            candidates = (node.node_id.lower(), *(alias.lower() for alias in node.aliases))
            if any(key == cand or key in cand for cand in candidates):
                return node
        return None

    def nearest_node(self, floor: str, x_m: float, y_m: float) -> WaypointNode | None:
        same_floor = [node for node in self.nodes.values() if node.floor == floor]
        if not same_floor:
            return None
        return min(same_floor, key=lambda n: math.hypot(n.x_m - x_m, n.y_m - y_m))

    def plan_route(
        self,
        start_floor: str,
        start_x_m: float,
        start_y_m: float,
        target_floor: str,
        target_x_m: float,
        target_y_m: float,
        *,
        target_yaw_deg: float = 0.0,
    ) -> RoutePlan:
        sx = _finite_float(start_x_m, 'start_x_m')
        sy = _finite_float(start_y_m, 'start_y_m')
        tx = _finite_float(target_x_m, 'target_x_m')
        ty = _finite_float(target_y_m, 'target_y_m')
        tyaw = _finite_float(target_yaw_deg, 'target_yaw_deg')

        if not self.nodes and start_floor == target_floor:
            dist = math.hypot(tx - sx, ty - sy)
            return RoutePlan(
                ok=True,
                legs=(RouteLeg(floor=target_floor, x_m=tx, y_m=ty, yaw_deg=tyaw, via='drive'),),
                total_cost_m=dist,
                floors_traversed=(start_floor,),
            )

        start_node = self.nearest_node(start_floor, sx, sy)
        goal_node = self.nearest_node(target_floor, tx, ty)
        if start_node is None or goal_node is None:
            return RoutePlan(
                ok=False,
                error=f'No navigable waypoint nodes on floor {start_floor!r} or {target_floor!r}',
            )

        pq: list[tuple[float, str]] = [(0.0, start_node.node_id)]
        dist_map: dict[str, float] = {start_node.node_id: 0.0}
        prev_map: dict[str, tuple[str, str]] = {}

        while pq:
            cost_so_far, current = heapq.heappop(pq)
            if current == goal_node.node_id:
                break
            if cost_so_far > dist_map.get(current, math.inf):
                continue
            for nxt, via, weight in self._adj.get(current, ()):
                cand = cost_so_far + weight
                if cand < dist_map.get(nxt, math.inf):
                    dist_map[nxt] = cand
                    prev_map[nxt] = (current, via)
                    heapq.heappush(pq, (cand, nxt))

        if goal_node.node_id not in dist_map:
            return RoutePlan(
                ok=False,
                error=f'No connected route from {start_floor} ({start_node.node_id}) to {target_floor} ({goal_node.node_id})',
            )

        chain: list[tuple[WaypointNode, str]] = []
        cursor = goal_node.node_id
        while cursor != start_node.node_id:
            parent, via = prev_map[cursor]
            chain.append((self.nodes[cursor], via))
            cursor = parent
        chain.reverse()

        legs: list[RouteLeg] = []
        total_cost = math.hypot(start_node.x_m - sx, start_node.y_m - sy)
        if total_cost > 0.05 or chain:
            legs.append(RouteLeg(
                floor=start_node.floor,
                x_m=start_node.x_m,
                y_m=start_node.y_m,
                yaw_deg=start_node.yaw_deg,
                via='drive',
                node_id=start_node.node_id,
            ))
        for node, via in chain:
            legs.append(RouteLeg(
                floor=node.floor,
                x_m=node.x_m,
                y_m=node.y_m,
                yaw_deg=node.yaw_deg,
                via=via,
                node_id=node.node_id,
            ))
        total_cost += dist_map[goal_node.node_id]
        tail_dist = math.hypot(tx - goal_node.x_m, ty - goal_node.y_m)
        if not legs or tail_dist > 0.02 or abs(tyaw - legs[-1].yaw_deg) > 1.0:
            legs.append(RouteLeg(
                floor=target_floor,
                x_m=tx,
                y_m=ty,
                yaw_deg=tyaw,
                via='drive',
                node_id=None,
            ))
            total_cost += tail_dist

        floors: list[str] = []
        for leg in legs:
            if not floors or floors[-1] != leg.floor:
                floors.append(leg.floor)

        return RoutePlan(
            ok=True,
            legs=tuple(legs),
            total_cost_m=total_cost,
            floors_traversed=tuple(floors),
        )

    def save(self, path: str | Path) -> None:
        payload = {
            'version': 1,
            'nodes': [{**asdict(node), 'aliases': list(node.aliases)} for node in self.nodes.values()],
            'edges': [asdict(edge) for edge in self._edges],
        }
        atomic_json(Path(path), payload)

    @classmethod
    def load(cls, path: str | Path) -> 'TopologicalMap':
        raw = json.loads(Path(path).read_text(encoding='utf-8'))
        topo = cls()
        for item in raw.get('nodes', []):
            topo.add_node(
                item['node_id'],
                floor=item['floor'],
                x_m=item['x_m'],
                y_m=item['y_m'],
                yaw_deg=item.get('yaw_deg', 0.0),
                kind=item.get('kind', 'waypoint'),
                aliases=item.get('aliases', ()),
            )
        for edge in raw.get('edges', []):
            topo.add_edge(
                edge['src'],
                edge['dst'],
                via=edge.get('via', 'drive'),
                cost_m=edge.get('cost_m'),
                bidirectional=bool(edge.get('bidirectional', True)),
            )
        return topo

