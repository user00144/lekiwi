"""ChromaDB spatial semantic map (`language query -> floor, x_m, y_m, yaw_deg`)."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
import re
import time
from typing import Any
from uuid import uuid4

import chromadb
from chromadb.api.types import Documents, EmbeddingFunction, Embeddings
import numpy as np


DEFAULT_EMBED_DIM = 512
_TOKEN_RE = re.compile(r'[0-9A-Za-z\uac00-\ud7a3_]+')


def _finite_float(value: float, label: str) -> float:
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{label} must be a finite number')
    return float(value)


class LocalMultilingualEmbeddingFunction(EmbeddingFunction[Documents]):
    """Deterministic offline char/word n-gram embedding for ChromaDB."""

    def __init__(self, dim: int = DEFAULT_EMBED_DIM):
        if type(dim) is not int or dim < 32:
            raise ValueError('dim must be an integer >= 32')
        self.dim = dim

    @staticmethod
    def name() -> str:
        return 'lekiwi_local_multilingual_ngram'

    def get_config(self) -> dict[str, Any]:
        return {'dim': self.dim}

    @classmethod
    def build_from_config(cls, config: dict[str, Any]) -> 'LocalMultilingualEmbeddingFunction':
        return cls(dim=int(config.get('dim', DEFAULT_EMBED_DIM)))

    def _embed_one(self, text: str) -> np.ndarray:
        normalized = str(text).strip().lower()
        vector = np.zeros(self.dim, dtype=np.float32)
        if not normalized:
            vector[0] = 1.0
            return vector

        tokens = _TOKEN_RE.findall(normalized)
        features: list[tuple[str, float]] = []
        for token in tokens:
            features.append((f'tok:{token}', 2.0))
            for n in (2, 3):
                if len(token) >= n:
                    for idx in range(len(token) - n + 1):
                        features.append((f'c{n}:{token[idx:idx + n]}', 1.0))
        for idx in range(len(tokens) - 1):
            features.append((f'bi:{tokens[idx]}_{tokens[idx + 1]}', 1.5))

        for feature, weight in features:
            digest = hashlib.blake2b(feature.encode('utf-8'), digest_size=8).digest()
            raw = int.from_bytes(digest, 'little')
            bucket = raw % self.dim
            sign = 1.0 if ((raw >> 63) & 1) == 0 else -1.0
            vector[bucket] += sign * weight

        norm = float(np.linalg.norm(vector))
        if norm > 0.0:
            vector /= norm
        else:
            vector[0] = 1.0
        return vector

    def __call__(self, input: Documents) -> Embeddings:
        return [self._embed_one(doc) for doc in input]


@dataclass(frozen=True)
class SemanticHit:
    record_id: str
    score: float
    text: str
    floor: str
    x_m: float
    y_m: float
    yaw_deg: float
    room: str = ''
    tags: tuple[str, ...] = ()
    status: str = 'present'
    robot_id: str = 'kiwi-01'
    observed_at: float = 0.0
    # False when saved without a global pose (SLAM-free): x_m/y_m are then not a map position.
    localized: bool = True

    def to_dict(self) -> dict[str, object]:
        payload = asdict(self)
        payload['tags'] = list(self.tags)
        return payload


def create_chroma_client(persist_dir: str | Path | None = None):
    if persist_dir is None:
        return chromadb.EphemeralClient()
    target = Path(persist_dir)
    target.mkdir(parents=True, exist_ok=True)
    return chromadb.PersistentClient(path=str(target))


class SemanticMemoryStore:
    """ChromaDB collection mapping scene/object descriptions to `(floor, x_m, y_m, yaw_deg)`."""

    COLLECTION_NAME = 'lekiwi_semantic_memory'

    def __init__(
        self,
        persist_dir: str | Path | None = None,
        *,
        client=None,
        embedding_function: EmbeddingFunction[Documents] | None = None,
        collection_name: str | None = None,
    ):
        self._client = client if client is not None else create_chroma_client(persist_dir)
        if collection_name is None:
            # chromadb.EphemeralClient() shares one in-process store, so a fixed name would
            # leak records between "fresh" in-memory stores (tests, repeated GUI sim runs).
            ephemeral = client is None and persist_dir is None
            collection_name = f'{self.COLLECTION_NAME}_{uuid4().hex[:12]}' if ephemeral else self.COLLECTION_NAME
        self._embedding_fn = embedding_function or LocalMultilingualEmbeddingFunction()
        self._collection = self._client.get_or_create_collection(
            name=collection_name,
            embedding_function=self._embedding_fn,
            metadata={'hnsw:space': 'cosine'},
        )

    def count(self) -> int:
        return int(self._collection.count())

    @staticmethod
    def _build_document(text: str, floor: str, room: str, tags: tuple[str, ...]) -> str:
        parts = [floor, room, text, *tags]
        return ' '.join(p for p in parts if p).strip()

    def upsert(
        self,
        text: str,
        *,
        floor: str,
        x_m: float,
        y_m: float,
        yaw_deg: float = 0.0,
        room: str = '',
        tags: tuple[str, ...] | list[str] = (),
        status: str = 'present',
        robot_id: str = 'kiwi-01',
        record_id: str | None = None,
        observed_at: float | None = None,
        localized: bool = True,
    ) -> SemanticHit:
        if not isinstance(text, str) or not text.strip():
            raise ValueError('text must be a non-empty string')
        if not isinstance(floor, str) or not floor.strip():
            raise ValueError('floor must be a non-empty string')
        if status not in ('present', 'absent'):
            raise ValueError("status must be 'present' or 'absent'")

        clean_text = text.strip()
        clean_floor = floor.strip()
        clean_room = str(room).strip()
        clean_tags = tuple(str(t).strip() for t in tags if str(t).strip())
        rid = record_id or f'mem_{uuid4().hex[:12]}'
        ts = time.time() if observed_at is None else _finite_float(observed_at, 'observed_at')
        x_val = _finite_float(x_m, 'x_m')
        y_val = _finite_float(y_m, 'y_m')
        yaw_val = _finite_float(yaw_deg, 'yaw_deg')

        metadata = {
            'raw_text': clean_text,
            'floor': clean_floor,
            'x_m': x_val,
            'y_m': y_val,
            'yaw_deg': yaw_val,
            'room': clean_room,
            'tags_csv': ','.join(clean_tags),
            'status': status,
            'robot_id': str(robot_id),
            'observed_at': ts,
            'localized': bool(localized),
        }
        doc = self._build_document(clean_text, clean_floor, clean_room, clean_tags)
        self._collection.upsert(ids=[rid], documents=[doc], metadatas=[metadata])
        return SemanticHit(
            record_id=rid,
            score=1.0,
            text=clean_text,
            floor=clean_floor,
            x_m=x_val,
            y_m=y_val,
            yaw_deg=yaw_val,
            room=clean_room,
            tags=clean_tags,
            status=status,
            robot_id=str(robot_id),
            observed_at=ts,
            localized=bool(localized),
        )

    @staticmethod
    def _where_clause(floor: str | None, include_absent: bool) -> dict[str, Any] | None:
        clauses: list[dict[str, Any]] = []
        if floor is not None:
            clauses.append({'floor': str(floor)})
        if not include_absent:
            clauses.append({'status': 'present'})
        if not clauses:
            return None
        if len(clauses) == 1:
            return clauses[0]
        return {'$and': clauses}

    @staticmethod
    def _meta_to_hit(rid: str, score: float, meta: dict[str, Any]) -> SemanticHit:
        tags_csv = str(meta.get('tags_csv', '')).strip()
        tags = tuple(t for t in tags_csv.split(',') if t) if tags_csv else ()
        return SemanticHit(
            record_id=rid,
            score=round(score, 4),
            text=str(meta.get('raw_text', '')),
            floor=str(meta.get('floor', '')),
            x_m=float(meta['x_m']),
            y_m=float(meta['y_m']),
            yaw_deg=float(meta.get('yaw_deg', 0.0)),
            room=str(meta.get('room', '')),
            tags=tags,
            status=str(meta.get('status', 'present')),
            robot_id=str(meta.get('robot_id', 'kiwi-01')),
            observed_at=float(meta.get('observed_at', 0.0)),
            localized=bool(meta.get('localized', True)),
        )

    def search(
        self,
        query: str,
        *,
        floor: str | None = None,
        top_k: int = 3,
        include_absent: bool = False,
        min_score: float = 0.05,
    ) -> list[SemanticHit]:
        if not isinstance(query, str) or not query.strip():
            raise ValueError('query must be a non-empty string')
        if type(top_k) is not int or top_k <= 0:
            raise ValueError('top_k must be a positive integer')
        total = self.count()
        if total == 0:
            return []

        where = self._where_clause(floor, include_absent)
        result = self._collection.query(
            query_texts=[query.strip()],
            n_results=min(max(top_k * 3, top_k), total),
            where=where,
            include=['metadatas', 'distances'],
        )
        ids = (result.get('ids') or [[]])[0]
        metas = (result.get('metadatas') or [[]])[0]
        distances = (result.get('distances') or [[]])[0]

        hits: list[SemanticHit] = []
        for rid, meta, dist in zip(ids, metas, distances):
            similarity = max(-1.0, min(1.0, 1.0 - float(dist)))
            if similarity < min_score:
                continue
            hits.append(self._meta_to_hit(str(rid), similarity, dict(meta)))

        hits.sort(key=lambda h: (h.score, h.observed_at), reverse=True)
        return hits[:top_k]

    def mark_absent(self, record_id_or_query: str, *, floor: str | None = None, note: str = 'not_found') -> int:
        if not isinstance(record_id_or_query, str) or not record_id_or_query.strip():
            raise ValueError('record_id_or_query must be a non-empty string')
        existing = self._collection.get(ids=[record_id_or_query], include=['metadatas', 'documents'])
        target_ids = list(existing.get('ids') or [])
        target_metas = list(existing.get('metadatas') or [])
        target_docs = list(existing.get('documents') or [])

        if not target_ids:
            matches = self.search(record_id_or_query, floor=floor, top_k=1, include_absent=False)
            if not matches:
                return 0
            fetched = self._collection.get(ids=[matches[0].record_id], include=['metadatas', 'documents'])
            target_ids = list(fetched.get('ids') or [])
            target_metas = list(fetched.get('metadatas') or [])
            target_docs = list(fetched.get('documents') or [])

        updated = 0
        for rid, meta, doc in zip(target_ids, target_metas, target_docs):
            if floor is not None and meta.get('floor') != floor:
                continue
            new_meta = dict(meta)
            new_meta['status'] = 'absent'
            new_meta['absent_note'] = str(note)
            new_meta['observed_at'] = time.time()
            self._collection.upsert(ids=[rid], documents=[doc], metadatas=[new_meta])
            updated += 1
        return updated
