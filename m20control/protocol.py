from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Any


class AsduType(IntEnum):
    JSON = 0x01
    XML = 0x02


SYNC_BYTES = b"\xeb\x90\xeb\x90"
ALT_SYNC_BYTES = b"\xeb\x91\xeb\x90"
HEADER_STRUCT = struct.Struct("<4sHH8s")
HEADER_SIZE = HEADER_STRUCT.size


@dataclass(frozen=True)
class Frame:
    message_id: int
    asdu_type: int
    raw_frame: bytes
    payload_raw: bytes
    payload: dict[str, Any] | None
    decode_error: str | None = None


def encode_json_frame(payload: dict[str, Any], message_id: int) -> bytes:
    payload_bytes = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    body = bytes([AsduType.JSON]) + payload_bytes
    header = HEADER_STRUCT.pack(SYNC_BYTES, len(body), message_id & 0xFFFF, b"\x00" * 8)
    return header + body


def _valid_sync(prefix: bytes) -> bool:
    return prefix in (SYNC_BYTES, ALT_SYNC_BYTES)


class FrameDecoder:
    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []

        while True:
            if len(self._buffer) < HEADER_SIZE:
                break

            if not _valid_sync(bytes(self._buffer[:4])):
                sync_index = self._find_next_sync()
                if sync_index < 0:
                    self._buffer.clear()
                    break
                del self._buffer[:sync_index]
                if len(self._buffer) < HEADER_SIZE:
                    break

            sync, body_len, message_id, _reserved = HEADER_STRUCT.unpack(self._buffer[:HEADER_SIZE])
            if not _valid_sync(sync):
                del self._buffer[0]
                continue

            total_size = HEADER_SIZE + body_len
            if len(self._buffer) < total_size:
                break

            raw_frame = bytes(self._buffer[:total_size])
            body = raw_frame[HEADER_SIZE:total_size]
            del self._buffer[:total_size]
            if not body:
                continue

            asdu_type = body[0]
            payload_raw = body[1:]
            payload: dict[str, Any] | None
            decode_error: str | None
            try:
                payload = self._decode_payload(asdu_type, payload_raw)
                decode_error = None
            except ValueError as exc:
                payload = None
                decode_error = str(exc)
            frames.append(
                Frame(
                    message_id=message_id,
                    asdu_type=asdu_type,
                    raw_frame=raw_frame,
                    payload_raw=payload_raw,
                    payload=payload,
                    decode_error=decode_error,
                )
            )

        return frames

    def _find_next_sync(self) -> int:
        for prefix in (SYNC_BYTES, ALT_SYNC_BYTES):
            index = self._buffer.find(prefix, 1)
            if index >= 0:
                return index
        return -1

    @staticmethod
    def _decode_payload(asdu_type: int, payload_raw: bytes) -> dict[str, Any]:
        if asdu_type != AsduType.JSON:
            raise ValueError(f"unsupported ASDU type: {asdu_type}")
        try:
            text = payload_raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError(f"invalid utf-8 payload for ASDU type {asdu_type}: {exc}") from exc
        try:
            return json.loads(text)
        except json.JSONDecodeError as exc:
            raise ValueError(f"invalid JSON payload: {exc.msg}") from exc
