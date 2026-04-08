from __future__ import annotations

import unittest

from m20control.protocol import AsduType, HEADER_STRUCT, SYNC_BYTES, FrameDecoder, encode_json_frame


class ProtocolTest(unittest.TestCase):
    def test_encode_and_decode_json_frame(self) -> None:
        payload = {"PatrolDevice": {"Type": 100, "Command": 100, "Time": "2026-04-08 12:00:00", "Items": {}}}
        frame_bytes = encode_json_frame(payload, message_id=7)

        decoder = FrameDecoder()
        frames = decoder.feed(frame_bytes)
        self.assertEqual(len(frames), 1)
        frame = frames[0]
        self.assertEqual(frame.message_id, 7)
        self.assertEqual(frame.asdu_type, AsduType.JSON)
        self.assertEqual(frame.raw_frame, frame_bytes)
        self.assertEqual(frame.payload, payload)

    def test_decoder_handles_tcp_fragmentation(self) -> None:
        payload = {"PatrolDevice": {"Type": 2, "Command": 21, "Time": "2026-04-08 12:00:00", "Items": {"X": 0.1}}}
        frame_bytes = encode_json_frame(payload, message_id=9)

        decoder = FrameDecoder()
        self.assertEqual(decoder.feed(frame_bytes[:10]), [])
        self.assertEqual(decoder.feed(frame_bytes[10:20]), [])
        frames = decoder.feed(frame_bytes[20:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].payload["PatrolDevice"]["Items"]["X"], 0.1)

    def test_decoder_returns_unknown_asdu_frame_without_raising(self) -> None:
        payload_raw = b"XYZ123"
        body = bytes([0x58]) + payload_raw
        frame_bytes = HEADER_STRUCT.pack(SYNC_BYTES, len(body), 11, b"\x00" * 8) + body

        decoder = FrameDecoder()
        frames = decoder.feed(frame_bytes)

        self.assertEqual(len(frames), 1)
        frame = frames[0]
        self.assertEqual(frame.message_id, 11)
        self.assertEqual(frame.asdu_type, 0x58)
        self.assertEqual(frame.raw_frame, frame_bytes)
        self.assertIsNone(frame.payload)
        self.assertEqual(frame.decode_error, "unsupported ASDU type: 88")

    def test_decoder_returns_invalid_json_frame_without_raising(self) -> None:
        body = bytes([AsduType.JSON]) + b"{bad json"
        frame_bytes = HEADER_STRUCT.pack(SYNC_BYTES, len(body), 12, b"\x00" * 8) + body

        decoder = FrameDecoder()
        frames = decoder.feed(frame_bytes)

        self.assertEqual(len(frames), 1)
        frame = frames[0]
        self.assertEqual(frame.message_id, 12)
        self.assertEqual(frame.asdu_type, AsduType.JSON)
        self.assertEqual(frame.raw_frame, frame_bytes)
        self.assertIsNone(frame.payload)
        self.assertTrue(frame.decode_error.startswith("invalid JSON payload:"))


if __name__ == "__main__":
    unittest.main()
