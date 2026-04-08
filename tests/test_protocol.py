from __future__ import annotations

import unittest

from m20control.protocol import AsduType, FrameDecoder, encode_json_frame


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


if __name__ == "__main__":
    unittest.main()
