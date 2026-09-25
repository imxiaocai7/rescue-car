import unittest

from motion_protocol import (
    ActuatorCommand,
    FRAME_SIZE,
    MotionCommand,
    MotionCommandDecoder,
    crc16_ccitt_false,
    encode_motion_command,
    lease_expired,
    sequence_is_newer,
)


class MotionProtocolTests(unittest.TestCase):
    def test_crc_matches_ccitt_false_check_value(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_command_round_trip(self):
        command = MotionCommand(42, 150, -240, 320, ActuatorCommand.OPEN)
        frame = encode_motion_command(command)
        self.assertEqual(len(frame), FRAME_SIZE)
        self.assertEqual(MotionCommandDecoder().feed(frame), [command])

    def test_incremental_and_concatenated_frames(self):
        first = encode_motion_command(MotionCommand(7, 100, 0, 0))
        second_command = MotionCommand(8, 80, 100, -100, ActuatorCommand.CLOSE)
        second = encode_motion_command(second_command)
        decoder = MotionCommandDecoder()
        self.assertEqual(decoder.feed(b"noise" + first[:6]), [])
        self.assertEqual(decoder.feed(first[6:] + second), [
            MotionCommand(7, 100, 0, 0), second_command
        ])

    def test_bad_crc_is_discarded_and_next_frame_recovers(self):
        bad = bytearray(encode_motion_command(MotionCommand(1, 100, 20, 20)))
        bad[-1] ^= 0x80
        valid_command = MotionCommand(2, 100, -10, 10)
        decoder = MotionCommandDecoder()
        self.assertEqual(decoder.feed(bytes(bad) + encode_motion_command(valid_command)), [valid_command])
        self.assertEqual(decoder.bad_checksums, 1)

    def test_invalid_frame_header_is_rejected_and_recovers(self):
        bad = bytearray(encode_motion_command(MotionCommand(1, 100, 20, 20)))
        bad[4] = 9  # unsupported version
        valid_command = MotionCommand(3, 100, 0, 1)
        decoder = MotionCommandDecoder()
        self.assertEqual(decoder.feed(bytes(bad) + encode_motion_command(valid_command)), [valid_command])
        self.assertGreaterEqual(decoder.bad_headers, 1)

    def test_encoder_rejects_out_of_range_fields(self):
        for command in (
            MotionCommand(-1, 100, 0, 0),
            MotionCommand(1, 0, 0, 0),
            MotionCommand(1, 1001, 0, 0),
            MotionCommand(1, 100, 32768, 0),
            MotionCommand(1, 100, 0, -32769),
            MotionCommand(1, 100, 0, 0, 3),
        ):
            with self.subTest(command=command), self.assertRaises(ValueError):
                encode_motion_command(command)

    def test_sequence_order_handles_wrap_and_replay(self):
        self.assertTrue(sequence_is_newer(0, 0xFFFFFFFF))
        self.assertTrue(sequence_is_newer(9, 8))
        self.assertFalse(sequence_is_newer(8, 8))
        self.assertFalse(sequence_is_newer(7, 8))

    def test_lease_watchdog_handles_wrap(self):
        self.assertFalse(lease_expired((0xFFFFFFF0 + 9) & 0xFFFFFFFF, 0xFFFFFFF0, 20))
        self.assertTrue(lease_expired((0xFFFFFFF0 + 20) & 0xFFFFFFFF, 0xFFFFFFF0, 20))
        self.assertTrue(lease_expired(501, 500, 1))


if __name__ == "__main__":
    unittest.main()
