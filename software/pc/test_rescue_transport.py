import unittest

from rescue_transport import (
    HEADER,
    ImageFrame,
    ImageFrameDecoder,
    encode_jpeg_frame,
)


def sample(seq=1):
    return ImageFrame(seq, 123456, 320, 240, b"\xff\xd8test-payload\xff\xd9")


class ImageFrameProtocolTests(unittest.TestCase):
    def test_round_trip_byte_by_byte(self):
        expected = sample()
        decoder = ImageFrameDecoder()
        actual = []
        for byte in encode_jpeg_frame(expected):
            actual.extend(decoder.feed(bytes([byte])))
        self.assertEqual(actual, [expected])

    def test_multiple_frames_and_noise(self):
        decoder = ImageFrameDecoder()
        wire = b"noise" + encode_jpeg_frame(sample(7)) + encode_jpeg_frame(sample(8))
        self.assertEqual(decoder.feed(wire), [sample(7), sample(8)])

    def test_bad_crc_then_recover(self):
        damaged = bytearray(encode_jpeg_frame(sample()))
        damaged[HEADER.size + 3] ^= 1
        decoder = ImageFrameDecoder()
        self.assertEqual(decoder.feed(damaged + encode_jpeg_frame(sample(2))), [sample(2)])
        self.assertEqual(decoder.bad_payloads, 1)

    def test_bad_size_then_recover(self):
        damaged = bytearray(encode_jpeg_frame(sample()))
        damaged[18:22] = (999_999).to_bytes(4, "little")
        decoder = ImageFrameDecoder()
        self.assertEqual(decoder.feed(damaged + encode_jpeg_frame(sample(2))), [sample(2)])
        self.assertGreaterEqual(decoder.bad_headers, 1)

    def test_reject_non_jpeg(self):
        with self.assertRaises(ValueError):
            encode_jpeg_frame(ImageFrame(1, 1, 320, 240, b"not jpeg"))


if __name__ == "__main__":
    unittest.main()
