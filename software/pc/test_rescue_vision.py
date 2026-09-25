import unittest

import cv2
import numpy as np

from rescue_transport import ImageFrame
from rescue_vision import Detection, annotate_frame, decode_jpeg, run_detector


class Scalar:
    def __init__(self, value):
        self.value = value

    def item(self):
        return self.value


class Coordinates:
    def tolist(self):
        return [4.2, 5.7, 32.4, 37.9]


class FakeBox:
    cls = [Scalar(2)]
    conf = [Scalar(0.91)]
    xyxy = [Coordinates()]


class FakeResult:
    names = {2: "casualty"}
    boxes = [FakeBox()]


class FakeModel:
    def __init__(self):
        self.options = None

    def predict(self, **options):
        self.options = options
        return [FakeResult()]


class RescueVisionHelperTests(unittest.TestCase):
    @staticmethod
    def sample_frame() -> ImageFrame:
        image = np.zeros((48, 64, 3), dtype=np.uint8)
        image[:, :] = (40, 120, 200)
        success, encoded = cv2.imencode(".jpg", image)
        if not success:
            raise RuntimeError("OpenCV failed to encode test image")
        return ImageFrame(7, 1234, 64, 48, encoded.tobytes())

    def test_decode_jpeg_returns_bgr_frame(self):
        image = decode_jpeg(self.sample_frame())
        self.assertEqual(image.shape, (48, 64, 3))
        self.assertGreater(int(image[20, 20, 2]), int(image[20, 20, 0]))

    def test_annotate_frame_draws_boxes_without_mutating_input(self):
        frame = self.sample_frame()
        image = decode_jpeg(frame)
        original = image.copy()
        result = annotate_frame(
            image,
            frame,
            [Detection(1, "casualty", 0.91, (8, 8, 40, 40))],
            fps=12.5,
            detector_enabled=True,
        )
        self.assertEqual(result.shape, image.shape)
        self.assertTrue(np.array_equal(image, original))
        self.assertFalse(np.array_equal(result, original))

    def test_detector_results_are_normalized_and_device_is_forwarded(self):
        model = FakeModel()
        image = decode_jpeg(self.sample_frame())
        detections = run_detector(image, model, confidence=0.4, image_size=320, device="0")
        self.assertEqual(detections, [Detection(2, "casualty", 0.91, (4, 6, 32, 38))])
        self.assertEqual(model.options["device"], "0")
        self.assertEqual(model.options["imgsz"], 320)


if __name__ == "__main__":
    unittest.main()
