#!/usr/bin/env python3

import io
import os
import struct
import sys
import time

from gz.msgs10.image_pb2 import Image as GazeboImage
from gz.transport13 import Node
from PIL import Image


FRAME_INTERVAL = 1 / 12
JPEG_QUALITY = 68
CAMERA_TOPIC_SUFFIX = "/sensor/camera/image"


class CameraBridge:
    def __init__(self):
        self.node = Node()
        self.last_frame_time = 0.0
        self.topic = None

    def find_camera_topic(self):
        configured_topic = os.environ.get("GZ_CAMERA_TOPIC")
        if configured_topic:
            return configured_topic

        topics = self.node.topic_list()
        camera_topics = [
            topic
            for topic in topics
            if topic.endswith(CAMERA_TOPIC_SUFFIX)
        ]

        return camera_topics[0] if camera_topics else None

    def connect(self):
        while self.topic is None:
            self.topic = self.find_camera_topic()
            if self.topic is None:
                time.sleep(1)

        self.node.subscribe(
            GazeboImage,
            self.topic,
            self.on_frame,
        )
        print(f"Camera connected: {self.topic}", file=sys.stderr, flush=True)

    def on_frame(self, message):
        now = time.monotonic()
        if now - self.last_frame_time < FRAME_INTERVAL:
            return

        self.last_frame_time = now
        frame = self.decode_frame(message)
        if frame is None:
            return

        output = io.BytesIO()
        frame.save(
            output,
            format="JPEG",
            quality=JPEG_QUALITY,
            optimize=False,
        )
        jpeg = output.getvalue()

        sys.stdout.buffer.write(struct.pack(">I", len(jpeg)))
        sys.stdout.buffer.write(jpeg)
        sys.stdout.buffer.flush()

    @staticmethod
    def decode_frame(message):
        pixel_formats = {
            1: ("L", "L"),
            3: ("RGB", "RGB"),
            4: ("RGBA", "RGBA"),
            5: ("RGBA", "BGRA"),
            8: ("RGB", "BGR"),
        }
        format_pair = pixel_formats.get(message.pixel_format_type)

        if format_pair is None:
            return None

        mode, raw_mode = format_pair

        try:
            image = Image.frombytes(
                mode,
                (message.width, message.height),
                message.data,
                "raw",
                raw_mode,
                message.step,
            )
        except ValueError:
            return None

        return image.convert("RGB")


def main():
    bridge = CameraBridge()
    bridge.connect()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
