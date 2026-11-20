"""Stream camera frames as MJPEG over HTTP with FastAPI.

The endpoints are the same as those of the Odysseus robot's server, so
the pi_camera_line_following example works with either:

    GET /odysseus/camera              the camera, as an MJPEG stream
    GET /odysseus/camera/square-path  generated frames of a square path, one
                                      side at a time, as an MJPEG stream
    GET /odysseus/camera/frame        the latest camera frame as one JPEG
    GET /odysseus/camera/info         the source, frame size and frame rate

On a Raspberry Pi 5 with a camera module the camera frames come from Picamera2.
Point the camera down at a sheet of paper with a square drawn on it in thick
dark lines. On any other computer, --synthetic draws such a square instead.

The square path needs no camera. Every --period seconds its frame shows the
next side of a square, counterclockwise as seen from above: bottom, right, top,
left. The square is centred in the frame.

Usage
    python camera_server.py                  # Pi camera, port 8000
    python camera_server.py --synthetic      # generated camera frames
    python camera_server.py --help
"""

import argparse
import asyncio
import math
import threading
import time

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, Response
from fastapi.responses import StreamingResponse

BOUNDARY = "frame"


class PiCameraSource:
    """Frames from the Pi camera through Picamera2."""

    name = "picamera2"

    def __init__(self, width, height):
        # Only on the Pi: sudo apt install python3-picamera2
        from picamera2 import Picamera2

        self._camera = Picamera2()
        # RGB888 gives the pixels in B, G, R order, which is what OpenCV uses
        config = self._camera.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self._camera.configure(config)
        self._camera.start()

    def read(self):
        return self._camera.capture_array()

    def close(self):
        self._camera.stop()


class SyntheticSource:
    """A square of thick dark lines on paper, as a camera above it would see it.

    The paper is slightly rotated and moves a little over time, and the frames
    have uneven lighting and noise, so the simulator cannot rely on a perfect
    picture.
    """

    name = "synthetic"

    def __init__(self, width, height, side=0.6, line_width=0.035, seed=1):
        self._width = width
        self._height = height
        self._side = side  # of the square, as a fraction of the frame height
        self._line_width = line_width  # as a fraction of the frame height
        self._rng = np.random.default_rng(seed)
        self._start = time.monotonic()

        # Lighting: brighter in one corner, like a desk lamp
        y, x = np.mgrid[0:height, 0:width].astype(np.float32)
        light = 225.0 - 35.0 * (x / width + y / height) / 2.0
        self._paper = np.repeat(light[:, :, None], 3, axis=2)

    def read(self):
        t = time.monotonic() - self._start
        h = self._height
        angle = math.radians(4.0 + 1.5 * math.sin(0.3 * t))
        centre = np.array([self._width / 2 + 0.01 * h * math.sin(0.2 * t),
                           h / 2 + 0.01 * h * math.cos(0.25 * t)])
        half = self._side * h / 2.0
        corners = np.array([[-half, -half], [half, -half], [half, half], [-half, half]])
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        corners = (corners @ rotation.T + centre).astype(np.int32)

        frame = self._paper.copy()
        thickness = max(2, int(round(self._line_width * h)))
        cv2.polylines(frame, [corners], isClosed=True, color=(40, 40, 40),
                      thickness=thickness, lineType=cv2.LINE_AA)
        frame += self._rng.normal(0.0, 6.0, frame.shape).astype(np.float32)
        time.sleep(1.0 / 60.0)  # a camera does not deliver frames instantly
        return np.clip(frame, 0, 255).astype(np.uint8)

    def close(self):
        pass


class SquarePath:
    """One side of a square at a time, the next one every period seconds."""

    def __init__(self, width, height, period, side=0.5, line_width=0.035):
        self.period = period
        self._start = time.monotonic()
        half = side * height / 2.0
        cx, cy = width / 2.0, height / 2.0
        # Counterclockwise as seen from above; image rows grow downwards
        corners = [(cx - half, cy + half), (cx + half, cy + half),
                   (cx + half, cy - half), (cx - half, cy - half)]
        y, x = np.mgrid[0:height, 0:width].astype(np.float32)
        paper = 225.0 - 30.0 * (x / width + y / height) / 2.0
        thickness = max(2, int(round(line_width * height)))
        self._frames = []
        for i in range(4):
            frame = np.repeat(paper[:, :, None], 3, axis=2).astype(np.uint8)
            start, end = corners[i], corners[(i + 1) % 4]
            cv2.line(frame, (int(round(start[0])), int(round(start[1]))),
                     (int(round(end[0])), int(round(end[1]))), color=(40, 40, 40),
                     thickness=thickness, lineType=cv2.LINE_AA)
            self._frames.append(cv2.imencode(".jpg", frame)[1].tobytes())

    def latest(self):
        """The JPEG of the current side, and a number that changes with it"""
        step = int((time.monotonic() - self._start) // self.period)
        return self._frames[step % 4], step


class FrameGrabber:
    """Reads frames on a thread of its own and keeps the latest one as JPEG."""

    def __init__(self, source, quality):
        self._source = source
        self._quality = quality
        self._lock = threading.Lock()
        self._jpeg = None
        self._count = 0
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self._running:
            frame = self._source.read()
            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
            if ok:
                with self._lock:
                    self._jpeg = jpeg.tobytes()
                    self._count += 1

    def latest(self):
        with self._lock:
            return self._jpeg, self._count

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self._source.close()


def mjpeg_part(jpeg):
    return (f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n"
            f"Content-Length: {len(jpeg)}\r\n\r\n").encode() + jpeg + b"\r\n"


def build_app(grabber, square_path, source_name, width, height, fps):
    app = FastAPI(title="bitsim pi_camera_line_following camera")
    media_type = f"multipart/x-mixed-replace; boundary={BOUNDARY}"

    @app.get("/odysseus/camera/info")
    def info():
        _, count = grabber.latest()
        return {"source": source_name, "width": width, "height": height,
                "fps": fps, "frames": count, "square_path_period": square_path.period}

    @app.get("/odysseus/camera/frame")
    def frame():
        jpeg, _ = grabber.latest()
        if jpeg is None:
            return Response(status_code=503)
        return Response(content=jpeg, media_type="image/jpeg")

    async def camera_stream():
        last = 0
        while True:
            jpeg, count = grabber.latest()
            if jpeg is not None and count != last:
                last = count
                yield mjpeg_part(jpeg)
            await asyncio.sleep(1.0 / fps)

    async def square_path_stream():
        # The frame does not change for a whole side, but send it at the frame
        # rate anyway, as a camera would
        while True:
            jpeg, _ = square_path.latest()
            yield mjpeg_part(jpeg)
            await asyncio.sleep(1.0 / fps)

    @app.get("/odysseus/camera")
    def camera():
        return StreamingResponse(camera_stream(), media_type=media_type)

    @app.get("/odysseus/camera/square-path")
    def square_path_endpoint():
        return StreamingResponse(square_path_stream(), media_type=media_type)

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--synthetic", action="store_true",
                        help="generate frames instead of using the Pi camera")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=20.0, help="frames per second to stream")
    parser.add_argument("--quality", type=int, default=80, help="JPEG quality, 1 to 100")
    parser.add_argument("--period", type=float, default=5.0,
                        help="seconds between the sides of the square path")
    args = parser.parse_args()

    if args.synthetic:
        source = SyntheticSource(args.width, args.height)
    else:
        source = PiCameraSource(args.width, args.height)

    grabber = FrameGrabber(source, args.quality)
    square_path = SquarePath(args.width, args.height, args.period)
    app = build_app(grabber, square_path, source.name, args.width, args.height, args.fps)
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    finally:
        grabber.stop()


if __name__ == "__main__":
    main()
