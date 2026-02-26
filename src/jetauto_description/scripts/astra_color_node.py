#!/usr/bin/env python3
"""
Astra Pro Plus RGB color stream via raw V4L2 (kernel uvcvideo driver).
MJPG capture via raw V4L2 mmap + Rockchip hardware JPEG decode via GStreamer mppjpegdec.
MJPG (~1-3 MB/s) vs YUYV (18.4 MB/s) avoids USB bandwidth contention with depth+IR streams.
Hardware decode via Rockchip MPP (mppjpegdec) reduces CPU load.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import fcntl
import struct
import os
import mmap
import select
import time

# V4L2 constants (AArch64: struct v4l2_buffer = 88 bytes)
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_PIX_FMT_MJPG = 0x47504a4d  # 'MJPG'
VIDIOC_S_FMT     = 0xc0d05605
VIDIOC_REQBUFS   = 0xc0145608
VIDIOC_QUERYBUF  = 0xc0585609  # 88-byte struct on AArch64
VIDIOC_QBUF      = 0xc058560f
VIDIOC_DQBUF     = 0xc0585611
VIDIOC_STREAMON  = 0x40045612
VIDIOC_STREAMOFF = 0x40045613

NUM_BUFS = 4


def _make_v4l2_buf(index=0):
    """Create an 88-byte v4l2_buffer for VIDEO_CAPTURE MMAP."""
    b = bytearray(88)
    struct.pack_into('I', b, 0, index)
    struct.pack_into('I', b, 4, V4L2_BUF_TYPE_VIDEO_CAPTURE)
    struct.pack_into('I', b, 60, V4L2_MEMORY_MMAP)
    return b


class V4L2MJPGCapture:
    """
    Raw V4L2 MJPG capture using mmap streaming.
    Bypasses OpenCV's broken V4L2 backend on AArch64 (wrong struct v4l2_buffer size).
    Returns raw JPEG bytes per frame for external decoding.
    """

    def __init__(self, device, width, height, fps):
        self.fd = -1
        self.buffers = []
        self._open(device, width, height, fps)

    def _open(self, device, width, height, fps):
        self.fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)

        # Request MJPG format: ~1-3 MB/s vs 18.4 MB/s YUYV → no USB contention with depth+IR
        fmt = bytearray(208)
        struct.pack_into('I', fmt, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into('I', fmt, 8, width)
        struct.pack_into('I', fmt, 12, height)
        struct.pack_into('I', fmt, 16, V4L2_PIX_FMT_MJPG)
        try:
            fcntl.ioctl(self.fd, VIDIOC_S_FMT, fmt)
        except OSError as e:
            os.close(self.fd)
            self.fd = -1
            raise OSError(e.errno, f'VIDIOC_S_FMT MJPG failed: {e}') from e

        # Request mmap buffers
        reqbuf = bytearray(struct.pack('IIII', NUM_BUFS,
                                       V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                       V4L2_MEMORY_MMAP, 0))
        try:
            fcntl.ioctl(self.fd, VIDIOC_REQBUFS, reqbuf)
        except OSError as e:
            os.close(self.fd)
            self.fd = -1
            raise OSError(e.errno, f'VIDIOC_REQBUFS failed: {e}') from e
        count = struct.unpack('I', reqbuf[:4])[0]

        # Query and mmap each buffer
        for i in range(count):
            b = _make_v4l2_buf(i)
            fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, b)
            length = struct.unpack('I', b[72:76])[0]
            offset = struct.unpack('I', b[64:68])[0]
            mm = mmap.mmap(self.fd, length, mmap.MAP_SHARED,
                           mmap.PROT_READ | mmap.PROT_WRITE, offset=offset)
            self.buffers.append((mm, length))

        # Queue all buffers
        for i in range(len(self.buffers)):
            b = _make_v4l2_buf(i)
            fcntl.ioctl(self.fd, VIDIOC_QBUF, b)

        # Start streaming
        streamtype = bytearray(struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE))
        try:
            fcntl.ioctl(self.fd, VIDIOC_STREAMON, streamtype)
        except OSError as e:
            os.close(self.fd)
            self.fd = -1
            raise OSError(e.errno, f'VIDIOC_STREAMON failed: {e}') from e

    def read_jpeg(self, timeout=1.0):
        """Dequeue one MJPG frame. Returns raw JPEG bytes or None on timeout/error."""
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return None

        b = _make_v4l2_buf(0)
        try:
            fcntl.ioctl(self.fd, VIDIOC_DQBUF, b)
        except OSError:
            return None

        idx = struct.unpack('I', b[0:4])[0]
        bytesused = struct.unpack('I', b[8:12])[0]

        mm, _ = self.buffers[idx]
        mm.seek(0)
        data = bytes(mm.read(bytesused))

        try:
            fcntl.ioctl(self.fd, VIDIOC_QBUF, b)
        except OSError:
            pass

        return data

    def close(self):
        if self.fd < 0:
            return
        try:
            streamtype = bytearray(struct.pack('I', V4L2_BUF_TYPE_VIDEO_CAPTURE))
            fcntl.ioctl(self.fd, VIDIOC_STREAMOFF, streamtype)
        except OSError:
            pass
        for mm, _ in self.buffers:
            mm.close()
        self.buffers.clear()
        os.close(self.fd)
        self.fd = -1


class RockchipJPEGDecoder:
    """
    Hardware JPEG decoder using Rockchip MPP via GStreamer.
    Tries decoders in order:
      1. mppjpegdec  - Rockchip VPU (MPP), lowest CPU
      2. v4l2jpegdec - V4L2 M2M hardware decoder
      3. jpegdec     - Software fallback (GStreamer)
      4. cv2.imdecode - Software fallback (OpenCV)

    Uses 'new-sample' signal + threading.Queue to avoid try_pull_sample
    which is not exposed in all PyGObject versions.
    """

    def __init__(self, width, height):
        self._width = width
        self._height = height
        self._pipeline = None
        self._appsrc = None
        self._decoder = 'cv2.imdecode'
        self._Gst = None
        self._sample_queue = None
        self._init_gst(width, height)

    def _init_gst(self, width, height):
        try:
            import gi
            gi.require_version('Gst', '1.0')
            from gi.repository import Gst
            Gst.init(None)
            self._Gst = Gst

            import queue
            sq = queue.Queue(maxsize=1)
            self._sample_queue = sq

            for decoder in ['mppjpegdec', 'v4l2jpegdec', 'jpegdec']:
                pipeline_str = (
                    f"appsrc name=src format=time is-live=true block=false do-timestamp=true "
                    f"max-buffers=2 leaky-type=downstream ! "
                    f"image/jpeg,width={width},height={height} ! "
                    f"jpegparse ! {decoder} ! "
                    f"videoconvert ! video/x-raw,format=BGR ! "
                    f"appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
                )
                try:
                    pipeline = Gst.parse_launch(pipeline_str)
                    appsrc = pipeline.get_by_name('src')
                    appsink = pipeline.get_by_name('sink')

                    # new-sample signal: called by GStreamer streaming thread.
                    # Use emit('pull-sample') — it's a GstAppSink action signal available
                    # without importing GstApp (pull_sample/try_pull_sample require GstApp typelib).
                    def on_new_sample(sink, _sq=sq, _Gst=Gst):
                        sample = sink.emit('pull-sample')
                        try:
                            _sq.put_nowait(sample)
                        except Exception:
                            pass  # queue full → frame dropped (drop=true on appsink)
                        return _Gst.FlowReturn.OK

                    appsink.connect('new-sample', on_new_sample)

                    ret = pipeline.set_state(Gst.State.PLAYING)
                    if ret == Gst.StateChangeReturn.FAILURE:
                        pipeline.set_state(Gst.State.NULL)
                        continue

                    self._pipeline = pipeline
                    self._appsrc = appsrc
                    self._decoder = decoder
                    return
                except Exception:
                    continue

        except (ImportError, Exception):
            pass
        # All GStreamer options failed → will use cv2.imdecode

    def decode(self, jpeg_bytes):
        """Decode JPEG bytes to BGR numpy array. Returns None on error."""
        if self._pipeline is None:
            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)

        Gst = self._Gst
        buf = Gst.Buffer.new_wrapped(jpeg_bytes)
        flow = self._appsrc.emit('push-buffer', buf)
        if flow != Gst.FlowReturn.OK:
            return None

        try:
            sample = self._sample_queue.get(timeout=1.0)
        except Exception:
            return None

        if sample is None:
            return None

        gbuf = sample.get_buffer()
        success, map_info = gbuf.map(Gst.MapFlags.READ)
        if not success:
            return None
        try:
            arr = np.frombuffer(map_info.data, dtype=np.uint8)
            return arr.reshape((self._height, self._width, 3)).copy()
        finally:
            gbuf.unmap(map_info)

    @property
    def decoder_name(self):
        return self._decoder

    def close(self):
        if self._pipeline is not None:
            self._pipeline.set_state(self._Gst.State.NULL)
            self._pipeline = None


class AstraColorNode(Node):
    def __init__(self):
        super().__init__('color', namespace='camera')
        self.declare_parameter(
            'video_device',
            '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN0001-video-index0',
        )
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        device = self.get_parameter('video_device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()
        self.pub_image = self.create_publisher(Image, '/camera/color/image_raw', 10)

        self._stop = threading.Event()
        self._cap = None
        self._dec = None
        self._thread = None

        # Init hardware decoder (GStreamer pipeline)
        self._dec = RockchipJPEGDecoder(self.width, self.height)
        self.get_logger().info(f'JPEG decoder backend: {self._dec.decoder_name}')

        # Open V4L2 MJPG capture with retry (device may need settling time)
        for attempt in range(10):
            try:
                self._cap = V4L2MJPGCapture(device, self.width, self.height, self.fps)
                self.get_logger().info(
                    f'Color camera opened: {device} @ {self.width}x{self.height} MJPG (raw V4L2)'
                )
                break
            except OSError as e:
                self.get_logger().warn(f'Open attempt {attempt+1}/10 failed: {e}')
                time.sleep(1.0)
        else:
            self.get_logger().error(f'Cannot open {device} after 10 attempts')
            return

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        while not self._stop.is_set() and rclpy.ok():
            jpeg = self._cap.read_jpeg(timeout=1.0)
            if jpeg is None:
                self.get_logger().warn('Frame timeout, retrying...')
                continue

            frame = self._dec.decode(jpeg)
            if frame is None:
                self.get_logger().warn('JPEG decode failed, skipping frame')
                continue

            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            try:
                self.pub_image.publish(msg)
            except Exception:
                break

    def destroy_node(self):
        self._stop.set()
        if self._cap is not None:
            self._cap.close()
        if self._dec is not None:
            self._dec.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AstraColorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
