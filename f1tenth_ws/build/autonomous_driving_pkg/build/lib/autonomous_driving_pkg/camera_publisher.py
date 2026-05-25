import math
import threading

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, CameraInfo

# Camera 0 on this RB5 board outputs native JPEG at 640x480 30fps via qtiqmmfsrc.
# Do NOT request NV12, H264, BGR, or non-native resolutions — causes QMMF EndConfigure
# assertion crash in qmmf_camera_context.cc.
# Since output is already JPEG, it goes directly to appsink with no decode/re-encode.
PIPELINE_STR = (
    'qtiqmmfsrc camera=0 '
    '! image/jpeg,width=640,height=480,framerate=30/1 '
    '! appsink name=sink emit-signals=true max-buffers=1 drop=true'
)

JPEG_QUALITY = 85


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Declare hfov_deg parameter — default 70.0, override at launch with:
        #   --ros-args -p hfov_deg:=84.0
        self.declare_parameter('hfov_deg', 70.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(CompressedImage, '/camera/image/compressed', qos)
        self._info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos)

        # Log the active FOV so it's visible at startup
        hfov = self.get_parameter('hfov_deg').value
        w = self.get_parameter('image_width').value
        h = self.get_parameter('image_height').value
        fx = (w / 2.0) / math.tan(math.radians(hfov) / 2.0)
        fy = fx  # square pixels assumed
        cx, cy = w / 2.0, h / 2.0
        self._camera_info = self._build_camera_info(w, h, fx, fy, cx, cy)
        self.get_logger().info(
            f'Camera parameters: {w}x{h}, hfov={hfov:.1f}deg, fx={fx:.1f}, fy={fy:.1f}'
        )

        Gst.init(None)
        self._pipeline = Gst.parse_launch(PIPELINE_STR)
        self._sink = self._pipeline.get_by_name('sink')
        self._sink.connect('new-sample', self._on_new_sample)

        self._lock = threading.Lock()
        self._running = False
        self._glib_loop = GLib.MainLoop()

        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self._on_bus_message)

        self._thread = threading.Thread(target=self._gst_thread, daemon=True)
        self._thread.start()

    def _gst_thread(self):
        ret = self._pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error('Failed to start GStreamer pipeline')
            return
        self._running = True
        self.get_logger().info('GStreamer pipeline started (640x480 BGR @ 30fps via qtivdec)')
        try:
            self._glib_loop.run()
        except Exception as e:
            self.get_logger().error(f'GLib loop error: {e}')
        finally:
            self._pipeline.set_state(Gst.State.NULL)

    def _on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf = sample.get_buffer()
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        try:
            # Camera outputs native JPEG — publish bytes directly, no re-encode needed
            stamp = self.get_clock().now().to_msg()

            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = 'camera'
            msg.format = 'jpeg'
            msg.data = bytes(map_info.data)
            self._pub.publish(msg)

            info = self._camera_info
            info.header.stamp = stamp
            self._info_pub.publish(info)
        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

    def _on_bus_message(self, bus, message):
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f'GStreamer error: {err.message} | {debug}')
            self._glib_loop.quit()
        elif t == Gst.MessageType.EOS:
            self.get_logger().warn('GStreamer EOS received')
            self._glib_loop.quit()

    @staticmethod
    def _build_camera_info(w, h, fx, fy, cx, cy):
        info = CameraInfo()
        info.header.frame_id = 'camera'
        info.width = w
        info.height = h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx,
                  0.0, fy, cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def destroy_node(self):
        if self._running:
            self._glib_loop.quit()
            self._thread.join(timeout=3.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
