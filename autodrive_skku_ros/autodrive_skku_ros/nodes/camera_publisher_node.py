import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

try:
    import cv2
except ImportError:
    cv2 = None

from .. import config
from .camera_node import CameraNode

# rear_camera_index 파라미터가 이 값이면 "후방 카메라 미사용"(config.REAR_CAMERA=None과 동일).
NO_REAR_CAMERA = -1


class CameraPublisherNode(Node):
    """CameraNode(캡처/상하분할/회전 보정)를 그대로 소유하고, 타이머로 최신 프레임을
    /camera/top, /camera/bottom, /camera/rear에 CompressedImage(jpeg)로 발행한다.

    cv_bridge 불필요 — CompressedImage는 순수 bytes라 cv2.imencode 결과를 그대로 담는다.
    """

    def __init__(self):
        super().__init__("camera_publisher_node")

        self.declare_parameter("front_camera_index", config.FRONT_CAMERA)
        self.declare_parameter("rear_camera_index", NO_REAR_CAMERA
                                if config.REAR_CAMERA is None else config.REAR_CAMERA)
        self.declare_parameter("split", config.CAMERA_SPLIT)
        self.declare_parameter("rotate", config.FRONT_CAMERA_ROTATE or "")
        self.declare_parameter("jpeg_quality", 80)

        rear_index = self.get_parameter("rear_camera_index").value
        rear_index = None if rear_index == NO_REAR_CAMERA else rear_index
        rotate = self.get_parameter("rotate").value or None
        self._jpeg_quality = self.get_parameter("jpeg_quality").value

        self._cameras = CameraNode(
            self.get_parameter("front_camera_index").value,
            rear_index,
            split=self.get_parameter("split").value,
            width=config.FRAME_WIDTH,
            height=config.FRAME_HEIGHT,
            rotate=rotate,
        )

        self._top_pub = self.create_publisher(CompressedImage, "/camera/top", 10)
        self._bottom_pub = self.create_publisher(CompressedImage, "/camera/bottom", 10)
        self._rear_pub = self.create_publisher(CompressedImage, "/camera/rear", 10)

        self.create_timer(1.0 / config.LOOP_HZ, self._tick)

    def _publish_frame(self, publisher, frame):
        if cv2 is None or frame is None:
            return
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        publisher.publish(msg)

    def _tick(self):
        top, bottom = self._cameras.latest()
        rear = self._cameras.rear()
        self._publish_frame(self._top_pub, top)
        self._publish_frame(self._bottom_pub, bottom)
        self._publish_frame(self._rear_pub, rear)

    def destroy_node(self):
        self._cameras.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
