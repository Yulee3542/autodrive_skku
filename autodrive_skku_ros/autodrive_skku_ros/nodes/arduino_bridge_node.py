import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Int8
from autodrive_msgs.msg import DriveCmd, SteerCmd

from .. import config
from .arduino_node import ArduinoNode
from .ports import autodetect_ports

# 아두이노 state(0 정지/1 전진/2 후진)가 None(미연결)일 때 Int8로 실어보낼 센티널.
# 구독 쪽(mission_node)에서 다시 None으로 복원한다.
STATE_UNKNOWN = -1


class ArduinoBridgeNode(Node):
    """ArduinoNode(시리얼 프로토콜)를 그대로 소유하고 ROS 토픽만 얹는 얇은 래퍼.

    /car/cmd/go, /car/cmd/stop, /car/cmd/drive, /car/cmd/steer 구독 → ArduinoNode의
    대응 메서드(go/stop/drive/steer/steer_pulse)를 그대로 호출한다. 시리얼
    프로토콜·워치독·dedupe 로직은 ArduinoNode에 손대지 않고 그대로 재사용한다.
    """

    def __init__(self):
        super().__init__("arduino_bridge_node")

        self.declare_parameter("port", "")
        self.declare_parameter("baud", config.ARDUINO_BAUD)

        port = self.get_parameter("port").value or None
        if port is None:
            port, _lidar = autodetect_ports()
        baud = self.get_parameter("baud").value

        self._car = ArduinoNode(port, baud)

        self.create_subscription(Empty, "/car/cmd/go", self._on_go, 10)
        self.create_subscription(Empty, "/car/cmd/stop", self._on_stop, 10)
        self.create_subscription(DriveCmd, "/car/cmd/drive", self._on_drive, 10)
        self.create_subscription(SteerCmd, "/car/cmd/steer", self._on_steer, 10)

        self._state_pub = self.create_publisher(Int8, "/car/state", 10)
        self.create_timer(1.0 / config.LOOP_HZ, self._publish_state)

    def _on_go(self, _msg):
        self._car.go()

    def _on_stop(self, _msg):
        self._car.stop()

    def _on_drive(self, msg):
        self._car.drive(msg.speed)

    def _on_steer(self, msg):
        if msg.pulse:
            self._car.steer_pulse(msg.direction)
        else:
            self._car.steer(msg.direction)

    def _publish_state(self):
        state = self._car.state
        self._state_pub.publish(Int8(data=STATE_UNKNOWN if state is None else state))

    def destroy_node(self):
        self._car.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
