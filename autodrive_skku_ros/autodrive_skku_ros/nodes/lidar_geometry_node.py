import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

from .. import config
from .lidar_node import laserscan_msg_to_tuples, rear_min_m, scan_to_ranges

N_BINS = 360


class LidarGeometryNode(Node):
    """공식 rplidar_ros가 발행하는 /scan(원시 LaserScan)을 구독해, 2026-07-09
    후방 장착 실측 보정(config.LIDAR_MOUNT/LIDAR_SELF_MASK_DEG/LIDAR_REAR_SECTOR)을
    적용한 결과를 재발행한다.

    - /lidar/rear_min_m: 뒤 범퍼 기준 후방 섹터 최소 거리 (T주차 후진용, 없으면 NaN)
    - /lidar/scan_corrected: 자차 반사 제거 + 차량 기준 bearing으로 정렬한 LaserScan
      — 미션이 실제로 "보는" 시야를 Foxglove/rviz에서 그대로 확인 가능.

    주의: yaw_offset_deg/invert 캘리브레이션은 기존 파이썬 rplidar 라이브러리의
    각도 규약 기준으로 잡힌 값이라, rplidar_ros의 /scan 각도 규약과 정확히
    일치하는지 실차에서 재확인이 필요하다 — 다르면 config.LIDAR_MOUNT만 조정.
    """

    def __init__(self):
        super().__init__("lidar_geometry_node")

        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self._rear_min_pub = self.create_publisher(Float32, "/lidar/rear_min_m", 10)
        self._corrected_pub = self.create_publisher(LaserScan, "/lidar/scan_corrected", 10)

    def _on_scan(self, msg):
        scan = laserscan_msg_to_tuples(msg)

        rear_min = rear_min_m(scan, config.LIDAR_MOUNT, config.LIDAR_REAR_SECTOR,
                               config.LIDAR_SELF_MASK_DEG)
        self._rear_min_pub.publish(Float32(data=float("nan") if rear_min is None else rear_min))

        start_angle, end_angle, ranges_m = scan_to_ranges(
            scan, config.LIDAR_MOUNT, config.LIDAR_SELF_MASK_DEG, N_BINS)
        corrected = LaserScan()
        corrected.header = msg.header
        corrected.angle_min = start_angle
        corrected.angle_max = end_angle
        corrected.angle_increment = (end_angle - start_angle) / N_BINS
        corrected.time_increment = msg.time_increment
        corrected.scan_time = msg.scan_time
        corrected.range_min = 0.05
        corrected.range_max = 12.0
        corrected.ranges = ranges_m
        self._corrected_pub.publish(corrected)


def main(args=None):
    rclpy.init(args=args)
    node = LidarGeometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
