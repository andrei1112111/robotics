import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class StopOnDepth(Node):
    def __init__(self):
        super().__init__('stop_on_depth')

        # Параметры поведения
        self.safe_distance = 2      # м: ближе этого — тормозим
        self.forward_speed = 0.3      # м/с: скорость вперёд
        self.max_depth = 5.0         # м: всё дальше считаем “нет препятствия”

        # Подписка на depth
        self.depth_sub = self.create_subscription(
            Image,
            '/robot/virtual_depth/image',
            self.depth_callback,
            10
        )

        # Публикация команд скорости
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Состояние
        self.obstacle_close = False
        self.last_min_dist = None

        # Таймер для периодической отправки cmd_vel
        self.timer = self.create_timer(0.1, self.control_loop)

    def depth_callback(self, msg: Image):
        """Обработка кадра глубины."""

        # Определяем тип данных в Image
        if msg.encoding == '32FC1':
            dtype = np.float32
            scale = 1.0       # уже в метрах
        elif msg.encoding in ('16UC1', 'mono16'):
            dtype = np.uint16
            scale = 0.001     # обычно мм -> метры
        else:
            self.get_logger().warn(f'Неизвестное кодирование depth: {msg.encoding}')
            return

        # Превращаем байты в массив
        try:
            depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
        except ValueError:
            self.get_logger().warn('Не удалось преобразовать depth image в массив')
            return

        depth = depth * scale

        # Центр по ширине (прямо перед роботом)
        h, w = depth.shape
        col_center = w // 2
        window = w // 10 if w >= 10 else 5  # примерно центральные 20% ширины

        col_start = max(0, col_center - window)
        col_end   = min(w, col_center + window)

        roi = depth[:, col_start:col_end]

        # Фильтрация “некорректных” значений
        valid = roi[(roi > 0.0) & (roi < self.max_depth) & ~np.isnan(roi)]

        if valid.size == 0:
            # Ничего не увидели впереди
            self.obstacle_close = False
            self.last_min_dist = None
            return

        min_dist = float(valid.min())
        self.last_min_dist = min_dist

        if min_dist < self.safe_distance:
            self.obstacle_close = True
            self.get_logger().info(f'!!! STOP: препятствие на {min_dist:.2f} м')
        else:
            self.obstacle_close = False
            self.get_logger().info(f'!!! GO: свободно, ближайшее на {min_dist:.2f} м !!!')

    def control_loop(self):
        """Публикация команд движения на основе флага obstacle_close."""
        twist = Twist()

        if self.obstacle_close:
            twist.linear.x = 0.0
        else:
            twist.linear.x = self.forward_speed

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = StopOnDepth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()