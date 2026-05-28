#!/usr/bin/env python3
import sys
import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QGroupBox, QLineEdit,
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QDoubleValidator

PUBLISH_HZ = 50


class JointControlNode(Node):

    def __init__(self):
        super().__init__('joint_state_publisher_gui')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        self.wheel_speed = [0.0, 0.0, 0.0]  # rad/s
        self.steering    = [0.0, 0.0, 0.0]  # rad
        self.wheel_pos   = [0.0, 0.0, 0.0]  # accumulated angle

        self.dt = 1.0 / PUBLISH_HZ
        self.create_timer(self.dt, self._publish)

    def _publish(self):
        # Integrate wheel speed
        for i in range(3):
            self.wheel_pos[i] += self.wheel_speed[i] * self.dt
            self.wheel_pos[i] = math.fmod(self.wheel_pos[i], 2.0 * math.pi)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'steering_1_joint', 'steering_2_joint', 'steering_3_joint',
            'wheel_1_joint',    'wheel_2_joint',    'wheel_3_joint',
        ]
        msg.position = [
            self.steering[0],  self.steering[1],  self.steering[2],
            self.wheel_pos[0], self.wheel_pos[1], self.wheel_pos[2],
        ]
        self.pub.publish(msg)


class SliderRow(QWidget):

    def __init__(self, name, min_val, max_val, default, scale, on_change):
        super().__init__()
        self.scale      = scale
        self.min_val    = min_val
        self.max_val    = max_val
        self._on_change = on_change
        self._updating  = False  # prevent recursive updates

        row = QHBoxLayout()
        row.setContentsMargins(4, 2, 4, 2)

        lbl = QLabel(name)
        lbl.setFixedWidth(120)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(int(min_val * scale))
        self.slider.setMaximum(int(max_val * scale))
        self.slider.setValue(int(default * scale))
        self.slider.valueChanged.connect(self._slider_moved)

        self.val_lbl = QLabel(f'{default:+.2f}')
        self.val_lbl.setFixedWidth(50)

        # Type value, press Enter
        self.text_input = QLineEdit(f'{default:.4f}')
        self.text_input.setFixedWidth(80)
        self.text_input.setPlaceholderText('type & ↵')
        self.text_input.setValidator(
            QDoubleValidator(min_val, max_val, 4, self.text_input)
        )
        self.text_input.returnPressed.connect(self._text_entered)

        row.addWidget(lbl)
        row.addWidget(self.slider)
        row.addWidget(self.val_lbl)
        row.addWidget(self.text_input)
        self.setLayout(row)

    def _slider_moved(self, raw):
        if self._updating:
            return
        val = raw / self.scale
        self._updating = True
        self.val_lbl.setText(f'{val:+.2f}')
        self.text_input.setText(f'{val:.4f}')
        self._updating = False
        self._on_change(val)

    def _text_entered(self):
        try:
            val = float(self.text_input.text().strip())
        except ValueError:
            return
        val = max(self.min_val, min(self.max_val, val))
        self._updating = True
        self.slider.setValue(int(val * self.scale))
        self.val_lbl.setText(f'{val:+.2f}')
        self.text_input.setText(f'{val:.4f}')
        self._updating = False
        self._on_change(val)


class ControlWindow(QWidget):

    def __init__(self, node: JointControlNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('3D3S Robot — Joint Control')
        self.setMinimumWidth(520)

        root = QVBoxLayout()

        # Wheel speed sliders
        wg = QGroupBox('Wheel Speed (rad/s)')
        wl = QVBoxLayout()
        for i in range(3):
            wl.addWidget(SliderRow(
                name=f'  wheel_{i + 1}',
                min_val=-10.0, max_val=10.0, default=0.0, scale=100,
                on_change=lambda v, idx=i: self._set(node.wheel_speed, idx, v),
            ))
        wg.setLayout(wl)

        # Steering angle sliders
        sg = QGroupBox('Steering Angle (rad)')
        sl = QVBoxLayout()
        for i in range(3):
            sl.addWidget(SliderRow(
                name=f'  steering_{i + 1}',
                min_val=-2.618, max_val=2.618, default=0.0, scale=1000,
                on_change=lambda v, idx=i: self._set(node.steering, idx, v),
            ))
        sg.setLayout(sl)

        root.addWidget(wg)
        root.addWidget(sg)
        self.setLayout(root)

    def _set(self, lst, idx, val):
        lst[idx] = val


def main():
    rclpy.init()
    node = JointControlNode()

    # ROS2 spins in background
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = QApplication(sys.argv)
    win = ControlWindow(node)
    win.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
