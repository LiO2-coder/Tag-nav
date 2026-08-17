#!/usr/bin/env python3
"""PyQt + rospy frontend for the tag-graph planner.

Pick a start and a goal tag on the grid, then "Plan & Go" calls the C++ planner
service /planner/plan_path (which plans A* over the tag graph and drives the
robot) and publishes start/goal arrow markers for rviz. The latest plan from
/planner/path is drawn over the grid.
"""

import argparse
import json
import math
import os
import sys
import threading

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, pyqtSignal

import rospy
import rospkg
import tf.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker
from tag_nav_planner.srv import PlanPath


def load_tag_map(path):
    """Returns (tags, grid, cols, rows, min_x, max_y) from apriltagMap.json."""
    with open(path) as f:
        data = json.load(f)
    if data.get('map_type') != '2d':
        raise ValueError('map_type must be "2d"')
    locations = data.get('tag_locations', {})
    if not isinstance(locations, dict) or not locations:
        raise ValueError('tag_locations is missing or empty')
    tags = {}
    for key, value in locations.items():
        if len(value) != 3:
            raise ValueError('tag %s must be [x, y, yaw]' % key)
        tags[int(key)] = (float(value[0]), float(value[1]), float(value[2]))
    xs = [t[0] for t in tags.values()]
    ys = [t[1] for t in tags.values()]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    cols = int(round(max_x - min_x)) + 1
    rows = int(round(max_y - min_y)) + 1
    grid = {}
    for tid, (x, y, _yaw) in tags.items():
        grid[tid] = (int(round(x - min_x)), int(round(max_y - y)))
    return tags, grid, cols, rows, min_x, max_y


class RosInterface(QtCore.QObject):
    """Owns rospy publishers/subscribers; callbacks only emit Qt signals."""

    path_signal = pyqtSignal(object)
    state_signal = pyqtSignal(str)
    pose_signal = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self.goal_marker_pub = rospy.Publisher('/nav/goal_marker', Marker,
                                               queue_size=1, latch=True)
        self.start_marker_pub = rospy.Publisher('/nav/start_marker', Marker,
                                                queue_size=1, latch=True)
        self.cancel_pub = rospy.Publisher('/planner/cancel', Empty, queue_size=1)
        rospy.Subscriber('/planner/path', Path, self._on_path)
        rospy.Subscriber('/planner/state', String, self._on_state)
        rospy.Subscriber('/apriltag_localization/pose', PoseWithCovarianceStamped,
                         self._on_pose)
        self._proxy = None

    def _on_path(self, msg):
        self.path_signal.emit(msg)

    def _on_state(self, msg):
        self.state_signal.emit(msg.data)

    def _on_pose(self, msg):
        self.pose_signal.emit(msg)

    def plan(self, start_tag, goal_tag):
        if self._proxy is None:
            rospy.wait_for_service('/planner/plan_path', timeout=3.0)
            self._proxy = rospy.ServiceProxy('/planner/plan_path', PlanPath)
        return self._proxy(start_tag=start_tag, goal_tag=goal_tag)

    def cancel(self):
        self.cancel_pub.publish(Empty())

    def publish_marker(self, x, y, is_goal):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'nav'
        marker.id = 0 if is_goal else 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.25
        # Point the arrow up (+z) so it is visible from a top-down rviz view.
        q = tf.transformations.quaternion_from_euler(0.0, -math.pi / 2.0, 0.0)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        marker.scale.x = 0.7
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        if is_goal:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.15, 0.15
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.1, 0.8, 0.2
        marker.lifetime = rospy.Duration(0)
        (self.goal_marker_pub if is_goal else self.start_marker_pub).publish(marker)


class TagNavView(QtWidgets.QWidget):
    def __init__(self, window):
        super().__init__(window)
        self.window_ = window
        self.setMinimumSize(400, 400)

    def _geometry(self):
        w = self.window_
        cell = min(self.width() / w.cols, self.height() / w.rows)
        ox = (self.width() - w.cols * cell) / 2.0
        oy = (self.height() - w.rows * cell) / 2.0
        return cell, ox, oy

    def _cell_at(self, x, y):
        w = self.window_
        cell, ox, oy = self._geometry()
        if x < ox or y < oy:
            return None
        col = int((x - ox) // cell)
        row = int((y - oy) // cell)
        if 0 <= col < w.cols and 0 <= row < w.rows:
            return row * w.cols + col
        return None

    def _screen_of_map(self, mx, my):
        w = self.window_
        cell, ox, oy = self._geometry()
        col = mx - w.min_x
        row = w.max_y - my
        return ox + (col + 0.5) * cell, oy + (row + 0.5) * cell

    def paintEvent(self, _event):
        w = self.window_
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), Qt.white)
        if not w.grid:
            return
        cell, ox, oy = self._geometry()

        for tid, (col, row) in w.grid.items():
            x = ox + col * cell
            y = oy + row * cell
            rect = QtCore.QRectF(x + 0.5, y + 0.5, cell - 1.0, cell - 1.0)
            if tid == w.goal_tag:
                painter.setBrush(QtGui.QColor('#e05a5a'))
                painter.setPen(QtGui.QPen(QtGui.QColor('#8a1a1a'), 2))
            elif tid == w.start_tag:
                painter.setBrush(QtGui.QColor('#54c46b'))
                painter.setPen(QtGui.QPen(QtGui.QColor('#1a6b2e'), 2))
            else:
                painter.setBrush(QtGui.QColor('#eef2f6'))
                painter.setPen(QtGui.QPen(QtGui.QColor('#c0c8d0'), 1))
            painter.drawRect(rect)

        if w.path_points:
            painter.setPen(QtGui.QPen(QtGui.QColor('#2563eb'), 3, Qt.SolidLine, Qt.RoundCap))
            points = [QtCore.QPointF(*self._screen_of_map(mx, my)) for mx, my in w.path_points]
            painter.drawPolyline(QtGui.QPolygonF(points))

    def mousePressEvent(self, event):
        w = self.window_
        if not w.grid:
            return
        tid = self._cell_at(event.x(), event.y())
        if tid is None:
            return
        if event.button() == Qt.RightButton:
            w.set_start(tid)
        else:
            w.set_goal(tid)


class NavGui(QtWidgets.QMainWindow):
    def __init__(self, map_path):
        super().__init__()
        self.setWindowTitle('Tag 导航')
        self.tags, self.grid, self.cols, self.rows, self.min_x, self.max_y = \
            load_tag_map(map_path)
        self.goal_tag = None
        self.start_tag = None
        self.path_points = []
        self.latest_pose = None  # (x, y) in map frame

        self.ros = RosInterface()
        self.ros.path_signal.connect(self.on_path)
        self.ros.state_signal.connect(self.on_state)
        self.ros.pose_signal.connect(self.on_pose)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        self.view = TagNavView(self)
        layout.addWidget(self.view, 1)
        layout.addWidget(self._build_side_panel())
        self.setCentralWidget(central)
        self.resize(920, 780)
        self.statusBar().showMessage('左键点 tag = 终点(红)，右键点 tag = 起点(绿)')

    def _build_side_panel(self):
        panel = QtWidgets.QWidget()
        panel.setFixedWidth(240)
        v = QtWidgets.QVBoxLayout(panel)

        self.start_label = QtWidgets.QLabel('起点：未设置')
        self.goal_label = QtWidgets.QLabel('终点：未设置')
        v.addWidget(self.start_label)
        v.addWidget(self.goal_label)

        use_pose = QtWidgets.QPushButton('用当前位姿作起点')
        use_pose.clicked.connect(self.on_use_pose)
        v.addWidget(use_pose)

        plan_btn = QtWidgets.QPushButton('规划并执行')
        plan_btn.clicked.connect(self.on_plan)
        v.addWidget(plan_btn)

        stop_btn = QtWidgets.QPushButton('停止')
        stop_btn.clicked.connect(self.on_stop)
        v.addWidget(stop_btn)

        v.addStretch(1)

        state_group = QtWidgets.QGroupBox('规划器状态')
        state_layout = QtWidgets.QVBoxLayout()
        self.state_dot = QtWidgets.QLabel()
        self.state_dot.setFixedSize(14, 14)
        self.state_text = QtWidgets.QLabel('IDLE')
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.state_dot)
        row.addWidget(self.state_text)
        row.addStretch(1)
        state_layout.addLayout(row)
        state_group.setLayout(state_layout)
        v.addWidget(state_group)

        self.msg_label = QtWidgets.QLabel('')
        self.msg_label.setWordWrap(True)
        v.addWidget(self.msg_label)
        return panel

    # ----- selection ------------------------------------------------------------
    def set_start(self, tid):
        self.start_tag = tid
        self.start_label.setText(self._tag_text('起点', tid))
        if tid is not None:
            self.ros.publish_marker(*self.tags[tid][:2], is_goal=False)
        self.view.update()

    def set_goal(self, tid):
        self.goal_tag = tid
        self.goal_label.setText(self._tag_text('终点', tid))
        if tid is not None:
            self.ros.publish_marker(*self.tags[tid][:2], is_goal=True)
        self.view.update()

    @staticmethod
    def _tag_text(label, tid):
        return '%s：未设置' % label if tid is None else '%s：tag %d' % (label, tid)

    # ----- ROS slots ------------------------------------------------------------
    def on_path(self, path_msg):
        self.path_points = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        self.view.update()

    def on_state(self, state):
        self.state_text.setText(state)
        colors = {
            'IDLE': '#b0b0b0', 'TURN': '#f0a020', 'DRIVE': '#20a0f0',
            'ARRIVED': '#20c050', 'NO_PATH': '#e04040',
            'LOCALIZATION_LOST': '#e04040',
        }
        self.state_dot.setStyleSheet(
            'background-color: %s; border-radius: 7px;' % colors.get(state, '#b0b0b0'))

    def on_pose(self, pose_msg):
        self.latest_pose = (pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y)

    # ----- actions --------------------------------------------------------------
    def on_use_pose(self):
        if self.latest_pose is None:
            self.msg_label.setText('还没有收到定位 pose')
            return
        px, py = self.latest_pose
        tid = min(self.tags, key=lambda i: math.hypot(self.tags[i][0] - px,
                                                      self.tags[i][1] - py))
        self.set_start(tid)

    def on_plan(self):
        if self.start_tag is None or self.goal_tag is None:
            self.msg_label.setText('请先设置起点和终点 tag')
            return
        try:
            resp = self.ros.plan(self.start_tag, self.goal_tag)
        except (rospy.ServiceException, rospy.ROSException) as error:
            self.msg_label.setText('规划失败：%s' % error)
            return
        if resp.success:
            self.msg_label.setText('已下发规划（%d 个路径点）' % len(resp.path.poses))
            self.on_path(resp.path)
        else:
            self.msg_label.setText('规划失败：%s' % (resp.message or '无路径'))

    def on_stop(self):
        self.ros.cancel()
        self.path_points = []
        self.msg_label.setText('已发送停止')
        self.view.update()

    def closeEvent(self, event):
        rospy.signal_shutdown('GUI closed')
        super().closeEvent(event)


def parse_args():
    default_map = os.path.join(rospkg.RosPack().get_path('tag_nav_bringup'),
                               'worlds', 'maps', 'apriltagMap.json')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', default=default_map, help='path to apriltagMap.json')
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node('tag_nav_gui', anonymous=True)
    app = QtWidgets.QApplication(sys.argv)
    try:
        window = NavGui(args.map)
    except Exception as error:
        QtWidgets.QMessageBox.critical(None, '启动失败', str(error))
        return 1
    window.show()
    threading.Thread(target=rospy.spin, daemon=True).start()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
