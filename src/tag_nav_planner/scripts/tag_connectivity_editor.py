#!/usr/bin/env python3
"""Interactive 8-direction connectivity editor for the tag-navigation planner.

Draws the tag grid from apriltagMap.json and lets the user toggle the 8 compass
directions (N/NE/E/SE/S/SW/W/NW) of each tag, then saves the result to
connectivity.json. This is an offline tool: the running planner picks the file
up on its next start (restart required).
"""

import argparse
import json
import math
import os
import sys

import rospkg
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

# (bit, name, dx, dy, dcol, drow). dx/dy are map coords (+x east, +y north);
# dcol/drow are grid coords (row increases southward). MUST match core/direction.h.
DIRS = [
    (0, 'N',   0, +1,  0, -1),
    (1, 'NE', +1, +1, +1, -1),
    (2, 'E',  +1,  0, +1,  0),
    (3, 'SE', +1, -1, +1, +1),
    (4, 'S',   0, -1,  0, +1),
    (5, 'SW', -1, -1, -1, +1),
    (6, 'W',  -1,  0, -1,  0),
    (7, 'NW', -1, +1, -1, -1),
]
OPP = [(i + 4) % 8 for i in range(8)]
DIR_NAME = {bit: name for bit, name, _dx, _dy, _dc, _dr in DIRS}
# Compass layout for the checkbox panel: (bit, grid_row, grid_col).
BOX_POSITIONS = [(7, 0, 0), (0, 0, 1), (1, 0, 2),
                 (6, 1, 0),             (2, 1, 2),
                 (5, 2, 0), (4, 2, 1), (3, 2, 2)]


class TagGridView(QtWidgets.QWidget):
    """Paints the tag grid and handles click-to-select / click-neighbor-to-toggle."""

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
            if tid == w.selected:
                painter.setBrush(QtGui.QColor('#4a90d9'))
                painter.setPen(QtGui.QPen(QtGui.QColor('#1a4a7a'), 2))
            else:
                painter.setBrush(QtGui.QColor('#eef2f6'))
                painter.setPen(QtGui.QPen(QtGui.QColor('#c0c8d0'), 1))
            painter.drawRect(rect)

        if w.show_all:
            painter.setPen(QtGui.QPen(QtGui.QColor('#b8c4d0'), 1))
            for tid, (col, row) in w.grid.items():
                mask = w.masks.get(tid, 0)
                cx = ox + (col + 0.5) * cell
                cy = oy + (row + 0.5) * cell
                for bit, _name, _dx, _dy, dcol, drow in DIRS:
                    if not (mask & (1 << bit)):
                        continue
                    nid = w.neighbor_id(tid, bit)
                    if nid is None:
                        continue
                    ncol, nrow = w.grid[nid]
                    painter.drawLine(QtCore.QPointF(cx, cy),
                                     QtCore.QPointF(ox + (ncol + 0.5) * cell,
                                                    oy + (nrow + 0.5) * cell))

        if w.selected is not None:
            col, row = w.grid[w.selected]
            mask = w.masks.get(w.selected, 0)
            cx = ox + (col + 0.5) * cell
            cy = oy + (row + 0.5) * cell
            for bit, _name, _dx, _dy, dcol, drow in DIRS:
                on = bool(mask & (1 << bit))
                color = QtGui.QColor('#e05a00') if on else QtGui.QColor('#d8dde3')
                end_x = cx + dcol * cell * 0.5
                end_y = cy + drow * cell * 0.5
                painter.setPen(QtGui.QPen(color, 3, Qt.SolidLine, Qt.RoundCap))
                painter.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(end_x, end_y))
                if on:
                    self._arrowhead(painter, cx, cy, end_x, end_y, color)

    @staticmethod
    def _arrowhead(painter, x1, y1, x2, y2, color):
        ang = math.atan2(y2 - y1, x2 - x1)
        head = 7.0
        p1 = QtCore.QPointF(x2 - head * math.cos(ang - math.pi / 6),
                            y2 - head * math.sin(ang - math.pi / 6))
        p2 = QtCore.QPointF(x2 - head * math.cos(ang + math.pi / 6),
                            y2 - head * math.sin(ang + math.pi / 6))
        painter.setBrush(color)
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(x2, y2), p1, p2]))

    def mousePressEvent(self, event):
        w = self.window_
        if not w.grid:
            return
        cell, ox, oy = self._geometry()
        if event.x() < ox or event.y() < oy:
            return
        col = int((event.x() - ox) // cell)
        row = int((event.y() - oy) // cell)
        if not (0 <= col < w.cols and 0 <= row < w.rows):
            return
        clicked = row * w.cols + col
        if w.selected is not None and clicked != w.selected:
            d = w.direction_between(w.selected, clicked)
            if d is not None:
                on = not (w.masks.get(w.selected, 0) & (1 << d))
                w.set_dir(w.selected, d, on)
                w.refresh_boxes()
                self.update()
                return
        w.selected = clicked
        w.refresh_boxes()
        self.update()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, map_path, connectivity_path):
        super().__init__()
        self.setWindowTitle('Tag 八向连通性编辑器')
        self.map_path = map_path
        self.connectivity_path = connectivity_path
        self.tags = {}
        self.grid = {}
        self.cols = 0
        self.rows = 0
        self.masks = {}
        self.selected = None
        self.mirror = True
        self.show_all = False

        self._load_map()
        self._load_connectivity(silent_missing=True)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(central)
        self.view = TagGridView(self)
        layout.addWidget(self.view, 1)
        layout.addWidget(self._build_side_panel())
        self.setCentralWidget(central)
        self.resize(920, 780)
        self.refresh_boxes()
        self.statusBar().showMessage('点击 tag 选择，点击其相邻 tag 切换边 | 保存后需重启 planner 生效')

    # ----- data loading / saving ------------------------------------------------
    def _load_map(self):
        with open(self.map_path) as f:
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
        cols = int(round(max(xs) - min(xs))) + 1
        rows = int(round(max(ys) - min(ys))) + 1
        grid = {}
        for tid, (x, y, _yaw) in tags.items():
            grid[tid] = (int(round(x - min(xs))), int(round(max(ys) - y)))
        self.tags, self.grid, self.cols, self.rows = tags, grid, cols, rows

    def _load_connectivity(self, silent_missing=False):
        if not os.path.exists(self.connectivity_path):
            self.masks = {}
            self._notify_load_result('未找到 %s，从空掩码开始' % self.connectivity_path,
                                     silent_missing)
            return
        try:
            with open(self.connectivity_path) as f:
                data = json.load(f)
            if data.get('schema_version', 1) != 1:
                raise ValueError('schema_version must be 1')
            conn = data.get('connectivity', {})
            masks = {}
            for key, value in conn.items():
                if not isinstance(value, int) or not (0 <= value <= 255):
                    raise ValueError('connectivity["%s"] must be an int in [0, 255]' % key)
                tid = int(key)
                if tid not in self.grid:
                    raise ValueError('connectivity id %s is not in the tag map' % key)
                masks[tid] = value
            self.masks = masks
            self.statusBar().showMessage('已载入 %d 个 tag 的连通性' % len(masks))
        except Exception as error:
            self.masks = {}
            QtWidgets.QMessageBox.critical(self, '载入 connectivity 失败', str(error))

    def _notify_load_result(self, message, silent):
        if silent:
            self.statusBar().showMessage(message)
        else:
            QtWidgets.QMessageBox.information(self, '未找到', message)

    def _save_connectivity(self, path):
        obj = {
            'schema_version': 1,
            'connectivity': {str(tid): self.masks.get(tid, 0) for tid in sorted(self.tags)},
        }
        tmp = path + '.tmp'
        try:
            with open(tmp, 'w') as f:
                json.dump(obj, f, indent=2)
                f.write('\n')
            os.replace(tmp, path)
        except Exception as error:
            QtWidgets.QMessageBox.critical(self, '保存失败', str(error))
            return
        self.connectivity_path = path
        self.statusBar().showMessage('已保存 %d 个 tag 到 %s（重启 planner 生效）'
                                     % (len(obj['connectivity']), path))

    # ----- graph helpers --------------------------------------------------------
    def neighbor_id(self, tid, bit):
        col, row = self.grid[tid]
        dcol = DIRS[bit][4]
        drow = DIRS[bit][5]
        ncol, nrow = col + dcol, row + drow
        if 0 <= ncol < self.cols and 0 <= nrow < self.rows:
            return nrow * self.cols + ncol
        return None

    def direction_between(self, a, b):
        col_a, row_a = self.grid[a]
        col_b, row_b = self.grid[b]
        dcol, drow = col_b - col_a, row_b - row_a
        for bit, _name, _dx, _dy, dc, dr in DIRS:
            if dc == dcol and dr == drow:
                return bit
        return None

    def set_dir(self, tid, bit, on):
        mask = self.masks.get(tid, 0)
        mask = (mask | (1 << bit)) if on else (mask & ~(1 << bit))
        self.masks[tid] = mask & 0xFF
        if self.mirror:
            nid = self.neighbor_id(tid, bit)
            if nid is not None:
                obit = OPP[bit]
                nmask = self.masks.get(nid, 0)
                nmask = (nmask | (1 << obit)) if on else (nmask & ~(1 << obit))
                self.masks[nid] = nmask & 0xFF

    # ----- UI handlers ----------------------------------------------------------
    def _build_side_panel(self):
        panel = QtWidgets.QWidget()
        panel.setFixedWidth(250)
        v = QtWidgets.QVBoxLayout(panel)

        group = QtWidgets.QGroupBox('方向（选中 tag）')
        grid = QtWidgets.QGridLayout()
        self.dir_boxes = {}
        for bit, row, col in BOX_POSITIONS:
            checkbox = QtWidgets.QCheckBox(DIR_NAME[bit])
            checkbox.toggled.connect(lambda checked, b=bit: self.on_dir_toggled(b, checked))
            self.dir_boxes[bit] = checkbox
            grid.addWidget(checkbox, row, col)
        group.setLayout(grid)
        v.addWidget(group)

        self.mirror_box = QtWidgets.QCheckBox('双向镜像（对称）')
        self.mirror_box.setChecked(True)
        self.mirror_box.toggled.connect(self.on_mirror_toggled)
        v.addWidget(self.mirror_box)

        self.show_all_box = QtWidgets.QCheckBox('显示所有连线')
        self.show_all_box.toggled.connect(self.on_show_all_toggled)
        v.addWidget(self.show_all_box)

        fill = QtWidgets.QPushButton('全选该 tag（界内方向）')
        fill.clicked.connect(self.on_fill)
        v.addWidget(fill)

        clear = QtWidgets.QPushButton('清空该 tag')
        clear.clicked.connect(self.on_clear)
        v.addWidget(clear)

        v.addStretch(1)

        reload_btn = QtWidgets.QPushButton('重新载入 connectivity.json')
        reload_btn.clicked.connect(self.on_reload)
        v.addWidget(reload_btn)

        save_btn = QtWidgets.QPushButton('保存')
        save_btn.clicked.connect(self.on_save)
        v.addWidget(save_btn)

        saveas_btn = QtWidgets.QPushButton('另存为…')
        saveas_btn.clicked.connect(self.on_save_as)
        v.addWidget(saveas_btn)

        self.selected_label = QtWidgets.QLabel('未选中 tag')
        v.addWidget(self.selected_label)
        return panel

    def refresh_boxes(self):
        for checkbox in self.dir_boxes.values():
            checkbox.blockSignals(True)
        try:
            if self.selected is None:
                for checkbox in self.dir_boxes.values():
                    checkbox.setChecked(False)
                    checkbox.setEnabled(False)
                self.selected_label.setText('未选中 tag')
            else:
                mask = self.masks.get(self.selected, 0)
                x, y, _yaw = self.tags[self.selected]
                self.selected_label.setText('tag %d  (%0.0f, %0.0f)  掩码 %d'
                                            % (self.selected, x, y, mask))
                for bit, checkbox in self.dir_boxes.items():
                    checkbox.setEnabled(True)
                    checkbox.setChecked(bool(mask & (1 << bit)))
        finally:
            for checkbox in self.dir_boxes.values():
                checkbox.blockSignals(False)

    def on_dir_toggled(self, bit, checked):
        if self.selected is None:
            return
        self.set_dir(self.selected, bit, checked)
        self.view.update()

    def on_mirror_toggled(self, checked):
        self.mirror = checked

    def on_show_all_toggled(self, checked):
        self.show_all = checked
        self.view.update()

    def on_fill(self):
        if self.selected is None:
            return
        mask = self.masks.get(self.selected, 0)
        for bit, _name, _dx, _dy, _dc, _dr in DIRS:
            if self.neighbor_id(self.selected, bit) is not None:
                mask |= (1 << bit)
        self.masks[self.selected] = mask & 0xFF
        self.refresh_boxes()
        self.view.update()

    def on_clear(self):
        if self.selected is None:
            return
        self.masks[self.selected] = 0
        self.refresh_boxes()
        self.view.update()

    def on_reload(self):
        self._load_connectivity(silent_missing=False)
        self.refresh_boxes()
        self.view.update()

    def on_save(self):
        self._save_connectivity(self.connectivity_path)

    def on_save_as(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, '保存 connectivity.json', self.connectivity_path, 'JSON (*.json)')
        if path:
            self._save_connectivity(path)


def parse_args():
    default_map = os.path.join(rospkg.RosPack().get_path('tag_nav_bringup'),
                               'worlds', 'maps', 'apriltagMap.json')
    default_conn = os.path.join(rospkg.RosPack().get_path('tag_nav_planner'),
                                'config', 'connectivity.json')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', default=default_map, help='path to apriltagMap.json')
    parser.add_argument('--connectivity', default=default_conn,
                        help='path to connectivity.json')
    return parser.parse_args()


def main():
    args = parse_args()
    app = QtWidgets.QApplication(sys.argv)
    try:
        window = MainWindow(args.map, args.connectivity)
    except Exception as error:
        QtWidgets.QMessageBox.critical(None, '启动失败', str(error))
        return 1
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
