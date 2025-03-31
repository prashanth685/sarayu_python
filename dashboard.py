import sys
import numpy as np
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QLabel, QPushButton, QHBoxLayout, QTableWidget,
                             QTableWidgetItem, QSplitter, QTextEdit, QInputDialog, QMessageBox,
                             QToolButton, QLineEdit, QComboBox, QHeaderView, QSizePolicy,
                             QToolBar, QAction, QScrollArea, QDateTimeEdit, QApplication, QTreeWidget, QTreeWidgetItem,
                             QListWidget, QListWidgetItem)
from PyQt5.QtCore import Qt, QSize, QDateTime, QTimer, QPointF
from PyQt5.QtGui import QIcon, QCursor
import os
import csv
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from datetime import datetime, timedelta
from mqtthandler import MQTTHandler
import logging
from collections import deque
from matplotlib.backends.backend_pdf import PdfPages

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class DashboardWindow(QWidget):
    def __init__(self, db, email):
        super().__init__()
        self.db = db
        self.email = email
        self.current_project = None
        self.current_feature = None
        self.selected_tags = []
        self.mqtt_handler = None
        self.mqtt_tag = None
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_mqtt_plot)
        self.time_timer = QTimer(self)
        self.time_timer.timeout.connect(self.update_time_view_plot)
        self.time_view_buffer = deque(maxlen=2048)
        self.time_view_timestamps = deque(maxlen=2048)
        
        self.time_report_buffer = {}
        self.time_report_timestamps = {}
        self.time_report_timer = QTimer(self)
        self.time_report_timer.timeout.connect(self.update_time_report_plot)
        
        self.figure = plt.Figure(figsize=(10, 6))
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setParent(self)
        self.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.canvas.mpl_connect('scroll_event', self.on_scroll)
        self.canvas.mpl_connect('button_press_event', self.on_press)
        self.canvas.mpl_connect('button_release_event', self.on_release)
        self.canvas.mpl_connect('motion_notify_event', self.on_drag)
        
        self.dragging = False
        self.press_x = None
        
        self.initUI()
        self.setup_mqtt()

    def setup_mqtt(self):
        if self.current_project:
            if self.mqtt_handler:
                self.mqtt_handler.stop()
            self.mqtt_handler = MQTTHandler(self.db, self.current_project)
            self.mqtt_handler.data_received.connect(self.on_data_received)
            self.mqtt_handler.start()
            logging.info(f"MQTT setup for project: {self.current_project}")

    def on_data_received(self, tag_name, values):
        timestamp = datetime.now().isoformat()
        if self.current_feature == "Time View" and tag_name == self.mqtt_tag:
            self.time_view_buffer.extend(values)
            self.time_view_timestamps.extend([timestamp] * len(values))
            logging.debug(f"Time View - Received {len(values)} values for {tag_name}")
        if self.current_feature == "Time Report" and tag_name in self.selected_tags:
            if tag_name not in self.time_report_buffer:
                self.time_report_buffer[tag_name] = deque(maxlen=10000)
                self.time_report_timestamps[tag_name] = deque(maxlen=10000)
            self.time_report_buffer[tag_name].extend(values)
            self.time_report_timestamps[tag_name].extend([timestamp] * len(values))
            logging.debug(f"Time Report - Received {len(values)} values for {tag_name}")

    def initUI(self):
        self.setWindowTitle('Sarayu Dashboard')
        self.setGeometry(100, 100, 1200, 800)

        main_layout = QVBoxLayout()
        self.setLayout(main_layout)

        self.file_bar = QToolBar("File")
        self.file_bar.setStyleSheet("""
            QToolBar { background-color: #c3cb9b; border: none; padding: 5px; spacing: 10px; }
            QToolBar QToolButton { font-size: 16px; font-weight: bold; padding: 5px; }
            QToolBar QToolButton:hover { background-color: #a9b37e; }
        """)
        self.file_bar.setFixedHeight(40)
        self.file_bar.setMovable(False)
        self.file_bar.setFloatable(False)

        actions = [
            ("Home", self.display_dashboard),
            ("New", self.create_project),
            ("Open", self.open_project_dialog),
            ("Save", self.save_action),
            ("Settings", self.settings_action),
            ("Refresh", self.refresh_action),
            ("Exit", self.close)
        ]
        for text, func in actions:
            action = QAction(text, self)
            action.triggered.connect(func)
            self.file_bar.addAction(action)
        main_layout.addWidget(self.file_bar)

        self.toolbar = QToolBar("Navigation")
        self.update_toolbar()
        main_layout.addWidget(self.toolbar)

        main_splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(main_splitter)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabel("Projects")
        self.tree.setStyleSheet("""
            QTreeWidget { background-color: #2c3e50; color: white; border: none;}
            QTreeWidget::item { padding: 5px; text-align: center; }
            QTreeWidget::item:hover { background-color: #4a6077; }
            QTreeWidget::item:selected { background-color: #1a73e8; }
        """)
        self.tree.setFixedWidth(300)
        self.tree.itemClicked.connect(self.on_tree_item_clicked)
        main_splitter.addWidget(self.tree)

        content_container = QWidget()
        content_layout = QVBoxLayout()
        content_container.setLayout(content_layout)
        content_container.setStyleSheet("background-color: #34495e;")

        self.content = QWidget()
        self.content_layout = QVBoxLayout()
        self.content.setLayout(self.content_layout)
        content_layout.addWidget(self.content)
        content_layout.addStretch()

        main_splitter.addWidget(content_container)
        main_splitter.setSizes([300, 900])
        main_splitter.setHandleWidth(0)

        self.load_projects()
        self.display_dashboard()

    def update_toolbar(self):
        self.toolbar.clear()
        self.toolbar.setStyleSheet("""
            QToolBar { background-color: #83afa5; border: none; padding: 5px; spacing: 5px; margin: 0; }
            QToolBar::separator { width: 1px; margin: 0; }
            QToolButton { border: none; padding: 8px; border: 1px solid black; margin: 0; border-radius: 5px; background-color: #1e2937; }
            QToolButton:hover { background-color: #e0e0e0; }
            QToolButton:pressed { background-color: #d0d0d0; }
            QToolButton:focus { outline: none; border: 1px solid #0078d7; }
        """)
        self.toolbar.setIconSize(QSize(24, 24))
        self.toolbar.setMovable(False)
        self.toolbar.setFloatable(False)

        def add_action(text, icon_path, callback, tooltip=None):
            icon = QIcon(icon_path) if os.path.exists(icon_path) else QIcon()
            action = QAction(icon, text, self)
            action.triggered.connect(callback)
            if tooltip:
                action.setToolTip(tooltip)
            self.toolbar.addAction(action)

        add_action("New", "icons/new.png", self.create_project, "Create a New Project")
        add_action("Open", "icons/open.png", self.open_project_dialog, "Open an Existing Project")
        add_action("", "icons/save.png", self.save_action, "Save Project")
        add_action("", "icons/refresh.png", self.refresh_action, "Refresh View")
        add_action("", "icons/edit.png", self.edit_project_dialog, "Edit Project Name")
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.toolbar.addWidget(spacer)
        add_action("Settings", "icons/settings.png", self.settings_action, "Settings")

    def close_project(self):
        if self.mqtt_handler:
            self.mqtt_handler.stop()
            self.mqtt_handler = None
        self.current_project = None
        self.current_feature = None
        self.selected_tags = []
        self.timer.stop()
        self.time_timer.stop()
        self.time_report_timer.stop()
        self.update_toolbar()
        self.display_dashboard()

    def open_project_dialog(self):
        project_name, ok = QInputDialog.getItem(self, "Open Project", "Select a project:", self.db.projects, 0, False)
        if ok and project_name:
            self.current_project = project_name
            self.current_feature = None
            self.selected_tags = []
            self.timer.stop()
            self.time_timer.stop()
            self.time_report_timer.stop()
            self.update_toolbar()
            self.setup_mqtt()
            self.display_feature_content("Create Tags", project_name)

    def display_dashboard(self):
        if self.mqtt_handler:
            self.mqtt_handler.stop()
            self.mqtt_handler = None
        self.current_project = None
        self.current_feature = None
        self.selected_tags = []
        self.timer.stop()
        self.time_timer.stop()
        self.time_report_timer.stop()
        if hasattr(self, 'canvas') and self.canvas.parent():
            self.canvas.setParent(None)
        self.update_toolbar()

        while self.content_layout.count():
            item = self.content_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        header = QLabel("Welcome to Sarayu Application")
        header.setStyleSheet("color: white; font-size: 24px; font-weight: bold; padding: 10px;")
        self.content_layout.addWidget(header, alignment=Qt.AlignCenter)

    def load_projects(self):
        self.db.load_projects()
        self.tree.clear()
        for project_name in self.db.projects:
            self.add_project_to_tree(project_name)

    def add_project_to_tree(self, project_name):
        project_item = QTreeWidgetItem(self.tree)
        project_item.setText(0, project_name)
        project_item.setIcon(0, QIcon("icons/folder.png") if os.path.exists("icons/folder.png") else QIcon())
        project_item.setData(0, Qt.UserRole, {"type": "project", "name": project_name})

        features = [
            ("Create Tags", "icons/tag.png"),
            ("Time View", "icons/time.png"),
            ("Tabular View", "icons/table.png"),
            ("FFT", "icons/fft.png"),
            ("Waterfall", "icons/waterfall.png"),
            ("Orbit", "icons/orbit.png"),
            ("Trend View", "icons/trend.png"),
            ("Multiple Trend View", "icons/multitrend.png"),
            ("Bode Plot", "icons/bode.png"),
            ("History Plot", "icons/history.png"),
            ("Time Report", "icons/report.png"),
            ("Report", "icons/report.png")
        ]

        for feature, icon_path in features:
            feature_item = QTreeWidgetItem(project_item)
            feature_item.setText(0, feature)
            feature_item.setIcon(0, QIcon(icon_path) if os.path.exists(icon_path) else QIcon())
            feature_item.setData(0, Qt.UserRole, {"type": "feature", "name": feature, "project": project_name})

    def on_tree_item_clicked(self, item, column):
        data = item.data(0, Qt.UserRole)
        if data["type"] == "project":
            self.current_project = data["name"]
            self.current_feature = None
            self.setup_mqtt()
            self.display_dashboard()
        elif data["type"] == "feature":
            self.current_project = data["project"]
            self.current_feature = data["name"]
            self.setup_mqtt()
            self.display_feature_content(data["name"], data["project"])

    def create_project(self):
        project_name, ok = QInputDialog.getText(self, "Create Project", "Enter project name:")
        if ok and project_name:
            success, message = self.db.create_project(project_name)
            if success:
                self.add_project_to_tree(project_name)
                QMessageBox.information(self, "Success", message)
                self.current_project = project_name
                self.current_feature = None
                self.update_toolbar()
                self.setup_mqtt()
                self.display_feature_content("Create Tags", project_name)
            else:
                QMessageBox.warning(self, "Error", message)

    def edit_project_dialog(self):
        if not self.current_project:
            QMessageBox.warning(self, "Error", "No project selected to edit!")
            return

        old_project_name = self.current_project
        new_project_name, ok = QInputDialog.getText(self, "Edit Project", "Enter new project name:", text=old_project_name)
        if not ok or not new_project_name or new_project_name == old_project_name:
            return

        success, message = self.db.edit_project(old_project_name, new_project_name)
        if success:
            for i in range(self.tree.topLevelItemCount()):
                item = self.tree.topLevelItem(i)
                if item.text(0) == old_project_name:
                    item.setText(0, new_project_name)
                    item.setData(0, Qt.UserRole, {"type": "project", "name": new_project_name})
                    for j in range(item.childCount()):
                        child = item.child(j)
                        child_data = child.data(0, Qt.UserRole)
                        child_data["project"] = new_project_name
                        child.setData(0, Qt.UserRole, child_data)
                    break
            
            self.current_project = new_project_name
            self.setup_mqtt()
            self.update_toolbar()
            self.display_feature_content(self.current_feature or "Create Tags", self.current_project)
            QMessageBox.information(self, "Success", message)
        else:
            QMessageBox.warning(self, "Error", message)

    def delete_project(self, project_name, project_button):
        reply = QMessageBox.question(self, "Confirm Delete", f"Are you sure you want to delete {project_name}?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            success, message = self.db.delete_project(project_name)
            if success:
                for i in range(self.tree.topLevelItemCount()):
                    if self.tree.topLevelItem(i).text(0) == project_name:
                        self.tree.takeTopLevelItem(i)
                        break
                if self.current_project == project_name:
                    self.current_project = None
                    self.current_feature = None
                    if self.mqtt_handler:
                        self.mqtt_handler.stop()
                        self.mqtt_handler = None
                    self.update_toolbar()
                    self.display_dashboard()
                QMessageBox.information(self, "Success", message)
            else:
                QMessageBox.warning(self, "Error", message)

    def start_mqtt_plotting(self, output_widget, tag_name):
        if not self.current_project or not tag_name:
            QMessageBox.warning(self, "Error", "No project or tag selected for MQTT plotting!")
            return
        self.output_widget = output_widget
        self.mqtt_tag = tag_name
        self.timer.stop()
        self.timer.setInterval(1000)
        
        if self.canvas.parent() != self.feature_widget:
            if self.canvas.parent():
                self.canvas.setParent(None)
            self.feature_layout.addWidget(self.canvas)
        self.figure.clear()
        self.timer.start()

    def update_mqtt_plot(self):
        if not self.current_project or not self.mqtt_tag:
            self.output_widget.setText("No project or tag selected for MQTT plotting.")
            return

        data = self.db.get_tag_values(self.current_project, self.mqtt_tag)
        if not data:
            self.output_widget.setText(f"No MQTT data received for {self.mqtt_tag} yet.")
            return

        latest_values = data[-1]["values"]
        self.output_widget.setText(f"MQTT Data for {self.mqtt_tag}:\nLatest 10 values: {latest_values[-10:]}")

        self.figure.clear()
        ax = self.figure.add_subplot(111, projection='3d' if self.current_feature == "Waterfall" else None)

        if self.current_feature == "Trend View":
            time_points = np.linspace(0, 10.24, 1024)
            ax.plot(time_points, latest_values, 'b-', label=f'{self.mqtt_tag}')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Value (m/s)')
            ax.set_title(f'Trend View for {self.mqtt_tag}')
            ax.set_xlim(0, 10.24)
            ax.set_ylim(16390, 46537)
            ax.grid(True)
            ax.legend()

        elif self.current_feature == "FFT":
            fft_data = np.abs(np.fft.fft(latest_values))[:512]
            freqs = np.fft.fftfreq(1024, 0.01)[:512]
            ax.plot(freqs, fft_data, 'b-')
            ax.set_xlabel('Frequency (Hz)')
            ax.set_ylabel('Magnitude')
            ax.set_title(f'FFT for {self.mqtt_tag}')
            ax.set_xlim(0, 50)
            ax.grid(True)

        elif self.current_feature == "Waterfall":
            waterfall_data = [d["values"] for d in data[-10:]]
            X = np.linspace(0, 10.24, 1024)
            Y = np.arange(len(waterfall_data))
            X, Y = np.meshgrid(X, Y)
            Z = np.array(waterfall_data)
            ax.plot_surface(X, Y, Z, cmap='viridis')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Message Index')
            ax.set_zlabel('Value (m/s)')
            ax.set_title(f'Waterfall for {self.mqtt_tag}')

        elif self.current_feature == "Orbit":
            x_data = self.db.get_tag_values(self.current_project, "tag2")
            y_data = self.db.get_tag_values(self.current_project, "tag3")
            if x_data and y_data:
                x_values = x_data[-1]["values"]
                y_values = y_data[-1]["values"]
                ax.plot(x_values, y_values, 'b-')
                ax.set_xlabel('X Value (m/s)')
                ax.set_ylabel('Y Value (m/s)')
                ax.set_title('Orbit Plot')
                ax.set_xlim(16390, 46537)
                ax.set_ylim(16390, 46537)
                ax.grid(True)
                ax.set_aspect('equal')
            else:
                self.output_widget.setText("Orbit requires data from tag2 and tag3.")

        elif self.current_feature == "Multiple Trend View":
            tags_data = list(self.db.tags_collection.find({"project_name": self.current_project}))
            time_points = np.linspace(0, 10.24, 1024)
            colors = ['b-', 'g-', 'r-']
            for i, tag in enumerate(tags_data[:3]):
                tag_data = self.db.get_tag_values(self.current_project, tag["tag_name"])
                if tag_data:
                    ax.plot(time_points, tag_data[-1]["values"], colors[i % len(colors)], label=tag["tag_name"])
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Value (m/s)')
            ax.set_title('Multiple Trend View')
            ax.set_xlim(0, 10.24)
            ax.set_ylim(16390, 46537)
            ax.grid(True)
            ax.legend()

        elif self.current_feature == "Bode Plot":
            fft_data = np.fft.fft(latest_values)
            freqs = np.fft.fftfreq(1024, 0.01)[:512]
            mag = 20 * np.log10(np.abs(fft_data[:512]) + 1e-10)
            phase = np.angle(fft_data[:512], deg=True)
            ax2 = ax.twinx()
            ax.plot(freqs, mag, 'b-', label='Magnitude (dB)')
            ax2.plot(freqs, phase, 'r-', label='Phase (deg)')
            ax.set_xlabel('Frequency (Hz)')
            ax.set_ylabel('Magnitude (dB)', color='b')
            ax2.set_ylabel('Phase (deg)', color='r')
            ax.set_title(f'Bode Plot for {self.mqtt_tag}')
            ax.set_xlim(0, 50)
            ax.grid(True)
            ax.legend(loc='upper left')
            ax2.legend(loc='upper right')

        self.canvas.draw()

    def setup_time_view_plot(self, tag_name):
        if not self.current_project or not tag_name:
            logging.warning("No project or tag selected for Time View!")
            return
        self.mqtt_tag = tag_name
        self.time_timer.stop()
        self.time_timer.setInterval(100)

        self.time_view_buffer.clear()
        self.time_view_timestamps.clear()
        data = self.db.get_tag_values(self.current_project, self.mqtt_tag)
        if data:
            for entry in data[-2:]:
                self.time_view_buffer.extend(entry["values"])
                self.time_view_timestamps.extend([entry["timestamp"]] * len(entry["values"]))

        if self.canvas.parent() != self.time_widget:
            if self.canvas.parent():
                self.canvas.setParent(None)
            self.time_layout.addWidget(self.canvas)
        
        self.figure.clear()
        self.ax = self.figure.add_subplot(111)
        self.line, = self.ax.plot([], [], 'b-', linewidth=1.5, color='darkblue')
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.set_ylabel("Values", rotation=90, labelpad=10)
        self.ax.yaxis.set_label_position("right")
        self.ax.yaxis.tick_right()
        self.ax.set_xlabel("Time (HH:MM:SSS)")
        self.ax.set_xlim(0, 1)  # Set default to 1-second window
        self.ax.set_xticks(np.linspace(0, 1, 10))  # 10 ticks across 1 second
        self.annotation = self.ax.annotate("", xy=(0, 0), xytext=(20, 20), textcoords="offset points",
                                          bbox=dict(boxstyle="round", fc="w"), arrowprops=dict(arrowstyle="->"))
        self.annotation.set_visible(False)
        self.figure.subplots_adjust(left=0.05, right=0.85, top=0.95, bottom=0.15)
        self.canvas.setMinimumSize(1000, 600)
        self.canvas.draw()

        self.time_timer.start()

    def generate_y_ticks(self, values):
        if not values:
            return np.arange(16390, 46538, 5000)
        y_max = max(values, default=46537)
        y_min = min(values, default=16390)
        padding = (y_max - y_min) * 0.1 if y_max != y_min else 5000
        y_max += padding
        y_min -= padding
        range_val = y_max - y_min
        step = max(range_val / 10, 1)
        step = np.ceil(step / 500) * 500
        ticks = []
        current = np.floor(y_min / step) * step
        while current <= y_max:
            ticks.append(current)
            current += step
        return ticks

    def update_time_view_plot(self):
        if not self.current_project or not self.mqtt_tag:
            self.time_result.setText("No project or tag selected for Time View.")
            return

        if len(self.time_view_buffer) < 1024:
            self.time_result.setText(f"Waiting for sufficient data for {self.mqtt_tag} (Current buffer: {len(self.time_view_buffer)}/1024).")
            return

        xlim = self.ax.get_xlim()
        window_size = xlim[1] - xlim[0]
        samples_per_window = int(1024 * window_size)
        
        if len(self.time_view_buffer) < samples_per_window:
            samples_per_window = len(self.time_view_buffer)

        window_values = list(self.time_view_buffer)[-samples_per_window:]
        window_timestamps = list(self.time_view_timestamps)[-samples_per_window:]
        
        time_points = np.linspace(xlim[0], xlim[1], samples_per_window)
        self.line.set_data(time_points, window_values)
        
        y_max = max(window_values)
        y_min = min(window_values)
        padding = (y_max - y_min) * 0.1 if y_max != y_min else 5000
        self.ax.set_ylim(y_min - padding, y_max + padding)
        self.ax.set_yticks(self.generate_y_ticks(window_values))

        if window_timestamps:
            latest_dt = datetime.strptime(window_timestamps[-1], "%Y-%m-%dT%H:%M:%S.%f")
            time_labels = []
            tick_positions = np.linspace(xlim[0], xlim[1], 10)  # 10 ticks
            for tick in tick_positions:
                delta_seconds = tick - xlim[1]
                tick_dt = latest_dt + timedelta(seconds=delta_seconds)
                milliseconds = tick_dt.microsecond // 1000
                time_labels.append(f"{tick_dt.strftime('%H:%M:')}{milliseconds:03d}")
            self.ax.set_xticks(tick_positions)
            self.ax.set_xticklabels(time_labels, rotation=0)

        for txt in self.ax.texts:
            txt.remove()

        if window_timestamps:
            left_ts = datetime.strptime(window_timestamps[0], "%Y-%m-%dT%H:%M:%S.%f")
            right_ts = datetime.strptime(window_timestamps[-1], "%Y-%m-%dT%H:%M:%S.%f")
            left_ms = left_ts.microsecond // 1000
            right_ms = right_ts.microsecond // 1000
            left_ts_ms = f"{left_ts.strftime('%H:%M:')}{left_ms:03d}"
            right_ts_ms = f"{right_ts.strftime('%H:%M:')}{right_ms:03d}"

        self.canvas.draw()
        self.time_result.setText(f"Time View Data for {self.mqtt_tag}, Latest value: {window_values[-1]}, Window: {window_size:.2f}s")

    def reset_time_view(self):
        if hasattr(self, 'ax'):
            self.ax.set_xlim(0, 1)  # Reset to default 1-second window
            self.ax.set_xticks(np.linspace(0, 1, 10))  # Reset to 10 ticks
            self.canvas.draw()
            logging.debug("Time View reset to default 1-second window with 10 ticks")

    def on_mouse_move(self, event):
        if event.inaxes == self.ax:
            x, y = event.xdata, event.ydata
            if x is not None and y is not None:
                xlim = self.ax.get_xlim()
                window_size = xlim[1] - xlim[0]
                samples_per_window = int(1024 * window_size)
                idx = int(round((x - xlim[0]) / window_size * (samples_per_window - 1)))
                window_values = list(self.time_view_buffer)[-samples_per_window:]
                if 0 <= idx < len(window_values):
                    value = window_values[idx]
                    self.annotation.xy = (x, y)
                    self.annotation.set_text(f"Value: {value:.2f}")
                    self.annotation.set_visible(True)
                    self.canvas.draw_idle()
            else:
                self.annotation.set_visible(False)
                self.canvas.draw_idle()

    def on_scroll(self, event):
        if event.inaxes:
            ax = event.inaxes
            xlim = ax.get_xlim()
            x_range = xlim[1] - xlim[0]
            center = event.xdata if event.xdata is not None else (xlim[0] + xlim[1]) / 2
            scale = 1.1 if event.button == 'down' else 0.9
            new_range = x_range * scale
            if self.current_feature == "Time View":
                if new_range < 0.1:
                    new_range = 0.1
                elif new_range > 10:
                    new_range = 10
            ax.set_xlim(center - new_range / 2, center + new_range / 2)
            self.canvas.draw()
            logging.debug(f"Zoomed: new window size {new_range:.2f}s")

    def on_press(self, event):
        if event.inaxes and event.button == 1:
            self.dragging = True
            self.press_x = event.xdata

    def on_release(self, event):
        self.dragging = False

    def on_drag(self, event):
        if self.dragging and event.inaxes:
            ax = event.inaxes
            if self.press_x is not None and event.xdata is not None:
                dx = self.press_x - event.xdata
                xlim = ax.get_xlim()
                new_left = xlim[0] + dx
                new_right = xlim[1] + dx
                if self.current_feature == "Time View" and new_left < 0:
                    new_left = 0
                    new_right = new_left + (xlim[1] - xlim[0])
                ax.set_xlim(new_left, new_right)
                self.press_x = event.xdata
                self.canvas.draw()
                logging.debug(f"Panned: new xlim [{new_left:.2f}, {new_right:.2f}]")

    def setup_time_report_plot(self, selected_tags):
        if not self.current_project or not selected_tags:
            logging.warning("No project or tags selected for Time Report!")
            return
        self.selected_tags = selected_tags
        self.time_report_timer.stop()
        self.time_report_timer.setInterval(100)

        self.time_report_buffer.clear()
        self.time_report_timestamps.clear()
        from_dt = self.time_from_date.dateTime().toPyDateTime()
        to_dt = self.time_to_date.dateTime().toPyDateTime()
        
        for tag in selected_tags:
            self.time_report_buffer[tag] = deque(maxlen=10000)
            self.time_report_timestamps[tag] = deque(maxlen=10000)
            data = self.db.get_tag_values(self.current_project, tag)
            if data:
                for entry in data:
                    ts = datetime.strptime(entry["timestamp"], "%Y-%m-%dT%H:%M:%S.%f")
                    if from_dt <= ts <= to_dt:
                        self.time_report_buffer[tag].extend(entry["values"])
                        self.time_report_timestamps[tag].extend([entry["timestamp"]] * len(entry["values"]))

        if self.canvas.parent() != self.time_report_widget:
            if self.canvas.parent():
                self.canvas.setParent(None)
            self.time_report_layout.addWidget(self.canvas)
        
        self.figure.clear()
        self.time_report_ax = self.figure.add_subplot(111)
        self.time_report_lines = {tag: self.time_report_ax.plot([], [], label=tag)[0] for tag in selected_tags}
        self.time_report_ax.grid(True)
        self.time_report_ax.set_xlabel("Time (HH:MM:SSS)")
        self.time_report_ax.set_ylabel("Value (m/s)")
        self.time_report_ax.set_title("Time Report")
        self.time_report_ax.legend()
        self.figure.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.2)
        self.canvas.setMinimumSize(1000, 600)
        self.canvas.draw()
        self.time_report_timer.start()

    def update_time_report_plot(self):
        if not self.current_project or not self.selected_tags:
            self.time_report_result.setText("No project or tags selected for Time Report.")
            return

        from_dt = self.time_from_date.dateTime().toPyDateTime()
        to_dt = self.time_to_date.dateTime().toPyDateTime()
        all_values = []

        for tag in self.selected_tags:
            if tag not in self.time_report_buffer:
                continue

            timestamps = list(self.time_report_timestamps[tag])
            values = list(self.time_report_buffer[tag])
            filtered_timestamps = []
            filtered_values = []

            for ts, val in zip(timestamps, values):
                dt = datetime.strptime(ts, "%Y-%m-%dT%H:%M:%S.%f")
                if from_dt <= dt <= to_dt:
                    filtered_timestamps.append(dt)
                    filtered_values.append(val)
                    all_values.append(val)

            if filtered_values:
                self.time_report_lines[tag].set_data(filtered_timestamps, filtered_values)
            else:
                self.time_report_lines[tag].set_data([], [])

        if all_values:
            y_min, y_max = min(all_values), max(all_values)
            padding = (y_max - y_min) * 0.1 if y_max != y_min else 5000
            self.time_report_ax.set_ylim(y_min - padding, y_max + padding)
            self.time_report_ax.set_xlim(from_dt, to_dt)
            self.time_report_result.setText(f"Time Report Data for {', '.join(self.selected_tags)}:\nData points: {len(all_values)}")
        else:
            self.time_report_ax.set_xlim(from_dt, to_dt)
            self.time_report_ax.set_ylim(16390, 46537)
            self.time_report_result.setText("No data received in the selected time range yet.")

        self.time_report_ax.relim()
        self.time_report_ax.autoscale_view()
        self.figure.autofmt_xdate()
        self.canvas.draw()

    def save_time_report_data(self, project_name):
        from_dt = self.time_from_date.dateTime().toString(Qt.ISODate)
        to_dt = self.time_to_date.dateTime().toString(Qt.ISODate)
        total_saved = 0

        for tag in self.selected_tags:
            timestamps = list(self.time_report_timestamps.get(tag, []))
            values = list(self.time_report_buffer.get(tag, []))
            filtered_data = []

            for ts, val in zip(timestamps, values):
                if from_dt <= ts <= to_dt:
                    filtered_data.append({"timestamp": ts, "value": val})

            if filtered_data:
                data_to_save = {}
                for entry in filtered_data:
                    ts = entry["timestamp"]
                    if ts not in data_to_save:
                        data_to_save[ts] = []
                    data_to_save[ts].append(entry["value"])

                for ts, vals in data_to_save.items():
                    self.db.save_tag_values(project_name, tag, {"timestamp": ts, "values": vals})
                total_saved += len(filtered_data)

        if total_saved > 0:
            QMessageBox.information(self, "Success", f"Saved {total_saved} data points for {', '.join(self.selected_tags)} to database.")
        else:
            QMessageBox.warning(self, "No Data", "No data to save in the selected time range.")

    def export_time_report_to_pdf(self, project_name):
        from_dt = self.time_from_date.dateTime().toPyDateTime()
        to_dt = self.time_to_date.dateTime().toPyDateTime()
        filename = f"{project_name}_time_report_{from_dt.strftime('%Y%m%d_%H%M%S')}.pdf"
        
        with PdfPages(filename) as pdf:
            fig = plt.figure(figsize=(11, 8))
            ax = fig.add_subplot(111)
            
            for tag in self.selected_tags:
                if tag in self.time_report_buffer:
                    timestamps = [datetime.strptime(ts, "%Y-%m-%dT%H:%M:%S.%f") 
                                 for ts in self.time_report_timestamps[tag]]
                    values = list(self.time_report_buffer[tag])
                    filtered_timestamps = [ts for ts in timestamps if from_dt <= ts <= to_dt]
                    filtered_values = [val for ts, val in zip(timestamps, values) if from_dt <= ts <= to_dt]
                    if filtered_values:
                        ax.plot(filtered_timestamps, filtered_values, label=tag)
            
            ax.set_xlabel("Time (HH:MM:SSS)")
            ax.set_ylabel("Value (m/s)")
            ax.set_title(f"Time Report for {project_name}")
            ax.legend()
            ax.grid(True)
            fig.autofmt_xdate()
            
            pdf.savefig(fig)
            plt.close(fig)
        
        QMessageBox.information(self, "Success", f"Time Report exported to {filename}")

    def parse_tag_string(self, tag_string):
        if not tag_string:
            QMessageBox.warning(self, "Error", "Tag cannot be empty!")
            return None
        return {"tag_name": tag_string}

    def display_feature_content(self, feature_name, project_name):
        self.current_project = project_name
        self.current_feature = feature_name
        self.update_toolbar()
        self.timer.stop()
        self.time_timer.stop()
        self.time_report_timer.stop()
        if self.canvas.parent():
            self.canvas.setParent(None)

        while self.content_layout.count():
            item = self.content_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        tags_data = list(self.db.tags_collection.find({"project_name": project_name}))

        if feature_name == "Create Tags":
            header = QLabel(f"MANAGE TAGS FOR {project_name.upper()}")
            header.setStyleSheet("color: white; font-size: 26px; font-weight: bold; padding: 8px;")
            self.content_layout.addWidget(header, alignment=Qt.AlignCenter)

            tags_widget = QWidget()
            tags_layout = QVBoxLayout()
            tags_widget.setLayout(tags_layout)
            tags_widget.setStyleSheet("background-color: #2c3e50; border-radius: 5px; padding: 10px;")

            add_tag_form = QHBoxLayout()
            tag_name_input = QLineEdit()
            tag_name_input.setPlaceholderText("Enter full tag (e.g., sarayu/tag1/topic1|m/s)")
            tag_name_input.setStyleSheet("background-color: #34495e; color: white; border: 1px solid #1a73e8; padding: 5px; height: 25px;")

            add_tag_btn = QPushButton("Add Tag")
            add_tag_btn.setStyleSheet("""
                QPushButton { background-color: #28a745; color: white; border: none; padding: 5px; border-radius: 5px; height: 25px; }
                QPushButton:hover { background-color: #218838; }
            """)
            add_tag_btn.clicked.connect(lambda: self.add_tag(project_name, tag_name_input))

            add_tag_form.addWidget(tag_name_input)
            add_tag_form.addWidget(add_tag_btn)
            add_tag_form.addStretch()

            tags_layout.addLayout(add_tag_form)

            tags_table = QTableWidget()
            tags_table.setColumnCount(3)
            tags_table.setHorizontalHeaderLabels(["FULL TAG", "VALUE", "ACTIONS"])
            tags_table.setStyleSheet("""
                QTableWidget { background-color: #34495e; color: white; border: none; gridline-color: #2c3e50; }
                QTableWidget::item { padding: 5px; border: none; }
                QHeaderView::section { background-color: #1a73e8; color: white; border: none; padding: 10px; font-size: 14px; }
            """)
            tags_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            tags_table.verticalHeader().setVisible(False)

            tags_table.setRowCount(len(tags_data))
            for row, tag in enumerate(tags_data):
                tags_table.setItem(row, 0, QTableWidgetItem(tag["tag_name"]))
                latest_data = self.db.get_tag_values(project_name, tag["tag_name"])
                value = latest_data[-1]["values"][-1] if latest_data else "N/A"
                tags_table.setItem(row, 1, QTableWidgetItem(str(value)))

                actions_widget = QWidget()
                actions_layout = QHBoxLayout()
                actions_widget.setLayout(actions_layout)
                actions_layout.setContentsMargins(0, 0, 0, 0)
                actions_layout.setSpacing(5)
                actions_layout.setAlignment(Qt.AlignCenter)

                edit_btn = QPushButton("Edit")
                edit_btn.setFixedSize(60, 30)
                edit_btn.setStyleSheet("""
                    QPushButton { background-color: #3498db; color: white; border: none; border-radius: 5px; padding: 5px; }
                    QPushButton:hover { background-color: #2980b9; }
                """)
                edit_btn.clicked.connect(lambda checked, r=row: self.edit_tag(project_name, r))

                delete_btn = QPushButton("Delete")
                delete_btn.setFixedSize(60, 30)
                delete_btn.setStyleSheet("""
                    QPushButton { background-color: #e74c3c; color: white; border: none; border-radius: 5px; padding: 5px; }
                    QPushButton:hover { background-color: #c0392b; }
                """)
                delete_btn.clicked.connect(lambda checked, r=row: self.delete_tag(project_name, r))

                actions_layout.addWidget(edit_btn)
                actions_layout.addWidget(delete_btn)
                tags_table.setCellWidget(row, 2, actions_widget)

            tags_layout.addWidget(tags_table)
            self.content_layout.addWidget(tags_widget)

        elif feature_name == "Tabular View":
            header = QLabel(f"TABULAR VIEW FOR {project_name.upper()}")
            header.setStyleSheet("color: white; font-size: 26px; font-weight: bold; padding: 8px;")
            self.content_layout.addWidget(header, alignment=Qt.AlignCenter)

            tags_widget = QWidget()
            tags_layout = QVBoxLayout()
            tags_widget.setLayout(tags_layout)
            tags_widget.setStyleSheet("background-color: #2c3e50; border-radius: 5px; padding: 10px;")

            filter_layout = QHBoxLayout()
            filter_label = QLabel("Select Tags:")
            filter_label.setStyleSheet("color: white; font-size: 14px;")
            self.tag_combo = QComboBox()
            self.tag_combo.addItem("All Tags")
            for tag in tags_data:
                self.tag_combo.addItem(tag["tag_name"])
            self.tag_combo.setStyleSheet("background-color: #34495e; color: white; border: 1px solid #1a73e8; padding: 5px;")
            self.tag_combo.currentTextChanged.connect(lambda: self.update_tabular_view(project_name))
            filter_layout.addWidget(filter_label)
            filter_layout.addWidget(self.tag_combo)
            filter_layout.addStretch()
            tags_layout.addLayout(filter_layout)

            self.tabular_table = QTableWidget()
            self.tabular_table.setColumnCount(3)
            self.tabular_table.setHorizontalHeaderLabels(["FULL TAG", "TIMESTAMP", "VALUE"])
            self.tabular_table.setStyleSheet("""
                QTableWidget { background-color: #34495e; color: white; border: none; gridline-color: #2c3e50; }
                QTableWidget::item { padding: 5px; border: none; }
                QHeaderView::section { background-color: #1a73e8; color: white; border: none; padding: 10px; font-size: 14px; }
            """)
            self.tabular_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.tabular_table.verticalHeader().setVisible(False)
            self.update_tabular_view(project_name)
            tags_layout.addWidget(self.tabular_table)

            self.content_layout.addWidget(tags_widget)

        elif feature_name == "Time View":
            header = QLabel(f"TIME VIEW FOR {project_name.upper()}")
            header.setStyleSheet("color: white; font-size: 26px; font-weight: bold; padding: 8px;")
            self.content_layout.addWidget(header, alignment=Qt.AlignCenter)

            self.time_widget = QWidget()
            self.time_layout = QVBoxLayout()
            self.time_widget.setLayout(self.time_layout)
            self.time_widget.setStyleSheet("background-color: #2c3e50; border-radius: 5px; padding: 10px;")
            self.time_widget.setMinimumHeight(600)

            tag_layout = QHBoxLayout()
            tag_label = QLabel("Select Tag:")
            tag_label.setStyleSheet("color: white; font-size: 14px;")
            self.tag_combo = QComboBox()
            if not tags_data:
                self.tag_combo.addItem("No Tags Available")
            else:
                for tag in tags_data:
                    self.tag_combo.addItem(tag["tag_name"])
            self.tag_combo.setStyleSheet("background-color: #34495e; color: white; border: 1px solid #1a73e8; padding: 5px;")
            self.tag_combo.currentTextChanged.connect(lambda tag: self.setup_time_view_plot(tag) if tag != "No Tags Available" else None)

            reset_btn = QPushButton("Reset")
            reset_btn.setStyleSheet("""
                QPushButton { background-color: #f39c12; color: white; border: none; padding: 5px; border-radius: 5px; }
                QPushButton:hover { background-color: #e67e22; }
            """)
            reset_btn.clicked.connect(self.reset_time_view)

            tag_layout.addWidget(tag_label)
            tag_layout.addWidget(self.tag_combo)
            tag_layout.addWidget(reset_btn)
            tag_layout.addStretch()
            self.time_layout.addLayout(tag_layout)

            self.time_result = QTextEdit()
            self.time_result.setReadOnly(True)
            self.time_result.setStyleSheet("background-color: #34495e; color: white; border-radius: 5px; padding: 10px;")
            self.time_result.setMinimumHeight(100)
            self.time_result.setText(f"Time View for {project_name}: Select a tag to start real-time plotting.\nUse mouse wheel to zoom, drag to pan, or reset to default.")
            self.time_layout.addWidget(self.time_result)
            self.time_layout.addStretch()

            self.content_layout.addWidget(self.time_widget)

            if tags_data:
                self.tag_combo.setCurrentIndex(0)
                self.setup_time_view_plot(self.tag_combo.currentText())

        elif feature_name in ["FFT", "Waterfall", "Orbit", "Trend View", "Multiple Trend View", "Bode Plot"]:
            header = QLabel(f"{feature_name.upper()} FOR {project_name.upper()}")
            header.setStyleSheet("color: white; font-size: 26px; font-weight: bold; padding: 8px;")
            self.content_layout.addWidget(header, alignment=Qt.AlignCenter)

            self.feature_widget = QWidget()
            self.feature_layout = QVBoxLayout()
            self.feature_widget.setLayout(self.feature_layout)
            self.feature_widget.setStyleSheet("background-color: #2c3e50; border-radius: 5px; padding: 10px;")

            tag_layout = QHBoxLayout()
            tag_label = QLabel("Select Tag:" if feature_name != "Multiple Trend View" else "Select Tags:")
            tag_label.setStyleSheet("color: white; font-size: 14px;")
            tag_combo = QComboBox()
            if not tags_data:
                tag_combo.addItem("No Tags Available")
            else:
                for tag in tags_data:
                    tag_combo.addItem(tag["tag_name"])
            tag_combo.setStyleSheet("background-color: #34495e; color: white; border: 1px solid #1a73e8; padding: 5px;")
            tag_layout.addWidget(tag_label)
            tag_layout.addWidget(tag_combo)
            tag_layout.addStretch()
            self.feature_layout.addLayout(tag_layout)

            button_layout = QHBoxLayout()
            mqtt_btn = QPushButton("Start MQTT Plotting")
            mqtt_btn.setStyleSheet("""
                QPushButton { background-color: #f39c12; color: white; border: none; padding: 5px; border-radius: 5px; }
                QPushButton:hover { background-color: #e67e22; }
            """)
            mqtt_btn.clicked.connect(lambda: self.start_mqtt_plotting(self.feature_result, tag_combo.currentText()))
            button_layout.addWidget(mqtt_btn)
            button_layout.addStretch()
            self.feature_layout.addLayout(button_layout)

            self.feature_result = QTextEdit()
            self.feature_result.setReadOnly(True)
            self.feature_result.setStyleSheet("background-color: #34495e; color: white; border-radius: 5px; padding: 10px;")
            self.feature_result.setText(f"{feature_name} data for {project_name}: Select a tag to begin.")
            self.feature_layout.addWidget(self.feature_result)

            self.content_layout.addWidget(self.feature_widget)

        elif feature_name == "History Plot":
            header = QLabel(f"HISTORY PLOT FOR {project_name.upper()}")
            header.setStyleSheet("color: white; font-size: 26px; font-weight: bold; padding: 8px;")
            self.content_layout.addWidget(header, alignment=Qt.AlignCenter)

            scroll_area = QScrollArea()
            scroll_area.setWidgetResizable(True)
            scroll_area.setStyleSheet("border: none;")

            history_widget = QWidget()
            history_layout = QVBoxLayout()
            history_widget.setLayout(history_layout)
            history_widget.setStyleSheet("background-color: #2c3e50; border-radius: 5px; padding: 10px;")

            scroll_area.setWidget(history_widget)

            filter_layout = QHBoxLayout()
            from_label = QLabel("From:")
            from_label.setStyleSheet("color: white; font-size: 14px; font-weight: bold;")
            self.from_date = QDateTimeEdit()
            self.from_date.setCalendarPopup(True)
            self.from_date.setDateTime(QDateTime.currentDateTime().addDays(-7))
            self.from_date.setFixedWidth(150)
            self.from_date.setStyleSheet("background-color: #34495e; color: white; border: 1px solid #1a73e8; padding: 5px; border-radius: 5px;")

            to_label = QLabel("To:")
            to_label.setStyleSheet("color: white; font-size: 14px; font-weight: bold;")
            self.to_date = QDateTimeEdit()
            self.to_date.setCalendarPopup(True)
            self.to_date.setDateTime(QDateTime.currentDateTime())
            self.to_date.setFixedWidth(150)
            self.to_date.setStyleSheet("background-color: #34495e; color: white; border: 1px solid #1a73e8; padding: 5px; border-radius: 5px;")

            tag_label = QLabel("Select Tag:")
            tag_label.setStyleSheet("color: white; font-size: 14px; font-weight: bold;")
            self.history_tag_combo = QComboBox()
            self.history_tag_combo.setFixedWidth(200)
            if not tags_data:
                self.history_tag_combo.addItem("No Tags Available")
            else:
                for tag in tags_data:
                    self.history_tag_combo.addItem(tag["tag_name"])
            self.history_tag_combo.setStyleSheet("background-color: #34495e; color: white; border: 1px solid #1a73e8; padding: 5px; border-radius: 5px;")

            filter_btn = QPushButton("Filter")
            filter_btn.setStyleSheet("""
                QPushButton { background-color: #3498db; color: white; border: none; padding: 7px 15px; border-radius: 5px; font-size: 14px; font-weight: bold; }
                QPushButton:hover { background-color: #2980b9; }
            """)
            filter_btn.clicked.connect(lambda: self.update_history_plot(project_name))

            filter_layout.addWidget(from_label)
            filter_layout.addWidget(self.from_date)
            filter_layout.addWidget(to_label)
            filter_layout.addWidget(self.to_date)
            filter_layout.addWidget(tag_label)
            filter_layout.addWidget(self.history_tag_combo)
            filter_layout.addWidget(filter_btn)
            filter_layout.addStretch()
            history_layout.addLayout(filter_layout)

            self.history_table = QTableWidget()
            self.history_table.setColumnCount(2)
            self.history_table.setHorizontalHeaderLabels(["FULL TAG", "TIMESTAMP"])
            self.history_table.setMinimumHeight(500)
            self.history_table.setMinimumWidth(700)
            self.history_table.setStyleSheet("""
                QTableWidget { background-color: #34495e; color: white; border: none; gridline-color: #2c3e50; }
                QTableWidget::item { padding: 10px; border: none; color: white; font-size: 14px; }
                QHeaderView::section { background-color: #1a73e8; color: white; border: none; padding: 10px; font-size: 14px; font-weight: bold; }
                QTableWidget::item:selected { background-color: #1a73e8; }
            """)
            self.history_table.setAlternatingRowColors(True)
            self.history_table.setStyleSheet("""
                QTableWidget::item { background-color: #2c3e50; }
                QTableWidget::item:alternate { background-color: #3b4c5e; }
            """)
            self.history_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.history_table.verticalHeader().setVisible(False)

            self.update_history_plot(project_name)
            history_layout.addWidget(self.history_table)

            self.content_layout.addWidget(scroll_area)

        elif feature_name == "Time Report":
            header = QLabel(f"TIME REPORT FOR {project_name.upper()}")
            header.setStyleSheet("color: white; font-size: 26px; font-weight: bold; padding: 8px;")
            self.content_layout.addWidget(header, alignment=Qt.AlignCenter)

            self.time_report_widget = QWidget()
            self.time_report_layout = QVBoxLayout()
            self.time_report_widget.setLayout(self.time_report_layout)
            self.time_report_widget.setStyleSheet("background-color: #2c3e50; border-radius: 5px; padding: 10px;")

            filter_layout = QHBoxLayout()
            from_label = QLabel("From:")
            from_label.setStyleSheet("color: white; font-size: 14px;")
            self.time_from_date = QDateTimeEdit()
            self.time_from_date.setCalendarPopup(True)
            self.time_from_date.setDateTime(QDateTime.currentDateTime().addDays(-1))
            self.time_from_date.setStyleSheet("background-color: #34495e; color: white; border: 1px solid #1a73e8; padding: 5px;")
            to_label = QLabel("To:")
            to_label.setStyleSheet("color: white; font-size: 14px;")
            self.time_to_date = QDateTimeEdit()
            self.time_to_date.setCalendarPopup(True)
            self.time_to_date.setDateTime(QDateTime.currentDateTime())
            self.time_to_date.setStyleSheet("background-color: #34495e; color: white; border: 1px solid #1a73e8; padding: 5px;")
            tag_label = QLabel("Select Tags:")
            tag_label.setStyleSheet("color: white; font-size: 14px;")
            self.time_report_tag_list = QListWidget()
            self.time_report_tag_list.setSelectionMode(QListWidget.MultiSelection)
            if not tags_data:
                self.time_report_tag_list.addItem("No Tags Available")
            else:
                for tag in tags_data:
                    item = QListWidgetItem(tag["tag_name"])
                    self.time_report_tag_list.addItem(item)
            self.time_report_tag_list.setStyleSheet("background-color: #34495e; color: white; border: 1px solid #1a73e8; padding: 5px;")
            self.time_report_tag_list.itemSelectionChanged.connect(lambda: self.setup_time_report_plot([item.text() for item in self.time_report_tag_list.selectedItems()]))

            filter_layout.addWidget(from_label)
            filter_layout.addWidget(self.time_from_date)
            filter_layout.addWidget(to_label)
            filter_layout.addWidget(self.time_to_date)
            filter_layout.addWidget(tag_label)
            filter_layout.addWidget(self.time_report_tag_list)
            filter_layout.addStretch()
            self.time_report_layout.addLayout(filter_layout)

            button_layout = QHBoxLayout()
            # save_btn = QPushButton("Save Data")
            # save_btn.setStyleSheet("""
            #     QPushButton { background-color: #28a745; color: white; border: none; padding: 5px; border-radius: 5px; }
            #     QPushButton:hover { background-color: #218838; }
            # """)
            # save_btn.clicked.connect(lambda: self.save_time_report_data(project_name))

            pdf_btn = QPushButton("Export to PDF")
            pdf_btn.setStyleSheet("""
                QPushButton { background-color: #3498db; color: white; border: none; padding: 5px; border-radius: 5px; }
                QPushButton:hover { background-color: #2980b9; }
            """)
            pdf_btn.clicked.connect(lambda: self.export_time_report_to_pdf(project_name))

            # button_layout.addWidget(save_btn)
            button_layout.addWidget(pdf_btn)
            button_layout.addStretch()
            self.time_report_layout.addLayout(button_layout)

            self.time_report_result = QTextEdit()
            self.time_report_result.setReadOnly(True)
            self.time_report_result.setStyleSheet("background-color: #34495e; color: white; border-radius: 5px; padding: 10px;")
            self.time_report_result.setMinimumHeight(100)
            self.time_report_result.setText(f"Time Report for {project_name}: Select tags to start plotting.\nUse mouse wheel to zoom, drag to pan.")
            self.time_report_layout.addWidget(self.time_report_result)
            self.time_report_layout.addStretch()

            self.content_layout.addWidget(self.time_report_widget)

            if tags_data:
                self.time_report_tag_list.item(0).setSelected(True)
                self.setup_time_report_plot([tags_data[0]["tag_name"]])

        elif feature_name == "Report":
            header = QLabel(f"REPORT FOR {project_name.upper()}")
            header.setStyleSheet("color: white; font-size: 26px; font-weight: bold; padding: 8px;")
            self.content_layout.addWidget(header, alignment=Qt.AlignCenter)

            report_widget = QWidget()
            report_layout = QVBoxLayout()
            report_widget.setLayout(report_layout)
            report_widget.setStyleSheet("background-color: #2c3e50; border-radius: 5px; padding: 10px;")

            report_table = QTableWidget()
            report_table.setColumnCount(3)
            report_table.setHorizontalHeaderLabels(["FULL TAG", "TIMESTAMP", "VALUE"])
            report_table.setStyleSheet("""
                QTableWidget { background-color: #34495e; color: white; border: none; gridline-color: #2c3e50; }
                QTableWidget::item { padding: 5px; border: none; }
                QHeaderView::section { background-color: #1a73e8; color: white; border: none; padding: 10px; font-size: 14px; }
            """)
            report_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            report_table.verticalHeader().setVisible(False)
            report_table.setRowCount(len(tags_data))
            for row, tag in enumerate(tags_data):
                report_table.setItem(row, 0, QTableWidgetItem(tag["tag_name"]))
                latest_data = self.db.get_tag_values(project_name, tag["tag_name"])
                timestamp = latest_data[-1]["timestamp"] if latest_data else "N/A"
                value = latest_data[-1]["values"][-1] if latest_data else "N/A"
                report_table.setItem(row, 1, QTableWidgetItem(timestamp))
                report_table.setItem(row, 2, QTableWidgetItem(str(value)))
            report_layout.addWidget(report_table)

            export_btn = QPushButton("Export to CSV")
            export_btn.setStyleSheet("""
                QPushButton { background-color: #28a745; color: white; border: none; padding: 5px; border-radius: 5px; }
                QPushButton:hover { background-color: #218838; }
            """)
            export_btn.clicked.connect(lambda: self.export_report_to_csv(project_name))
            report_layout.addWidget(export_btn)

            self.content_layout.addWidget(report_widget)

    def update_tabular_view(self, project_name):
        tags_data = list(self.db.tags_collection.find({"project_name": project_name}))
        selected_tag = self.tag_combo.currentText()

        filtered_tags = tags_data if selected_tag == "All Tags" else [tag for tag in tags_data if tag["tag_name"] == selected_tag]
        self.tabular_table.setRowCount(len(filtered_tags))
        for row, tag in enumerate(filtered_tags):
            self.tabular_table.setItem(row, 0, QTableWidgetItem(tag["tag_name"]))
            latest_data = self.db.get_tag_values(project_name, tag["tag_name"])
            timestamp = latest_data[-1]["timestamp"] if latest_data else "N/A"
            value = latest_data[-1]["values"][-1] if latest_data else "N/A"
            self.tabular_table.setItem(row, 1, QTableWidgetItem(timestamp))
            self.tabular_table.setItem(row, 2, QTableWidgetItem(str(value)))

    def update_history_plot(self, project_name):
        tags_data = list(self.db.tags_collection.find({"project_name": project_name}))

        from_date_str = self.from_date.dateTime().toString(Qt.ISODate)
        to_date_str = self.to_date.dateTime().toString(Qt.ISODate)
        selected_tag = self.history_tag_combo.currentText()

        filtered_tags = [tag for tag in tags_data if tag["tag_name"] == selected_tag]
        if filtered_tags:
            tag = filtered_tags[0]
            filtered_data = [entry for entry in self.db.get_tag_values(project_name, tag["tag_name"]) if from_date_str <= entry["timestamp"] <= to_date_str]
            self.history_table.setRowCount(len(filtered_data))
            for row, entry in enumerate(filtered_data):
                self.history_table.setItem(row, 0, QTableWidgetItem(tag["tag_name"]))
                self.history_table.setItem(row, 1, QTableWidgetItem(entry["timestamp"]))
        else:
            self.history_table.setRowCount(1)
            self.history_table.setItem(0, 0, QTableWidgetItem(selected_tag if selected_tag != "No Tags Available" else "N/A"))
            self.history_table.setItem(0, 1, QTableWidgetItem("No data found for this tag and date range"))

    def export_report_to_csv(self, project_name):
        tags_data = list(self.db.tags_collection.find({"project_name": project_name}))

        filename = f"{project_name}_report.csv"
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Full Tag", "Timestamp", "Value"])
            for tag in tags_data:
                latest_data = self.db.get_tag_values(project_name, tag["tag_name"])
                timestamp = latest_data[-1]["timestamp"] if latest_data else "N/A"
                value = latest_data[-1]["values"][-1] if latest_data else "N/A"
                writer.writerow([tag["tag_name"], timestamp, value])
        QMessageBox.information(self, "Success", f"Report exported to {filename}")

    def add_tag(self, project_name, tag_name_input):
        tag_string = tag_name_input.text().strip()
        tag_data = self.parse_tag_string(tag_string)
        if tag_data is None:
            return

        success, message = self.db.add_tag(project_name, tag_data)
        if success:
            tag_name_input.clear()
            if self.mqtt_handler:
                self.mqtt_handler.client.subscribe(tag_data["tag_name"])
            self.display_feature_content("Create Tags", project_name)
        else:
            QMessageBox.warning(self, "Error", message)

    def edit_tag(self, project_name, row):
        tags_data = list(self.db.tags_collection.find({"project_name": project_name}))
        if row >= len(tags_data):
            return
        tag = tags_data[row]
        old_tag_string = tag["tag_name"]
        new_tag_string, ok = QInputDialog.getText(self, "Edit Tag", "Enter new tag (e.g., sarayu/tag1/topic1|m/s):", text=old_tag_string)
        if ok and new_tag_string:
            new_tag_data = self.parse_tag_string(new_tag_string)
            if new_tag_data is None:
                return
            if self.mqtt_handler:
                self.mqtt_handler.client.unsubscribe(tag["tag_name"])
                self.mqtt_handler.client.subscribe(new_tag_data["tag_name"])
            success, message = self.db.edit_tag(project_name, row, new_tag_data)
            if success:
                self.display_feature_content("Create Tags", project_name)
            else:
                QMessageBox.warning(self, "Error", message)

    def delete_tag(self, project_name, row):
        reply = QMessageBox.question(self, "Confirm Delete", "Are you sure you want to delete this tag?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            tags_data = list(self.db.tags_collection.find({"project_name": project_name}))
            tag = tags_data[row]
            if self.mqtt_handler:
                self.mqtt_handler.client.unsubscribe(tag["tag_name"])
            success, message = self.db.delete_tag(project_name, row)
            if success:
                self.display_feature_content("Create Tags", project_name)
            else:
                QMessageBox.warning(self, "Error", message)

    def save_action(self):
        if self.current_project and self.db.get_project_data(self.current_project):
            QMessageBox.information(self, "Save", f"Data for project '{self.current_project}' saved successfully!")
        else:
            QMessageBox.warning(self, "Save Error", "No project selected to save!")

    def refresh_action(self):
        if self.current_project and self.current_feature:
            self.display_feature_content(self.current_feature, self.current_project)
            QMessageBox.information(self, "Refresh", f"Refreshed view for '{self.current_feature}'!")
        else:
            self.display_dashboard()
            QMessageBox.information(self, "Refresh", "Refreshed dashboard view!")

    def settings_action(self):
        QMessageBox.information(self, "Settings", "Settings functionality not implemented yet.")

    def closeEvent(self, event):
        self.timer.stop()
        self.time_timer.stop()
        self.time_report_timer.stop()
        if self.mqtt_handler:
            self.mqtt_handler.stop()
        self.db.close_connection()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    from database import Database
    db = Database(email="user@example.com")
    window = DashboardWindow(db=db, email="user@example.com")
    window.show()
    sys.exit(app.exec_())