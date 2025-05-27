import sys
import requests
from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QInputDialog,
    QMessageBox, QPushButton, QComboBox, QHBoxLayout, QTableWidget,
    QTableWidgetItem
)
from PySide6.QtCore import Qt, QTimer


def is_valid_ip(ip: str) -> bool:
    import re
    pattern = re.compile(
        r'^('
        r'(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)\.){3}'
        r'(25[0-5]|2[0-4]\d|1\d{2}|[1-9]?\d)$'
    )
    return pattern.match(ip) is not None


def prompt_for_ip():
    while True:
        ip, ok = QInputDialog.getText(None, "Enter Server IP", "Flask Server IP:", text="127.0.0.1")
        if not ok:
            return None
        ip = ip.strip()
        if is_valid_ip(ip):
            return ip
        QMessageBox.critical(None, "Invalid IP", f"'{ip}' is not a valid IPv4 address. Please try again.")


def get_room_list(server_ip):
    try:
        response = requests.get(f"http://{server_ip}:5000/get_rooms", timeout=2)
        response.raise_for_status()
        return response.json()
    except Exception as e:
        QMessageBox.critical(None, "Connection Error", f"Could not fetch rooms:\n{e}")
        return None


def prompt_select_room(room_list):
    if not room_list:
        return None
    room, ok = QInputDialog.getItem(None, "Select Room", "Available Rooms:", room_list, editable=False)
    return room if ok else None


def notifyRoomSelection(server_ip, room):
    try:
        response = requests.post(
            f"http://{server_ip}:5000/notify_room_selection",
            json={"room": room},
            timeout=2
        )
        if response.status_code != 200:
            QMessageBox.warning(None, "Room Error", f"Server responded: {response.text}")
    except Exception as e:
        QMessageBox.critical(None, "Notification Error", f"Could not notify room selection:\n{e}")


class MarkerManager(QWidget):
    def __init__(self, server_ip, room):
        super().__init__()
        self.server_ip = server_ip
        self.room = room
        self.server_url = f"http://{server_ip}:5000"
        self.setWindowTitle(f"Manage Markers for Room: {room}")
        self.resize(400, 300)

        self.layout = QVBoxLayout(self)

        self.marker_table = QTableWidget()
        self.marker_table.setColumnCount(4)
        self.marker_table.setHorizontalHeaderLabels(["ID", "X", "Y", "Z"])
        self.layout.addWidget(self.marker_table)

        btn_layout = QHBoxLayout()
        self.add_btn = QPushButton("Add Marker")
        self.del_btn = QPushButton("Delete Marker")
        self.update_btn = QPushButton("Update Marker")

        btn_layout.addWidget(self.add_btn)
        btn_layout.addWidget(self.del_btn)
        btn_layout.addWidget(self.update_btn)

        self.layout.addLayout(btn_layout)

        self.add_btn.clicked.connect(self.add_marker)
        self.del_btn.clicked.connect(self.delete_marker)
        self.update_btn.clicked.connect(self.update_marker)

        self.load_markers()

    def set_room(self, room):
        self.room = room
        self.setWindowTitle(f"Manage Markers for Room: {room}")
        self.load_markers()

    def load_markers(self):
        try:
            resp = requests.get(f"{self.server_url}/get_Known_Markers/{self.room}", timeout=2)
            resp.raise_for_status()
            markers = resp.json()  # Expects dict like {"1": {"x":1, "y":1, "z":1}, ...}

            self.marker_table.setRowCount(0)
            for row, (mid, coords) in enumerate(markers.items()):
                self.marker_table.insertRow(row)
                self.marker_table.setItem(row, 0, QTableWidgetItem(mid))
                self.marker_table.setItem(row, 1, QTableWidgetItem(str(coords["x"])))
                self.marker_table.setItem(row, 2, QTableWidgetItem(str(coords["y"])))
                self.marker_table.setItem(row, 3, QTableWidgetItem(str(coords["z"])))
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load markers:\n{e}")

    def add_marker(self):
        from PySide6.QtWidgets import QInputDialog

        marker_id, ok = QInputDialog.getText(self, "Add Marker", "Marker ID:")
        if not ok or not marker_id.strip():
            return
        marker_id = marker_id.strip()

        x, ok_x = QInputDialog.getDouble(self, "Add Marker", "X coordinate:")
        if not ok_x:
            return
        y, ok_y = QInputDialog.getDouble(self, "Add Marker", "Y coordinate:")
        if not ok_y:
            return
        z, ok_z = QInputDialog.getDouble(self, "Add Marker", "Z coordinate:")
        if not ok_z:
            return

        try:
            data = {"room": self.room, "id": marker_id, "x": x, "y": y, "z": z}
            resp = requests.post(f"{self.server_url}/add_marker", json=data, timeout=2)
            resp.raise_for_status()
            self.load_markers()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to add marker:\n{e}")

    def delete_marker(self):
        selected = self.marker_table.currentRow()
        if selected < 0:
            QMessageBox.warning(self, "Warning", "Select a marker to delete.")
            return
        marker_id = self.marker_table.item(selected, 0).text()
        confirm = QMessageBox.question(self, "Confirm Delete", f"Delete marker {marker_id}?")
        if confirm != QMessageBox.Yes:
            return
        try:
            data = {"room": self.room, "id": marker_id}
            resp = requests.post(f"{self.server_url}/delete_marker", json=data, timeout=2)
            resp.raise_for_status()
            self.load_markers()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to delete marker:\n{e}")

    def update_marker(self):
        from PySide6.QtWidgets import QInputDialog

        selected = self.marker_table.currentRow()
        if selected < 0:
            QMessageBox.warning(self, "Warning", "Select a marker to update.")
            return
        marker_id = self.marker_table.item(selected, 0).text()

        x, ok_x = QInputDialog.getDouble(self, "Update Marker", "New X coordinate:",
                                         float(self.marker_table.item(selected, 1).text()))
        if not ok_x:
            return
        y, ok_y = QInputDialog.getDouble(self, "Update Marker", "New Y coordinate:",
                                         float(self.marker_table.item(selected, 2).text()))
        if not ok_y:
            return
        z, ok_z = QInputDialog.getDouble(self, "Update Marker", "New Z coordinate:",
                                         float(self.marker_table.item(selected, 3).text()))
        if not ok_z:
            return

        try:
            data = {"room": self.room, "id": marker_id, "x": x, "y": y, "z": z}
            resp = requests.post(f"{self.server_url}/update_marker", json=data, timeout=2)
            resp.raise_for_status()
            self.load_markers()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to update marker:\n{e}")


class FlaskClientUI(QWidget):
    def __init__(self, server_ip, selected_room):
        super().__init__()
        self.server_ip = server_ip
        self.room = selected_room
        self.server_url = f"http://{server_ip}:5000"
        self.setWindowTitle("Barcode Locolaization Client")
        self.setFixedSize(500, 400)

        self.layout = QVBoxLayout()

        self.room_selector = QComboBox()
        self.refresh_rooms()
        self.room_selector.setCurrentText(self.room)
        self.room_selector.currentTextChanged.connect(self.change_room)

        self.position_label = QLabel("Waiting for coordinates...")
        self.position_label.setAlignment(Qt.AlignCenter)

        self.status_label = QLabel("Not Connected")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.set_status_connected(False)

        self.add_button = QPushButton("Add Room")
        self.add_button.clicked.connect(self.add_room)

        self.delete_button = QPushButton("Delete Room")
        self.delete_button.clicked.connect(self.delete_room)

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(self.add_button)
        btn_layout.addWidget(self.delete_button)

        self.layout.addWidget(QLabel("Room:"))
        self.layout.addWidget(self.room_selector)
        self.layout.addWidget(self.position_label)
        self.layout.addWidget(self.status_label)
        self.layout.addLayout(btn_layout)

        # Add MarkerManager widget below the room controls
        self.marker_manager = MarkerManager(server_ip, selected_room)
        self.layout.addWidget(self.marker_manager)

        self.setLayout(self.layout)

        self.setup_timer()
        self.fetch_position()
        notifyRoomSelection(self.server_ip, self.room)

    def setup_timer(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.fetch_position)
        self.timer.start(1000)  # update every 1 second

    def set_status_connected(self, connected):
        self.status_label.setText("Connected" if connected else "Not Connected")
        self.status_label.setStyleSheet(
            "color: green;" if connected else "color: red;"
        )

    def fetch_position(self):
        try:
            response = requests.get(f"{self.server_url}/get_position", timeout=2)
            response.raise_for_status()
            data = response.json()
            x, y, z = data.get("x"), data.get("y"), data.get("z")
            self.position_label.setText(f"Coordinates: X={x}, Y={y}, Z={z}")
            self.set_status_connected(True)
        except Exception:
            self.position_label.setText("Coordinates: N/A")
            self.set_status_connected(False)

    def refresh_rooms(self):
        rooms = get_room_list(self.server_ip)
        if rooms is None:
            rooms = []
        self.room_selector.clear()
        self.room_selector.addItems(rooms)

    def change_room(self, room):
        if not room:
            return
        self.room = room
        notifyRoomSelection(self.server_ip, self.room)
        self.marker_manager.set_room(room)

    def add_room(self):
        text, ok = QInputDialog.getText(self, "Add Room", "New Room Name:")
        if not ok or not text.strip():
            return
        room = text.strip()
        try:
            resp = requests.post(f"{self.server_url}/add_room", json={"room": room}, timeout=2)
            resp.raise_for_status()
            self.refresh_rooms()
            self.room_selector.setCurrentText(room)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to add room:\n{e}")

    def delete_room(self):
        room = self.room_selector.currentText()
        if not room:
            return
        confirm = QMessageBox.question(self, "Confirm Delete", f"Delete room '{room}'?")
        if confirm != QMessageBox.Yes:
            return
        try:
            resp = requests.post(f"{self.server_url}/delete_room", json={"room": room}, timeout=2)
            resp.raise_for_status()
            self.refresh_rooms()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to delete room:\n{e}")


def main():
    app = QApplication(sys.argv)
    ip = prompt_for_ip()
    if ip is None:
        sys.exit()

    room_list = get_room_list(ip)
    if room_list is None or len(room_list) == 0:
        QMessageBox.critical(None, "Error", "No rooms available from the server.")
        sys.exit()

    selected_room = prompt_select_room(room_list)
    if selected_room is None:
        sys.exit()

    ui = FlaskClientUI(ip, selected_room)
    ui.show()
    sys.exit(app.exec())
    

if __name__ == "__main__":
    main()
