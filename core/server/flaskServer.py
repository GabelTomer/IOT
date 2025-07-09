from flask import Flask, jsonify, request
import json
import threading
import time

import requests
from flask import after_this_request


class server:
    def __init__(self,  port = 5000, known_markers_path=None, detector=None, left_camera_ip=None, right_camera_ip=None):
        self.target_position = None
        self.detector = detector
        self.chosen_room = None
        self.known_markers = self.load_known_markers(known_markers_path) if known_markers_path else {}
        self.known_markers_path = known_markers_path
        self.lock = threading.Lock()
        self.port = port
        self.roomChanged = False
        self.arucoList = None
        self.left_camera_ip = left_camera_ip
        self.right_camera_ip = right_camera_ip
        self.position = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'heading' : 0.0
        }
        self.last_command = {"command": None, "timestamp": None}
        if self.known_markers:
            self.rooms = list(self.known_markers.keys())
        else:
            self.rooms = []
        self.app = Flask(__name__)
    
   
    def get_target(self):
        with self.lock:
            return self.target_position
        

    def load_known_markers(self, path):
        with open(path, 'r') as file:
            data = json.load(file)
        if not data:
            raise ValueError("[ERROR] No known markers found in the file.")
            return {}
        return data
    
    
    def save_markers(self, data, room="1"):
        with open(self.known_markers_path, "w") as f:
            json.dump(data, f, indent=2)
        self.known_markers = data
        self.chosen_room = room
        self.rooms = list(self.known_markers.keys())
        self.detector.update_known_markers(room=room)
    
    def run(self):
          self.app.run(host='0.0.0.0', port=self.port)
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return "Welcome to the Flask Server!"
        
        @self.app.route('/get_Known_Markers/<room>', methods=['GET'])
        def get_known_markers(room):
            if room in self.known_markers:
                return jsonify(self.known_markers[room])
            else:
                return jsonify({"error": "Room not found"}), 404
        
        
        @self.app.route('/get_rooms')
        def get_rooms():
            return jsonify(self.rooms)
        
        @self.app.route('/notify_room_selection', methods=['POST'])
        def notify_room_selection():
            data = request.get_json()
            room = data.get('room')
            if room in self.known_markers:
                self.roomChanged = True
                self.detector.update_known_markers(room=room)
                @after_this_request
                def notify(response):  # <- executes after Flask finishes response
                    notify_camera_room_selection(self.left_camera_ip, room)
                    notify_camera_room_selection(self.right_camera_ip, room)
                    return response
                return jsonify({"status": "success", "room": room})
            else:
                return jsonify({"error": "Room not found"}), 404
            
        def notify_camera_room_selection(camera_ip, room):
            if camera_ip is not None:
                try:
                    requests.post(
                        f"http://{camera_ip}:5000/notify_room_selection",
                        json={"room": room},
                        timeout=1
                    )
                    print(f"Camera at {camera_ip} notified of room selection: {room}")
                except requests.exceptions.RequestException as e:
                    print(f"Failed to notify camera at {camera_ip}: {e}")
        
        @self.app.route('/update_position', methods=['POST']) #gets input from application on where to move
        def update_position():
            # Example data to update position
            data = request.get_json()
            if 'x' in data and 'y' in data:
                print(f"this is data {data} \n")
                return jsonify({'status': 'success', 'position': data}) #dont change returns from this function
            return jsonify({'status': 'error', 'message': 'Invalid data'}), 400
        
        
        @self.app.route('/update_marker', methods=['POST'])
        def update_marker():
            data = request.get_json()
            marker_id = str(data.get("id"))
            x = data.get("x")
            y = data.get("y")
            z = data.get("z")
            room = data.get("room")
            yaw = data.get("yaw")
            pitch = data.get("pitch")
            roll = data.get("roll")
            if None in [marker_id, x, y, z, room,yaw, pitch, roll]:
                return jsonify({"error": "Missing fields"}), 400

            markers = self.known_markers[room]
            markers[marker_id] = {"x": x, "y": y, "z": z,"yaw":yaw, "pitch": pitch, "roll": roll}
            self.save_markers(self.known_markers, room=room)
            @after_this_request
            def notify(response):  # <- executes after Flask finishes response
                notify_camera_marker_update(self.left_camera_ip, marker_id, x, y, z,yaw, pitch, roll)
                notify_camera_marker_update(self.right_camera_ip, marker_id, x, y, z,yaw, pitch, roll)
                return response
            return jsonify({"status": "updated", "id": marker_id}), 200
        

        def notify_camera_marker_update(camera_ip, marker_id, x, y, z,yaw, pitch, roll):
            if camera_ip is not None:
                try:
                    requests.post(
                        f"http://{camera_ip}:5000/update_marker",
                        json={"id": marker_id, "x": x, "y": y, "z": z,"yaw":yaw, "pitch": pitch, "roll": roll},
                        timeout=1  # short timeout
                    )
                except requests.exceptions.RequestException as e:
                    print(f"[WARN] Failed to notify {camera_ip} marker update: {e}")
        
        

        @self.app.route('/change_room_shape', methods=['POST'])
        def change_room_shape():
            data = request.get_json()
            room_name = str(data.get("room"))
            boundry = data.get("boundry")
            origin = data.get("origin")
            width = data.get("width")
            height = data.get("height")
            if None in [room_name, boundry, origin, width, height]:
                return jsonify({"error": "Missing fields"}), 400

            markers = self.known_markers[room_name]
            markers.update({
                "boundry": boundry,
                "origin": origin,
                "width": width,
                "height": height
            })
            self.save_markers(self.known_markers, room=room_name)
            return jsonify({"status": "updated", "room": room_name}), 200
        
        @self.app.route('/delete_room', methods=['POST'])
        def delete_room():
            data = request.get_json()
            room_name = str(data.get("room"))
            if room_name not in self.known_markers:
                return jsonify({"error": "Room not found"}), 404
            del self.known_markers[room_name]
            self.save_markers(self.known_markers)
            @after_this_request
            def notify(response):  # <- executes after Flask finishes response
                notify_camera_room_deletion(self.left_camera_ip, room_name)
                notify_camera_room_deletion(self.right_camera_ip, room_name)
                return response
            return jsonify({"status": "deleted", "name": room_name}), 200
            
        def notify_camera_room_deletion(camera_ip, room_name):
            if camera_ip is not None:
                try:
                    response = requests.post(
                        f"http://{camera_ip}:5000/delete_room",
                        json={"room": room_name},
                        timeout=1
                    )
                    print(f"Camera at {camera_ip} notified of room deletion: {room_name}")
                except requests.exceptions.RequestException as e:
                    print(f"Failed to notify camera at {camera_ip}: {e}")
        
        
        @self.app.route('/delete_marker', methods=['POST'])
        def delete_marker():
            data = request.get_json()
            marker_id = str(data.get("id"))
            room = data.get("room")
            if room not in self.known_markers:
                return jsonify({"error": "Room not found"}), 404
            if marker_id is None:
                return jsonify({"error": "Missing ID"}), 400
            markers = self.known_markers[room]
            if marker_id in markers:
                del markers[marker_id]
                self.save_markers(self.known_markers, room=room)
                @after_this_request
                def notify(response):  # <- executes after Flask finishes response
                    notify_camera_marker_deletion(self.left_camera_ip, marker_id, room)
                    notify_camera_marker_deletion(self.right_camera_ip, marker_id, room)
                    return response
                return jsonify({"status": "deleted", "id": marker_id}), 200
            else:
                return jsonify({"error": "Marker not found"}), 404
        
        def notify_camera_marker_deletion(camera_ip, marker_id, room):
            if camera_ip is not None:
                try:
                    response = requests.post(
                        f"http://{camera_ip}:5000/delete_marker",
                        json={"id": marker_id, "room": room},
                        timeout=1
                    )
                    print(f"Camera at {camera_ip} notified of marker deletion: {marker_id}")
                except requests.exceptions.RequestException as e:
                    print(f"Failed to notify camera at {camera_ip}: {e}")
                    
        
        @self.app.route('/get_position')
        def get_position():
            dataToSend = self.getPos()

            return jsonify(dataToSend)
        
        @self.app.route('/get_aruco_list')
        def show_visualization():
            dataToSend = []
            dataToSend = self.getArucoList()
            if dataToSend is None:
                dataToSend = "GOT NONE"

            return jsonify(dataToSend)
        
        @self.app.route('/add_marker', methods=['POST'])
        def add_marker():
            data = request.get_json()
            marker_id = data.get('id')
            x, y, z = data.get('x'), data.get('y'), data.get('z')
            room = data.get('room')
            yaw = data.get('yaw')
            pitch = data.get('pitch')
            roll = data.get('roll')
            if marker_id and all(v is not None for v in [x, y, z, room, yaw, pitch, roll]):
                self.known_markers[room][marker_id] = {'x': x, 'y': y, 'z': z, 'yaw':yaw, 'pitch': pitch, 'roll': roll}
                self.save_markers(self.known_markers, room=room)
                @after_this_request
                def notify(response):  # <- executes after Flask finishes response
                    notify_camera_marker_addition(self.left_camera_ip, marker_id, x, y, z, room,yaw, pitch, roll)
                    notify_camera_marker_addition(self.right_camera_ip, marker_id, x, y, z, room,yaw, pitch, roll)
                    return response
                return jsonify({'status': 'success'}), 200
            return jsonify({'error': 'Invalid input'}), 400
        
        def notify_camera_marker_addition(camera_ip, marker_id, x, y, z, room,yaw, pitch, roll):
            if camera_ip is not None:
                try:
                    response = requests.post(
                        f"http://{camera_ip}:5000/add_marker",
                        json={"id": marker_id, "x": x, "y": y, "z": z, "room": room, 'yaw':yaw, 'pitch': pitch, 'roll': roll},
                        timeout=1
                    )
                    print(f"Camera at {camera_ip} notified of marker addition: {marker_id}")
                except requests.exceptions.RequestException as e:
                    print(f"Failed to notify camera at {camera_ip}: {e}")
        
        @self.app.route('/add_room', methods=['POST'])
        def add_room():
            data = request.get_json()
            room_name = data.get('room')
            if room_name:
                self.known_markers[room_name] = {}
                self.save_markers(self.known_markers, room = self.chosen_room)
                @after_this_request
                def notify(response):  # <- executes after Flask finishes response
                    notify_camera_room_addition(self.left_camera_ip, room_name)
                    notify_camera_room_addition(self.right_camera_ip, room_name)
                    return response
                return jsonify({'status': 'success'}), 200
            return jsonify({'error': 'Invalid input'}), 400
        
        def notify_camera_room_addition(camera_ip, room_name):
            if camera_ip is not None:
                try:
                    response = requests.post(
                        f"http://{camera_ip}:5000/add_room",
                        json={"room": room_name},
                        timeout=1
                    )
                    print(f"Camera at {camera_ip} notified of room addition: {room_name}")
                except requests.exceptions.RequestException as e:
                    print(f"Failed to notify camera at {camera_ip}: {e}")
        
        @self.app.route('/set_target', methods=['POST'])
        def set_target():
            data = request.get_json()
            if 'x' in data and 'y' in data:
                if self.target_position is None:
                    self.target_position = {}
                with self.lock:
                    self.target_position['x'] = data['x']
                    self.target_position['y'] = data['y']
                    self.target_position['z'] = data.get('z', 0.0)
                return jsonify({'status': 'success', 'target': self.target_position})
            return jsonify({'error': 'Invalid data'}), 400

        
        

        @self.app.route("/get_last_command", methods=["GET"])
        def get_last_command():
            return jsonify(self.last_command)

    def update_last_command(self, cmd, cmd_time):
            command = cmd
            if command:
                self.last_command["command"] = command
                self.last_command["timestamp"] = cmd_time
    
    def updatePosition(self, x, y, z, heading=None):
        with self.lock:
            self.position['x'] = x
            self.position['y'] = y
            self.position['z'] = z
            self.position['heading'] = heading if heading is not None else 0.0
    def getPos(self):
        with self.lock:
            return self.position
        
    def getArucoList(self):
        with self.lock:
            return self.arucoList
        
    def updateIds(self, arucoList):
        with self.lock:
            self.arucoList = arucoList
#for debug only use with out raspberry pi
'''
def main():
    # Initialize the server
    server_instance = server(port=5000)
    
    # Setup routes
    server_instance.setup_routes()
    
    # Run the server
    server_instance.run()

if __name__ == "__main__":
    main()
'''
