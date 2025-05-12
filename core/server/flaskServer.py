from flask import Flask, jsonify, request
import json
import threading
class server:
    def __init__(self,  port = 5000, known_markers_path=None, detector=None):
        self.detector = detector
        self.known_markers = self.load_known_markers(known_markers_path) if known_markers_path else {}
        self.known_markers_path = known_markers_path
        self.lock = threading.Lock()
        self.port = port
        self.position = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0
        }
        self.app = Flask(__name__)
    
    def load_known_markers(self, path):
        with open(path, 'r') as file:
            data = json.load(file)
        if not data:
            raise ValueError("[ERROR] No known markers found in the file.")
            return {}
        return data

    
    def save_markers(self, data):
        with open(self.known_markers_path, "w") as f:
            json.dump(data, f, indent=2)
        self.known_markers = data
        self.detector.update_known_markers()
    
    def run(self):
          self.app.run(host='0.0.0.0', port=self.port)
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return "Welcome to the Flask Server!"
        @self.app.route('/get_Known_Markers')
        def get_known_markers():
            return jsonify(self.known_markers)
        
        
        @self.app.route('/update_position', methods=['POST'])
        def update_position():
            # Example data to update position
            data = request.get_json()
            if 'x' in data and 'y' in data:
                print(f"this is data {data} \n")
                return jsonify({'status': 'success', 'position': data})
            return jsonify({'status': 'error', 'message': 'Invalid data'}), 400
        
        
        @self.app.route('/update_marker', methods=['POST'])
        def update_marker():
            data = request.get_json()
            marker_id = str(data.get("id"))
            x = data.get("x")
            y = data.get("y")
            z = data.get("z")

            if None in [marker_id, x, y, z]:
                return jsonify({"error": "Missing fields"}), 400

            markers = self.known_markers
            markers[marker_id] = {"x": x, "y": y, "z": z}
            self.save_markers(markers)
            return jsonify({"status": "updated", "id": marker_id}), 200
        
        
        @self.app.route('/delete_marker', methods=['POST'])
        def delete_marker():
            data = request.get_json()
            marker_id = str(data.get("id"))
    
            if marker_id is None:
                return jsonify({"error": "Missing ID"}), 400

            if marker_id in self.known_markers:
                del self.known_markers[marker_id]
                self.save_markers(self.known_markers)
                return jsonify({"status": "deleted", "id": marker_id}), 200
            else:
                return jsonify({"error": "Marker not found"}), 404
        
        
        @self.app.route('/get_position')
        def get_position():
            print(f"this is jsonify {self.position} \n")
            # Example position data
            return jsonify(self.getPos())
        
        @self.app.route('/add_marker', methods=['POST'])
        def add_marker():
            data = request.get_json()
            marker_id = data.get('id')
            x, y, z = data.get('x'), data.get('y'), data.get('z')
            if marker_id and all(v is not None for v in [x, y, z]):
                self.known_markers[marker_id] = {'x': x, 'y': y, 'z': z}
                with open(self.known_markers_path, 'w') as f:
                    json.dump(self.known_markers, f)
                return jsonify({'status': 'success'}), 200
            return jsonify({'error': 'Invalid input'}), 400
    
    def updatePosition(self, x, y, z):
        with self.lock:
            self.position['x'] = x
            self.position['y'] = y
            self.position['z'] = z
    def getPos(self):
        with self.lock:
            return self.position

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
