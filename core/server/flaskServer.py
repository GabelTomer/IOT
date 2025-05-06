from flask import Flask, jsonify, request
import json
import threading
class server:
    def __init__(self,  port = 5000, known_markers_path=None):
        self.known_markers = self.load_known_markers(known_markers_path) if known_markers_path else {}
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
        @self.app.route('/get_data')
        def get_data():
            # Example data to return
            data = {
                'key1': 'value1',
                'key2': 'value2'
            }
            return jsonify(data)

        @self.app.route('/get_position')
        def get_position():
            print(f"this is jsonify {self.position} \n")
            # Example position data
            return jsonify(self.getPos())
    
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
