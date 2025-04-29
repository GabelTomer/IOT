from flask import Flask, jsonify
class server:
    def __init__(self, port = 5000):
        self.port = port
        self.position = {
            'x': 0,
            'y': 0,
            'z': 0
        }
        self.app = Flask(__name__)
    
    def run(self):
          self.app.run(host='0.0.0.0', port=self.port)
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return "Welcome to the Flask Server!"

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
            # Example position data
            return jsonify(self.position)
    
        def updatePosition(self, x, y, z):
            self.position['x'] = x
            self.position['y'] = y
            self.position['z'] = z
#for debug only use with out raspberry pi
def main():
    # Initialize the server
    server_instance = server(port=5000)
    
    # Setup routes
    server_instance.setup_routes()
    
    # Run the server
    server_instance.run()

if __name__ == "__main__":
    main()