from flask import Flask, jsonify

app = Flask(__name__)

# Example robot position
robot_position = {
    "x": 1.2,
    "y": 1.8,
    "z": -1.0
}

@app.route('/get_position')
def get_position():
    return jsonify(robot_position)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)