from flask import Flask, request

app = Flask(__name__)

@app.route('/', methods=['GET'])
def index():
    return '''
        <form action="/send" method="post">
            <input type="text" name="message" placeholder="Enter message">
            <input type="submit" value="Send">
        </form>
    '''

@app.route('/send', methods=['POST'])
def handle_data():
    message = request.form['message']
    print(f"Received message: {message}")
    return f"Message received: {message}"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=4000)


# ------ TEST NOTES ------
# Open browser and go to: http://<raspberry-pi-ip>:4000
# Enter a message & then submit the form.
# The message should be printed in the Raspberry Pi's terminal.
