import time

from person_and_depth import get_all_people
from flask import Flask, jsonify
from threading import Thread

app = Flask(__name__)
data_store = []

def data_updater():
    global data_store
    while True:
        data_store = get_all_people()
        time.sleep(0.5)

@app.route('/get_people', methods=['GET'])
def get_people():
    """API endpoint to get the latest data of people detected."""
    return jsonify(data_store)

if __name__ == "__main__":
    Thread(target=data_updater, daemon=True).start()
    app.run(host='0.0.0.0', port=5000)
