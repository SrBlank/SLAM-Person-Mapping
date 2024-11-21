import time
from flask import Flask, jsonify

from threading import Thread

app = Flask(__name__)

# Sample data
sample_data = [
    [{'depth': 1.28, 'confidence': 0.9998899698257446}],
    [{'depth': 1.28, 'confidence': 0.9998899698257446}],
    [{'depth': 1.28, 'confidence': 0.9998899698257446}],
    [{'depth': 1.15, 'confidence': 0.942328691482544}],
    [{'depth': 1.15, 'confidence': 0.942328691482544}],
    [{'depth': 1.0, 'confidence': 0.942328691482544}],
    [{'depth': .5, 'confidence': 0.942328691482544}],
    [{'depth': .5, 'confidence': 0.942328691482544}],
]

data_store = []

def fake_data_updater():
    global data_store
    while True:
        for data in sample_data:
            data_store = data
            time.sleep(1)

@app.route('/get_people', methods=['GET'])
def get_people():
    """API endpoint to get the latest data of people detected."""
    return jsonify(data_store)

if __name__ == "__main__":
    Thread(target=fake_data_updater, daemon=True).start()
    app.run(host='0.0.0.0', port=5000)
