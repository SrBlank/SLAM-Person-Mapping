import time
from flask import Flask, jsonify

from threading import Thread

app = Flask(__name__)

# Sample data
sample_data = [
    [{'depth': 0.5260000228881836, 'confidence': 0.9105256199836731}],
    [],
    [],
    [{'depth': 0.27400001883506775, 'confidence': 0.2727588713169098}],
    [{'depth': 1.8620001077651978, 'confidence': 0.9919590950012207}],
    [{'depth': 1.2780001163482666, 'confidence': 0.9998899698257446}],
    [{'depth': 1.255000114440918, 'confidence': 0.942328691482544}],
    [{'depth': 1.4800000190734863, 'confidence': 0.9984509944915771}],
    [{'depth': 1.4940000772476196, 'confidence': 0.9989467263221741}],
    [{'depth': 1.5190000534057617, 'confidence': 0.9986746311187744}],
    [],
    [{'depth': 1.7810001373291016, 'confidence': 0.9949648976325989}],
    [{'depth': 1.7510000467300415, 'confidence': 0.9739450216293335}, {'depth': 2.5830001831054688, 'confidence': 0.92947918176651}],
    [{'depth': 1.7610000371932983, 'confidence': 0.9768665432929993}, {'depth': 2.7920000553131104, 'confidence': 0.9095771312713623}],
    [{'depth': 1.7660000324249268, 'confidence': 0.9749782085418701}, {'depth': 2.5300002098083496, 'confidence': 0.9720422625541687}],
    [{'depth': 1.787000060081482, 'confidence': 0.9496432542800903}, {'depth': 2.616000175476074, 'confidence': 0.9482998251914978}],
    [{'depth': 1.7510000467300415, 'confidence': 0.995796799659729}]
]

data_store = []

def fake_data_updater():
    global data_store
    while True:
        for data in sample_data:
            data_store = data
            time.sleep(2)

@app.route('/get_people', methods=['GET'])
def get_people():
    """API endpoint to get the latest data of people detected."""
    return jsonify(data_store)

if __name__ == "__main__":
    Thread(target=fake_data_updater, daemon=True).start()
    app.run(host='0.0.0.0', port=5000)
