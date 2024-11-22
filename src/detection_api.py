import time
from person_and_depth import get_all_people
from flask import Flask, jsonify
from threading import Thread
from typing import List, Dict

app = Flask(__name__)
data_store: List[Dict[str, float]] = [] # Stores detected people

def data_updater() -> None:
    """
    Continuously updates the global data_store with the latest data of detected people.
        Runs in a separate thread and polls for new data every 0.5 seconds to prevent overloading.
    """
    global data_store
    while True:
        data_store = get_all_people()
        time.sleep(0.5)

@app.route('/get_people', methods=['GET'])
def get_people() -> jsonify:
    """
    API endpoint to retrieve the latest data of people detected.
    
    Returns:
        JSON representation of the data_store containing people detection information.
    """    
    return jsonify(data_store)

if __name__ == "__main__":
    # Start the data updater thread to update detection data in the background.
    Thread(target=data_updater, daemon=True).start()
    # Run the Flask application on all available network interfaces on port 5000.
    app.run(host='0.0.0.0', port=5000)
