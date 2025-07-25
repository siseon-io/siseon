# gaze_detection/inference/webcam/sender.py

import logging
import requests

def send_to_backend(url: str, payload: dict, timeout: float=0.1):
    try:
        requests.post(url, json=payload,
                      headers={"Content-Type":"application/json"},
                      timeout=timeout)
    except requests.RequestException as e:
        logging.warning(f"Failed to send data: {e}")
