# test_app.py

import requests

def test_chat_endpoint():
    url = "http://localhost:8000/chat"
    payload = {"question": "SISEON 매뉴얼의 모델 이름은?"}
    headers = {"Content-Type": "application/json"}

    resp = requests.post(url, json=payload, headers=headers)
    print("Status code:", resp.status_code)
    try:
        print("Response JSON:", resp.json())
    except ValueError:
        print("Non-JSON response:", resp.text)

if __name__ == "__main__":
    test_chat_endpoint()
