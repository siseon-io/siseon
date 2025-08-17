# chatbot/llm.py
import requests
from typing import List, Optional
from langchain.llms.base import LLM
from settings import OPENAI_API_KEY, OPENAI_API_BASE, GMS_MODEL, TEMPERATURE

class GMSChat(LLM):
    model_name: str
    api_base:   str
    api_key:    str
    temperature: float = 0.0

    def __init__(
        self,
        model_name: str = GMS_MODEL,
        api_base:   str = OPENAI_API_BASE,
        api_key:    str = OPENAI_API_KEY,
        temperature: float = TEMPERATURE,
    ):
        super().__init__(
            model_name=model_name,
            api_base=api_base,
            api_key=api_key,
            temperature=temperature,
        )

    @property
    def _identifying_params(self):
        return {"model_name": self.model_name, "temperature": self.temperature}

    @property
    def _llm_type(self) -> str:
        return "gms"

    def _call(self, prompt: str, stop: Optional[List[str]] = None) -> str:
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.api_key}"
        }
        payload = {
            "model": self.model_name,
            "input": prompt,
            "temperature": self.temperature
        }
        url = self.api_base
        resp = requests.post(url, json=payload, headers=headers)
        if resp.status_code != 200:
            print(f"🔴 LLM API Error {resp.status_code}:\n{resp.text}")
            resp.raise_for_status()
        data = resp.json()

        # 1) OpenAI-style choices
        if "choices" in data:
            choice = data["choices"][0]
            if "message" in choice:
                return choice["message"]["content"].strip()
            if "text" in choice:
                return choice["text"].strip()

        # 2) GMS Proxy ‘output’ 처리
        out_list = data.get("output")
        if isinstance(out_list, list) and out_list:
            first = out_list[0]
            # content가 리스트 형태로 들어있으면
            content = first.get("content")
            if isinstance(content, list) and content:
                text = content[0].get("text")
                if isinstance(text, str):
                    return text.strip()

        # 3) 기타 필드
        for key in ("text", "response"):
            val = data.get(key)
            if isinstance(val, str):
                return val.strip()
            if isinstance(val, list) and val and isinstance(val[0], str):
                return val[0].strip()

        # 4) 그래도 못 찾으면 에러
        raise ValueError(f"Unexpected payload structure:\n{data!r}")
