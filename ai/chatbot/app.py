# chatbot/app.py
from fastapi import FastAPI
from pydantic import BaseModel
from service import answer        # 절대 임포트

app = FastAPI(title="SISEON Chatbot Service")

class ChatRequest(BaseModel):
    question: str

class ChatResponse(BaseModel):
    summary: str
    details: dict

@app.post("/chat", response_model=ChatResponse)
def chat(req: ChatRequest):
    resp = answer(req.question)
    return ChatResponse(summary=resp.get("summary", ""), details=resp.get("details", {}))
