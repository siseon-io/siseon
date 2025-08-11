from fastapi import FastAPI, Depends, APIRouter, Query, HTTPException
from pydantic import BaseModel, Field, conint
from security import get_current_user, TokenData
from service import answer

from contextlib import asynccontextmanager
from db import SessionLocal, engine
from models import Base, ChatLog
from sqlalchemy.orm import Session
from sqlalchemy.exc import SQLAlchemyError
import json
import time
import logging
from typing import Any

# ── logging
log = logging.getLogger("chatbot.app")

@asynccontextmanager
async def lifespan(app: FastAPI):
    Base.metadata.create_all(bind=engine)
    yield

app = FastAPI(title="SISEON Chatbot Service", lifespan=lifespan)
api_router = APIRouter(prefix="/api")

# ====== Schemas ======
class ChatRequest(BaseModel):
    question: str
    profile_id: conint(gt=0) = Field(..., alias="profileId")

class ChatResponse(BaseModel):
    summary: str
    created_at: str

class ChatMessage(BaseModel):
    role: str
    content: str
    created_at: str

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def _safe_json(obj: Any) -> dict:
    try:
        return json.loads(json.dumps(obj, ensure_ascii=False, default=str))
    except Exception:
        log.exception("details not JSON-serializable; fallback to string")
        return {"error": "details_not_serializable", "details_str": str(obj)}

# ====== Endpoints ======
@api_router.post("/chat", response_model=ChatResponse)
def chat(req: ChatRequest,
         user: TokenData = Depends(get_current_user),
         db: Session = Depends(get_db)):

    t0 = time.time()
    log.info("[chat] start user=%s profile_id=%s question_len=%s",
             user.sub, req.profile_id, len(req.question))

    # 1) LLM 호출
    try:
        resp = answer(req.question, session_id=str(req.profile_id))
    except Exception as e:
        log.exception("[chat] answer() failed user=%s profile_id=%s", user.sub, req.profile_id)
        raise HTTPException(status_code=502, detail=f"LLM/answer failed: {type(e).__name__}: {e}")

    # 2) 결과 가드 + 직렬화 방어
    try:
        summary = resp.get("summary") if isinstance(resp, dict) else None
        details = resp.get("details") if isinstance(resp, dict) else None
    except Exception:
        log.exception("[chat] resp structure invalid user=%s profile_id=%s resp=%r", user.sub, req.profile_id, resp)
        raise HTTPException(status_code=500, detail="answer() returned invalid structure")

    if summary is None:
        summary = ""
    if not isinstance(details, dict):
        details = {"raw": resp}

    latency = int((time.time() - t0) * 1000)
    safe_details = _safe_json(details)  # DB 저장용(응답에는 포함하지 않음)

    # 3) DB 저장
    try:
        row = ChatLog(
            profile_id=req.profile_id,
            question=req.question,
            answer_summary=summary,
            answer_details=safe_details,
            latency_ms=latency,
            status="success",
        )
        db.add(row)
        db.commit()
        db.refresh(row)
    except SQLAlchemyError as e:
        db.rollback()
        log.exception("[chat] DB error user=%s profile_id=%s", user.sub, req.profile_id)
        raise HTTPException(status_code=500, detail=f"DB error: {type(e).__name__}: {e}")
    except Exception as e:
        db.rollback()
        log.exception("[chat] unexpected DB save error user=%s profile_id=%s", user.sub, req.profile_id)
        raise HTTPException(status_code=500, detail=f"Unexpected DB error: {type(e).__name__}: {e}")

    created_iso = row.created_at.replace(tzinfo=None).isoformat() + "Z"
    log.info("[chat] done user=%s profile_id=%s latency_ms=%s row_id=%s",
             user.sub, req.profile_id, latency, row.id)

    return ChatResponse(summary=row.answer_summary, created_at=created_iso)

@api_router.get("/chat/history", response_model=list[ChatMessage])
def history(profile_id: conint(gt=0) = Query(..., alias="profileId"),
            limit: int = Query(200, ge=1, le=1000),
            user: TokenData = Depends(get_current_user),
            db: Session = Depends(get_db)):

    log.info("[history] start user=%s profile_id=%s limit=%s", user.sub, profile_id, limit)
    try:
        rows = (db.query(ChatLog)
                  .filter(ChatLog.profile_id == profile_id)
                  .order_by(ChatLog.created_at.asc())
                  .limit(limit).all())
    except SQLAlchemyError as e:
        log.exception("[history] DB error user=%s profile_id=%s", user.sub, profile_id)
        raise HTTPException(status_code=500, detail=f"DB error: {type(e).__name__}: {e}")
    except Exception as e:
        log.exception("[history] unexpected error user=%s profile_id=%s", user.sub, profile_id)
        raise HTTPException(status_code=500, detail=f"Unexpected error: {type(e).__name__}: {e}")

    messages: list[ChatMessage] = []
    try:
        for r in rows:
            t = r.created_at.replace(tzinfo=None).isoformat() + "Z"
            messages.append(ChatMessage(role="user", content=r.question, created_at=t))
            assistant_text = r.answer_summary or ""
            messages.append(ChatMessage(role="assistant", content=assistant_text, created_at=t))
    except Exception:
        log.exception("[history] serialization error user=%s profile_id=%s", user.sub, profile_id)
        raise HTTPException(status_code=500, detail="Serialization error")

    log.info("[history] done user=%s profile_id=%s count=%s", user.sub, profile_id, len(messages))
    return messages

app.include_router(api_router)
