from fastapi import FastAPI, Depends, APIRouter, HTTPException, Query, Response
from pydantic import BaseModel, Field, conint
from contextlib import asynccontextmanager
from sqlalchemy.orm import Session
from sqlalchemy.exc import SQLAlchemyError
from typing import Any, Optional, List
from datetime import datetime, timedelta
import hashlib
import json
import time
import logging

from db import SessionLocal, engine
from models import Base, ChatLog, Faq
from service import answer
from security import get_current_user, TokenData
from seed import seed_faqs
from sqlalchemy import text

# ──────────────────────
# Logging
# ──────────────────────
log = logging.getLogger("chatbot.app")

# ──────────────────────
# App / Lifespan
# ──────────────────────
@asynccontextmanager
async def lifespan(app: FastAPI):
    # DB 테이블 생성
    Base.metadata.create_all(bind=engine)

    # 서버 기동 시 FAQ 시드 데이터 업서트
    db = SessionLocal()
    try:
        db.execute(text("SET NAMES utf8mb4"))  # 인코딩 방어
        n = seed_faqs(db)
        db.commit()
        log.info("[startup] FAQ seed upserted/updated=%s", n)
    except Exception:
        db.rollback()
        log.exception("[startup] FAQ seed failed")
    finally:
        db.close()

    yield

app = FastAPI(title="SISEON Chatbot Service", lifespan=lifespan)
api_router = APIRouter(prefix="/api")

# ──────────────────────
# Schemas
# ──────────────────────
class ChatRequest(BaseModel):
    question: str
    profile_id: conint(gt=0) = Field(..., alias="profileId")

class ChatResponse(BaseModel):
    summary: str
    created_at: datetime

class ChatMessage(BaseModel):
    role: str
    content: str
    created_at: datetime

class FaqOut(BaseModel):
    id: int
    question: str
    answer: str
    category: Optional[str] = None
    tags: Optional[List[str]] = None
    display_ord: int
    updated_at: datetime
    created_at: datetime
    class Config:
        from_attributes = True

class QuickAnswerIn(BaseModel):
    profileId: conint(gt=0)

class QuickAnswerOut(BaseModel):
    answer: str
    faq_id: int

# ──────────────────────
# DB Dependency
# ──────────────────────
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# ──────────────────────
# Utils
# ──────────────────────
def _safe_json(obj: Any) -> dict:
    try:
        return json.loads(json.dumps(obj, ensure_ascii=False, default=str))
    except Exception:
        log.exception("details not JSON-serializable; fallback to string")
        return {"error": "details_not_serializable", "details_str": str(obj)}

# ──────────────────────
# Chat
# ──────────────────────
@api_router.post("/chat", response_model=ChatResponse)
def chat(req: ChatRequest,
         user: TokenData = Depends(get_current_user),
         db: Session = Depends(get_db)):

    t0 = time.time()
    log.info("[chat] start user=%s profile_id=%s question_len=%s",
             user.sub, req.profile_id, len(req.question))

    try:
        resp = answer(req.question, session_id=str(req.profile_id))
    except Exception as e:
        log.exception("[chat] answer() failed user=%s profile_id=%s", user.sub, req.profile_id)
        raise HTTPException(status_code=502, detail=f"LLM/answer failed: {type(e).__name__}: {e}")

    try:
        summary = resp.get("summary") if isinstance(resp, dict) else None
        details = resp.get("details") if isinstance(resp, dict) else None
    except Exception:
        log.exception("[chat] resp invalid user=%s profile_id=%s resp=%r", user.sub, req.profile_id, resp)
        raise HTTPException(status_code=500, detail="answer() returned invalid structure")

    if summary is None:
        summary = ""
    if not isinstance(details, dict):
        details = {"raw": resp}

    latency = int((time.time() - t0) * 1000)
    safe_details = _safe_json(details)

    try:
        row = ChatLog(
            profile_id=req.profile_id,
            question=req.question,
            answer_summary=summary,
            answer_details=safe_details,
            latency_ms=latency,
            status="success",
            source="ai",
            faq_id=None,
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

    log.info("[chat] done user=%s profile_id=%s latency_ms=%s row_id=%s",
             user.sub, req.profile_id, latency, row.id)
    return ChatResponse(summary=row.answer_summary, created_at=row.created_at)

# ──────────────────────
# Chat History
# ──────────────────────
@api_router.get("/chat/history", response_model=List[ChatMessage])
def history(profileId: int = Query(..., gt=0),
            limit: int = Query(200, ge=1, le=1000),
            user: TokenData = Depends(get_current_user),
            db: Session = Depends(get_db)):

    rows = (
        db.query(ChatLog)
          .filter(ChatLog.profile_id == profileId)
          .order_by(ChatLog.created_at.asc())
          .limit(limit)
          .all()
    )

    messages: List[ChatMessage] = []
    for r in rows:
        asked_dt = r.created_at - timedelta(milliseconds=(r.latency_ms or 0))
        answered_dt = r.created_at
        messages.append(ChatMessage(role="user", content=r.question, created_at=asked_dt))
        messages.append(ChatMessage(role="assistant", content=r.answer_summary or "", created_at=answered_dt))

    return messages

# ──────────────────────
# FAQ
# ──────────────────────
@api_router.get("/faq", response_model=List[FaqOut])
def list_faqs(category: Optional[str] = None,
              q: Optional[str] = Query(None),
              response: Response = None,
              user: TokenData = Depends(get_current_user),
              db: Session = Depends(get_db)):

    stmt = db.query(Faq)
    if category:
        stmt = stmt.filter(Faq.category == category)
    if q:
        like = f"%{q}%"
        stmt = stmt.filter((Faq.question.like(like)) | (Faq.answer.like(like)))

    rows = stmt.order_by(Faq.display_ord.asc(), Faq.id.asc()).all()

    etag_src = "".join([f"{r.id}:{r.updated_at}" for r in rows]).encode("utf-8")
    etag = hashlib.md5(etag_src).hexdigest()

    if response is not None:
        response.headers["Cache-Control"] = "max-age=600"
        response.headers["ETag"] = etag

    return [FaqOut.model_validate(r) for r in rows]

@api_router.post("/faq/{faq_id}/answer", response_model=QuickAnswerOut)
def answer_from_faq(faq_id: int,
                    body: QuickAnswerIn,
                    user: TokenData = Depends(get_current_user),
                    db: Session = Depends(get_db)):

    faq = db.get(Faq, faq_id)
    if not faq:
        raise HTTPException(status_code=404, detail="FAQ not found")

    try:
        log_row = ChatLog(
            profile_id=body.profileId,
            question=f"[FAQ] {faq.question}",
            answer_summary=faq.answer,
            answer_details=None,
            latency_ms=0,
            status="success",
            source="faq",
            faq_id=faq.id,
        )
        db.add(log_row)
        db.commit()
    except SQLAlchemyError as e:
        db.rollback()
        log.exception("[faq.answer] DB error user=%s profile_id=%s", user.sub, body.profileId)
        raise HTTPException(status_code=500, detail=f"DB error: {type(e).__name__}: {e}")

    return QuickAnswerOut(answer=faq.answer, faq_id=faq.id)

# ──────────────────────
# Router Mount
# ──────────────────────
app.include_router(api_router)
