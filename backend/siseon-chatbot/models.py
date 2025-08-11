# chatbot/models.py
from sqlalchemy import Column, BigInteger, Integer, Text, JSON, TIMESTAMP, String, func, Index
from sqlalchemy.orm import declarative_base

Base = declarative_base()

class ChatLog(Base):
    __tablename__ = "chat_logs"

    id             = Column(BigInteger, primary_key=True, autoincrement=True)
    profile_id     = Column(BigInteger, nullable=False)
    question       = Column(Text, nullable=False)
    answer_summary = Column(Text)
    answer_details = Column(JSON)
    latency_ms     = Column(Integer)
    status         = Column(String(32), default="success")
    created_at     = Column(TIMESTAMP, nullable=False, server_default=func.now())

    __table_args__ = (
        Index("idx_profile_created", "profile_id", "created_at"),
    )
