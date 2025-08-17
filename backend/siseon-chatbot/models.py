# chatbot/models.py
from sqlalchemy import (
    Column, BigInteger, Integer, String, Text, JSON, DateTime,
    ForeignKey, func, Index
)
from sqlalchemy.orm import declarative_base, relationship

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
    source         = Column(String(16), default="ai")  # 'ai' | 'faq'
    faq_id         = Column(BigInteger, ForeignKey("faqs.id", ondelete="SET NULL"), nullable=True)
    created_at     = Column(DateTime, nullable=False, server_default=func.now())

    faq = relationship("Faq", back_populates="logs")

    __table_args__ = (
        Index("idx_profile_created", "profile_id", "created_at"),
    )

class Faq(Base):
    __tablename__ = "faqs"

    id          = Column(BigInteger, primary_key=True, autoincrement=True)
    question    = Column(Text, nullable=False)
    answer      = Column(Text, nullable=False)
    category    = Column(String(64), nullable=True)
    tags        = Column(JSON, nullable=True)           # ["프리셋", "알림"]
    display_ord = Column(Integer, nullable=False, default=0)
    updated_at  = Column(DateTime, nullable=False, server_default=func.now(), onupdate=func.now())
    created_at  = Column(DateTime, nullable=False, server_default=func.now())

    logs = relationship("ChatLog", back_populates="faq")

Index("idx_faq_cat_ord", Faq.category, Faq.display_ord)