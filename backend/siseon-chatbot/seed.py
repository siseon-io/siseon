# chatbot/seed.py
from typing import List
from sqlalchemy.orm import Session
from sqlalchemy import select
from models import Faq

FAQ_SEED = [
    # ── 시작/홈/모드. ─────────────────────────────────────────────────────
    {
        "question": "처음 설치하면 무엇부터 하면 되나요?",
        "answer": "앱 설치 후 로그인 → 프로필 설정 → 홈에서 자동 제어(가운데 눈 버튼)를 켠 뒤 기본 동작을 확인하세요. 프리셋으로 저장 후 다음부터 원터치로 불러올 수도 있습니다.",
        "category": "시작하기",
        "tags": ["설치", "로그인", "프로필", "첫사용"],
        "display_ord": 1,
    },
    {
        "question": "수동 제어로 들어가면 자동 제어는 어떻게 되나요?",
        "answer": "수동 제어 화면으로 이동하면 자동 조절이 일시 정지되고, 화면을 나가면 자동 제어로 자동 복귀합니다.",
        "category": "홈/모드",
        "tags": ["자동모드", "수동모드", "전환"],
        "display_ord": 2,
    },

    # ── 프리셋 ─────────────────────────────────────────────────────────────
    {
        "question": "프리셋은 어떻게 저장하나요?",
        "answer": "홈화면에서 프리셋 탭에서 [+]버튼을 눌러 현재 모니터 위치를 저장하거나 [설정 > 프리셋] 페이지에서 [+] 버튼을 누를시 현재 모니터 위치를 저장합니다.",
        "category": "프리셋",
        "tags": ["프리셋", "저장"],
        "display_ord": 1,
    },
    {
        "question": "저장한 프리셋은 몇 개까지 가능한가요?",
        "answer": "3개까지 저장할 수 있고, 빈 프리셋이 있어야 추가 저장이 가능합니다.",
        "category": "프리셋",
        "tags": ["프리셋", "개수", "제한"],
        "display_ord": 2,
    },
    {
        "question": "프리셋을 불러오면 어떻게 동작하나요?",
        "answer": "프리셋 목록에서 항목을 탭하면 모니터가 해당 위치로 자동 이동합니다. Preset 모드 상태가 화면에 표시됩니다.",
        "category": "프리셋",
        "tags": ["프리셋", "호출", "자동이동"],
        "display_ord": 3,
    },
    {
        "question": "프리셋 이름 변경이나 삭제는 어디서 하나요?",
        "answer": "[설정 > 프리셋] 화면에서 편집 아이콘을 통해 이름 변경, 삭제가 가능합니다.",
        "category": "프리셋",
        "tags": ["프리셋", "편집", "삭제", "이름변경"],
        "display_ord": 4,
    },

    # ── 수동 제어 ─────────────────────────────────────────────────────────
    {
        "question": "수동으로 어떻게 움직이나요?",
        "answer": "수동 제어 화면에서 조이스틱으로 X/Y/Z 이동을 할 수 있습니다. 만족스러운 위치는 프리셋으로 저장해 두세요.",
        "category": "수동 제어",
        "tags": ["수동제어", "슬라이더", "화살표", "미세조정"],
        "display_ord": 1,
    },
    {
        "question": "좌우/거리 조절도 가능한가요?",
        "answer": "가능합니다. 좌우 이동과 앞뒤(거리) 조절 컨트롤이 제공되며, 장치의 물리적 범위 내에서 동작합니다.",
        "category": "수동 제어",
        "tags": ["좌우", "거리", "범위제한"],
        "display_ord": 2,
    },

    # ── 자동 제어 & 알림 ──────────────────────────────────────────────────
    {
        "question": "앱이 내 자세를 자동으로 맞춰주나요?",
        "answer": "네. 카메라로 눈·상체를 추적해 모니터 높이/각도를 자동 보정합니다. 사용자가 움직여도 실시간으로 따라갑니다.",
        "category": "자동 제어",
        "tags": ["자동조정", "시선추적", "자세인식"],
        "display_ord": 1,
    },
    {
        "question": "잘못된 자세 알림은 어떻게 오나요?",
        "answer": "잘못된 자세가 일정 시간 (30분 이상) 지속되면 푸시 알림이 전송됩니다.",
        "category": "알림",
        "tags": ["자세", "푸시", "알림"],
        "display_ord": 1,
    },

    # ── 통계 ──────────────────────────────────────────────────────────────
    {
        "question": "자세 통계는 어디서 보나요?",
        "answer": "[설정 > 통계]에서 일/주/월 단위로 바른 자세와 올바르지 않은 자세의 사용 시간 비율을 그래프로 확인할 수 있습니다.",
        "category": "통계",
        "tags": ["통계", "설정", "그래프", "경고추세"],
        "display_ord": 1,
    },
    {
        "question": "통계 데이터는 자동으로 쌓이나요?",
        "answer": "AI 모니터링으로 자동 수집됩니다. 더 정확하게 측정하려면 카메라 렌즈를 주기적으로 청소해 주세요.",
        "category": "통계",
        "tags": ["통계", "자동수집", "정확도"],
        "display_ord": 2,
    },

    # ── 계정/프로필 & 장치 ────────────────────────────────────────────────
    {
        "question": "프로필은 어디서 관리하나요?",
        "answer": "[설정 > 프로필]에서 프로필의 이름·생년월일·시력 등 수정이 가능합니다.",
        "category": "계정",
        "tags": ["계정", "프로필", "설정", "수정"],
        "display_ord": 1,
    },
    {
        "question": "프로필을 여러 개 만들 수 있나요?",
        "answer": "하나의 계정에서 최대 4개의 프로필을 생성해 서로 다른 환경을 저장할 수 있습니다.",
        "category": "계정",
        "tags": ["계정", "프로필", "멀티프로필"],
        "display_ord": 2,
    },
    {
        "question": "펌웨어 업데이트는 가능한가요?",
        "answer": "[설정 > 기기정보]에서 시리얼넘버 확인 및 펌웨어 업데이트를 진행할 수 있습니다.",
        "category": "장치",
        "tags": ["펌웨어", "기기정보", "업데이트"],
        "display_ord": 1,
    },
    {
        "question": "블루투스 연결이 안돼요",
        "answer": "[설정 > 기기정보]에 가서 시리얼넘버가 정확한지 확인해주세요. 정확하지 않다면, 오른쪽 위 기기 삭제를 하고 다시 홈화면에서 기기 등록을 진행해주세요.",
        "category": "장치",
        "tags": ["블루투스", "연결", "기기등록", "시리얼넘버"],
        "display_ord": 2,
    },

    # ── 문제 해결 & 챗봇 ─────────────────────────────────────────────────
    {
        "question": "로봇 팔이 전혀 안 움직여요. 어떻게 해야 하나요?",
        "answer": "전원과 네트워크 상태를 확인하고 홈의 블루투스 버튼을 누른 뒤, 수동 제어에서 직접 조작을 시도해 보세요. 반응이 없으면 앱을 재부팅한 뒤 다시 시도해 보세요.",
        "category": "문제 해결",
        "tags": ["로봇팔", "동작불가", "재부팅"],
        "display_ord": 1,
    },
    {
        "question": "앱에서 바로 도움을 받을 수 있나요?",
        "answer": "앱 내 챗봇에서 FAQ 기반 빠른 답변을 즉시 받을 수 있고, 심화 질문은 가이드 매뉴얼을 바탕으로 상세히 안내해 줍니다.",
        "category": "챗봇",
        "tags": ["챗봇", "FAQ", "LLM"],
        "display_ord": 1,
    },

    # ── 1:1 문의 ─────────────────────────────────────────────────────────
    {
        "question": "1:1 문의는 어디로 하면 되나요?",
        "answer": "서비스 관련 문의는 siseon.service@gmail.com로 보내주세요. 가능한 한 빠르게 답변드리겠습니다.",
        "category": "문의",
        "tags": ["문의", "고객센터", "1:1문의"],
        "display_ord": 2,
    },
]


def seed_faqs(db: Session) -> int:
    changed = 0
    for item in FAQ_SEED:
        q = item["question"]
        existing = db.execute(select(Faq).where(Faq.question == q)).scalar_one_or_none()
        if existing:
            dirty = False
            if existing.answer != item["answer"]:
                existing.answer = item["answer"]; dirty = True
            if existing.category != item["category"]:
                existing.category = item["category"]; dirty = True
            if existing.tags != item["tags"]:
                existing.tags = item["tags"]; dirty = True
            if existing.display_ord != item["display_ord"]:
                existing.display_ord = item["display_ord"]; dirty = True
            if dirty:
                changed += 1
        else:
            db.add(Faq(**item))
            changed += 1
    return changed

def delete_faqs_by_questions(db: Session, questions: List[str]) -> int:
    rows = db.execute(select(Faq).where(Faq.question.in_(questions))).scalars().all()
    n = 0
    for r in rows:
        db.delete(r)
        n += 1
    return n
