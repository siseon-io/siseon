# db.py
from sqlalchemy import create_engine, event, text
from sqlalchemy.engine import URL
from sqlalchemy.orm import sessionmaker
from settings import DB_HOST, DB_PORT, DB_USER, DB_PASSWORD, DB_NAME

# mysql+pymysql + utf8mb4
DATABASE_URL = URL.create(
    drivername="mysql+pymysql",
    username=DB_USER,
    password=DB_PASSWORD,
    host=DB_HOST,
    port=DB_PORT,
    database=DB_NAME,
    query={"charset": "utf8mb4"},
)

# 연결 안정성/풀 설정 (컨테이너 환경 권장값)
engine = create_engine(
    DATABASE_URL,
    pool_pre_ping=True,   # 커넥션 살아있나 헬스체크
    pool_recycle=300,     # 오래된 커넥션 재활용(네트워크 장비 타임아웃 회피)
    pool_size=5,          # 기본 풀 사이즈
    max_overflow=10,      # 스파이크 대응
    future=True,
)

# 세션 타임존/문자셋 보장 (NOW(), CURRENT_TIMESTAMP 등 서버함수 일관성)
@event.listens_for(engine, "connect")
def set_session_settings(dbapi_connection, connection_record):
    with dbapi_connection.cursor() as cur:
        # 세션 타임존을 KST로 고정 (DB 서버 타임존이 달라도 안전)
        cur.execute("SET time_zone = '+09:00'")
        # 문자셋 일치 (드라이버가 대부분 처리하지만 이중안전)
        cur.execute("SET NAMES utf8mb4")
        # 필요시 엄격모드 선호 시 주석 해제
        # cur.execute("SET SESSION sql_mode = 'STRICT_TRANS_TABLES'")

SessionLocal = sessionmaker(
    bind=engine,
    autocommit=False,
    autoflush=False,
    future=True,
)
