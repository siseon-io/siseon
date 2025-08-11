FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 타임존 설정 추가 (KST)
RUN apt-get update && apt-get install -y --no-install-recommends tzdata \
 && ln -sf /usr/share/zoneinfo/Asia/Seoul /etc/localtime \
 && echo "Asia/Seoul" > /etc/timezone \
 && rm -rf /var/lib/apt/lists/*

# 매뉴얼 포함 및 존재 검증 (없으면 빌드 실패)
RUN mkdir -p /app/manuals
COPY manuals/siseon_manual.pdf /app/manuals/siseon_manual.pdf
RUN test -f /app/manuals/siseon_manual.pdf || (echo "❌ Missing manuals/siseon_manual.pdf" && exit 1)
ENV MANUAL_PATH=/app/manuals/siseon_manual.pdf

# 앱 소스 복사
COPY . /app

EXPOSE 8000
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8000"]
