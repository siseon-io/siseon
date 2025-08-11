FROM python:3.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# 매뉴얼 포함 및 존재 검증 (없으면 빌드 실패)
RUN mkdir -p /app/manuals
COPY manuals/siseon_manual.pdf /app/manuals/siseon_manual.pdf
RUN test -f /app/manuals/siseon_manual.pdf || (echo "❌ Missing manuals/siseon_manual.pdf" && exit 1)
ENV MANUAL_PATH=/app/manuals/siseon_manual.pdf

# 앱 소스 복사
COPY . /app

EXPOSE 8000
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8000"]
