FROM python:3.12-slim-bookworm
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 HOME=/tmp MAVLINK20=1
RUN pip install --no-cache-dir pymavlink==2.4.50
WORKDIR /app
COPY planner.py index.html /app/
USER 1001
EXPOSE 8080/tcp
ENTRYPOINT ["python3", "/app/planner.py"]
