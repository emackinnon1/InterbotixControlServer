start_prod:
    @echo "Starting FastAPI and Caddy..."
    @trap 'kill %1 %2 2>/dev/null || true' EXIT; \
    uv run uvicorn main:app --host 0.0.0.0 --port 8000 & \
    caddy run & \
    wait

.PHONY: start_prod