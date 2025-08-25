.PHONY: start_prod
.PHONY: start_runner
.PHONY: deploy

deploy:
	@echo "Deploying app: syncing dependencies and starting services..."
	@uv sync
	@caddy start
	@echo "Starting FastAPI..."
	@uv run uvicorn main:app --host 0.0.0.0 --port 8000

start_prod:
	@echo "Starting FastAPI and Caddy..."
	@trap 'kill %1 %2 2>/dev/null || true' EXIT; \
	uv run uvicorn main:app --host 0.0.0.0 --port 8000 & \
	caddy run & \
	wait

start_runner:
	@echo "Starting GH runner..."
	@trap 'kill %1 %2 2>/dev/null || true' EXIT; \
	./actions-runner/run.sh & \
	wait