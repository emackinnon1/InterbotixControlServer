.PHONY: start_prod stop_prod
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
	@trap 'echo "Stopping services..."; make stop_prod 2>/dev/null || true' EXIT INT TERM; \
	caddy start && \
	uv run uvicorn main:app --host 0.0.0.0 --port 8000 --pid-file /tmp/uvicorn.pid & \
	echo $$! > /tmp/uvicorn.pid && \
	wait

stop_prod:
	@echo "Stopping production services..."
	@caddy stop 2>/dev/null || true
	@if [ -f /tmp/uvicorn.pid ]; then \
		kill $$(cat /tmp/uvicorn.pid) 2>/dev/null || true; \
		rm -f /tmp/uvicorn.pid; \
	fi
	@pkill -f "uvicorn main:app" 2>/dev/null || true
	@echo "Services stopped."

cleanup:
	@echo "Cleaning up any stuck processes..."
	@caddy stop 2>/dev/null || true
	@pkill -f "uvicorn main:app" 2>/dev/null || true
	@pkill -f "caddy" 2>/dev/null || true
	@rm -f /tmp/uvicorn.pid
	@echo "Cleanup complete."

start_runner:
	@echo "Starting GH runner..."
	@trap 'kill %1 %2 2>/dev/null || true' EXIT; \
	./actions-runner/run.sh & \
	wait