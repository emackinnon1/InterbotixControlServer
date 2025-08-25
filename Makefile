.PHONY: start_prod stop_prod
.PHONY: start_runner
.PHONY: deploy

deploy:
	@echo "Deploying app: stopping existing services, syncing dependencies, and restarting..."
	@make cleanup
	@uv sync
	@echo "Starting services..."
	@caddy start
	@echo "Starting FastAPI in detached shell..."
	@nohup uv run uvicorn main:app --host 0.0.0.0 --port 8000 --pid-file /tmp/uvicorn.pid > /tmp/uvicorn.log 2>&1 &
# 	@bash -c 'cd $(PWD) && exec setsid sh -c "exec nohup uv run uvicorn main:app --host 0.0.0.0 --port 8000 & > /tmp/uvicorn.log 2>&1 &" < /dev/null > /dev/null 2>&1 & echo $$! > /tmp/uvicorn_parent.pid'
	@sleep 3
	@echo "Services started successfully"

start_prod:
	@echo "Starting FastAPI and Caddy..."
	@trap 'echo "Stopping services..."; make stop_prod 2>/dev/null || true' EXIT INT TERM; \
	caddy start && \
	nohup uv run uvicorn main:app --host 0.0.0.0 --port 8000 --pid-file /tmp/uvicorn.pid & \
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
	@./scripts/cleanup.sh

start_runner:
	@echo "Starting GH runner..."
	@trap 'kill %1 %2 2>/dev/null || true' EXIT; \
	./actions-runner/run.sh & \
	wait