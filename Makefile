.PHONY: test test-verbose deploy clean install install-boot-service

test:
	PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --with pytest pytest tests -q

test-verbose:
	PYTHONPATH=. PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run --with pytest pytest tests -vv

deploy:
	bash scripts/deploy.sh

clean:
	bash scripts/cleanup.sh

install:
	bash xsarm_rpi4_install.sh

install-boot-service:
	bash scripts/install-boot-service.sh