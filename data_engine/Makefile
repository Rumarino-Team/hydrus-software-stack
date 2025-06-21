# Makefile for SAM2 Data Engine

.PHONY: install setup run clean test help

# Default Python interpreter
PYTHON := python3

# Virtual environment name
VENV := venv

help: ## Show this help message
	@echo "SAM2 Data Engine - Available commands:"
	@echo
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'

install: ## Install requirements
	$(PYTHON) -m pip install -r requirements.txt

setup: ## Run full setup (install + download models)
	$(PYTHON) setup.py

run: ## Run the data engine application
	$(PYTHON) run.py

test: ## Run example/test script
	$(PYTHON) example.py

clean: ## Clean cache and temporary files
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete
	find . -type f -name "*.pyo" -delete
	rm -rf test_output/
	rm -rf test_config/

venv: ## Create virtual environment
	$(PYTHON) -m venv $(VENV)
	@echo "Activate with: source $(VENV)/bin/activate"

venv-install: venv ## Create venv and install requirements
	$(VENV)/bin/pip install -r requirements.txt

venv-run: ## Run application in virtual environment
	$(VENV)/bin/python run.py

dev-install: ## Install development dependencies
	$(PYTHON) -m pip install -r requirements.txt
	$(PYTHON) -m pip install black flake8 mypy

format: ## Format code with black
	black *.py

lint: ## Lint code with flake8
	flake8 *.py

type-check: ## Type check with mypy
	mypy *.py --ignore-missing-imports

check: format lint type-check ## Run all code quality checks

# Docker commands (if needed)
docker-build: ## Build Docker image
	docker build -t sam2-data-engine .

docker-run: ## Run in Docker container
	docker run -it --rm -v $(PWD):/app sam2-data-engine

# Documentation
docs: ## Generate documentation
	@echo "Documentation available in README.md"
	@echo "For detailed usage, see: python example.py"

# System info
info: ## Show system information
	@echo "Python version:"
	@$(PYTHON) --version
	@echo
	@echo "Pip packages:"
	@$(PYTHON) -m pip list | grep -E "(torch|opencv|PySide6|ultralytics|numpy|Pillow)"
	@echo
	@echo "CUDA availability:"
	@$(PYTHON) -c "import torch; print('CUDA available:', torch.cuda.is_available())" 2>/dev/null || echo "PyTorch not installed"

# Quick start
quickstart: ## Quick start guide
	@echo "SAM2 Data Engine - Quick Start"
	@echo "=============================="
	@echo
	@echo "1. Install dependencies:"
	@echo "   make install"
	@echo
	@echo "2. Run setup (download models):"
	@echo "   make setup"
	@echo
	@echo "3. Launch application:"
	@echo "   make run"
	@echo
	@echo "4. For testing without GUI:"
	@echo "   make test"
	@echo
	@echo "For more commands: make help"
