#!/bin/bash
# Quick activation script for formatter virtual environment
echo "🔌 Activating formatter virtual environment..."
source .venv-formatters/bin/activate
echo "✅ Formatter environment activated!"
echo "💡 Available commands:"
echo "   - black . --line-length 88 --target-version py38"
echo "   - isort . --profile black"
echo "   - flake8 ."
echo "   - pre-commit run --all-files"
