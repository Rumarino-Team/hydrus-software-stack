#!/bin/bash
# Format script using the dedicated virtual environment
set -e

echo "🔧 Running Python formatters..."

# Check if virtual environment exists
if [ ! -d ".venv-formatters" ]; then
    echo "❌ Virtual environment not found. Run './setup_venv_formatters.sh' first"
    exit 1
fi

# Activate virtual environment
source .venv-formatters/bin/activate

# Run formatters
echo "🎨 Formatting with Black..."
black . --line-length 88 --target-version py38

echo "📝 Sorting imports with isort..."
isort . --profile black

echo "🔍 Running flake8 linting..."
flake8 . --statistics

echo "✅ Code formatting complete!"
