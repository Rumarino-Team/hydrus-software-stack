#!/bin/bash
# Python Code Formatting Script for Hydrus Software Stack

echo "🔧 Running Python code formatters..."

# Check if we're in the right directory
if [ ! -f "requirements.txt" ]; then
    echo "❌ Error: Please run this script from the project root directory"
    exit 1
fi

# Activate virtual environment if it exists, otherwise try system packages
if [ -d ".venv" ]; then
    echo "🔧 Activating virtual environment..."
    source .venv/bin/activate
else
    echo "⚠️  Virtual environment not found, trying system packages..."
    echo "💡 Run './setup_formatting.sh' to set up the virtual environment"
fi

# Check if tools are available
if ! command -v black &> /dev/null; then
    echo "❌ Black not found. Please run './setup_formatting.sh' first"
    exit 1
fi

echo "🎨 Formatting Python code with Black..."
black . --line-length 88 --target-version py38

echo "📝 Sorting imports with isort..."
isort . --profile black

echo "🔍 Running flake8 linting..."
flake8 . --max-line-length=88 --extend-ignore=E203,W503

echo "✅ Code formatting complete!"
echo ""
echo "📋 Summary:"
echo "   - Black: Code formatted to 88 character lines"
echo "   - isort: Import statements sorted and organized"
echo "   - flake8: Linting checks completed"
echo ""
echo "💡 To set up automatic formatting on git commits, run:"
echo "   pre-commit install"
