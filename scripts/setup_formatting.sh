#!/bin/bash
# Setup Script for Python Code Formatting Tools

echo "🚀 Setting up Python code formatting for Hydrus Software Stack..."

# Check if we need to install python3-venv
if ! python3 -c "import venv" 2>/dev/null; then
    echo "📦 Installing python3-venv package..."
    sudo apt update && sudo apt install -y python3-venv python3-pip
fi

# Create virtual environment if it doesn't exist
if [ ! -d ".venv" ]; then
    echo "🔧 Creating Python virtual environment..."
    python3 -m venv .venv
fi

# Activate virtual environment and install dependencies
echo "📦 Installing formatting dependencies in virtual environment..."
source .venv/bin/activate
pip install --upgrade pip
pip install black isort flake8 pre-commit mypy

# Setup pre-commit hooks
echo "🔗 Setting up pre-commit hooks..."
pre-commit install

echo "🎯 Running initial format on existing code..."
# Run formatting with virtual environment
source .venv/bin/activate && ./format.sh

echo "✅ Setup complete!"
echo ""
echo "🎉 Your Python formatting tools are now configured:"
echo "   • Black: Automatic code formatting"
echo "   • isort: Import statement organization"
echo "   • flake8: Code linting and style checking"
echo "   • pre-commit: Automatic formatting before commits"
echo ""
echo "📝 Usage:"
echo "   • Run './format.sh' to format all Python files"
echo "   • Formatting will run automatically on git commits"
echo "   • Use 'source .venv/bin/activate && pre-commit run --all-files' to check all files"
echo ""
echo "💡 Note: The tools are installed in a virtual environment (.venv)"
