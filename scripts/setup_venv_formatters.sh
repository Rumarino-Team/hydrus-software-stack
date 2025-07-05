#!/bin/bash
# Virtual Environment Setup Script for Python Formatters
# This script creates a virtual environment and installs the latest available formatting tools

set -e  # Exit on any error

echo "🚀 Setting up virtual environment with Python formatters..."

# Script configuration
VENV_NAME=".venv-formatters"
PYTHON_VERSION="python3"

# Check if Python is available
if ! command -v $PYTHON_VERSION &> /dev/null; then
    echo "❌ Error: $PYTHON_VERSION is not installed or not in PATH"
    exit 1
fi

# Remove existing virtual environment if it exists
if [ -d "$VENV_NAME" ]; then
    echo "🗑️  Removing existing virtual environment..."
    rm -rf "$VENV_NAME"
fi

# Create virtual environment
echo "🔧 Creating virtual environment: $VENV_NAME"
$PYTHON_VERSION -m venv "$VENV_NAME"

# Activate virtual environment
echo "🔌 Activating virtual environment..."
source "$VENV_NAME/bin/activate"

# Upgrade pip
echo "📦 Upgrading pip..."
python -m pip install --upgrade pip

# Install formatting tools with latest compatible versions
echo "🎨 Installing Python formatting tools..."
echo "   - Installing Black (Python code formatter)..."
pip install "black>=24.0.0,<25.0.0"

echo "   - Installing isort (Import sorter)..."
pip install "isort>=5.12.0,<7.0.0"

echo "   - Installing flake8 (Linter)..."
pip install "flake8>=6.0.0,<8.0.0"

echo "   - Installing pre-commit (Git hooks)..."
pip install "pre-commit>=3.0.0,<4.0.0"

echo "   - Installing mypy (Type checker)..."
pip install "mypy>=1.0.0,<2.0.0"

# Show installed versions
echo ""
echo "📋 Installed formatter versions:"
pip list | grep -E "(black|isort|flake8|pre-commit|mypy)" || echo "No formatters found"

# Create activation script
cat > activate_formatters.sh << 'EOF'
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
EOF

chmod +x activate_formatters.sh

# Create a simple format script that uses this venv
cat > format_with_venv.sh << 'EOF'
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
EOF

chmod +x format_with_venv.sh

echo ""
echo "✅ Virtual environment setup complete!"
echo ""
echo "📁 Created files:"
echo "   - $VENV_NAME/ (virtual environment)"
echo "   - activate_formatters.sh (quick activation)"
echo "   - format_with_venv.sh (formatting script)"
echo ""
echo "🚀 Usage:"
echo "   1. To activate: source activate_formatters.sh"
echo "   2. To format code: ./format_with_venv.sh"
echo "   3. To install pre-commit hooks: source activate_formatters.sh && pre-commit install"
echo ""
echo "💡 This environment uses the latest publicly available versions of formatting tools"
