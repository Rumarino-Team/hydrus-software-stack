# Python Code Formatting Guide

This project uses automated Python code formatting to ensure consistent code style across the entire codebase.

## 🚀 Quick Start

### Option 1: Use the Virtual Environment (Recommended)
```bash
# One-time setup
./setup_venv_formatters.sh

# Format your code
./format_with_venv.sh

# Or activate and use manually
source activate_formatters.sh
black . --line-length 88 --target-version py38
isort . --profile black
flake8 .
```

### Option 2: Use Existing Environment
```bash
# If you already have the tools installed
./format.sh
```

## 🔧 Tools Used

- **Black** (24.10.0+): Automatic Python code formatter
- **isort** (6.0.1+): Import statement organizer
- **flake8** (7.2.0+): Code linter and style checker
- **pre-commit** (3.8.0+): Git hooks for automatic formatting

## 📁 Files in This Setup

| File | Purpose |
|------|---------|
| `setup_venv_formatters.sh` | Creates virtual environment with formatters |
| `activate_formatters.sh` | Quick activation script |
| `format_with_venv.sh` | Formats code using virtual environment |
| `format.sh` | Legacy formatting script |
| `pyproject.toml` | Tool configuration |
| `.flake8` | Flake8-specific configuration |
| `.pre-commit-config.yaml` | Git hooks configuration |

## 🔄 CI/CD Integration

The CI pipeline automatically checks code formatting on every pull request:

1. **Black formatting check** - Ensures consistent code style
2. **Import sorting check** - Validates import organization
3. **Flake8 linting** - Checks for code quality issues

If any check fails, the CI will block the PR with clear instructions to run the formatter.

## 🛠️ Configuration

### Black Settings
- Line length: 88 characters
- Target Python version: 3.8
- Profile: Compatible with isort

### Flake8 Settings
- Ignores common style issues that don't affect functionality
- Focuses on important code quality checks
- Uses same line length as Black

## 💡 Usage Tips

1. **Before committing**: Pre-commit hooks will automatically format your code
2. **If CI fails**: Run `./format_with_venv.sh` to fix all issues
3. **For specific files**: Use tools directly after activating the environment

## 🔍 Troubleshooting

### Virtual Environment Issues
```bash
# If the environment is corrupted, recreate it
rm -rf .venv-formatters
./setup_venv_formatters.sh
```

### CI Version Conflicts
The CI uses flexible version ranges to ensure compatibility with the latest available public versions of the formatting tools.

### Pre-commit Hook Issues
```bash
# Reinstall hooks if needed
source activate_formatters.sh
pre-commit install --overwrite
```

## 📊 Version Compatibility

This setup is designed to work with the latest publicly available versions:
- Black: `>=24.0.0,<25.0.0`
- isort: `>=5.12.0,<7.0.0`
- flake8: `>=6.0.0,<8.0.0`

The virtual environment approach ensures consistent versions across all developers and CI environments.
