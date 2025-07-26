#!/bin/bash

# Hydrus Software Stack Doctor
# Checks and installs required dependencies for the Hydrus project
# Dependencies: Docker, Python3, and Git

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Emojis for better UX
CHECK="✅"
CROSS="❌"
WARNING="⚠️"
INFO="ℹ️"
ROCKET="🚀"
WRENCH="🔧"
DOWNLOAD="📥"

print_header() {
    echo ""
    echo -e "${BLUE}=================================${NC}"
    echo -e "${BLUE}🏥 Hydrus Software Stack Doctor${NC}"
    echo -e "${BLUE}=================================${NC}"
    echo ""
    echo -e "${CYAN}Checking system dependencies...${NC}"
    echo ""
}

print_section() {
    echo -e "\n${PURPLE}--- $1 ---${NC}"
}

print_success() {
    echo -e "${GREEN}${CHECK} $1${NC}"
}

print_error() {
    echo -e "${RED}${CROSS} $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}${WARNING} $1${NC}"
}

print_info() {
    echo -e "${CYAN}${INFO} $1${NC}"
}

ask_install() {
    local package=$1
    local description=$2
    echo ""
    echo -e "${YELLOW}${DOWNLOAD} $package is required but not installed.${NC}"
    echo -e "${CYAN}Description: $description${NC}"
    echo ""
    read -p "Would you like to install $package? (y/N): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        return 0
    else
        return 1
    fi
}

detect_os() {
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        if command -v apt-get &> /dev/null; then
            echo "ubuntu"
        elif command -v yum &> /dev/null; then
            echo "centos"
        elif command -v pacman &> /dev/null; then
            echo "arch"
        else
            echo "linux"
        fi
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        echo "macos"
    elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "win32" ]]; then
        echo "windows"
    else
        echo "unknown"
    fi
}

install_python3() {
    local os=$(detect_os)
    print_info "Installing Python3 for $os..."

    case $os in
        ubuntu)
            sudo apt update
            sudo apt install -y python3 python3-pip python3-venv
            ;;
        centos)
            sudo yum install -y python3 python3-pip
            ;;
        arch)
            sudo pacman -S python python-pip
            ;;
        macos)
            if command -v brew &> /dev/null; then
                brew install python3
            else
                print_error "Homebrew not found. Please install Python3 manually from python.org"
                return 1
            fi
            ;;
        *)
            print_error "Unsupported OS for automatic Python3 installation"
            print_info "Please install Python3 manually from https://python.org"
            return 1
            ;;
    esac
}

install_docker() {
    local os=$(detect_os)
    print_info "Installing Docker for $os..."

    case $os in
        ubuntu)
            # Official Docker installation for Ubuntu
            sudo apt update
            sudo apt install -y ca-certificates curl gnupg lsb-release

            # Add Docker's official GPG key
            sudo mkdir -p /etc/apt/keyrings
            curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

            # Set up repository
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

            # Install Docker Engine
            sudo apt update
            sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

            # Add user to docker group
            sudo usermod -aG docker $USER

            print_success "Docker installed successfully!"
            print_warning "Please log out and log back in for Docker group changes to take effect"
            ;;
        centos)
            sudo yum install -y yum-utils
            sudo yum-config-manager --add-repo https://download.docker.com/linux/centos/docker-ce.repo
            sudo yum install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
            sudo systemctl start docker
            sudo systemctl enable docker
            sudo usermod -aG docker $USER
            ;;
        arch)
            sudo pacman -S docker docker-compose
            sudo systemctl start docker
            sudo systemctl enable docker
            sudo usermod -aG docker $USER
            ;;
        macos)
            print_info "Please download Docker Desktop from https://docker.com/products/docker-desktop"
            print_info "Docker Desktop includes Docker Engine and Docker Compose"
            open "https://docker.com/products/docker-desktop" 2>/dev/null || true
            ;;
        *)
            print_error "Unsupported OS for automatic Docker installation"
            print_info "Please install Docker manually from https://docker.com"
            return 1
            ;;
    esac
}

install_git() {
    local os=$(detect_os)
    print_info "Installing Git for $os..."

    case $os in
        ubuntu)
            sudo apt update
            sudo apt install -y git
            ;;
        centos)
            sudo yum install -y git
            ;;
        arch)
            sudo pacman -S git
            ;;
        macos)
            if command -v brew &> /dev/null; then
                brew install git
            else
                print_info "Git should be available via Xcode Command Line Tools"
                xcode-select --install 2>/dev/null || true
            fi
            ;;
        *)
            print_error "Unsupported OS for automatic Git installation"
            print_info "Please install Git manually"
            return 1
            ;;
    esac
}

check_python3() {
    print_section "Python3"

    if command -v python3 &> /dev/null; then
        local version=$(python3 --version 2>&1 | cut -d' ' -f2)
        print_success "Python3 is installed (version $version)"

        # Check if pip is available
        if command -v pip3 &> /dev/null; then
            print_success "pip3 is available"
        else
            print_warning "pip3 is not available"
            if ask_install "pip3" "Python package manager"; then
                local os=$(detect_os)
                case $os in
                    ubuntu) sudo apt install -y python3-pip ;;
                    centos) sudo yum install -y python3-pip ;;
                    arch) sudo pacman -S python-pip ;;
                    macos) python3 -m ensurepip --upgrade ;;
                esac
            fi
        fi
        return 0
    else
        print_error "Python3 is not installed"
        if ask_install "Python3" "Required for running Hocker and various scripts"; then
            install_python3
            return $?
        else
            print_warning "Python3 installation skipped"
            return 1
        fi
    fi
}

check_docker() {
    print_section "Docker"

    if command -v docker &> /dev/null; then
        local version=$(docker --version 2>&1 | cut -d' ' -f3 | tr -d ',')
        print_success "Docker is installed (version $version)"

        # Check if Docker daemon is running
        if docker info &> /dev/null; then
            print_success "Docker daemon is running"
        else
            print_warning "Docker daemon is not running"
            print_info "You may need to start Docker or add yourself to the docker group"
        fi

        # Check Docker Compose
        if docker compose version &> /dev/null; then
            local compose_version=$(docker compose version --short 2>&1)
            print_success "Docker Compose is available (version $compose_version)"
        else
            print_warning "Docker Compose is not available"
            print_info "Modern Docker installations include Compose as a plugin"
        fi

        return 0
    else
        print_error "Docker is not installed"
        if ask_install "Docker" "Container platform for running Hydrus in isolated environments"; then
            install_docker
            return $?
        else
            print_warning "Docker installation skipped"
            return 1
        fi
    fi
}

check_git() {
    print_section "Git"

    if command -v git &> /dev/null; then
        local version=$(git --version 2>&1 | cut -d' ' -f3)
        print_success "Git is installed (version $version)"
        return 0
    else
        print_error "Git is not installed"
        if ask_install "Git" "Version control system for cloning and managing the repository"; then
            install_git
            return $?
        else
            print_warning "Git installation skipped"
            return 1
        fi
    fi
}

check_system_requirements() {
    print_section "System Requirements"

    # Check available disk space
    local available_space=$(df . | tail -1 | awk '{print $4}')
    local available_gb=$((available_space / 1024 / 1024))

    if [ $available_gb -gt 5 ]; then
        print_success "Sufficient disk space available (~${available_gb}GB)"
    else
        print_warning "Limited disk space available (~${available_gb}GB)"
        print_info "Docker images require ~3-5GB of space"
    fi

    # Check available memory
    if command -v free &> /dev/null; then
        local available_ram=$(free -g | awk '/^Mem:/{print $2}')
        if [ $available_ram -gt 3 ]; then
            print_success "Sufficient RAM available (~${available_ram}GB)"
        else
            print_warning "Limited RAM available (~${available_ram}GB)"
            print_info "Hydrus containers work best with 4GB+ RAM"
        fi
    fi
}

test_installation() {
    print_section "Testing Installation"

    print_info "Testing Hocker deployment tool..."

    if [ -f "docker/hydrus-docker/hocker.py" ]; then
        if chmod +x docker/hydrus-docker/hocker.py; then
            print_success "Hocker is executable"

            # Test help command
            if ./docker/hydrus-docker/hocker.py --help &> /dev/null; then
                print_success "Hocker help command works"
            else
                print_warning "Hocker help command failed - this may be due to missing dependencies"
            fi
        else
            print_error "Failed to make Hocker executable"
        fi
    else
        print_error "Hocker not found at docker/hydrus-docker/hocker.py"
        print_info "Make sure you're in the hydrus-software-stack directory"
    fi
}

print_final_instructions() {
    echo ""
    echo -e "${GREEN}=================================${NC}"
    echo -e "${GREEN}${ROCKET} Setup Complete!${NC}"
    echo -e "${GREEN}=================================${NC}"
    echo ""
    echo -e "${CYAN}Environment Setup (Recommended):${NC}"
    echo ""
    echo -e "${YELLOW}# Set up environment for global access to hocker and hydrus-cli${NC}"
    echo -e "${WHITE}source ./envsetup${NC}"
    echo ""
    echo -e "${CYAN}Quick Start Commands:${NC}"
    echo ""
    echo -e "${YELLOW}# Make hocker executable (if not already)${NC}"
    echo -e "${WHITE}chmod +x docker/hydrus-docker/hocker.py${NC}"
    echo ""
    echo -e "${YELLOW}# Enter container shell (workspace root)${NC}"
    echo -e "${WHITE}./docker/hydrus-docker/hocker.py --exec bash --it${NC}"
    echo ""
    echo -e "${YELLOW}# Enter container shell (development volume)${NC}"
    echo -e "${WHITE}./docker/hydrus-docker/hocker.py --exec bash --it --dev${NC}"
    echo ""
    echo -e "${YELLOW}# Start development environment (detached)${NC}"
    echo -e "${WHITE}./docker/hydrus-docker/hocker.py --detach${NC}"
    echo ""
    echo -e "${YELLOW}# View all options${NC}"
    echo -e "${WHITE}./docker/hydrus-docker/hocker.py --help${NC}"
    echo ""
    echo -e "${CYAN}After running 'source ./envsetup':${NC}"
    echo -e "${WHITE}hocker --exec bash --it --dev    # Enter dev container${NC}"
    echo -e "${WHITE}hydrus-cli --help                # Show CLI help${NC}"
    echo -e "${WHITE}hydrus-root                      # Navigate to project root${NC}"
    echo ""
    echo -e "${CYAN}Documentation:${NC}"
    echo -e "${WHITE}• Main README: README.md${NC}"
    echo -e "${WHITE}• Docker Guide: docker/README.md${NC}"
    echo -e "${WHITE}• Autonomy System: autonomy/README.md${NC}"
    echo ""
    if groups $USER | grep -q docker; then
        print_success "You're in the docker group - ready to go!"
    else
        print_warning "You may need to log out and back in for Docker group permissions"
    fi
    echo ""
}

main() {
    print_header

    local python_ok=false
    local docker_ok=false
    local git_ok=false

    # Check all dependencies
    if check_python3; then python_ok=true; fi
    if check_docker; then docker_ok=true; fi
    if check_git; then git_ok=true; fi

    check_system_requirements

    # Test installation if basic deps are available
    if $python_ok && $docker_ok; then
        test_installation
    fi

    # Final summary
    echo ""
    print_section "Summary"

    if $python_ok && $docker_ok && $git_ok; then
        print_success "All dependencies are installed!"
        print_final_instructions
    else
        print_warning "Some dependencies are missing:"
        if ! $python_ok; then print_error "Python3 is required"; fi
        if ! $docker_ok; then print_error "Docker is required"; fi
        if ! $git_ok; then print_error "Git is required"; fi
        echo ""
        print_info "Please install missing dependencies and run this script again"
        print_info "Or install them manually using your system's package manager"
    fi
}

# Run main function
main "$@"
