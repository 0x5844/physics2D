#!/bin/bash

set -euo pipefail

# Configuration
BINARY_NAME="physics2d"
VERSION=$(git describe --tags --always --dirty 2>/dev/null || echo "dev")
BUILD_TIME=$(date -u '+%Y-%m-%d_%H:%M:%S')
GO_VERSION=$(go version | awk '{print $3}')

# Build flags for maximum performance
LDFLAGS="-s -w -X main.Version=${VERSION} -X main.BuildTime=${BUILD_TIME} -X main.GoVersion=${GO_VERSION}"
GCFLAGS="-B -C"
ASMFLAGS="-B"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check Go version
check_go_version() {
    local required_version="1.19"
    local current_version=$(go version | grep -oE '[0-9]+\.[0-9]+' | head -1)
    
    if ! printf '%s\n%s\n' "$required_version" "$current_version" | sort -V -C; then
        print_error "Go version $required_version or higher required. Current: $current_version"
        exit 1
    fi
}

# Clean previous builds
clean() {
    print_status "Cleaning previous builds..."
    rm -f ${BINARY_NAME}
    rm -rf build/
    go clean -cache
}

# Run tests
test() {
    print_status "Running tests..."
    go test -v -race -coverprofile=coverage.out ./...
    if [ $? -eq 0 ]; then
        print_success "All tests passed"
    else
        print_error "Tests failed"
        exit 1
    fi
}

# Run benchmarks
benchmark() {
    print_status "Running benchmarks..."
    go test -bench=. -benchmem -cpuprofile=cpu.prof -memprofile=mem.prof
}

# Build for current platform
build_native() {
    print_status "Building native binary..."
    CGO_ENABLED=0 go build \
        -ldflags="${LDFLAGS}" \
        -gcflags="${GCFLAGS}" \
        -asmflags="${ASMFLAGS}" \
        -trimpath \
        -o ${BINARY_NAME} \
        main.go
    
    print_success "Native build completed: ${BINARY_NAME}"
}

# Build for multiple platforms
build_cross() {
    print_status "Cross-compiling for multiple platforms..."
    
    mkdir -p build
    
    platforms=(
        "linux/amd64"
        "linux/arm64"
        "darwin/amd64"
        "darwin/arm64"
        "windows/amd64"
        "freebsd/amd64"
    )
    
    for platform in "${platforms[@]}"; do
        IFS='/' read -r os arch <<< "$platform"
        output="build/${BINARY_NAME}-${os}-${arch}"
        
        if [ "$os" = "windows" ]; then
            output="${output}.exe"
        fi
        
        print_status "Building for ${os}/${arch}..."
        
        CGO_ENABLED=0 GOOS=$os GOARCH=$arch go build \
            -ldflags="${LDFLAGS}" \
            -gcflags="${GCFLAGS}" \
            -asmflags="${ASMFLAGS}" \
            -trimpath \
            -o $output \
            main.go
        
        # Compress binary
        if command -v upx >/dev/null 2>&1; then
            upx --best --lzma $output 2>/dev/null || true
        fi
    done
    
    print_success "Cross-compilation completed"
}

# Show build info
info() {
    print_status "Build Information:"
    echo "  Version: $VERSION"
    echo "  Build Time: $BUILD_TIME"
    echo "  Go Version: $GO_VERSION"
    echo "  Binary Name: $BINARY_NAME"
}

# Optimize binary
optimize() {
    if [ -f "${BINARY_NAME}" ]; then
        print_status "Optimizing binary..."
        
        # Strip symbols if not already done
        if command -v strip >/dev/null 2>&1; then
            strip ${BINARY_NAME} 2>/dev/null || true
        fi
        
        # Compress with UPX if available
        if command -v upx >/dev/null 2>&1; then
            print_status "Compressing with UPX..."
            upx --best --lzma ${BINARY_NAME} 2>/dev/null || print_warning "UPX compression failed"
        fi
        
        print_success "Binary optimization completed"
    fi
}

# Performance profiling build
profile() {
    print_status "Building with profiling enabled..."
    go build \
        -ldflags="${LDFLAGS}" \
        -gcflags="-m=2" \
        -race \
        -o ${BINARY_NAME}-profile \
        main.go
    
    print_success "Profile build completed: ${BINARY_NAME}-profile"
}

# Release build
release() {
    clean
    test
    build_cross
    info
    
    print_success "Release build completed successfully"
}

# Development build
dev() {
    clean
    build_native
    print_success "Development build completed"
}

# Show usage
usage() {
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  dev         Build for development (default)"
    echo "  release     Full release build with tests and cross-compilation"
    echo "  native      Build for current platform only"
    echo "  cross       Cross-compile for multiple platforms"
    echo "  test        Run tests only"
    echo "  benchmark   Run benchmarks"
    echo "  clean       Clean build artifacts"
    echo "  optimize    Optimize existing binary"
    echo "  profile     Build with profiling enabled"
    echo "  info        Show build information"
    echo "  help        Show this help"
}

# Main execution
main() {
    check_go_version
    
    case "${1:-dev}" in
        "dev"|"development")
            dev
            ;;
        "release")
            release
            ;;
        "native")
            clean
            build_native
            optimize
            ;;
        "cross")
            clean
            build_cross
            ;;
        "test")
            test
            ;;
        "benchmark")
            benchmark
            ;;
        "clean")
            clean
            ;;
        "optimize")
            optimize
            ;;
        "profile")
            profile
            ;;
        "info")
            info
            ;;
        "help"|"-h"|"--help")
            usage
            ;;
        *)
            print_error "Unknown command: $1"
            usage
            exit 1
            ;;
    esac
}

main "$@"
