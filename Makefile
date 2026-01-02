# Makefile for Ornithopter Flight Control System
# Provides convenient targets for building, testing, and verification

.PHONY: all build release debug upload clean test docs verify format help

# Default target
all: build

# Build release version
build: release

release:
	@echo "Building release version..."
	pio run -e pico32_release

# Build debug version
debug:
	@echo "Building debug version..."
	pio run -e pico32_debug

# Upload firmware to device
upload:
	@echo "Uploading firmware..."
	pio run -e pico32_release --target upload

# Upload debug version
upload-debug:
	@echo "Uploading debug firmware..."
	pio run -e pico32_debug --target upload

# Static code analysis
check:
	@echo "Running static analysis..."
	pio check -e check --skip-packages

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	pio run --target clean
	rm -rf .pio/build
	rm -rf docs/html docs/latex

# Monitor serial output
monitor:
	@echo "Starting serial monitor..."
	pio device monitor

# Run formal verification
verify:
	@echo "Running formal verification..."
	@if command -v python3 >/dev/null 2>&1; then \
		python3 docs/formal_verification/verify_control.py; \
	else \
		echo "Error: Python 3 not found"; \
		exit 1; \
	fi

# Generate documentation
docs:
	@echo "Generating documentation..."
	@if command -v doxygen >/dev/null 2>&1; then \
		doxygen Doxyfile 2>/dev/null || echo "Note: Some doxygen warnings (normal)"; \
	else \
		echo "Error: Doxygen not found. Install with: apt-get install doxygen"; \
		exit 1; \
	fi

# Format code (requires clang-format)
format:
	@echo "Formatting code..."
	@if command -v clang-format >/dev/null 2>&1; then \
		find src include -name "*.cpp" -o -name "*.h" | xargs clang-format -i; \
		echo "Code formatted successfully"; \
	else \
		echo "Error: clang-format not found"; \
		exit 1; \
	fi

# Display binary size
size:
	@echo "Analyzing binary size..."
	pio run --target size -e pico32_release

# Memory analysis
memanalyze:
	@echo "Analyzing memory usage..."
	pio run --target memanalyze -e pico32_release

# List all available targets
help:
	@echo "Ornithopter Flight Control System - Build Targets"
	@echo "=================================================="
	@echo ""
	@echo "Build targets:"
	@echo "  make build       - Build release version (default)"
	@echo "  make release     - Build release version"
	@echo "  make debug       - Build debug version"
	@echo "  make upload      - Upload firmware to device"
	@echo "  make upload-debug- Upload debug firmware"
	@echo ""
	@echo "Analysis targets:"
	@echo "  make check       - Run static code analysis"
	@echo "  make verify      - Run formal verification (Z3)"
	@echo "  make size        - Show binary size information"
	@echo "  make memanalyze  - Analyze memory usage"
	@echo ""
	@echo "Maintenance targets:"
	@echo "  make clean       - Remove build artifacts"
	@echo "  make format      - Format code with clang-format"
	@echo "  make docs        - Generate Doxygen documentation"
	@echo ""
	@echo "Development targets:"
	@echo "  make monitor     - Open serial monitor"
	@echo "  make help        - Show this help message"
	@echo ""
