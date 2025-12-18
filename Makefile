# Makefile for PlcDataplaneEsp8266 Arduino Library
# Build system for ESP8266 library with syntax checking and example compilation

# Configuration
LIBRARY_NAME = PlcDataplaneEsp8266
ARDUINO_CLI = arduino-cli
FQBN = esp8266:esp8266:generic
SKETCH_DIR = examples

# Source files
SRC_DIR = src
SOURCES = $(wildcard $(SRC_DIR)/*.c $(SRC_DIR)/*.cpp)
HEADERS = $(wildcard $(SRC_DIR)/*.h)

# Examples
EXAMPLES = $(wildcard $(SKETCH_DIR)/*)

# Colors for output
RED = \033[0;31m
GREEN = \033[0;32m
YELLOW = \033[0;33m
NC = \033[0m # No Color

.PHONY: all check install-deps compile-examples clean help

all: check

help:
	@echo "$(GREEN)PlcDataplaneEsp8266 Library Build System$(NC)"
	@echo ""
	@echo "Available targets:"
	@echo "  $(YELLOW)make all$(NC)              - Run syntax check (default)"
	@echo "  $(YELLOW)make check$(NC)            - Verify library structure and syntax"
	@echo "  $(YELLOW)make compile-examples$(NC) - Compile all example sketches"
	@echo "  $(YELLOW)make install-deps$(NC)     - Install ESP8266 core via arduino-cli"
	@echo "  $(YELLOW)make clean$(NC)            - Clean build artifacts"
	@echo "  $(YELLOW)make help$(NC)             - Show this help message"

check:
	@echo "$(GREEN)Checking library structure...$(NC)"
	@test -f library.properties || (echo "$(RED)Error: library.properties not found$(NC)" && exit 1)
	@test -d $(SRC_DIR) || (echo "$(RED)Error: src directory not found$(NC)" && exit 1)
	@echo "$(GREEN)✓ Library structure OK$(NC)"
	@echo ""
	@echo "$(GREEN)Source files:$(NC)"
	@for file in $(SOURCES) $(HEADERS); do echo "  - $$file"; done
	@echo ""
	@echo "$(GREEN)✓ Library check passed$(NC)"

install-deps:
	@echo "$(GREEN)Installing ESP8266 core...$(NC)"
	@if ! command -v $(ARDUINO_CLI) >/dev/null 2>&1; then \
		echo "$(RED)Error: arduino-cli not found. Please install it first.$(NC)"; \
		echo "Visit: https://arduino.github.io/arduino-cli/"; \
		exit 1; \
	fi
	$(ARDUINO_CLI) core update-index
	$(ARDUINO_CLI) core install esp8266:esp8266
	@echo "$(GREEN)✓ Dependencies installed$(NC)"

compile-examples: check
	@echo "$(GREEN)Compiling example sketches...$(NC)"
	@if ! command -v $(ARDUINO_CLI) >/dev/null 2>&1; then \
		echo "$(RED)Error: arduino-cli not found$(NC)"; \
		echo "Run 'make install-deps' first or install arduino-cli manually"; \
		exit 1; \
	fi
	@failed=0; \
	for example in $(EXAMPLES); do \
		if [ -d "$$example" ]; then \
			sketch=$$(basename $$example); \
			echo "$(YELLOW)Compiling $$sketch...$(NC)"; \
			if $(ARDUINO_CLI) compile --fqbn $(FQBN) "$$example" 2>&1 | grep -v "^$$"; then \
				echo "$(GREEN)✓ $$sketch compiled successfully$(NC)"; \
			else \
				echo "$(RED)✗ $$sketch compilation failed$(NC)"; \
				failed=$$((failed + 1)); \
			fi; \
			echo ""; \
		fi; \
	done; \
	if [ $$failed -eq 0 ]; then \
		echo "$(GREEN)✓ All examples compiled successfully$(NC)"; \
	else \
		echo "$(RED)✗ $$failed example(s) failed to compile$(NC)"; \
		exit 1; \
	fi

clean:
	@echo "$(GREEN)Cleaning build artifacts...$(NC)"
	@find $(SKETCH_DIR) -type d -name "build" -exec rm -rf {} + 2>/dev/null || true
	@rm -rf build/ 2>/dev/null || true
	@echo "$(GREEN)✓ Clean complete$(NC)"
