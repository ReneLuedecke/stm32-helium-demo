# Xi 640 ETH - Quick Build Commands

.PHONY: help init build-native run-native build-hw flash clean docker-build docker-run

help:
	@echo "╔════════════════════════════════════════════════════════════╗"
	@echo "║   Xi 640 ETH - Build Commands                             ║"
	@echo "╚════════════════════════════════════════════════════════════╝"
	@echo ""
	@echo "Native PC Simulation (no hardware):"
	@echo "  make init          - Initialize west workspace"
	@echo "  make build-native  - Build for PC simulation"
	@echo "  make run-native    - Run native simulation"
	@echo "  make test          - Build and run native"
	@echo ""
	@echo "Hardware (STM32N6570-DK):"
	@echo "  make build-hw      - Build for STM32N6570-DK"
	@echo "  make flash         - Flash to hardware"
	@echo "  make monitor       - Open serial monitor"
	@echo ""
	@echo "Docker:"
	@echo "  make docker-build  - Build Docker container"
	@echo "  make docker-run    - Run Docker container"
	@echo ""
	@echo "Maintenance:"
	@echo "  make clean         - Clean build artifacts"
	@echo "  make pristine      - Deep clean (removes .west)"
	@echo ""

init:
	@echo "Initializing west workspace..."
	west init -l .
	west update

build-native:
	@echo "Building for native_sim..."
	west build -b native_sim -p auto -- -DCONF_FILE=prj_native.conf

run-native: build-native
	@echo "Running native simulation..."
	@echo "Press Ctrl+C to exit"
	@echo "════════════════════════════════════════════════════════════"
	./build/zephyr/zephyr.exe

test: build-native
	@echo "Running tests..."
	./build/zephyr/zephyr.exe --help || ./build/zephyr/zephyr.exe

build-hw:
	@echo "Building for STM32N6570-DK..."
	west build -b stm32n6570_dk -p auto

flash: build-hw
	@echo "Flashing to hardware..."
	west flash

monitor:
	@echo "Opening serial monitor (115200 baud)..."
	@echo "Press Ctrl+A then K to exit"
	screen /dev/ttyACM0 115200

docker-build:
	@echo "Building Docker container..."
	docker build -t xi640-zephyr .

docker-run: docker-build
	@echo "Starting Docker container..."
	docker run -it --rm -v $$(pwd):/workspace xi640-zephyr

clean:
	@echo "Cleaning build artifacts..."
	rm -rf build build-*

pristine: clean
	@echo "Deep clean (removing .west)..."
	rm -rf .west zephyr modules tools bootloader

# Quick patterns for testing
pattern-gradient: build-native
	@echo "Testing gradient pattern..."
	./build/zephyr/zephyr.exe

pattern-hotspot:
	@west build -b native_sim -- -DCONFIG_THERMAL_PATTERN_HOTSPOT=y
	./build/zephyr/zephyr.exe

pattern-checker:
	@west build -b native_sim -- -DCONFIG_THERMAL_PATTERN_CHECKERBOARD=y
	./build/zephyr/zephyr.exe
