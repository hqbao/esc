#!/bin/bash
# Build script for B-G431B-ESC1
# Usage: ./build.sh [clean|encoder|no_feedback|bemf]
#
# FOC mode selection:
#   ./build.sh              # default: encoder
#   ./build.sh encoder      # closed-loop with AS5048A
#   ./build.sh no_feedback  # open-loop only
#   ./build.sh bemf         # sensorless (placeholder)

set -e

# macOS: Prioritize Homebrew ARM toolchain
if [[ "$OSTYPE" == "darwin"* ]]; then
    CASK_GCC=$(ls -d /Applications/ArmGNUToolchain/*/arm-none-eabi/bin 2>/dev/null | head -n 1)
    if [[ -d "$CASK_GCC" ]]; then
        export PATH="$CASK_GCC:$PATH"
    fi
fi

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
TARGET="g4esc1"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Parse argument
FOC_MODE="encoder"
if [[ "$1" == "clean" ]]; then
    echo -e "${YELLOW}Cleaning build artifacts...${NC}"
    cd "$PROJECT_DIR"
    make clean
    echo -e "${GREEN}✓ Clean complete${NC}"
    exit 0
elif [[ "$1" == "no_feedback" || "$1" == "encoder" || "$1" == "bemf" ]]; then
    FOC_MODE="$1"
fi

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}B-G431B-ESC1 — Build (FOC_MODE=$FOC_MODE)${NC}"
echo -e "${YELLOW}========================================${NC}"

# Clean + rebuild to avoid stale object from different mode
echo -e "${YELLOW}Building ($FOC_MODE)...${NC}"
cd "$PROJECT_DIR"
make clean 2>/dev/null || true
make -j8 all FOC_MODE=$FOC_MODE

# Verify
ELF="$BUILD_DIR/$TARGET.elf"
BIN="$BUILD_DIR/$TARGET.bin"

if [[ ! -f "$ELF" ]]; then
    echo -e "${RED}✗ Build failed: $ELF not found${NC}"
    exit 1
fi

ELF_SIZE=$(ls -lh "$ELF" | awk '{print $5}')
BIN_SIZE=$(ls -lh "$BIN" | awk '{print $5}')
echo -e "${GREEN}✓ Build successful${NC}"
echo -e "  ELF: $ELF ($ELF_SIZE)"
echo -e "  BIN: $BIN ($BIN_SIZE)"

echo -e "${YELLOW}========================================${NC}"
echo -e "${GREEN}Ready to flash${NC}"
echo -e "${YELLOW}========================================${NC}"
