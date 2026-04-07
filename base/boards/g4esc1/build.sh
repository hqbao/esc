#!/bin/bash
# Build script for B-G431B-ESC1
# Usage: ./build.sh [clean]

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

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}B-G431B-ESC1 — Build${NC}"
echo -e "${YELLOW}========================================${NC}"

# Clean if requested
if [[ "$1" == "clean" ]]; then
    echo -e "${YELLOW}Cleaning build artifacts...${NC}"
    cd "$PROJECT_DIR"
    make clean
    echo -e "${GREEN}✓ Clean complete${NC}"
    exit 0
fi

# Build
echo -e "${YELLOW}Building...${NC}"
cd "$PROJECT_DIR"
make -j8 all

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
