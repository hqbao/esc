#!/bin/bash
# Build, flash, and optionally debug B-G431B-ESC1
# Usage:
#   ./build-flash.sh              # Build and flash
#   ./build-flash.sh --verify     # Build, flash, and verify
#   ./build-flash.sh --debug      # Build, flash, and debug with GDB

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
TARGET="g4esc1"
BIN_FILE="$BUILD_DIR/$TARGET.bin"
ELF_FILE="$BUILD_DIR/$TARGET.elf"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

VERIFY=false
DEBUG=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --verify) VERIFY=true; shift ;;
        --debug) DEBUG=true; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}B-G431B-ESC1 — Build & Flash${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# Step 1: Build
echo -e "${YELLOW}[1/2] Building...${NC}"
"$PROJECT_DIR/build.sh"
echo ""

# Step 2: Flash
echo -e "${YELLOW}[2/2] Flashing to device...${NC}"

if [[ ! -f "$BIN_FILE" ]]; then
    echo -e "${RED}✗ Binary not found: $BIN_FILE${NC}"
    exit 1
fi

if ! st-info --probe &> /dev/null; then
    echo -e "${RED}✗ ST-Link debugger not found${NC}"
    exit 1
fi

echo -e "${YELLOW}Flashing to 0x8000000...${NC}"
st-flash write "$BIN_FILE" 0x8000000

if [[ $? -eq 0 ]]; then
    echo -e "${GREEN}✓ Flash successful${NC}"

    if [[ "$VERIFY" == true ]]; then
        echo -e "${YELLOW}Verifying flash...${NC}"
        st-flash verify "$BIN_FILE" 0x8000000
        echo -e "${GREEN}✓ Verification successful${NC}"
    fi
else
    echo -e "${RED}✗ Flash failed${NC}"
    exit 1
fi

# Step 3: Debug (if requested)
if [[ "$DEBUG" == true ]]; then
    echo ""
    echo -e "${YELLOW}[3/3] Starting debugger...${NC}"

    if ! command -v openocd &> /dev/null; then
        echo -e "${RED}✗ OpenOCD not found. Install with: brew install openocd${NC}"
        exit 1
    fi

    openocd -f interface/stlink.cfg -f target/stm32g4x.cfg > /tmp/openocd_esc.log 2>&1 &
    OPENOCD_PID=$!
    sleep 2

    if ! kill -0 $OPENOCD_PID 2>/dev/null; then
        echo -e "${RED}✗ OpenOCD failed to start. Check /tmp/openocd_esc.log${NC}"
        exit 1
    fi

    echo -e "${GREEN}✓ OpenOCD running (PID $OPENOCD_PID)${NC}"
    echo -e "${YELLOW}Starting GDB...${NC}"
    echo ""
    echo -e "  GDB tips:"
    echo -e "    (gdb) monitor reset halt   — reset MCU"
    echo -e "    (gdb) continue              — run"
    echo -e "    (gdb) break main            — set breakpoint"
    echo -e "    (gdb) quit                  — exit"
    echo ""

    arm-none-eabi-gdb "$ELF_FILE" \
        -ex "target remote :3333" \
        -ex "monitor reset halt" \
        -ex "break main" \
        -ex "continue"

    kill $OPENOCD_PID 2>/dev/null || true
fi
