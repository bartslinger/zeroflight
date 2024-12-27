#!/bin/bash

# Arguments passed by cargo
BINARY_PATH=$1

# Raspberry Pi details
RPI_HOST="bali250"
RPI_USER="pi"
REMOTE_DIR="/home/pi/remote-run"

# Upload the binary to the Raspberry Pi
scp "${BINARY_PATH}" "${RPI_USER}@${RPI_HOST}:${REMOTE_DIR}/firmware.bin"

# Run probe-rs on the Raspberry Pi
ssh -t "${RPI_USER}@${RPI_HOST}" "~/.cargo/bin/probe-rs run --chip STM32F405RG --log-format '{L} {s}' ${REMOTE_DIR}/firmware.bin"

