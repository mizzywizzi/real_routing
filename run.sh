#!/bin/bash

# Always work from the script's own directory so JSON files land in the right place
cd "$(dirname "$0")"

echo "Compiling storm2.c..."
gcc storm.c -o storm.out -lSDL2 -lSDL2_ttf -lm

if [ $? -ne 0 ]; then
    echo "Compilation failed."
    exit 1
fi

echo "Compilation successful."

# Runs the app immediately
RUNNER='./storm.out'

# Try terminal emulators in order of preference
if command -v qterminal &>/dev/null; then
    qterminal -e bash -c "$RUNNER"
elif command -v x-terminal-emulator &>/dev/null; then
    x-terminal-emulator -e bash -c "$RUNNER"
elif command -v xterm &>/dev/null; then
    xterm -hold -e bash -c "$RUNNER"
elif command -v gnome-terminal &>/dev/null; then
    gnome-terminal -- bash -c "$RUNNER"
elif command -v konsole &>/dev/null; then
    konsole -e bash -c "$RUNNER"
else
    # No external terminal found — run inline (already in a terminal)
    echo "No terminal emulator found. Running inline:"
    ./storm.out
fi
