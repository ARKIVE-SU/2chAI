#!/bin/sh
# Сборка: C++ -> WebAssembly голым clang (без Emscripten), затем wasm вшивается в index.html.
set -e
cd "$(dirname "$0")"
mkdir -p build
clang++ --target=wasm32 -std=c++17 -O3 -ffast-math -nostdlib -ffreestanding -fno-builtin \
  -fno-exceptions -fno-rtti -Wall -Wno-unused-function \
  -Wl,--no-entry -Wl,--initial-memory=50331648 -Wl,-z,stack-size=1048576 -Wl,--strip-all \
  -o build/table.wasm src/table.cpp
python3 inline.py web/index.html build/table.wasm index.html
ls -l build/table.wasm index.html
