#!/bin/bash

# emcc src/main.cpp -s WASM=1 -s USE_SDL=2 -s USE_SDL_IMAGE=2 -s SDL2_IMAGE_FORMATS='["png"]' -s USE_SDL_TTF=2 --preload-file resources -o robotarm.js
# emcc src/main.cpp -s WASM=1 -s USE_SDL=2 -O3 -o robotarm.js
emcc src/*.cpp -s WASM=1 -s USE_SDL=2 -O3 -o robotarm.js
# g++ src/*.cpp -std=c++17 -I/usr/include/SDL2/ -I. -Iinclude/ -lSDL2main -lSDL2 -lSDL2_ttf -o bin/robotarm
g++ src/*.cpp -std=c++17 -I. -Iinclude/ -lSDL2main -lSDL2 -lSDL2_ttf -O2 -o bin/robotarm
