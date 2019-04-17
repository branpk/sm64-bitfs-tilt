#!/usr/bin/env bash

gcc \
  -std=c99 \
  -O3 \
  -Wall -Wextra \
  -Wno-missing-braces \
  -Wno-return-stack-address \
  -Wno-incompatible-pointer-types \
  -lglfw \
  -framework OpenGL \
  -fwrapv \
  -fno-strict-aliasing \
  source/*.c \
  -o bitfs-tilt
