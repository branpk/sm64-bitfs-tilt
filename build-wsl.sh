#!/usr/bin/env bash

gcc -std=c99 -fwrapv -fno-strict-aliasing -lm -O3 source/*.c -o bitfs-tilt
