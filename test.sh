#!/bin/sh
# Script to test components

# Go into the directory where this bash script is contained.
cd `dirname $0`

# Create binary output path
mkdir -p build/test

# Compile
g++ -std=c++17 \
    test/main.cpp \
    src/behavior.cpp \
    -L/usr/include \
    -I/usr/local/lib \
    -lstdc++ \
    -lgtest \
    -lgtest_main \
    -pthread \
    -o build/test/test_bin

# Run
#./build/test/test_bin