#!/bin/bash
mkdir -p build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_KIT=TRUE
make
