#!/bin/bash

cd `dirname $0`
rm -rf build

mkdir -p build
cd build
cmake ..
make $*

./pid