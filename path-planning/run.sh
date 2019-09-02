#!/bin/bash

clear

cd `dirname $0`
rm -rf build

mkdir -p build
cd build
cmake ..
make $*

./path_planning
