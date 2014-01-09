#!/bin/sh
mkdir -p lib
mkdir -p obj
gcc -c -o obj/c_box2d.o src/Box2D/c_box2d.cpp -Isrc/Box2D
ar rcs lib/libcbox2d.a obj/c_box2d.o
