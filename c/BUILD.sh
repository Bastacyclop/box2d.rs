#!/bin/sh
mkdir -p lib
mkdir -p obj
mkdir -p bin
gcc -c -o obj/c_box2d.o src/Box2D/c_box2d.cpp -Isrc
ar rcs lib/libcbox2d.a obj/c_box2d.o
gcc -o bin/HelloWorld src/HelloWorld.c -Isrc -Llib -lcbox2d -lBox2D
