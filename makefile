CC=g++
CFLAGS=-std=c++17 -Wall
TARGET=main

$(TARGET): main.cpp missile.o
	g++ -std=c++17 -o main main.cpp missile.o

missile.o: missile.hpp
	g++ -std=c++17 -c -o missile.o missile.hpp
