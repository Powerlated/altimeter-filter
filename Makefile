build/libaltimeterfilter.a: build/AltimeterFilter.o
	ar rs build/libaltimeterfilter.a build/AltimeterFilter.o

build/AltimeterFilter.o: AltimeterFilter/AltimeterFilter.cpp
	gcc -Wall -g -c -O3 $< -o $@

build/:
	mkdir build
