.PHONY:test

LDFLAGS=-I include/ -lGL -lglut -lGLU -L./Debug -std=c++20
CXX=clang++

test:
	$(CXX) test/test.cpp $(LDFLAGS)

clean:
	rm *.out