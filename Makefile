.PHONY:test

LDFLAGS=-I include/ -I src/ -lGL -lglut -lGLU -L./Debug -std=c++20 
CXX=clang++

DEMOS=ballistic bigballistic blob bridge explosion fireworks flightsim fracture platform ragdoll sailboat

$(DEMOS):
	$(CXX) src/*.cpp src/demos/$@.cpp $(LDFLAGS) -o $@.o

test:
	$(CXX) test/test.cpp $(LDFLAGS)

clean:
	rm *.out
	rm *.o