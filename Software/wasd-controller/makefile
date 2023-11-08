
SFMLLIBS=-lsfml-graphics -lsfml-window -lsfml-system -lsfml-network
COMPFLAGS=-Iinclude -Wall -Wextra -Werror
objects=$(patsubst src/%.cpp,objects/%.o,$(wildcard src/*.cpp))

objects:
	mkdir -p objects

objects/%.o: src/%.cpp objects
	g++ -c $< $(COMPFLAGS) -o $@ 

objects/wasd-controller: $(objects)
	g++ $^ $(SFMLLIBS) -o $@

run: objects/wasd-controller
	./objects/wasd-controller

clean:
	rm -rf objects
