all:
	gcc -g quat.cpp -lSDL2 -lm -o quat
	./quat

.PHONY:
	all
