kdtree.exe : kdtree.c
	gcc -Wall -g -o $@ $^

clean :
	rm kdtree.exe
