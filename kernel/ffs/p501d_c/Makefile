CFLAGS ?= -O2 -Wall -Wno-misleading-indentation
p501d: p501d.c ad9361.c ad9361.h ad9361_tables.h
	$(CC) $(CFLAGS) -o $@ p501d.c ad9361.c -lpthread -lm
clean:
	rm -f p501d
