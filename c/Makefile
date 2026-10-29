# -Wdouble-promotion matters here: on a Cortex-M7 a stray double silently
# drops the whole expression into software emulation.
CFLAGS = -std=c99 -O2 -Wall -Wextra -Wpedantic -Wdouble-promotion -Wconversion
LDLIBS = -lm

replay: replay.o attitude.o

clean:
	rm -f replay *.o

.PHONY: clean
