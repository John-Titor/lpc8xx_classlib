CHIP		= LPC810
LIB		= obj/lpc8xx_classlib.a
SRCS		= $(wildcard src/*.cpp)
EXTRA_FMT_SRCS	= $(wildcard include/*.h)


.PHONY:	all test

all: $(LIB) test

test:
	make -C ./test

include make.inc
