CFLAGS = -I../../include -I../../remoteApi -Wall -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -fPIC
LDFLAGS = -lpthread -ldl

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	CFLAGS += -D__linux
	LDFLAGS += -shared
	EXT = so
else
	CFLAGS += -D__APPLE__ 
	LDFLAGS += -dynamiclib -current_version 1.0
	EXT = dylib
endif

all: 
	@rm -f lib/*.$(EXT)
	@rm -f *.o 
	gcc $(CFLAGS) -c ../../remoteApi/extApi.c -o extApi.o
	gcc $(CFLAGS) -c ../../remoteApi/extApiPlatform.c -o extApiPlatform.o
	@mkdir -p lib
	gcc extApi.o extApiPlatform.o -o lib/remoteApi.$(EXT) -lpthread -ldl $(CFLAGS) $(LDFLAGS)
