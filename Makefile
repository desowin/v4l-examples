CC = gcc
EXTRA_CFLAGS ?=
EXTRA_LDFLAGS ?=
CFLAGS := -Wall -g -ansi -std=c99 $(EXTRA_CFLAGS)
LDFLAGS = $(EXTRA_LDFLAGS) -Wl,--as-needed
LDADD := -lSDL
VIEWER_OBJECTS = sdlvideoviewer.o
VIEWER_RGB565X_OBJECTS = sdlvideoviewer-rgb565x.o
M2MTESTER_OBJECTS = sdlm2mtester-rgb565x.o

.PHONY : clean distclean all
%.o : %.c
	$(CC) $(CFLAGS) -c $<

all: sdlvideoviewer sdlvideoviewer-rgb565x sdlm2mtester-rgb565x

sdlvideoviewer: $(VIEWER_OBJECTS) 
	$(CC) $(LDFLAGS) -o $@ $+ $(LDADD)

sdlvideoviewer-rgb565x: $(VIEWER_RGB565X_OBJECTS) 
	$(CC) $(LDFLAGS) -o $@ $+ $(LDADD)

sdlm2mtester-rgb565x: $(M2MTESTER_OBJECTS) 
	$(CC) $(LDFLAGS) -o $@ $+ $(LDADD)

clean:
	rm -f *.o

distclean : clean
	rm -f sdlvideoviewer sdlvideoviewer-rgb565x sdlm2mtester-rgb565x

