# sw/mld/Makefile — see sw/driver/Makefile for the build-flavour pattern.

CROSS    ?=
CC       ?= $(CROSS)gcc
AR       ?= $(CROSS)ar
RANLIB   ?= $(CROSS)ranlib

CFLAGS   ?= -std=c11 -Wall -Wextra -Os -ffunction-sections -fdata-sections
CPPFLAGS ?= -I. -I../driver
LDFLAGS  ?=
LDLIBS   ?=

LIB       = libdcf77_mld.a
TEST_BIN  = dcf77_mld_test

.PHONY: all lib test clean help

all: $(TEST_BIN)

lib: $(LIB)

$(LIB): dcf77_mld.o
	$(AR) rcs $@ $^
	-$(RANLIB) $@

dcf77_mld.o: dcf77_mld.c dcf77_mld.h ../driver/dcf77.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# Host test build: rebuild the MLD + driver objects with DCF77_HOST_TEST
# so dcf77_io.h picks the function-pointer shim.
dcf77_mld_test.o: dcf77_mld_test.c dcf77_mld.h ../driver/dcf77.h
	$(CC) $(CPPFLAGS) -DDCF77_HOST_TEST $(CFLAGS) -c $< -o $@

dcf77_mld.host.o: dcf77_mld.c dcf77_mld.h ../driver/dcf77.h
	$(CC) $(CPPFLAGS) -DDCF77_HOST_TEST $(CFLAGS) -c dcf77_mld.c -o $@

dcf77.host.o: ../driver/dcf77.c ../driver/dcf77.h ../driver/dcf77_io.h
	$(CC) $(CPPFLAGS) -DDCF77_HOST_TEST $(CFLAGS) -c ../driver/dcf77.c -o $@

$(TEST_BIN): dcf77_mld_test.o dcf77_mld.host.o dcf77.host.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -f *.o *.host.o $(LIB) $(TEST_BIN)

help:
	@echo "Targets:"
	@echo "  make / make all     - host unit-test binary"
	@echo "  make test           - build + run unit tests"
	@echo "  make lib            - build libdcf77_mld.a"
	@echo "  make clean          - remove build artefacts"
