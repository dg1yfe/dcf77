# sw/driver/Makefile
#
# Two build flavours:
#   make            -> host unit-test binary (test_dcf77)
#   make test       -> build + run the unit tests
#   make lib        -> libdcf77.a for host linking
#   make CROSS=mb-  -> cross-build libdcf77.a for Microblaze
#
# Override CC / CFLAGS as needed for your toolchain.

CROSS    ?=
CC       ?= $(CROSS)gcc
AR       ?= $(CROSS)ar
RANLIB   ?= $(CROSS)ranlib

CFLAGS   ?= -std=c11 -Wall -Wextra -Os -ffunction-sections -fdata-sections
CPPFLAGS ?=
LDFLAGS  ?=
LDLIBS   ?=

OBJS_LIB  = dcf77.o
OBJS_TEST = dcf77.o test_dcf77.o

LIB       = libdcf77.a
TEST_BIN  = test_dcf77

.PHONY: all lib test clean help

all: $(TEST_BIN)

lib: $(LIB)

$(LIB): $(OBJS_LIB)
	$(AR) rcs $@ $^
	-$(RANLIB) $@

dcf77.o: dcf77.c dcf77.h dcf77_io.h
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

# Host unit-test build forces DCF77_HOST_TEST (fake register file).
test_dcf77.o: test_dcf77.c dcf77.h dcf77_io.h
	$(CC) $(CPPFLAGS) -DDCF77_HOST_TEST $(CFLAGS) -c $< -o $@

# The driver object needs the same define so dcf77_io.h picks the
# function-pointer shim. Rebuild as a separate object to avoid
# polluting the libdcf77.a flavour.
test_dcf77.host.o: dcf77.c dcf77.h dcf77_io.h
	$(CC) $(CPPFLAGS) -DDCF77_HOST_TEST $(CFLAGS) -c dcf77.c -o $@

$(TEST_BIN): test_dcf77.o test_dcf77.host.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -f $(OBJS_LIB) $(OBJS_TEST) test_dcf77.host.o $(LIB) $(TEST_BIN)

help:
	@echo "Targets:"
	@echo "  make        - host unit-test binary"
	@echo "  make test   - build + run unit tests"
	@echo "  make lib    - build libdcf77.a"
	@echo "  make clean  - remove build artefacts"
	@echo ""
	@echo "Cross-compile examples:"
	@echo "  make lib CROSS=mb-                     # Microblaze"
	@echo "  make lib CROSS=arm-linux-gnueabihf-    # Linux ARM"
