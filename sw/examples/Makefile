# sw/examples/Makefile
#
# Bare-metal Microblaze demo. Cross-build only — there's no host
# equivalent (depends on xil_printf / xparameters.h from the BSP).
#
#   make CROSS=mb-                            # default Vitis toolchain
#   make CROSS=mb- BSP_INC=path/to/bsp/include LIBS="-lxil"
#
# Expected workflow:
#   1. Generate a Vitis BSP for your hardware design (xparameters.h,
#      bspconfig.h, libxil.a).
#   2. Point BSP_INC at the BSP's include dir and adjust LDFLAGS to
#      include its lib path.
#   3. make
#
# A cleaner long-term option is to import dcf77_demo.c, libdcf77.a
# and libdcf77_mld.a into a Vitis application project — that picks
# up the BSP automatically. See README.md for that flow.

CROSS    ?= mb-
CC       ?= $(CROSS)gcc

BSP_INC  ?=
DRV_INC  := -I../driver -I../mld
LIBS     ?= -lxil
LDFLAGS  ?=

CFLAGS   ?= -mlittle-endian -mcpu=v11.0 -mno-xl-soft-mul \
            -std=c11 -Wall -Wextra -Os -ffunction-sections -fdata-sections \
            -ffreestanding

CPPFLAGS += $(DRV_INC) $(if $(BSP_INC),-I$(BSP_INC))

DRV_LIB  := ../driver/libdcf77.a
MLD_LIB  := ../mld/libdcf77_mld.a

ELF      := dcf77_demo.elf

.PHONY: all clean libs help

all: $(ELF)

libs:
	$(MAKE) -C ../driver lib CROSS=$(CROSS)
	$(MAKE) -C ../mld    lib CROSS=$(CROSS)

$(ELF): dcf77_demo.c $(DRV_LIB) $(MLD_LIB)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) \
	    dcf77_demo.c $(MLD_LIB) $(DRV_LIB) $(LIBS) -o $@

$(DRV_LIB) $(MLD_LIB):
	@echo "Missing $@.  Run 'make libs CROSS=$(CROSS)' first."
	@false

clean:
	rm -f $(ELF)

help:
	@echo "Targets:"
	@echo "  make libs CROSS=mb-       build libdcf77 + libdcf77_mld for Microblaze"
	@echo "  make CROSS=mb-            link dcf77_demo.elf against the libs + BSP"
	@echo "  make clean                remove dcf77_demo.elf"
	@echo ""
	@echo "Variables:"
	@echo "  BSP_INC                  path to the Vitis BSP include directory"
	@echo "  LIBS                     extra link libraries (default: -lxil)"
	@echo "  LDFLAGS                  forwarded to gcc"
