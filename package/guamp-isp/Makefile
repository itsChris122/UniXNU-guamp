# guamp-isp: build sul PC (x86_64) e per arm64 (Kryo 260 = Cortex-A73/A53, ARMv8.0) con prova sotto qemu-user.
#   make            libreria + test nativi, esegue i test
#   make arm64      libreria + test arm64 (NEON e C puro), esegue i test sotto qemu e confronta NEON con C
#   make bench      tempi nativi
#   make shaders    compila gli shader Vulkan (GLSL -> SPIR-V) con glslangValidator
CC      ?= cc
CFLAGS  ?= -O3 -std=c11 -Wall -Wextra -Werror
ACC     ?= aarch64-linux-gnu-gcc
ACFLAGS ?= -O3 -std=c11 -Wall -Wextra -Werror -mcpu=cortex-a73
QEMU    ?= $(HOME)/moto/tools/qemu-user/x/usr/bin/qemu-aarch64
QEMUCPU ?= cortex-a53
GLSLANG ?= $(HOME)/moto/tools/glslang/x/usr/bin/glslangValidator
SRC  = src/gisp_core.c src/gisp_stats.c src/gisp_fast.c
HDR  = include/gisp.h src/gisp_internal.h
B    = build

all: test

$(B)/x86/libgisp.a: $(SRC) $(HDR)
	@mkdir -p $(@D)
	for f in $(SRC); do $(CC) $(CFLAGS) -Iinclude -c $$f -o $(@D)/$$(basename $$f .c).o || exit 1; done
	ar rcs $@ $(addprefix $(@D)/,$(notdir $(SRC:.c=.o)))

$(B)/x86/test_gisp: tests/test_gisp.c tests/synth.h $(B)/x86/libgisp.a
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=200809L -Iinclude -Itests $< $(B)/x86/libgisp.a -lm -lpthread -o $@

$(B)/x86/gisp_cli: tools/gisp_cli.c tests/synth.h $(B)/x86/libgisp.a
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=200809L -Iinclude -Itests $< $(B)/x86/libgisp.a -lm -lpthread -o $@

test: $(B)/x86/test_gisp $(B)/x86/gisp_cli
	$(B)/x86/test_gisp

bench: $(B)/x86/test_gisp
	$(B)/x86/test_gisp bench 20

# arm64: due varianti, con NEON e con -DGISP_NO_NEON
$(B)/a64/test_gisp: $(SRC) $(HDR) tests/test_gisp.c tests/synth.h
	@mkdir -p $(@D)
	$(ACC) $(ACFLAGS) -static -D_POSIX_C_SOURCE=200809L -Iinclude -Itests $(SRC) tests/test_gisp.c -lm -lpthread -o $@

$(B)/a64c/test_gisp: $(SRC) $(HDR) tests/test_gisp.c tests/synth.h
	@mkdir -p $(@D)
	$(ACC) $(ACFLAGS) -DGISP_NO_NEON -static -D_POSIX_C_SOURCE=200809L -Iinclude -Itests $(SRC) tests/test_gisp.c -lm -lpthread -o $@

arm64: $(B)/a64/test_gisp $(B)/a64c/test_gisp
	$(QEMU) -cpu $(QEMUCPU) $(B)/a64/test_gisp quick
	rm -rf $(B)/dump-neon $(B)/dump-c && mkdir -p $(B)/dump-neon $(B)/dump-c
	$(QEMU) -cpu $(QEMUCPU) $(B)/a64/test_gisp dump $(B)/dump-neon
	$(QEMU) -cpu $(QEMUCPU) $(B)/a64c/test_gisp dump $(B)/dump-c
	for f in $(B)/dump-neon/*.nv12; do cmp $$f $(B)/dump-c/$$(basename $$f) || exit 1; done; echo "NEON = C: identici"

shaders: shaders/gisp_demosaic.comp shaders/gisp_nv12.comp
	@mkdir -p $(B)/spv
	$(GLSLANG) -V --target-env vulkan1.1 shaders/gisp_demosaic.comp -o $(B)/spv/gisp_demosaic.spv
	$(GLSLANG) -V --target-env vulkan1.1 shaders/gisp_nv12.comp -o $(B)/spv/gisp_nv12.spv

clean:
	rm -rf $(B)

.PHONY: all test bench arm64 shaders clean
