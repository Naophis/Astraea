#!/bin/bash


xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
 0x42015a2f:0x3fcca270 0x4201a035:0x3fcca420