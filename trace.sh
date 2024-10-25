#!/bin/bash

xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x42017712:0x3fcbe8c0 0x42017a74:0x3fcbea80