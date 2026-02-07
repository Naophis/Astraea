#!/bin/bash


xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x42017e20:0x3fcca060 0x4201bb80:0x3fcca210 