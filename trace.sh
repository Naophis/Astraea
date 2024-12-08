#!/bin/bash


xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x42024b49:0x3fcbf530