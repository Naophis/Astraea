#!/bin/bash


xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x42010e82:0x3fcc2110 0x42078d69:0x3fcc2170