#!/bin/bash


xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x42017a14:0x3fcc8800 0x4201b138:0x3fcc89a0 