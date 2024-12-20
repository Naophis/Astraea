#!/bin/bash


xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
0x42012b61:0x3fcbe190 0x42017405:0x3fcbe330