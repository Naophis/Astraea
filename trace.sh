#!/bin/bash

xtensa-esp32-elf-addr2line -pfiaC -e build/main.elf \
 0x40375f86:0x3fcbe650 0x40383ecd:0x3fcbe670 0x4038b03d:0x3fcbe690 0x40389de1:0x3fcbe7b0 0x40389c9f:0x3fcbe7d0 0x403767f5:0x3fcbe7f0 0x4038b095:0x3fcbe810 0x4202bc5a:0x3fcbe830 0x4202bc25:0x3fcbe850 0x42013f2b:0x3fcbe870 0x42016a3b:0x3fcbea10 