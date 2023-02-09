#! /bin/sh

rm mbed_config.cmake
mbed-tools configure --mbed-os-path ../mbed-os --custom-targets-json ../mbed-external-targets/TARGET_Atmel/custom_targets.json -m XIAO_SAMD21 -t GCC_ARM
mv cmake_build/XIAO_SAMD21/develop/GCC_ARM/mbed_config.cmake ./
rm -r cmake_build/