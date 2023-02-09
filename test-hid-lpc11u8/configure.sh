#! /bin/sh

rm mbed_config.cmake
mbed-tools configure --mbed-os-path ../mbed-os --custom-targets-json ../mbed-external-targets/TARGET_NXP/custom_targets.json -m LPC11U68 -t GCC_ARM
mv cmake_build/LPC11U68/develop/GCC_ARM/mbed_config.cmake ./
rm -r cmake_build/